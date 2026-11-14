"""Versioned inspection points and geometric route previews on the desktop."""
from contextlib import contextmanager
import fcntl
from functools import lru_cache
import heapq
import itertools
import json
import math
from threading import RLock

import numpy as np

from map_cloud import read_pcd


STEP = .10
HORIZONTAL_CLEARANCE = .40
VERTICAL_CLEARANCE = .28
SUPPORT_RADIUS = .55
MAX_CELLS = 360_000
MAX_POINTS = 100
NEIGHBORS = [(dx, dy, math.hypot(dx, dy)) for dx, dy in itertools.product((-1, 0, 1), repeat=2) if dx or dy]


class Conflict(ValueError):
    pass


def number(value, low, high):
    if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError("坐标、航向或停留时间超出范围")
    return float(value)


def validate_points(points):
    if not isinstance(points, list) or len(points) > MAX_POINTS:
        raise ValueError("每个地图版本最多保存 100 个点")
    clean = []
    ids = set()
    for point in points:
        ident, name, position = point["id"], point["name"], point["position"]
        if not isinstance(ident, str) or not 1 <= len(ident) <= 60 or ident in ids:
            raise ValueError("巡检点 ID 重复或无效")
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 60:
            raise ValueError("点位名称应为 1–60 个字符")
        if not isinstance(position, list) or len(position) != 3 or type(point["selected"]) is not bool:
            raise ValueError("巡检点格式不正确")
        ids.add(ident)
        clean.append(dict(id=ident, name=name.strip(), position=[number(v, -1000, 1000) for v in position],
                          yaw=number(point["yaw"], -180, 180), dwell=number(point["dwell"], 0, 3600),
                          selected=point["selected"]))
    return clean


def expand(mask, cells):
    """Square expansion in XY; conservative for arbitrary horizontal heading."""
    padded = np.pad(mask, cells)
    window = np.lib.stride_tricks.sliding_window_view(padded, 2 * cells + 1, axis=0)
    along_x = window.any(axis=-1)
    window = np.lib.stride_tricks.sliding_window_view(along_x, 2 * cells + 1, axis=1)
    return window.any(axis=-1)


class MapGrid:
    def __init__(self, cloud, bounds):
        lower, upper = np.asarray(bounds, dtype=float)
        self.origin = np.floor(lower[:2] / STEP) * STEP
        self.lo, self.hi = lower, upper
        self.shape = tuple(np.ceil((upper[:2] - self.origin) / STEP).astype(int))
        if min(self.shape) < 1 or math.prod(self.shape) > MAX_CELLS:
            raise ValueError("地图范围过大或无效，无法建立 0.10 m 预览栅格")
        ij = np.floor((cloud[:, :2] - self.origin) / STEP).astype(int)
        inside = np.all((ij >= 0) & (ij < self.shape), axis=1)
        self.x, self.y = ij[inside].T
        self.z = cloud[inside, 2]
        support = np.zeros(self.shape, dtype=bool)
        support[self.x, self.y] = True
        # A PCD contains returns, not a visibility ray history. Projected
        # support limits extrapolation outside the scan's surveyed footprint.
        self.support = expand(support, math.floor(SUPPORT_RADIUS / STEP))
        self.free = lru_cache(maxsize=24)(self.build_free)

    def build_free(self, low, high):
        hits = (self.z >= low - VERTICAL_CLEARANCE) & (self.z <= high + VERTICAL_CLEARANCE)
        occupied = np.zeros(self.shape, dtype=bool)
        occupied[self.x[hits], self.y[hits]] = True
        cells = math.ceil(HORIZONTAL_CLEARANCE / STEP)
        blocked = expand(occupied | ~self.support, cells)
        ix, iy = np.ogrid[:self.shape[0], :self.shape[1]]
        for axis, indices in enumerate((ix, iy)):
            begin = self.origin[axis] + indices * STEP
            blocked |= ((begin < self.lo[axis] + HORIZONTAL_CLEARANCE - 1e-8)
                        | (begin + STEP > self.hi[axis] - HORIZONTAL_CLEARANCE + 1e-8))
        return ~blocked

    def cell(self, xy):
        return tuple(np.floor((np.asarray(xy) - self.origin) / STEP).astype(int))

    def xy(self, cell):
        return self.origin + (np.asarray(cell) + .5) * STEP

    def in_bounds(self, cell):
        return all(0 <= value < length for value, length in zip(cell, self.shape))

    def height_ok(self, z):
        return self.lo[2] + VERTICAL_CLEARANCE <= z <= self.hi[2] - VERTICAL_CLEARANCE


def find_path(free, start, target):
    if start == target:
        return [start]
    queue = [(math.dist(start, target), 0., start)]
    parents = {}
    best = {start: 0.}
    nx, ny = free.shape
    while queue:
        _, length, (x, y) = heapq.heappop(queue)
        cell = x, y
        if length != best[cell]:
            continue
        if cell == target:
            path = [cell]
            while cell != start:
                cell = parents[cell]
                path.append(cell)
            return list(reversed(path))
        for dx, dy, cost in NEIGHBORS:
            a, b = x + dx, y + dy
            if not (0 <= a < nx and 0 <= b < ny and free[a, b]):
                continue
            if dx and dy and not (free[a, y] and free[x, b]):
                continue
            nxt = a, b
            new = length + cost
            if new < best.get(nxt, math.inf):
                parents[nxt], best[nxt] = cell, new
                heapq.heappush(queue, (new + math.dist(nxt, target), new, nxt))
    return None


def corners(path):
    if len(path) <= 2:
        return path
    keep = [path[0]]
    for a, b, c in zip(path, path[1:], path[2:]):
        if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
            keep.append(b)
    return keep + [path[-1]]


def preview(grid, points, cruise):
    if len(points) < 2:
        raise ValueError("至少勾选两个巡检点")
    cruise = number(cruise, -1000, 1000)
    legs, errors = [], []
    if not grid.height_ok(cruise):
        return dict(cruise=cruise, legs=[], errors=["巡航 Z 超出地图的垂直净空范围"], length=0, complete=False)
    free = grid.free(cruise, cruise)
    endpoints = []
    for i, point in enumerate(points, 1):
        x, y, z = point["position"]
        cell = grid.cell([x, y])
        error = None
        if not grid.height_ok(z):
            error = "点位 Z 超出地图的垂直净空范围"
        elif not grid.in_bounds(cell):
            error = "点位超出地图范围"
        elif not free[cell]:
            error = "巡航高度处被占据、超出边界净空或缺少扫描覆盖"
        elif not grid.free(min(z, cruise), max(z, cruise))[cell]:
            error = "点位与巡航高度之间的垂直段净空不足"
        if error:
            errors.append(f"{i}. {point['name']} · {error}")
        endpoints.append((cell, error))

    for i, (a, b) in enumerate(zip(points, points[1:])):
        ca, ea = endpoints[i]
        cb, eb = endpoints[i + 1]
        if ea or eb:
            continue
        path = find_path(free, ca, cb)
        if path is None:
            errors.append(f"{a['name']} → {b['name']} · 当前巡航高度下没有连通的通道")
            continue
        start, end = a["position"], b["position"]
        points3 = [start, [*start[:2], cruise]]
        if ca != cb:
            points3.extend([*grid.xy(c), cruise] for c in corners(path))
        points3 += [[*end[:2], cruise], end]
        positions = [list(map(float, points3[0]))]
        for p in points3[1:]:
            if math.dist(positions[-1], p) > 1e-6:
                positions.append(list(map(float, p)))
        legs.append(dict(start=a["id"], end=b["id"], positions=positions,
                         length=sum(math.dist(p, q) for p, q in zip(positions, positions[1:]))))
    return dict(cruise=cruise, legs=legs, errors=errors, length=sum(leg["length"] for leg in legs),
                complete=not errors, dwell=sum(p["dwell"] for p in points))


class Inspection:
    def __init__(self, cache, rviz_maps, map_key, fingerprint, temporary_path):
        self.cache = cache
        self.rviz_maps = rviz_maps
        self.map_key = map_key
        self.fingerprint = fingerprint
        self.temporary_path = temporary_path
        self.lock = RLock()
        self.grid_lock = RLock()
        self.grid_cache = None

    def catalog(self):
        found = []
        for path in sorted((self.cache / "display").glob("*.bin"), reverse=True):
            match = self.map_key.fullmatch(path.stem)
            if match:
                found.append(dict(key=path.stem, id=match[1], version=match[2],
                                  planning=path.with_suffix(".json").is_file()))
        return found

    def map_path(self, key):
        if not isinstance(key, str) or not self.map_key.fullmatch(key):
            raise ValueError("地图版本无效")
        path = self.cache / "display" / f"{key}.bin"
        if not path.is_file():
            raise FileNotFoundError("未缓存这个地图版本")
        return path

    def read(self, key):
        self.map_path(key)
        path = self.cache / "inspection" / f"{key}.json"
        if path.is_file():
            return json.loads(path.read_text())
        return dict(revision=0, points=[], preview=None)

    def write(self, key, state):
        path = self.cache / "inspection" / f"{key}.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        with self.temporary_path(path.parent) as temp:
            temp.write_text(json.dumps(state, ensure_ascii=False, allow_nan=False, indent=2) + "\n")
            temp.replace(path)

    @contextmanager
    def edit_lock(self, key):
        self.map_path(key)
        folder = self.cache / "inspection"
        folder.mkdir(parents=True, exist_ok=True)
        with self.lock, (folder / f"{key}.lock").open("a") as file:
            fcntl.flock(file.fileno(), fcntl.LOCK_EX)
            yield

    def save(self, key, revision, points):
        points = validate_points(points)
        with self.edit_lock(key):
            if self.read(key)["revision"] != revision:
                raise Conflict("点位已在另一个窗口更新，请重新加载地图后再编辑")
            state = dict(revision=revision + 1, points=points, preview=None)
            self.write(key, state)
            return state

    def grid(self, key):
        with self.grid_lock:
            if self.grid_cache and self.grid_cache[0] == key:
                return self.grid_cache[1]
            display = self.map_path(key)
            if not display.with_suffix(".json").is_file():
                raise ValueError("缺少原始 PCD 的版本校验，请先重新同步该地图")
            manifest = json.loads(display.with_suffix(".json").read_text())
            map_id = self.map_key.fullmatch(key)[1]
            candidates = (self.cache / "maps" / map_id / "map_world.pcd",
                          self.rviz_maps / map_id / "map_world.pcd")
            path = next((p for p in candidates if p.is_file() and self.fingerprint(p) == manifest["sha256"]), None)
            if path is None:
                raise ValueError("本机原始 PCD 与地图版本不符，请重新同步")
            cloud = read_pcd(path)
            if self.fingerprint(path) != manifest["sha256"]:
                raise ValueError("读取时原始 PCD 已更新，请重新同步")
            with display.open("rb") as file:
                header = json.loads(file.read(int.from_bytes(file.read(4), "little")))
            grid = MapGrid(cloud, header["bounds"])
            self.grid_cache = key, grid
            return grid

    def plan(self, key, revision, cruise):
        with self.lock:
            state = self.read(key)
            if state["revision"] != revision:
                raise Conflict("点位已更新，请重新加载后生成路线")
        points = [p for p in state["points"] if p["selected"]]
        if len(points) < 2:
            raise ValueError("至少勾选两个巡检点")
        result = preview(self.grid(key), points, cruise)
        with self.edit_lock(key):
            current = self.read(key)
            if current["revision"] != revision:
                raise Conflict("规划时点位已更新，请重新生成路线")
            current["preview"] = result
            current["revision"] += 1
            self.write(key, current)
            return current
