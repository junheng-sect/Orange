"""Build browser display layers from a map selected by the relocalizer."""
import json
from pathlib import Path

import numpy as np


# These scans share the room and nominal origin used by the display bounds below.
INDOOR_TEST_ROOM_MAPS = {
    "20261021_110311_indoor",
    "20261114_101654_indoor_fly_raw_000001",
}
MIXED_MAP_ROOMS = json.loads(Path(__file__).with_name("mixed_map_rooms.json").read_text())


def read_pcd(path):
    with path.open("rb") as file:
        header = {}
        for _ in range(40):
            line = file.readline().decode("ascii").split()
            if line and not line[0].startswith("#"):
                header[line[0]] = line[1:]
            if line[:1] == ["DATA"]:
                break
        fields = header.get("FIELDS", [])
        if header.get("DATA") != ["binary"] or not {"x", "y", "z"} <= set(fields):
            raise ValueError("地图需要 XYZ 二进制 PCD")
        sizes = list(map(int, header["SIZE"]))
        counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
        layout = dict(zip(fields, zip(header["TYPE"], sizes, counts)))
        if any(layout[key] != ("F", 4, 1) for key in ("x", "y", "z")):
            raise ValueError("PCD 的 XYZ 需为 float32")
        steps = [size * count for size, count in zip(sizes, counts)]
        offsets = np.cumsum([0] + steps[:-1])
        dtype = np.dtype(dict(names=["x", "y", "z"], formats=["<f4"] * 3,
                             offsets=[int(offsets[fields.index(key)]) for key in ("x", "y", "z")],
                             itemsize=sum(steps)))
        data = np.fromfile(file, dtype=dtype)
        if len(data) != int(header["POINTS"][0]) or not len(data):
            raise ValueError("PCD 点数无效")
        points = np.column_stack((data["x"], data["y"], data["z"]))
        return points[np.isfinite(points).all(axis=1)]


def sample(points, size, limit):
    if not len(points):
        return points
    cells = np.floor(points / size).astype(np.int64)
    cells -= cells.min(axis=0)
    _, ny, nz = cells.max(axis=0) + 1
    keys = (cells[:, 0] * ny + cells[:, 1]) * nz + cells[:, 2]
    _, indices = np.unique(keys, return_index=True)
    if len(indices) > limit:
        indices = indices[np.linspace(0, len(indices) - 1, limit, dtype=int)]
    return points[indices]


def display_map(path):
    cloud = read_pcd(path)
    if not len(cloud):
        raise ValueError("地图没有有效点")
    if path.parent.name in INDOOR_TEST_ROOM_MAPS:
        cloud = cloud[np.all((cloud >= [-4.25, -2.05, -.24]) & (cloud <= [2.55, 1.85, 2.68]), axis=1)]
        x, y, z = cloud.T
        floor, roof = z <= -.105, z >= 2.405
        near = (y >= 1.42) & ~(floor | roof)
        walls = ((x < -3.93) | (y < -1.78)) & ~(floor | roof | near)
        layers = {
            "floor": sample(cloud[floor], [.12, .12, .08], 16000),
            "structure": sample(cloud[~(floor | roof | near | walls)], [.048, .048, .04], 80000),
            "walls": sample(cloud[walls], [.075, .075, .065], 26000),
            "near": sample(cloud[near], [.06, .06, .05], 18000),
            "roof": sample(cloud[roof], [.10, .10, .07], 10000),
        }
        bounds = [[-4.25, -2.05, -.24], [2.55, 1.85, 2.68]]
    else:
        # Only a display heuristic. Navigation keeps using its own original map.
        outside = cloud
        room = MIXED_MAP_ROOMS.get(path.parent.name)
        if room:
            lower, upper = room["bounds"]
            inside = np.all((cloud >= lower) & (cloud <= upper), axis=1)
            outside = cloud[~inside]
        ground = np.percentile(outside[:, 2], 10) + .22 if len(outside) else 0
        low = outside[:, 2] <= ground
        layers = {
            "floor": sample(outside[low], [.30, .30, .16], 26000),
            "structure": sample(outside[~low], [.12, .12, .10], 105000),
        }
        if room:
            # Preserve room detail independently of the much larger outdoor cloud.
            indoor = cloud[inside]
            floor = indoor[:, 2] <= room["floorMaxZ"]
            for name, points, size, limit in (
                    ("floor", indoor[floor], [.10, .10, .06], 16000),
                    ("structure", indoor[~floor], [.05, .05, .05], 80000)):
                layers[name] = np.concatenate((layers[name], sample(points, size, limit)))
        bounds = np.percentile(cloud, [.5, 99.5], axis=0).tolist()
    metadata = dict(bounds=bounds, counts={key: len(pts) for key, pts in layers.items()})
    raw = b"".join(np.asarray(pts, dtype="<f4").tobytes() for pts in layers.values())
    return metadata, raw
