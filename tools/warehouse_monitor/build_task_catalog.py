#!/usr/bin/env python3
"""Export the catalogue for offline use from the board's existing task definitions."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shlex

import yaml


MAPS = {
    "outdoor": "20261105_114605_outdoor",
    "indoor": "20261114_101654_indoor_fly_raw_000001",
}
HOLD = "到点悬停，飞手接管降落"


def literal(path, name):
    for statement in ast.parse(path.read_text()).body:
        if isinstance(statement, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name for target in statement.targets
        ):
            return ast.literal_eval(statement.value)
    raise ValueError(f"{path.name}: missing {name}")


def build(scripts, maps):
    nav = scripts / "nav_test"
    square = literal(nav / "nav_global.py", "DEFAULT_ROUTE")
    full = literal(nav / "nav_outdoor_full.py", "ROUTE")
    full_holds = literal(nav / "nav_outdoor_full.py", "HOLDS")
    inspection_file = nav / "inspection_route.yaml"
    inspection = yaml.safe_load(inspection_file.read_text())
    if Path(inspection["map_id"]).name != MAPS["outdoor"]:
        raise ValueError("test11 航线地图不匹配")
    points = [inspection["start"]]
    stops = []
    for leg in inspection["segments"]:
        points.extend(leg["points"])
        stops.append(dict(index=len(points) - 1, name=leg["name"], seconds=leg["hold_s"]))
    stops[-1].update(name="返回后恢复 0°", seconds=literal(nav / "nav_outdoor_inspection.py", "FINAL_HOLD"))
    vertical_def = next(line for line in (scripts / "nav_test9.sh").read_text().splitlines()
                        if line.startswith("DEFAULT_POINTS=("))
    vertical = [tuple(float(x) for x in p.split(","))
                for p in shlex.split(vertical_def[len("DEFAULT_POINTS=("):-1])]
    origin = (0, 0, 0)
    preview = [
        (1, "直线 1 m", "前进 1 m，下一次触发反向", "切出 OFFBOARD 后由飞手接管", [origin, (1, 0, 0)]),
        (2, "1 m 方形", "前、右、后、左各 1 m，每个点停 5 s", HOLD, [origin, (1, 0, 0), (1, -1, 0), (0, -1, 0), origin]),
        (3, "原地降落", "切 OFFBOARD 后悬停 3 s", "PX4 AUTO.LAND，着地后自动上锁", [origin]),
        (4, "2 m / 4 m 往返", "前方 2 m 往返，再前方 4 m 往返，每段停 5 s", HOLD, [origin, (2, 0, 0), origin, (4, 0, 0), origin]),
        (5, "升降、斜飞、转向", "升降 0.5 m、向前斜升 0.5 m 再返回，原地依次左转四次 90°，每段停 5 s", HOLD, [origin, (0, 0, .5), origin, (1, 0, .5), origin]),
        (6, "自动起飞悬停", "地面解锁 3 s 后起飞到约 1 m", "悬停，飞手接管降落", [origin, (0, 0, 1)]),
        (7, "固定方形航点", "每点停 3 s，目标航向 0°", HOLD, square),
        (8, "方形航点与段前转向", "同 test7 方形，在拐点转向，平移到点后无额外停留", HOLD, square),
        (9, "竖直矩形", "沿 X=2 m 平面升到 3 m 后回到 1 m，每点停 3 s", HOLD, vertical),
        (10, "自动起降综合航线", "自动起飞到 1 m 后执行六段固定航点", "返回后 OFFBOARD 慢降，着地确认后上锁", [(0, 0, 1), *full]),
        (11, "连续航段巡检", "四段连续三维航线，中途三处各停 3 s", "返航恢复 0°，停 2 s 后慢降并上锁", points),
        (1, "室内前进 1 m", "沿当前机头前进 1 m，稳定到点后停 5 s", HOLD, [origin, (1, 0, 0)]),
        (2, "室内 0.5 m 方形", "前、右、后、左各 0.5 m，每个点停 2 s", HOLD, [origin, (.5, 0, 0), (.5, -.5, 0), (0, -.5, 0), origin]),
    ]
    indoor_photo = yaml.safe_load((nav / 'indoor3_route.yaml').read_text())
    indoor_map = MAPS['indoor']
    if indoor_photo['map_id'] != indoor_map:
        raise ValueError('室内 test3 航线地图不匹配')
    map_versions = {}
    for map_id in MAPS.values():
        path = maps / map_id / "map_world.pcd"
        info = path.stat()
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        map_versions[map_id] = dict(key=f"{map_id}-{info.st_mtime_ns}-{info.st_size}",
                                    sha256=digest.hexdigest())
    tasks = []
    for index, (number, title, route, finish, path) in enumerate(preview):
        indoor = index >= 11
        map_id = MAPS["indoor" if indoor else "outdoor"]
        fixed = not indoor and number >= 7
        automatic = not indoor and number in (6, 10, 11)
        holds = []
        if fixed and number in (7, 9):
            holds = [dict(index=i, seconds=3) for i in range(len(path))]
        if number == 10:
            holds = [dict(index=i, seconds=s) for i, s in enumerate(full_holds, 1) if s]
        if number == 11:
            holds = stops
        tasks.append(dict(id=f"test{'_indoor' if indoor else ''}{number}", label=f"test{number}",
                          title=title, mapId=map_id, automatic=automatic, route=route, finish=finish,
                          fixed=fixed, points=[list(p[:3]) for p in path], stops=holds))
    if map_versions[indoor_map]['sha256'] != indoor_photo['map_sha256']:
        raise ValueError('室内 test3 航线地图 SHA-256 不匹配')
    stops = [dict(index=i, name=point['name'], seconds=indoor_photo['hold_s'])
             for i, point in enumerate(indoor_photo['points'])]
    tasks.append(dict(id='test_indoor3', label='test3', title='货架四点自动拍照',
                      mapId=indoor_map, automatic=False, fixed=True,
                      route='左下 → 左上 → 右上 → 右下，每点停 3 s 并保存全图',
                      finish=HOLD, points=[point['xyz'] for point in indoor_photo['points']], stops=stops))
    points = [point['xyz'] for point in indoor_photo['points']]
    tasks.append(dict(id='test_indoor3_1', label='test3_1', title='四点 AprilTag 识别',
                      mapId=indoor_map, automatic=False, fixed=True,
                      route='左下 → 左上 → 右上 → 右下，每点拍照并观察 AprilTag 2 s',
                      finish='右下悬停，汇总六个标签的覆盖情况，飞手接管降落',
                      points=points, stops=stops))
    tasks.append(dict(id='test_indoor3_2', label='test3_2', title='双相机 AprilTag 对比',
                      mapId=indoor_map, automatic=False, fixed=True,
                      route='左下 → 左上 → 右上 → 右下，每点同时用 OV13855 和 OV9281 识别 2 s',
                      finish='右下悬停，分别汇总两路标签覆盖，飞手接管降落',
                      points=points, stops=stops))
    tasks.append(dict(id='test_indoor3_3', label='test3_3', title='Tag 1 双相机读 QR',
                      mapId=indoor_map, automatic=False, fixed=False, points=[], stops=[],
                      route='OV9281 锁定 Tag 1，前往正前方 0.8 m，地图航向 −89.5°，两路各读 2 s',
                      finish='到点悬停，切出再切入 OFFBOARD 开始下一轮'))
    tasks.append(dict(id='test_indoor4', label='test4', title='货架标签分批扫码',
                      mapId=indoor_map, automatic=False, fixed=True,
                      route='左下 → 左上 → 右上 → 右下，各点识别后扫描新标签',
                      finish='返回右下朝向货架悬停，飞手接管降落',
                      points=points, stops=stops))
    tasks.insert(11, dict(id="test12", label="test12", title="手飞定位与录包",
                          mapId=MAPS["outdoor"], automatic=False, fixed=False, manual=True,
                          points=[], stops=[], route="全程遥控手飞，实时显示已飞轨迹",
                          finish="落地上锁后关通道 6，待封包后从网页收尾"))
    tasks.insert(12, dict(tasks[11], id="test12_ywl", label="test12 · YWL",
                          title="YWL 手飞定位与录包", observer="ywl"))
    return dict(maps=map_versions, tasks=tasks)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scripts", type=Path, default=Path.home() / "mid360s_ws/scripts")
    parser.add_argument("--maps", type=Path, default=Path.home() / "test_data/mapping")
    args = parser.parse_args()
    print(json.dumps(build(args.scripts, args.maps), indent=2, ensure_ascii=False))
