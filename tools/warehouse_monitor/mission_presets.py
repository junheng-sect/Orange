"""Read the same mission definition as the CLI without requiring ROS on the desktop."""
import importlib.util
import json
from pathlib import Path
import sys

import yaml


def extend(data, root, cache=None, board=False):
    package = root.parents[1] / 'src/navigation/drone_inspection'
    directory = package / 'missions'
    if not directory.is_dir():
        return data
    spec = importlib.util.spec_from_file_location('inspection_config', package / 'src/drone_inspection/config.py')
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    for path in sorted(directory.glob('*.yaml')):
        try:
            mission = config.load(path)
            map_dir = (path.parent / Path(mission['map']['dir']).expanduser()).resolve()
            map_id = map_dir.name
            sha = mission['map']['sha256']
            version = data['maps'].get(map_id, {})
            key = version.get('key') if version.get('sha256') == sha else None
            if board:
                info = (map_dir / 'map_world.pcd').stat()
                key = f'{map_id}-{info.st_mtime_ns}-{info.st_size}'
            elif cache is not None:
                key = None
                for manifest in sorted((cache / 'display').glob('*.json'), reverse=True):
                    if (manifest.stem.rsplit('-', 2)[0] == map_id and manifest.with_suffix('.bin').is_file()
                            and json.loads(manifest.read_text()).get('sha256') == sha):
                        key = manifest.stem
                        break
            if not key:
                continue
            f, pts = mission['flight'], mission['waypoints']
            digest = mission['mission_sha256']
            data['maps'].setdefault(map_id, dict(key=key, sha256=sha))
            data['tasks'].append(dict(id=f'mission:{path.stem}:{digest}', mission=path.name,
                missionSha256=digest, mapSha256=sha, mapKey=key, mapId=map_id,
                title=mission['title'], label='巡检', automatic=f['auto_takeoff'], fixed=True,
                delay=f['takeoff_delay_s'], route=' → '.join(p['name'] for p in pts),
                finish=('等高返航，' if f['return_home'] else '') + ('慢降至着地上锁' if f['auto_land'] else '终点悬停，飞手接管降落'),
                points=[p['xyz'] for p in pts],
                stops=[dict(index=i, name=p['name'], seconds=p['hold_s']) for i,p in enumerate(pts) if p['action'] != 'pass']))
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            print(f'跳过巡检预设 {path.name}：{error}', file=sys.stderr)
    return data
