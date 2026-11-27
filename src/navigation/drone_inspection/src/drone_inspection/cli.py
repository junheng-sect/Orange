"""Public CLI. Resolution and dry-run never initialize a ROS node."""
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import sys
import uuid

import rospkg
import yaml
from .config import resolve, segments


def workspace(share):
    for p in (*share.parents, *(Path(d).parent for d in os.getenv('CMAKE_PREFIX_PATH', '').split(':') if d)):
        if (p / 'scripts/nav_test/common.sh').is_file():
            return p
    raise ValueError('找不到包含 fusion.sh 和导航启动器的 Orange 工作空间')


def main():
    parser = argparse.ArgumentParser(description='运行 YAML 巡检任务')
    parser.add_argument('--mission', required=True, type=Path)
    parser.add_argument('--map', type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--preview', action='store_true')
    mode.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    try:
        share = Path(rospkg.RosPack().get_path('drone_inspection'))
        path = args.mission.expanduser()
        if not path.is_absolute() and not path.is_file():
            path = share / path
        path = path.resolve(strict=True)
        cfg = resolve(path, args.map, share)
        expected = os.getenv('WAREHOUSE_MISSION_SHA256')
        if expected is not None and expected != cfg['mission_sha256']:
            raise ValueError('网页所选任务版本已改变')
        if args.dry_run:
            print(json.dumps(dict(mission=cfg, segments=[[cfg['waypoints'][i]['name'] for i in group]
                  for group in segments(cfg['waypoints'])]), ensure_ascii=False, indent=2, allow_nan=False))
            return 0
        if sys.stdin.isatty() and not os.getenv('TMUX') and shutil.which('tmux'):
            command = ['rosrun', 'drone_inspection', 'run_mission', '--mission', str(path)]
            if args.map:
                command.extend(['--map', str(args.map.expanduser().resolve())])
            if args.preview:
                command.append('--preview')
            ws = workspace(share)
            launch = 'source %s && exec %s' % (shlex.quote(str(ws / 'devel/setup.bash')), shlex.join(command))
            os.execlp('tmux', 'tmux', 'new-session', '-s', 'nav', 'bash -c ' + shlex.quote(launch))
        rec, m = cfg['recording'], cfg['map']
        os.environ.update(INSPECTION_CONFIG=json.dumps(cfg, ensure_ascii=False, allow_nan=False),
                          INSPECTION_WORKSPACE=str(workspace(share)), INSPECTION_PKG=str(share),
                          TEST_NAME='mission_' + cfg['name'], DEFAULT_MAP=m['dir'],
                          DEFAULT_RELOC_CONFIG=m['reloc_config'], DEFAULT_EV_MODE=m['ev_mode'],
                          INSPECTION_SUFFIX=uuid.uuid4().hex[:10],
                          INSPECTION_CAMERA=str(int(any(p['action']=='scan_shelf' for p in cfg['waypoints']))),
                          INSPECTION_DLL=str(int(rec['dll_observer'])), INSPECTION_FRAMES=rec['save_frames'],
                          RC_RECORD_NAV_CLOUD=str(rec['record_nav_cloud']).lower())
        entry = str(share / 'lib/bringup.sh')
        os.execl('/bin/bash', 'bash', entry, *(['--preview'] if args.preview else []))
    except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError, rospkg.ResourceNotFound) as error:
        parser.exit(2, '巡检配置错误：%s\n' % error)
