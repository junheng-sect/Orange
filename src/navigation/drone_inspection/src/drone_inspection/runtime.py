"""Private phases of the supervised board process."""
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import yaml
from .records import Records


def arguments(cfg, live, archive, events, preview):
    return SimpleNamespace(mission=cfg, points=[(*p['xyz'], p.get('yaw_deg', 0.)) for p in cfg['waypoints']],
                           out=str(live), archive=str(archive), events=events, preview=preview,
                           hold=0., delay=cfg['flight']['takeoff_delay_s'], check_segments=False,
                           max_start_offset=math.inf)


def main():
    cfg = json.loads(os.environ['INSPECTION_CONFIG'])
    live = Path(os.environ['INSPECTION_LIVE'])
    archive = Path(os.environ['INSPECTION_ARCHIVE'])
    preview = os.environ['INSPECTION_PREVIEW'] == '1'
    phase = sys.argv[1]
    # Shared, guarded Orange navigation clients also used by the historical flight entry points.
    sys.path.insert(0, str(Path(os.environ['INSPECTION_WORKSPACE']) / 'scripts/nav_test'))
    if phase == 'probe':
        from .vision import probe
        probe(archive)
        return 0
    events = Records(live, archive.name, cfg, preview)
    if phase == 'setup':
        folder = Path(sys.argv[2])
        nav = dict(cfg['navigation'], planner_limits={k: cfg['flight']['planner_' + k] for k in ('max_vel','max_acc')})
        (folder / 'navigation.yaml').write_text(yaml.safe_dump(dict(config=nav)))
        if 'frames' in cfg['map']:
            (folder / 'frames.yaml').write_text(yaml.safe_dump(cfg['map']['frames']))
        (archive / 'mission.resolved.yaml').write_text(yaml.safe_dump(cfg, allow_unicode=True))
        (archive / '.inspection_live').write_text(str(live))
        return 0
    if phase == 'finalize':
        if events.state['end'] == 'RUNNING':
            reason = sys.argv[2] if len(sys.argv) > 2 else ''
            events.update(end='ABORTED' if reason else 'INTERRUPTED', phase='ABORTED',
                          reason=reason or '任务在执行结束前收尾')
        return 0
    try:
        release = Path(sys.argv[2]) if phase == 'warm' else None
        auto = cfg['flight']['auto_takeoff'] and not preview
        gate = release.with_name('gateway_started') if release and auto else release

        def wait_release(_state):
            import rospy
            while not gate.is_file():
                if rospy.is_shutdown() or not release.parent.is_dir():
                    raise RuntimeError('巡检启动已收尾')
                time.sleep(.05)
        barrier = wait_release if release is not None else None
        args = arguments(cfg, live, archive, events, preview)
        if auto:
            from .lifecycle import FlightLifecycle
            from nav_outdoor_full import run_full
            code = run_full(args, FlightLifecycle, 'inspection', wait_release=barrier,
                            released=release.is_file if release else None)
        else:
            from .route import MissionRoute
            from nav_global import serve
            code = serve('inspection_route', args, MissionRoute, wait_release=barrier)
        if events.state['end'] == 'RUNNING':
            events.update(end='SUCCEEDED' if code == 0 else 'ABORTED', phase='DONE' if code == 0 else 'ABORTED')
        return 0 if events.state['end'] == 'SUCCEEDED' else 1
    except (Exception, KeyboardInterrupt) as error:
        events.update(end='INTERRUPTED' if isinstance(error, KeyboardInterrupt) else 'ABORTED',
                      phase='ABORTED', reason=str(error) or '操作员中断')
        return 130 if isinstance(error, KeyboardInterrupt) else 1


if __name__ == '__main__':
    raise SystemExit(main())
