"""Mission loading shared by the board launcher and the desktop preset catalogue."""
import hashlib
import math
from pathlib import Path
import re

import yaml


def fields(value, required, optional=()):
    if not isinstance(value, dict):
        raise ValueError('配置段必须是字典')
    missing = set(required) - value.keys()
    extra = value.keys() - set(required) - set(optional)
    if missing or extra:
        raise ValueError('缺少字段 %s，多余字段 %s' % (missing, extra))


def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('需要有限数值，收到 %r' % value)
    return value


def vector(value):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError('需要三维坐标')
    return [number(v) for v in value]


def boolean(value):
    if type(value) is not bool:
        raise ValueError('需要布尔值，收到 %r' % value)


def label(value):
    if not isinstance(value, str) or not value.strip() or len(value) > 80 or any(
            not c.isprintable() or c in '/\\' for c in value):
        raise ValueError('名称无效')


def segments(points):
    start = 0
    for end, point in enumerate(points):
        if point['action'] != 'pass' or end == len(points) - 1:
            yield list(range(start, end + 1))
            start = end + 1


def load(path):
    raw = Path(path).read_bytes()
    cfg = yaml.safe_load(raw)
    fields(cfg, ('name', 'map', 'flight', 'waypoints', 'recording'), ('title',))
    if not isinstance(cfg['name'], str) or not re.fullmatch('[a-z][a-z0-9_]{0,47}', cfg['name']):
        raise ValueError('name 需要小写英文任务标识')
    label(cfg.setdefault('title', cfg['name']))
    m, f, rec = cfg['map'], cfg['flight'], cfg['recording']
    fields(m, ('dir', 'sha256', 'reloc_config', 'ev_mode'), ('map_frame',))
    if (not isinstance(m['sha256'], str) or not re.fullmatch('[0-9a-f]{64}', m['sha256'])
            or m['ev_mode'] not in ('pose', 'odom')):
        raise ValueError('地图 SHA-256 或 EV 模式无效')
    for key in ('dir', 'reloc_config', 'map_frame'):
        if key in m and (not isinstance(m[key], str) or not m[key].strip()):
            raise ValueError('地图路径字段无效：' + key)
    flags = ('auto_takeoff', 'return_home', 'auto_land')
    limits = ('max_vel', 'max_acc', 'planner_max_vel', 'planner_max_acc', 'max_vel_z', 'max_acc_z')
    fields(f, flags + limits + ('max_control_z', 'bounds_min', 'bounds_max', 'takeoff_delay_s', 'takeoff_height'))
    for k in flags:
        boolean(f[k])
    if f['auto_land'] and not f['return_home'] or f['return_home'] and not f['auto_takeoff']:
        raise ValueError('返航需要地面起飞参考；自动慢降要求先返航')
    if any(number(f[k]) <= 0 for k in limits) or number(f['takeoff_delay_s']) <= 0:
        raise ValueError('速度、加速度和倒计时必须大于零')
    if not .3 <= number(f['takeoff_height']) <= 2.5 or max(f['max_vel_z'], f['max_acc_z']) > 1.:
        raise ValueError('网关要求起飞高度在 0.30–2.50 m，竖直速度与加速度上限均不能大于 1')
    for key in ('max_vel', 'max_acc'):
        if f['planner_' + key] > f[key] or f[key + '_z'] > f[key]:
            raise ValueError('规划或竖直上限超过三维上限 ' + key)
    if any(b <= a for a, b in zip(vector(f['bounds_min']), vector(f['bounds_max']))):
        raise ValueError('导航边界无效')
    ceiling = number(f['max_control_z'])
    fields(rec, ('save_frames', 'capture_waypoints', 'record_nav_cloud', 'dll_observer'))
    for k in ('capture_waypoints', 'record_nav_cloud', 'dll_observer'):
        boolean(rec[k])
    if rec['save_frames'] not in ('off', 'failed', 'all'):
        raise ValueError("save_frames 使用字符串 'off'、'failed' 或 'all'")
    points = cfg['waypoints']
    if not isinstance(points, list) or not 1 <= len(points) <= 50:
        raise ValueError('任务需要 1 至 50 个航点')
    for p in points:
        fields(p, ('name', 'xyz', 'action'), ('yaw_deg', 'hold_s'))
        label(p['name'])
        if vector(p['xyz'])[2] > ceiling:
            raise ValueError(p['name'] + ' 超出地图限高')
        if p['action'] == 'pass':
            if 'yaw_deg' in p or 'hold_s' in p:
                raise ValueError('连续通行点只指定 XYZ')
        elif p['action'] in ('hover', 'scan_shelf'):
            number(p.get('yaw_deg'))
            hold = number(p.get('hold_s'))
            if hold < (3 if p['action'] == 'scan_shelf' else 0):
                raise ValueError('停留过短，扫码需要 1 s 稳定和至少 2 s 观察')
        else:
            raise ValueError('无法识别航点动作 ' + str(p['action']))
    for group in segments(points):
        for a, b in zip(group, group[1:]):
            if math.dist(points[a]['xyz'], points[b]['xyz']) < .2:
                raise ValueError('连续航段相邻点距离小于 0.20 m')
    cfg['mission_sha256'] = hashlib.sha256(raw).hexdigest()
    return cfg


def location(value, base):
    if value.startswith('package://'):
        import rospkg
        package, sep, name = value[10:].partition('/')
        if not sep:
            raise ValueError('package:// 缺少包内路径')
        return (Path(rospkg.RosPack().get_path(package)) / name).resolve(strict=True)
    return (base / Path(value).expanduser()).resolve(strict=True)


def resolve(path, map_override, share):
    from drone_mapping.prior_grid import verify
    from drone_navigation.core import deployment
    from car_bridge import calib
    from drone_relocalization.settings import load as load_reloc
    cfg = load(path)
    m, f = cfg['map'], cfg['flight']
    base = path.parent
    folder = location(str(map_override), Path.cwd()) if map_override else location(m['dir'], base)
    if not (folder / 'meta.json').is_file() or verify(folder) != m['sha256']:
        raise ValueError('任务地图指纹或 meta.json 不匹配')
    if 'map_frame' in m:
        frame_file = location(m['map_frame'], base)
        frames = calib.load(str(frame_file))
        if frames['drone_map'] != Path(m['dir']).name:
            raise ValueError('地图标定对应另一张图')
        frames = yaml.safe_load(frame_file.read_text())
        frames['drone_map'] = folder.name
        m['frames'] = frames
    m['dir'] = str(folder)
    m['reloc_config'] = str(location(m['reloc_config'], base))
    load_reloc(m['reloc_config'])
    nav = yaml.safe_load((share / 'config/airframe.yaml').read_text())
    nav.update({k: f[k] for k in ('takeoff_height', 'max_vel', 'max_acc', 'max_vel_z', 'max_acc_z',
                                   'bounds_min', 'bounds_max')})
    nav.update(map_id=str(folder), allow_short_goals=True)
    deployment(nav)
    cfg['navigation'] = nav
    return cfg
