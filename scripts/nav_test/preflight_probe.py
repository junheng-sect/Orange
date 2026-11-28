#!/usr/bin/env python3
"""Short-lived, read-only ROS probes for the board startup supervisors."""
import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import threading
import time

import yaml


class Deadline(BaseException):
    """Interrupt blocking ROS XML-RPC/TCP calls, including calls inside rospy."""


@contextmanager
def bounded(seconds):
    def expire(*_):
        raise Deadline()
    previous = signal.signal(signal.SIGALRM, expire)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)


def report(message):
    print('[%s] %s' % (time.strftime('%H:%M:%S'), message), file=sys.stderr, flush=True)


def system_state():
    import rosgraph
    master = rosgraph.Master('/orange_startup_probe')
    try:
        with bounded(5):
            pubs, subs, services = master.getSystemState()
    except (Deadline, rosgraph.MasterException, OSError) as exc:
        raise ValueError('常驻 roscore 不可达（systemctl status mid360s-roscore）') from exc
    nodes = {name for _, owners in pubs + subs + services for name in owners}
    return master, nodes, {topic for topic, _ in pubs + subs}


def param(master, name):
    import rosgraph
    try:
        with bounded(5):
            return master.getParam(name)
    except (Deadline, OSError, rosgraph.MasterException):
        return None


def fcu_connected():
    import rospy
    from mavros_msgs.msg import State
    try:
        with bounded(5):
            rospy.init_node('orange_startup_probe', anonymous=True, disable_signals=True, disable_rostime=True)
            return rospy.wait_for_message('/mavros/state', State, timeout=5).connected
    except (Deadline, rospy.ROSException, OSError):
        return False


def ego_limits(cfg, task):
    overrides = cfg.get('planner_limits', cfg.get('planner_limits_by_task', {}).get(task, {}))
    limits = []
    for key in ('max_vel', 'max_acc'):
        hard = cfg[key]
        value = overrides.get(key, hard)
        if not math.isfinite(value) or not 0 < value <= hard:
            raise ValueError('EGO %s 必须大于零并且不超过导航上限 %s' % (key, hard))
        limits.append(value)
    return (*limits, cfg.get('max_vel_z', -1), cfg.get('max_acc_z', -1))


def write_config(document, folder, preview, destination, task, height):
    cfg = document['config']
    if os.path.expanduser(cfg.get('map_id', '')) != str(folder):
        raise ValueError('导航配置 map_id %s 和地图 %s 不一致' % (cfg.get('map_id'), folder))
    if not preview and cfg.get('deployment_verified') is not True:
        raise ValueError('导航配置 deployment_verified 不是 true，真实输出会被网关拒绝（只预览加 --preview）')
    cfg['map_id'] = str(folder)
    if task == 'test_indoor6':
        cfg['takeoff_height'] = .50
    elif task == 'test_indoor_takeoff_land':
        cfg.update(takeoff_height=float(height), vertical_test_guard=True)
    with open(destination, 'w') as stream:
        yaml.safe_dump(document, stream, allow_unicode=True)
    vertical = ('%.2f m/s、%.2f m/s²' % (cfg['max_vel_z'], cfg.get('max_acc_z', -1))
                if 'max_vel_z' in cfg else '同水平')
    return '导航配置：三维限速 %.2f m/s、%.2f m/s²，竖直 %s，转向 %s°/s，目标范围 %s ~ %s（drone_odom）' % (
        cfg['max_vel'], cfg['max_acc'], vertical, cfg.get('yaw_rate_deg', 10), cfg['bounds_min'], cfg['bounds_max'])


def verified_prior(folder, expected, receipt):
    from drone_mapping.prior_grid import file_stamp, verify
    if expected and re.fullmatch('[0-9a-f]{64}', expected) and receipt == expected + ':' + file_stamp(folder):
        return expected
    actual = verify(folder)
    if expected is not None and actual != expected:
        raise ValueError('先验栅格与冻结任务的地图 SHA-256 不符')
    return actual


def preflight(a):
    from drone_relocalization.matching import MapModel, cache_valid
    from drone_relocalization.settings import load
    sha = ''
    if a.live_grid_only:
        report('[WARN] 本次仅使用实时栅格（--live-grid-only）')
    else:
        sha = verified_prior(a.map, a.expected_sha, os.getenv('INSPECTION_PRIOR_VERIFIED'))
        report('先验栅格 %s（地图 SHA-256 %s）' % (a.map / 'prior_grid.bin', sha))
    master, nodes, topics = system_state()
    for node in ('/drone_reloc', '/car_bridge', '/waypoint_manager', '/offboard_gateway'):
        if node in nodes:
            raise ValueError('已有 %s 在跑（上一次没收尾？），先停掉' % node)
    if a.forbid_topics_regex and any(re.fullmatch(a.forbid_topics_regex, t) for t in topics):
        raise ValueError('相机或读码服务已有任务')
    if not fcu_connected():
        raise ValueError('常驻 MAVROS 没连上飞控（飞控没上电？）：导航要用飞控的位置')
    report('[ OK ] ROS master、MAVROS 和飞控都在')
    if a.ev_mode == 'odom':
        protocol = param(master, '/mavros/fcu_protocol')
        if protocol != 'v2.0':
            raise ValueError('odom 方式要 MAVROS 用 MAVLink2，常驻 MAVROS 是 fcu_protocol=%s' % (protocol or '读不到'))
    document = yaml.safe_load(a.config.read_text())
    report(write_config(document, a.map, a.preview, a.output, a.task, a.height))
    limits = ego_limits(document['config'], a.task)
    report('EGO 三维规划上限：速度 %s m/s，加速度 %s m/s²' % limits[:2])
    report('步骤 1/4：载入地图（缓存不对就重建，室外图约 70 s，只第一次）')
    cfg = load(a.reloc_config)
    if cache_valid(a.map, cfg):
        report('重定位缓存指纹命中')
    else:
        model = MapModel(a.map, cfg)
        report('地图 %d 点，载入 %.1f s，%s' % (len(model.P), model.load_s,
                                           '缓存命中' if model.cache_hit else '已重建缓存'))
    print(json.dumps(dict(prior_map_sha=sha, limits=limits)))


def fusion(a):
    import rospkg
    packages = rospkg.RosPack()
    for name in ('mavros', 'livox_ros_driver2', 'fast_lio', 'fastlio_gravity_bridge', 'rc_record'):
        try:
            packages.get_path(name)
        except rospkg.ResourceNotFound:
            raise ValueError('rospack 找不到包 ' + name)
    if a.winch:
        try:
            packages.get_path('tether_winch_control')
        except rospkg.ResourceNotFound:
            raise ValueError('找不到 tether_winch_control，请在 orange 构建')
        try:
            from tether_winch_control.msg import WinchStatus  # noqa: F401
        except ImportError as exc:
            raise ValueError('绕线器消息不可用，请重新构建 tether_winch_control') from exc
    report('[ OK ] ROS Noetic 与所选功能包齐备')
    try:
        if not a.roscore_active:
            raise ValueError()
        master, nodes, _ = system_state()
    except ValueError:
        raise ValueError('常驻 roscore 不可达：systemctl status %s（没装过就跑 %s/resident/install.sh）' %
                         (a.roscore_unit, Path(__file__).resolve().parents[1]))
    report('[ OK ] 常驻 roscore 可达（master 走 127.0.0.1:11311）')
    if param(master, '/use_sim_time') in (True, 'true'):
        raise ValueError('常驻 master 上 /use_sim_time=true（多半是回放留下的）：rosparam set /use_sim_time false 后再启动')
    if not a.mavros_pid:
        report('[WARN] 常驻 MAVROS 不在运行（systemctl status %s）：核心照常起，飞控收不到外部定位' % a.mavros_unit)
    else:
        if fcu_connected():
            report('[ OK ] 常驻 MAVROS 已连上飞控（mavros_node pid %s）' % a.mavros_pid)
        else:
            report('[WARN] 常驻 MAVROS 在跑但没连上飞控（飞控没上电？）：核心照常起，飞控收不到外部定位')
        if a.ev_mode == 'odom':
            protocol = param(master, '/mavros/fcu_protocol')
            if protocol != 'v2.0':
                raise ValueError('--ev-mode odom 要 MAVROS 用 MAVLink2，常驻 MAVROS 是 fcu_protocol=%s\n改 scripts/launch/mavros.launch 的 fcu_protocol 后 sudo systemctl restart %s，或改用 --ev-mode pose' %
                                 (protocol or '读不到', a.mavros_unit))
            report('[ OK ] 常驻 MAVROS 用 MAVLink2（odom 方式可用）')
    busy = [node for node in a.nodes if node in nodes and node != '/mavros']
    if busy:
        raise ValueError('ROS Master 上已挂着本链路节点：%s\n先停掉旧实例再启动' % ' '.join(busy))
    report('[ OK ] ROS Master 上没有本链路节点')


def check_ev_ctrl(a):
    import rospy
    from mavros_msgs.srv import ParamGet
    try:
        with bounded(a.timeout):
            rospy.init_node('nav_test_ev_ctrl', anonymous=True, disable_signals=True, disable_rostime=True)
            get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
            while not rospy.is_shutdown():
                try:
                    response = get(param_id='EKF2_EV_CTRL')
                    if response.success:
                        value = response.value.integer
                        print(value, flush=True)
                        return 0 if value & 4 else 2
                except rospy.ServiceException:
                    pass
                time.sleep(2)
    except (Deadline, rospy.ROSException, OSError):
        pass
    print("unavailable", flush=True)
    return 1


def wait_odom(a):
    import rospy
    from nav_msgs.msg import Odometry
    pid, start = a.pidfile.read_text().split()
    stat = Path('/proc', pid, 'stat')
    received = threading.Event()
    try:
        with bounded(a.timeout):
            rospy.init_node('nav_test_wait_odom', anonymous=True, disable_signals=True, disable_rostime=True)
            rospy.Subscriber('/fastlio/odom_world', Odometry, lambda _: received.set(), queue_size=1)
            while not rospy.is_shutdown():
                try:
                    fields = stat.read_text().rsplit(')', 1)[1].split()
                    if fields[0] == 'Z' or fields[19] != start:
                        return 1
                except FileNotFoundError:
                    return 1
                if received.wait(.2):
                    return 0
    except Deadline:
        return 2
    return 1


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='command', required=True)
    pf = sub.add_parser('preflight')
    pf.add_argument('--map', required=True, type=Path)
    pf.add_argument('--config', required=True, type=Path)
    pf.add_argument('--reloc-config', required=True)
    pf.add_argument('--output', required=True)
    pf.add_argument('--task', required=True)
    pf.add_argument('--height', default='')
    pf.add_argument('--ev-mode', choices=('pose','odom'), required=True)
    pf.add_argument('--expected-sha')
    pf.add_argument('--forbid-topics-regex')
    pf.add_argument('--preview', action='store_true')
    pf.add_argument('--live-grid-only', action='store_true')
    pf.set_defaults(run=preflight)
    f = sub.add_parser('fusion')
    f.add_argument('--ev-mode', required=True)
    f.add_argument('--roscore-unit', required=True)
    f.add_argument('--mavros-unit', required=True)
    f.add_argument('--mavros-pid', required=True)
    f.add_argument('--roscore-active', action='store_true')
    f.add_argument('--winch', action='store_true')
    f.add_argument('nodes', nargs='+')
    f.set_defaults(run=fusion)
    ev = sub.add_parser('check-ev-ctrl')
    ev.add_argument('--timeout', type=float, default=45.)
    ev.set_defaults(run=check_ev_ctrl)
    od = sub.add_parser('wait-odom')
    od.add_argument('pidfile', type=Path)
    od.add_argument('--timeout', type=float, default=180.)
    od.set_defaults(run=wait_odom)
    limits = sub.add_parser('ego-limits')
    limits.add_argument('config', type=Path)
    limits.add_argument('task')
    limits.set_defaults(run=lambda a: print(*ego_limits(yaml.safe_load(a.config.read_text())['config'], a.task)))
    cfg = sub.add_parser('write-config')
    for arg in ('config','map','preview','output','task','height'):
        cfg.add_argument(arg, nargs='?' if arg == 'height' else None)
    cfg.set_defaults(run=lambda a: print(write_config(yaml.safe_load(Path(a.config).read_text()), a.map,
                                                      a.preview == '1', a.output, a.task, a.height)))
    a = parser.parse_args()
    try:
        return a.run(a)
    except (ValueError, TypeError, KeyError, OSError, yaml.YAMLError) as exc:
        report('[FAIL] ' + str(exc))
        return 3 if a.command == 'check-ev-ctrl' else 1


if __name__ == '__main__':
    sys.exit(main())
