#!/usr/bin/env python3
"""Score a complete outdoor A-B-A replay in common sensor-time windows."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import tempfile

import numpy as np
import rosbag
import rospkg
from drone_mapping import mapio
from drone_relocalization.geometry import quat_R, rotz, unpack, yaw

from aba_reference import register, static_cloud

LIDAR = '/livox/lidar'
IMU = '/livox/imu'
METHODS = ('fastlio2', 'dll', 'ywl')


def read_json(path):
    return json.loads(path.read_text())


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(2**20), b''):
            digest.update(block)
    return digest.hexdigest()


def replay_inputs(pair):
    dll = read_json(pair / 'dll_seed.json')
    run = read_json(pair / 'ywl/run.json')
    seed = read_json(Path(run['seed']))
    bag, folder = Path(dll['bag']), Path(dll['map'])
    if (Path(run['bag']).resolve() != bag.resolve() or Path(run['map']).resolve() != folder.resolve()
            or run.get('requested_seconds') is not None):
        raise ValueError('DLL and YWL must use the same full bag and tracking map')
    if not (seed['result'].get('ok') and seed['result'].get('confirmed')
            and seed['result']['transform'] == dll['results'][0]['transform']
            and seed['anchor_ns'] == dll['anchor_ns']):
        raise ValueError('Common confirmed A seed required')
    applied = read_json(pair / 'dll/applied_seed.json')
    tf = seed['result']['transform']
    if (int((pair / 'dll/seed_anchor_ns.txt').read_text()) != seed['anchor_ns']
            or applied != tf['translation'] + tf['q_xyzw']):
        raise ValueError('Applied DLL seed differs from the confirmed seed')
    digests = [line.split(maxsplit=1)[0] for line in (pair / 'dll/inputs.sha256').read_text().splitlines()]
    hashes = {name: sha(folder / name) for name in ('map_world.pcd', 'meta.json')}
    bag_hash = sha(bag)
    if hashes != seed['map_sha256'] or digests != [bag_hash, hashes['map_world.pcd']]:
        raise ValueError('Bag or map hashes differ from replay inputs')
    return bag, folder, seed, dict(bag=bag_hash, **hashes)


def sensors(bag):
    topics = bag.get_type_and_topic_info().topics
    for name, ros_type in [(LIDAR, 'livox_ros_driver2/CustomMsg'), (IMU, 'sensor_msgs/Imu')]:
        if name not in topics or topics[name].msg_type != ros_type:
            raise ValueError('Missing or wrong ROS type for ' + name)
    timeline = {LIDAR: [], IMU: []}
    received = []
    for topic, raw, rec in bag.read_messages(topics=list(timeline), raw=True):
        sec, ns = struct.unpack_from('<II', raw[1], 4)
        timeline[topic].append(sec + ns*1e-9)
        if topic == LIDAR:
            received.append(rec.to_sec())
    stats = {}
    for topic, values in timeline.items():
        t = np.asarray(values)
        gap = np.diff(t)
        if len(t) < 2 or np.any(gap <= 0):
            raise ValueError('Missing or nonmonotonic sensor timestamps on ' + topic)
        limit = .25 if topic == LIDAR else .05
        stats[topic] = dict(count=len(t), first=t[0], last=t[-1], max_gap_s=gap.max(),
                            gaps_over_limit=int((gap > limit).sum()), gap_limit_s=limit)
    laser, imu = np.array(timeline[LIDAR]), np.array(timeline[IMU])
    ok = (imu[0] <= laser[0]+.15 and imu[-1] >= laser[-1]-.05
          and not any(s['gaps_over_limit'] for s in stats.values()))
    return laser, np.array(received), dict(valid=bool(ok), topics=stats)


def stops(bag):
    rows = []
    for _, msg, _ in bag.read_messages(topics=['/Odometry']):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        rows.append([msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    data = np.array(rows)
    if len(data) < 10 or not np.isfinite(data).all() or np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError('Recorded /Odometry needed to identify physical stops')
    t, pos, quat = data[:, 0], data[:, 1:4], data[:, 4:]
    quat = quat / np.linalg.norm(quat, axis=1)[:, None]
    intervals = []
    begin = None
    for i, now in enumerate(t):
        left, right = np.searchsorted(t, [now-.5, now+.5])
        local = slice(left, right)
        still = (right-left >= 5 and np.max(np.diff(t[local])) <= .24
                 and np.linalg.norm(np.ptp(pos[local], axis=0)) <= .06
                 and np.min(abs(quat[local] @ quat[i])) >= math.cos(math.radians(1.5)/2))
        if still and begin is None:
            begin = i
        if begin is not None and (not still or i == len(t)-1):
            end = i if still else i-1
            if t[end]-t[begin] >= 8.5:
                intervals.append((t[begin], t[end]))
            begin = None
    return intervals, data


def select_stops(intervals, laser, anchor, b_at):
    a0 = next((w for w in intervals if w[0] <= laser[0]+1 and w[0] <= anchor < w[1]-1
               and w[1]-laser[0] >= 18.5), None)
    a1 = next((w for w in reversed(intervals) if w[1] >= laser[-1]-.75 and w != a0), None)
    candidates = [w for w in intervals if w != a0 and w != a1 and
                  w[0] > (a0[1] if a0 else laser[0]+18.5) and
                  w[1] < (a1[0] if a1 else laser[-1]-8.5)]
    if b_at is not None:
        candidates = [w for w in candidates if w[0] <= laser[0]+b_at <= w[1]]
    return a0, candidates[0] if len(candidates) == 1 else None, a1


def dll_health(pair):
    stamps, states = [], []
    with rosbag.Bag(str(pair / 'dll/result.bag')) as bag:
        for _, msg, _ in bag.read_messages(topics=['/dll_localization/status']):
            for status in msg.status:
                if status.name == 'fastlio_dll':
                    if status.level == status.STALE and stamps and msg.header.stamp.to_sec() == stamps[-1]:
                        continue
                    stamps.append(msg.header.stamp.to_sec())
                    states.append(status.message if status.level == 0 else 'invalid')
    order = np.argsort(stamps, kind='stable')
    return np.asarray(stamps)[order], np.asarray(states)[order]


def trajectories(pair, seed):
    rotation, translation = unpack(seed['result'])
    stamp, local, corrected = [], [], []
    with (pair / 'dll/trajectory.csv').open() as f:
        for row in csv.DictReader(f):
            ns = int(row['stamp_ns'])
            if ns < seed['anchor_ns']:
                continue
            stamp.append(ns * 1e-9)
            local.append([float(row['odom_'+k]) for k in 'xyz'])
            corrected.append([float(row['dll_map_'+k] or 'nan') for k in 'xyz'])
    times = np.array(stamp)
    poses = np.array(corrected).reshape(-1, 3)
    health_t, states = dll_health(pair)
    valid = np.zeros(len(times), bool)
    if len(states):
        index = np.searchsorted(health_t, times, side='right')-1
        valid = (index >= 0) & (times-health_t[index] <= 1.) & (states[index] == 'TRACKING')
    poses[~valid] = np.nan
    ytime, ypose = [], []
    with (pair / 'ywl/pose.csv').open() as f:
        for row in csv.DictReader(f):
            if int(row['stamp_ns']) >= seed['anchor_ns']:
                ytime.append(int(row['stamp_ns'])*1e-9)
                ypose.append([float(row[k]) for k in 'xyz'])
    return dict(fastlio2=(times, np.array(local).reshape(-1, 3) @ rotation.T + translation),
                dll=(times, poses), ywl=(np.array(ytime), np.array(ypose).reshape(-1, 3)))


def window_pose(series, window, laser):
    if window is None:
        return None, dict(valid=False, reason='missing_stop')
    lo, hi = window
    # Each LiDAR pose is stamped at scan end. Use the measured scan period to
    # pair with the expected raw scan. Missing frames never get interpolated.
    ends = laser + np.median(np.diff(laser))
    expected = ends[(ends >= lo) & (ends < hi)]
    t, xyz = series
    if len(t) and len(expected):
        right = np.searchsorted(t, expected).clip(0, len(t)-1)
        left = np.maximum(0, right-1)
        idx = np.where(abs(t[left]-expected) < abs(t[right]-expected), left, right)
        ok = (abs(t[idx]-expected) < .025) & (t[idx] >= lo-.025) & (t[idx] < hi+.025)
        points = xyz[idx[ok]]
        points = points[np.isfinite(points).all(axis=1)]
    else:
        points = np.empty((0, 3))
    valid = len(expected) >= 5 and len(points) == len(expected)
    return (np.median(points, axis=0) if valid else None,
            dict(valid=valid, expected=len(expected), usable=len(points)))


def difference(current, reference):
    if current is None or reference is None:
        return None
    delta = current-reference
    return dict(xyz_m=delta.tolist(), horizontal_m=float(np.linalg.norm(delta[:2])),
                dz_m=float(delta[2]), distance_m=float(np.linalg.norm(delta)))


def initial_guess(bag, recorded, b, position, seed, hint):
    # Roll/pitch come from gravity during the stationary raw IMU window.
    # Unless the user supplies an independent landmark hint, propagate ONLY yaw
    # from the recorded IMU-body local rotations between the seed and B.
    vectors = []
    gyro = []
    for _, msg, _ in bag.read_messages(topics=[IMU]):
        if b[0] <= msg.header.stamp.to_sec() <= b[1]:
            v = msg.linear_acceleration
            vectors.append([v.x, v.y, v.z])
            w = msg.angular_velocity
            gyro.append([w.x, w.y, w.z])
    if len(vectors) < 10 or np.linalg.norm(np.mean(vectors, axis=0)) < .5:
        raise ValueError('Missing stationary gravity at B')
    if np.percentile(np.linalg.norm(gyro, axis=1), 95) > math.radians(3):
        raise ValueError('B raw IMU is rotating within the ICP window')
    leveling = mapio.R_from_gravity(-np.mean(vectors, axis=0))
    if hint is not None:
        target_yaw = math.radians(hint[3])
        position = np.array(hint[:3])
    else:
        target_times = [seed['anchor_ns']*1e-9, np.mean(b)]
        indices = [np.argmin(abs(recorded[:, 0]-when)) for when in target_times]
        rows = recorded[indices]
        if np.max(abs(rows[:, 0]-target_times)) > .15 or position is None:
            raise ValueError('No odometry near B/anchor; supply --b-hint X Y Z IMU_YAW_DEG')
        initial = quat_R(seed['map_from_imu']['q_xyzw'])
        target_yaw = yaw(initial @ quat_R(rows[0, 4:]).T @ quat_R(rows[1, 4:]))
    return position, rotz(target_yaw-yaw(leveling)) @ leveling


def analyze(pair, b_at, hint):
    bag_path, folder, seed, hashes = replay_inputs(pair)
    paths = trajectories(pair, seed)
    anchor = seed['anchor_ns']*1e-9
    with rosbag.Bag(str(bag_path)) as bag:
        laser, received, coverage = sensors(bag)
        intervals, recorded = stops(bag)
        coverage['covers_recorded_odom'] = bool(laser[0] <= recorded[0,0]+.25 and
                                                laser[-1]+.35 >= recorded[-1,0])
        coverage['valid'] = coverage['valid'] and coverage['covers_recorded_odom']
        a0, b, a1 = select_stops(intervals, laser, anchor, b_at)
        windows = dict(a_start=(a0[1]-2, a0[1]-1) if a0 else None,
                       b_arrival=(b[0], b[0]+1) if b else None,
                       b_end=(b[1]-1, b[1]) if b else None,
                       a_return=(a1[1]-1, a1[1]) if a1 else None)
        ref = dict(valid=False, reasons=['missing_or_ambiguous_B'])
        if b:
            mid = np.mean(b)
            use = (laser >= mid-2.2) & (laser <= mid+2.2)
            try:
                held = recorded[(recorded[:, 0] >= mid-2) & (recorded[:, 0] < mid+2)]
                if len(held) < 30 or np.linalg.norm(np.ptp(held[:, 1:4], axis=0)) > .07:
                    raise ValueError('B moved within the four-second ICP window')
                if np.min(abs(held[:, 4:] @ held[len(held)//2, 4:])) < math.cos(math.radians(2)/2):
                    raise ValueError('B rotated within the four-second ICP window')
                if not use.any():
                    raise ValueError('Missing B scans')
                calib = Path(rospkg.RosPack().get_path('fast_lio'))/'config/mid360s.yaml'
                cloud = static_cloud(bag, (received[use].min(), received[use].max()), mid-2, mid+2, calib)
                guess, _ = window_pose(paths['fastlio2'], (mid-.5, mid+.5), laser)
                position, rotation = initial_guess(bag, recorded, (mid-2,mid+2), guess, seed, hint)
                ref = register(cloud, folder, position, rotation)
                ref.update(calibration_sha256=sha(calib), point_window_s=[mid-2-laser[0],mid+2-laser[0]],
                           hint='manual' if hint else 'start_aligned_local_odometry')
            except ValueError as exc:
                ref = dict(valid=False, reasons=[str(exc)])
    reference = np.array(ref['pose']['translation']) if ref['valid'] else None
    readings = {}
    for name in METHODS:
        points, counts = {}, {}
        for label, window in windows.items():
            points[label], counts[label] = window_pose(paths[name], window, laser)
        for label, point in points.items():
            counts[label]['map_xyz_m'] = point.tolist() if point is not None else None
        readings[name] = dict(b_arrival=difference(points['b_arrival'], reference),
                              b_end=difference(points['b_end'], reference),
                              a_return=difference(points['a_return'], points['a_start']), windows=counts)
    complete = coverage['valid'] and all(all(row[k] is not None for k in ('b_arrival','b_end','a_return'))
                                         for row in readings.values())
    def relative(window):
        return [round(float(t-laser[0]), 3) for t in window] if window else None
    issues = []
    if not coverage['valid']:
        issues.append('incomplete_raw_sensor_coverage')
    if a0 is None:
        issues.append('missing_initial_A_stop_or_seed')
    if a1 is None:
        issues.append('missing_final_A_stop')
    if not ref['valid']:
        issues.extend(ref['reasons'])
    for name, row in readings.items():
        for label, stats in row['windows'].items():
            if windows[label] and not stats['valid']:
                issues.append(f'{name}/{label}: {stats["usable"]}/{stats["expected"]} valid frames')
    return dict(status='complete' if complete else 'partial', issues=issues, bag=str(bag_path), map=str(folder),
                sha256=hashes, anchor_ns=seed['anchor_ns'], seed=seed['result']['transform'],
                sensor_coverage=coverage, lidar_start_s=laser[0],
                stationary_intervals_s=[relative(w) for w in intervals],
                windows_s={k: relative(w) for k,w in windows.items()},
                b_reference=ref, readings=readings)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pair_dir', type=Path)
    parser.add_argument('--b-at', type=float, help='seconds after first raw LiDAR stamp, inside the B stop')
    parser.add_argument('--b-hint', type=float, nargs=4, metavar=('X','Y','Z','IMU_YAW_DEG'))
    parser.add_argument('--output', type=Path, help='default: PAIR_DIR/aba.json')
    args = parser.parse_args()
    if args.b_at is not None and not math.isfinite(args.b_at):
        parser.error('--b-at must be finite')
    if args.b_hint is not None and not np.isfinite(args.b_hint).all():
        parser.error('--b-hint must be finite')
    report = analyze(args.pair_dir, args.b_at, args.b_hint)
    output = args.output or args.pair_dir/'aba.json'
    with tempfile.NamedTemporaryFile(mode='w', dir=output.parent, delete=False) as tmp:
        tmp.write(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    Path(tmp.name).replace(output)
    print(report['status'], output, *report['issues'])
    print('method   B-arrival / B-end / A-return (3D)')
    for name, row in report['readings'].items():
        print(name, *(f"{row[k]['distance_m']:.3f} m" if row[k] else 'N/A'
                      for k in ('b_arrival','b_end','a_return')))
    return 0 if report['status'] == 'complete' else 2


if __name__ == '__main__':
    sys.exit(main())
