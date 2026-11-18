#!/usr/bin/env python3
"""ROS-bag scoring regression: known B reference, injected errors, missing poses and wrong map."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import rosbag
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from aba_reference import register
from analyze_aba import analyze, read_json, select_stops, window_pose
from drone_mapping.mapio import write_pcd
from drone_relocalization.geometry import rotz

EPOCH = 1700000000
ANGLE = .2
TRANSLATION = np.array([1., -2., .2])
B = np.array([7., 0., .2])
LEVER = np.array([-.011, -.02329, .04412])


def write_json(path, data):
    path.write_text(json.dumps(data))


def stamp(t):
    return rospy.Time.from_sec(EPOCH + t)


def position(t):
    fraction = min(1., max(0., (t-22)/3.), max(0., (39-t)/3.))
    return TRANSLATION + fraction*(B-TRANSLATION)


def fixture(root):
    folder = root/'map'
    for path in (folder, root/'dll', root/'ywl'):
        path.mkdir()
    rng = np.random.RandomState(42)
    surfaces = []
    for axis, constant in [(0, 10.), (1, 4.), (2, -.7)]:
        p = rng.uniform([-4,-7,-.7], [12,6,5], (9000,3))
        p[:,axis] = constant
        surfaces.append(p)
    points = np.vstack(surfaces).astype('f4')
    write_pcd(folder/'map_world.pcd', points, np.ones(len(points),'f4'))
    write_json(folder/'meta.json', {})
    (folder/'outdoor_reloc.yaml').write_text('split_ground: 1\n')
    raw = root/'raw.bag'
    odom_header = ['odom_'+k for k in 'xyz'] + ['dll_map_'+k for k in 'xyz']
    path_rows, y_rows = [], []
    with rosbag.Bag(str(raw),'w') as bag, rosbag.Bag(str(root/'dll/result.bag'),'w') as results:
        for i in range(520):
            t = i/10.
            center = position(t)
            header = stamp(t)
            lidar = CustomMsg()
            lidar.header.stamp = header
            # The static B scan is generated in the LiDAR frame from a map with a known offset.
            sample = points[(i%30)::30] - center - LEVER
            lidar.points = [CustomPoint(0, x,y,z, 50,0,0) for x,y,z in sample]
            lidar.point_num = len(lidar.points)
            bag.write('/livox/lidar', lidar, header)
            for n in range(10):
                imu = Imu()
                imu.header.stamp = stamp(t+.01*n)
                imu.linear_acceleration.z = 9.81
                bag.write('/livox/imu', imu, imu.header.stamp)
            m = Odometry()
            m.header.stamp = header
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = center
            m.pose.pose.orientation.w = 1.
            bag.write('/Odometry',m, header)
            time = stamp(t+.1)
            diag = DiagnosticArray()
            diag.header.stamp = time
            diag.status = [DiagnosticStatus(level=0,name='fastlio_dll',message='TRACKING')]
            if i > 45:
                results.write('/dll_localization/status',diag,time)
            # Three distinct errors only at B, plus a different repeatable offset on return.
            bias = float(25 <= t <= 36)
            returned = .03 if t >= 39 else 0.
            local = rotz(ANGLE).T @ (center + [bias*.3, 0, returned] - TRANSLATION)
            dll = center + [0,.2*bias, returned]
            ywl = center + [0,0,.1*bias+returned]
            path_rows.append([time.to_nsec(), *local, *dll])
            if i >= 46:
                y_rows.append([time.to_nsec(), *ywl])
    for filename, keys, rows in [(root/'dll/trajectory.csv',odom_header,path_rows),
                                  (root/'ywl/pose.csv',list('xyz'),y_rows)]:
        with filename.open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['stamp_ns',*keys])
            writer.writerows(rows)
    seed_pose=dict(translation=TRANSLATION.tolist(), q_xyzw=[0,0,math.sin(ANGLE/2),math.cos(ANGLE/2)])
    result=dict(ok=True,confirmed=True,transform=seed_pose)
    anchor=stamp(5).to_nsec()
    hashes={s:hashlib.sha256((folder/s).read_bytes()).hexdigest() for s in ('map_world.pcd','meta.json')}
    write_json(root/'ywl_seed.json',dict(anchor_ns=anchor,result=result,map_sha256=hashes,
               map_from_imu=dict(q_xyzw=[0,0,0,1], translation=TRANSLATION.tolist())))
    write_json(root/'dll_seed.json',dict(bag=str(raw),map=str(folder),results=[result],anchor_ns=anchor))
    write_json(root/'dll/applied_seed.json',seed_pose['translation']+seed_pose['q_xyzw'])
    (root/'dll/seed_anchor_ns.txt').write_text(str(anchor))
    (root/'dll/inputs.sha256').write_text(hashlib.sha256(raw.read_bytes()).hexdigest()+'  raw.bag\n'+
                                          hashes['map_world.pcd']+'  map_world.pcd\n')
    write_json(root/'ywl/run.json',dict(bag=str(raw),map=str(folder),seed=str(root/'ywl_seed.json')))


class CheckpointTests(unittest.TestCase):
    def test_full_bag(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            fixture(root)
            report=analyze(root,None,None)
            self.assertEqual(report['status'],'complete',report)
            np.testing.assert_allclose(report['b_reference']['pose']['translation'], B, atol=.012)
            for method, delta in [('fastlio2',[.3,0,0]),('dll',[0,.2,0]),('ywl',[0,0,.1])]:
                row=report['readings'][method]
                for key in ('b_arrival','b_end'):
                    np.testing.assert_allclose(row[key]['xyz_m'], delta, atol=.012)
                self.assertAlmostEqual(row['a_return']['distance_m'],.03,places=3)
            wrong = read_json(root/'ywl_seed.json')
            wrong['anchor_ns'] += 1
            write_json(root/'ywl_seed.json',wrong)
            with self.assertRaisesRegex(ValueError,'Common confirmed'):
                analyze(root,None,None)

    def test_missing_or_ambiguous(self):
        start,b1,b2,end = [(0,21),(30,40),(46,56),(64,75)]
        frames=np.arange(0,75.1,.1)
        self.assertIsNone(select_stops([start,b1,b2,end],frames,5,None)[1])
        self.assertEqual(select_stops([start,b1,b2,end],frames,5,49)[1],b2)
        self.assertIsNone(select_stops([start,end],frames,5,None)[1])
        pts=np.zeros((len(frames),3))
        pts[310]=np.nan
        p,n=window_pose((frames,pts),(31,32),frames)
        self.assertIsNone(p)
        self.assertLess(n['usable'], n['expected'])

    def test_floor_only(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            rng=np.random.RandomState(5)
            pts=np.c_[rng.uniform(-5,5,(4000,2)),np.zeros(4000)].astype('f4')
            write_pcd(root/'map_world.pcd',pts,np.ones(len(pts),'f4'))
            (root/'outdoor_reloc.yaml').write_text('split_ground: 1\n')
            write_json(root/'meta.json', {})
            reference=register(pts,root,np.zeros(3),np.eye(3))
            self.assertFalse(reference['valid'])
            self.assertIn('degen_xy',reference['reasons'])


if __name__ == '__main__':
    unittest.main()
