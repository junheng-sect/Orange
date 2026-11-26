#!/usr/bin/env python3
"""Four shelf stops with timestamped full-resolution samples and an independent CH6 recorder."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import math
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry

from indoor3_runner import Frame, Indoor3Runner, save_photo, serve_route
from nav_indoor3 import load_route

SECOND = 1_000_000_000
OFFSETS = (2, 3, 4)


class Samples:
    """Keep one candidate per target time, dropping the candidate pass after a radius exit."""
    def __init__(self):
        self.entry = None
        self.confirmed = False
        self.last = None
        self.was_outside = False
        self.images = {}
        self.odom = []

    def observe(self, stamp, distance, speed, yaw_error):
        if self.last is not None and stamp <= self.last:
            return
        if not all(math.isfinite(value) for value in (distance, speed, yaw_error)):
            self.last = None
            if not self.confirmed:
                self.entry, self.was_outside = None, False
                self.images.clear()
            return
        if not self.confirmed:
            if self.last is None or stamp - self.last > 250_000_000:
                self.entry, self.was_outside = None, False
                self.images.clear()
            if distance > .15:
                self.entry, self.was_outside = None, True
                self.images.clear()
            elif self.entry is None and self.was_outside:
                self.entry = stamp
        self.last = stamp
        self.odom.append((stamp, distance, speed, yaw_error))

    def consider(self, msg):
        if self.entry is None:
            return
        stamp = msg.header.stamp.to_nsec()
        for offset in OFFSETS:
            delta = abs(stamp - self.entry - offset * SECOND)
            old = self.images.get(offset)
            if delta <= 100_000_000 and (old is None or delta < old[0]):
                self.images[offset] = (delta, msg)


class PhotoStream(Frame):
    def __init__(self, collect):
        self.collect = collect
        super().__init__()

    def receive(self, msg):
        super().receive(msg)
        if self.recent and self.recent[-1] is msg:
            self.collect(msg)


class DamperRunner(Indoor3Runner):
    test_name = record_prefix = 'test_indoor3_5'
    route_name = '室内 test3_5 四点减振对照'

    def __init__(self, *args):
        super().__init__(*args)
        self.sample_lock = threading.Lock()
        self.samples = None
        self.target = None
        self.target_yaw = 0.
        self.photo_stream = None
        self.sub = rospy.Subscriber('/drone_nav/odom', Odometry, self.observe, queue_size=40)

    def observe(self, msg):
        if msg.header.frame_id != 'drone_odom' or msg.child_frame_id != 'drone_nav_base':
            return
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        with self.sample_lock:
            if self.samples is not None:
                angle = yaw - self.target_yaw
                self.samples.observe(msg.header.stamp.to_nsec(), float(np.linalg.norm(self.target-[p.x,p.y,p.z])),
                                     math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
                                     abs(math.degrees(math.atan2(math.sin(angle), math.cos(angle)))))

    def collect(self, msg):
        with self.sample_lock:
            if self.samples is not None:
                self.samples.consider(msg)

    def guard(self, **kwargs):
        super().guard(**kwargs)
        # Pull 13 MP buffers only around the three samples, leaving the moving legs lightweight.
        with self.sample_lock:
            entry = self.samples.entry if self.samples is not None else None
        elapsed = None if entry is None else (rospy.Time.now().to_nsec() - entry) / SECOND
        needed = elapsed is not None and .7 <= elapsed <= 4.16
        if needed and self.photo_stream is None:
            self.photo_stream = PhotoStream(self.collect)
        elif not needed and self.photo_stream is not None:
            self.photo_stream.close()
            self.photo_stream = None

    def send_leg(self, i, goal, goal_map, yaw_map):
        with self.sample_lock:
            self.target, self.target_yaw, self.samples = goal, self.yaws[i], Samples()
        try:
            super().send_leg(i, goal, goal_map, yaw_map)
        finally:
            if self.photo_stream is not None:
                self.photo_stream.close()
                self.photo_stream = None
            with self.sample_lock:
                samples, self.samples = self.samples, None
            if self.record['legs'] and self.record['legs'][-1]['index'] == i+1:
                leg = self.record['legs'][-1]
                leg['odom_samples'] = samples.odom
                if i:
                    self.record['legs'][-2]['depart_ns'] = round(leg['sent_ros']*SECOND)
                self.save()

    def hold_at_point(self, i, leg, seconds):
        with self.sample_lock:
            entry = self.samples.entry
            self.samples.confirmed = True
        confirmed = round(leg['end_ros']*SECOND)
        leg.update(t0_ns=entry, confirmed_ns=confirmed, photos=[])
        end = (entry if entry is not None else confirmed) + 5*SECOND
        self.save()
        jobs = []
        with ThreadPoolExecutor(max_workers=1) as writer:
            try:
                for offset in OFFSETS:
                    row = dict(offset_s=offset, planned_ns=None if entry is None else entry+offset*SECOND)
                    leg['photos'].append(row)
                    if entry is None:
                        row['status'] = 'missing_entry'
                        continue
                    deadline = row['planned_ns'] + 160_000_000
                    while rospy.Time.now().to_nsec() < deadline:
                        self.hold_guard()
                        time.sleep(.02)
                    self.hold_guard()
                    with self.sample_lock:
                        best = self.samples.images.pop(offset, None)
                    if best is None:
                        row['status'] = 'missing_frame'
                        continue
                    msg = best[1]
                    path = Path(self.record_path).with_name('%s_p%d_%ds.png' % (Path(self.record_path).stem,i+1,offset))
                    row.update(file=path.name, stamp_ns=msg.header.stamp.to_nsec(), status='writing')
                    jobs.append((row, writer.submit(save_photo, msg, path, lambda: None)))
                while rospy.Time.now().to_nsec() < end:
                    self.hold_guard()
                    time.sleep(.02)
                self.hold_guard()
                leg['score_end_ns'] = rospy.Time.now().to_nsec()
            finally:
                for row, job in jobs:
                    try:
                        job.result()
                        row['status'] = 'saved'
                    except Exception as error:
                        row.update(status='write_error', error=str(error))
                self.save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--preview', action='store_true')
    args = parser.parse_args()
    route = load_route()
    route['hold_s'] = 5.
    return serve_route('nav_indoor3_5', args, route, DamperRunner)


if __name__ == '__main__':
    raise SystemExit(main())
