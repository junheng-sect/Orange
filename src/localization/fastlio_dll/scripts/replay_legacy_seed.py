#!/usr/bin/env python3
"""Replay fixture for the legacy result/TF contract. Use on an isolated master only."""
import argparse
import json
import math
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def multiply(left, right):
    x, y, z, w = left
    a, b, c, d = right
    return [
        w * a + x * d + y * c - z * b,
        w * b - x * c + y * d + z * a,
        w * c + x * b - y * a + z * d,
        w * d - x * a - y * b - z * c,
    ]


class ReplayLegacy:
    def __init__(self, map_dir, seed, invalidate_at):
        self.map_dir = map_dir
        self.t, self.q = seed[:3], seed[3:]
        self.invalidate_at = invalidate_at
        self.phase = None
        self.start = None
        self.last_status_at = 0.0
        self.results = rospy.Publisher("/drone_reloc/result", String, queue_size=1, latch=True)
        self.status = rospy.Publisher("/drone_reloc/status", String, queue_size=1, latch=True)
        self.poses = rospy.Publisher("/drone_reloc/pose", PoseStamped, queue_size=10)
        self.transforms = rospy.Publisher("/tf", TFMessage, queue_size=10)
        self.sub = rospy.Subscriber("/fastlio/odom_world", Odometry, self.on_odom, queue_size=20)

    def on_odom(self, odom):
        if self.start is None:
            self.start = odom.header.stamp
        elapsed = (odom.header.stamp - self.start).to_sec()
        if elapsed < 0.4:
            return
        phase = ("wrong_map" if elapsed < 1.0 else "old_epoch" if elapsed < 2.0 else
                 "matched" if elapsed < self.invalidate_at else "invalid")
        now = rospy.Time.now().to_sec()
        valid = phase != "invalid"
        if self.phase != phase:
            map_dir = self.map_dir + ".wrong" if phase == "wrong_map" else self.map_dir
            stamp = self.start.to_sec() - 1.0 if phase == "old_epoch" else now
            self.results.publish(json.dumps(dict(
                schema_version=2, stamp=stamp, map=map_dir, ok=valid, confirmed=valid,
                transform=dict(translation=self.t, q_xyzw=self.q))))
            self.last_status_at = 0.0
            self.phase = phase
        wall = time.monotonic()
        if wall - self.last_status_at >= 0.8:
            self.status.publish(json.dumps(dict(
                schema_version=2, stamp=now, valid=valid,
                state="LOCALIZED" if valid else "STALE", why="replay_fixture")))
            self.last_status_at = wall
        if not valid:
            return

        edge = TransformStamped()
        edge.header = odom.header
        edge.header.frame_id = "drone_map"
        edge.child_frame_id = "drone_odom"
        edge.transform.translation.x, edge.transform.translation.y, edge.transform.translation.z = self.t
        edge.transform.rotation.x, edge.transform.rotation.y, edge.transform.rotation.z, edge.transform.rotation.w = self.q
        self.transforms.publish(TFMessage([edge]))
        if phase != "matched":
            return

        pose = PoseStamped()
        pose.header = edge.header
        p = odom.pose.pose.position
        rotation = self.q
        conjugate = [-rotation[0], -rotation[1], -rotation[2], rotation[3]]
        point = multiply(multiply(rotation, [p.x, p.y, p.z, 0.0]), conjugate)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            point[i] + self.t[i] for i in range(3))
        q = odom.pose.pose.orientation
        orientation = multiply(rotation, [q.x, q.y, q.z, q.w])
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = orientation
        self.poses.publish(pose)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_dir")
    parser.add_argument("seed", type=json.loads)
    parser.add_argument("invalidate_at", type=float)
    args = parser.parse_args(rospy.myargv()[1:])
    if len(args.seed) != 7 or not all(math.isfinite(v) for v in args.seed) or args.invalidate_at <= 5:
        parser.error("expected a valid seven-element seed and replay duration greater than five seconds")
    rospy.init_node("dll_replay_legacy_seed")
    if not rospy.get_param("/use_sim_time", False):
        raise SystemExit("The legacy seed fixture requires simulated time")
    fixture = ReplayLegacy(args.map_dir, args.seed, args.invalidate_at)
    rospy.spin()


if __name__ == "__main__":
    main()
