#!/usr/bin/env python3
"""Read ROS navigation telemetry and serve a loopback-only browser viewer."""
import argparse
import base64
from collections import OrderedDict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
import math
import os
from pathlib import Path
import threading
import sys
import time
from urllib.parse import parse_qs, urlsplit
import uuid

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from traj_utils.msg import Bspline
try:
    from traj_utils.msg import PathTrajectory
except ImportError:
    PathTrajectory = None
from drone_navigation.core import Spline, rotation

from map_cloud import display_map, sample


MAP_FRESH = 1.25
POSE_FRESH = .8
CLOUD_FRESH = 1.25
PLAN_FRESH = .8
TRAIL_LENGTH = 4800
PATH_GENERATION = 1 << 32
OUTDOOR_OBSERVER_MAP = "20261105_114605_outdoor"


def xyz(p):
    return [p.x, p.y, p.z]


def xyzw(q):
    return np.array([q.x, q.y, q.z, q.w], dtype=float)


def quaternion(value):
    q = np.asarray(value, dtype=float)
    rotation(q)
    return q / np.linalg.norm(q)


def multiply(a, b):
    u, v = a[:3], b[:3]
    return np.r_[a[3] * v + b[3] * u + np.cross(u, v), a[3] * b[3] - u @ v]


def floats(points):
    return base64.b64encode(np.asarray(points, dtype="<f4").tobytes()).decode("ascii")


def cloud_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    offsets = []
    for key in ("x", "y", "z"):
        field = fields.get(key)
        if not field or field.datatype != 7 or field.count != 1 or field.offset + 4 > msg.point_step:
            raise ValueError("PointCloud2 XYZ fields")
        offsets.append(field.offset)
    if not msg.width or not msg.height:
        return np.empty((0, 3))
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.row_step * msg.height:
        raise ValueError("PointCloud2 dimensions")
    dtype = np.dtype(dict(names=["x", "y", "z"], formats=[(">f4" if msg.is_bigendian else "<f4")] * 3,
                         offsets=offsets, itemsize=msg.point_step))
    records = np.ndarray((msg.height, msg.width), dtype, buffer=msg.data,
                         strides=(msg.row_step, msg.point_step))
    points = np.column_stack([records[k].ravel() for k in ("x", "y", "z")])
    return points[np.isfinite(points).all(axis=1)]


class Monitor:
    def __init__(self, maps):
        self.maps = maps.resolve()
        self.lock = threading.RLock()
        self.map_lock = threading.Lock()
        self.map_cache = None
        self.epoch = uuid.uuid4().hex
        self.sequence = 0
        self.map_path = None
        self.map_key = None
        self.map_error = None
        self.solution = None
        self.transform = None
        self.relocation = (0, False, "WAITING")
        self.executor = (0, {})
        self.poses = {}
        self.source = None
        self.pose = None
        self.trail = deque(maxlen=TRAIL_LENGTH)
        self.dll_samples = OrderedDict()
        self.dll_status = None
        self.ywl_samples = OrderedDict()
        self.ywl_status = None
        self.ywl_seed_stamp = 0
        self.ywl_result_stamp = 0
        self.ywl_prior_hashes = None
        self.imu_to_body = None
        self.viewing_observer = None
        self.cloud = None
        self.cloud_at = 0
        self.plans = OrderedDict()
        self.aligned = False
        self.subs = []

    def subscribe(self):
        topics = [
            ("/drone_reloc/result", String, self.on_result, {}),
            ("/drone_reloc/status", String, self.on_relocation, {}),
            ("/tf", TFMessage, self.on_tf, {}),
            ("/tf_static", TFMessage, self.on_mount, {}),
            ("/drone_nav/odom", Odometry, self.on_pose, {"callback_args": "navigation"}),
            ("/fastlio/odom_world", Odometry, self.on_pose, {"callback_args": "fastlio"}),
            ("/dll_localization/pose", PoseStamped, self.on_dll_sample, {"callback_args": "pose"}),
            ("/dll_localization/correction", TransformStamped, self.on_dll_sample, {"callback_args": "correction"}),
            ("/dll_localization/status", DiagnosticArray, self.on_dll_status, {}),
            ("/ywl/odom", Odometry, self.on_ywl_pose, {}),
            ("/ywl/status", DiagnosticArray, self.on_ywl_status, {}),
            ("/ywl/initial_transform", TransformStamped, self.on_ywl_init, {}),
            ("/drone_nav/cloud", PointCloud2, self.on_cloud, {"buff_size": 4 * 1024 * 1024}),
            ("/drone_nav/bspline", Bspline, self.on_legacy_plan, {}),
            ("/drone_nav/executor_status", String, self.on_executor, {}),
        ]
        if PathTrajectory is not None:
            topics.append(("/drone_nav/path_trajectory", PathTrajectory, self.on_path_plan, {}))
        self.subs = [rospy.Subscriber(topic, kind, callback, queue_size=1, **options)
                     for topic, kind, callback, options in topics]

    def next_id(self):
        self.sequence += 1
        return self.sequence

    def clear_motion(self):
        self.epoch = uuid.uuid4().hex
        self.pose = self.cloud = None
        self.trail.clear()
        self.dll_samples.clear()
        self.ywl_samples.clear()
        self.viewing_observer = None
        self.plans.clear()
        self.source = None
        self.aligned = False
        self.cloud_at = 0

    def on_result(self, msg):
        try:
            value = json.loads(msg.data)
            if value.get("ok") is not True or value.get("confirmed") is not True:
                with self.lock:
                    self.solution = None
                    self.ywl_result_stamp = 0
                    self.clear_motion()
                return
            folder = Path(value["map"])
            if not folder.is_absolute():
                folder = self.maps / folder
            path = (folder if folder.suffix == ".pcd" else folder / "map_world.pcd").resolve()
            q = quaternion(value["transform"]["q_xyzw"])
            offset = np.asarray(value["transform"]["translation"], dtype=float)
            if offset.shape != (3,) or not np.isfinite(offset).all():
                raise ValueError("invalid map translation")
            allowed = path.name == "map_world.pcd" and self.maps in path.parents
            try:
                info = path.stat() if allowed and path.is_file() else None
            except OSError:
                info = None
            stamp = value.get("stamp")
            result_stamp = round(stamp * 1e9) if type(stamp) in (int, float) and math.isfinite(stamp) else 0
            hashes = self.ywl_crop_hashes(path.parent) if info and path.parent.name == OUTDOOR_OBSERVER_MAP else None
            with self.lock:
                self.ywl_result_stamp = result_stamp
                self.ywl_prior_hashes = hashes
                self.map_error = None if info else "定位地图不在地图目录内，或缺少 map_world.pcd"
                self.map_path = path if info else None
                self.map_key = ("%s-%d-%d" % (path.parent.name, info.st_mtime_ns, info.st_size)) if info else None
                self.solution = (q, offset)
                self.clear_motion()
        except (KeyError, ValueError, TypeError, OSError, AttributeError) as exc:
            rospy.logwarn_throttle(5, "monitor relocation result: %s", exc)

    def on_relocation(self, msg):
        try:
            value = json.loads(msg.data)
            with self.lock:
                self.relocation = (time.monotonic(), value["valid"] is True and value["state"] == "LOCALIZED",
                                   str(value["state"]))
                if not self.relocation[1] and self.aligned:
                    self.clear_motion()
        except (KeyError, ValueError, TypeError):
            return

    def on_tf(self, msg):
        for tf in msg.transforms:
            if (tf.header.frame_id, tf.child_frame_id) == ("drone_map", "drone_odom"):
                try:
                    q = quaternion(xyzw(tf.transform.rotation))
                    offset = np.asarray(xyz(tf.transform.translation), dtype=float)
                    if not np.isfinite(offset).all():
                        return
                except ValueError:
                    return
                with self.lock:
                    self.transform = (time.monotonic(), q, offset)
                return

    def valid_alignment(self, now):
        if not self.map_key or not self.solution or not self.transform:
            return False
        when, valid, _ = self.relocation
        tf_when, tf_q, tf_offset = self.transform
        q, offset = self.solution
        return (valid and now - when <= 2.5 and now - tf_when <= MAP_FRESH
                and abs(q @ tf_q) >= 1 - 1e-8 and float(np.max(np.abs(offset - tf_offset))) <= 1e-4)

    def on_dll_sample(self, msg, kind):
        stamp = msg.header.stamp.to_nsec()
        if stamp <= 0 or msg.header.frame_id != "drone_map":
            return
        if kind == "correction" and msg.child_frame_id != "drone_odom":
            return
        point = msg.pose.position if kind == "pose" else msg.transform.translation
        orientation = msg.pose.orientation if kind == "pose" else msg.transform.rotation
        position = xyz(point)
        if not np.isfinite(position).all():
            return
        try:
            body = quaternion(xyzw(orientation)).tolist()
        except ValueError:
            return
        sample = [msg.header.stamp.to_sec(), *position, *body] if kind == "pose" else True
        with self.lock:
            self.dll_samples.setdefault(stamp, {})[kind] = (time.monotonic(), sample)
            while len(self.dll_samples) > 8:
                self.dll_samples.popitem(last=False)

    def on_dll_status(self, msg):
        for status in msg.status:
            if status.name != "fastlio_dll":
                continue
            fields = {item.key: item.value for item in status.values}
            try:
                accepted = int(fields.get("accepted_stamp_ns", "0"))
            except ValueError:
                accepted = 0
            with self.lock:
                self.dll_status = (time.monotonic(), msg.header.stamp.to_nsec(), status.message,
                                   fields.get("reason", ""), accepted,
                                   status.level == 0 and fields.get("local_input") == "fresh")
                if status.message != "TRACKING" or status.level != 0:
                    self.dll_samples.clear()
            return

    def dll_view(self, now):
        if self.dll_status is None or self.map_path is None or self.map_path.parent.name != OUTDOOR_OBSERVER_MAP:
            return None, None
        at, status_stamp, state, reason, accepted, valid = self.dll_status
        if now - at > 1.5:
            return None, "DLL 状态已过期"
        if state != "TRACKING" or not valid:
            return None, f"{state} · {reason}"
        ros_now = rospy.Time.now().to_nsec()
        if not 0 < status_stamp or not -50_000_000 <= ros_now - status_stamp <= 1_500_000_000:
            return None, "DLL 状态时间戳过期"
        for stamp in sorted(self.dll_samples, reverse=True):
            sample = self.dll_samples[stamp]
            if (set(sample) == {"pose", "correction"} and accepted > 0
                    and 0 <= stamp - accepted < 1_500_000_000
                    and -50_000_000 <= ros_now - stamp <= 500_000_000
                    and all(now - received <= .5 for received, _ in sample.values())):
                return sample["pose"][1], None
        return None, "等待同刻 DLL 位姿与修正"

    @staticmethod
    def ywl_crop_hashes(source):
        crop = source.with_name(source.name + "_screenshot_roi")
        try:
            manifest = json.loads((crop / "manifest.json").read_text())
            if (manifest["map_frame"] != "drone_map" or Path(manifest["source"]["map_dir"]).resolve() != source
                    or Path(manifest["map_dir"]).resolve() != crop):
                return None
            return manifest["sha256"]["map_world.pcd"], manifest["sha256"]["meta.json"]
        except (KeyError, ValueError, TypeError, OSError):
            return None

    def on_mount(self, msg):
        for tf in msg.transforms:
            if (tf.header.frame_id, tf.child_frame_id) == ("body", "drone_base"):
                try:
                    mount = quaternion(xyzw(tf.transform.rotation))
                except ValueError:
                    return
                with self.lock:
                    self.imu_to_body = (tf.header.stamp.to_nsec(), mount)
                return

    def on_ywl_init(self, msg):
        if (msg.header.frame_id, msg.child_frame_id) == ("drone_map", "ywl_odom"):
            with self.lock:
                self.ywl_seed_stamp = msg.header.stamp.to_nsec()

    def cache_ywl(self, stamp, kind, value):
        self.ywl_samples.setdefault(stamp, {})[kind] = (time.monotonic(), value)
        while len(self.ywl_samples) > 8:
            self.ywl_samples.popitem(last=False)

    def on_ywl_pose(self, msg):
        if (msg.header.frame_id, msg.child_frame_id) != ("drone_map", "ywl_imu"):
            return
        point = xyz(msg.pose.pose.position)
        if not np.isfinite(point).all():
            return
        try:
            quat = quaternion(xyzw(msg.pose.pose.orientation))
        except ValueError:
            return
        with self.lock:
            self.cache_ywl(msg.header.stamp.to_nsec(), "pose", (point, quat))

    def on_ywl_status(self, msg):
        for status in msg.status:
            if status.name != "ywl_localization":
                continue
            values = {item.key: item.value for item in status.values}
            stamp = msg.header.stamp.to_nsec()
            with self.lock:
                valid = status.message == "TRACKING" and status.level == 0
                self.ywl_status = (time.monotonic(), status.message, values.get("reason", ""), valid)
                if valid:
                    self.cache_ywl(stamp, "status", (values.get("map_pcd_sha256"), values.get("map_meta_sha256")))
                else:
                    self.ywl_samples.clear()
            return

    def ywl_view(self, now):
        if self.ywl_status is None or self.map_path is None or self.map_path.parent.name != OUTDOOR_OBSERVER_MAP:
            return None, None
        received, state, reason, valid = self.ywl_status
        if not valid:
            return None, f"YWL {state} · {reason}"
        if now - received > .5:
            return None, "YWL 状态已过期"
        if (not self.ywl_result_stamp or not self.ywl_seed_stamp or not self.imu_to_body
                or self.ywl_seed_stamp < max(self.ywl_result_stamp - 500_000_000, self.imu_to_body[0])):
            return None, "等待本轮 YWL 初值与机体朝向"
        ros_now = rospy.Time.now().to_nsec()
        for stamp in sorted(self.ywl_samples, reverse=True):
            sample = self.ywl_samples[stamp]
            if (set(sample) == {"pose", "status"} and stamp >= self.ywl_seed_stamp
                    and -50_000_000 <= ros_now - stamp <= 500_000_000
                    and all(now - at <= .5 for at, _ in sample.values())):
                if sample["status"][1] != self.ywl_prior_hashes:
                    return None, "YWL 地图校验不一致"
                point, quat = sample["pose"][1]
                body = multiply(quat, self.imu_to_body[1])
                return [stamp / 1e9, *point, *body.tolist()], None
        return None, "等待同刻 YWL 位姿与状态"

    def on_pose(self, msg, source):
        expected = "drone_nav_base" if source == "navigation" else "drone_base"
        if (msg.header.frame_id, msg.child_frame_id) != ("drone_odom", expected):
            return
        point = np.asarray(xyz(msg.pose.pose.position))
        if not np.isfinite(point).all():
            return
        try:
            q = quaternion(xyzw(msg.pose.pose.orientation))
        except ValueError:
            return
        now, stamp = time.monotonic(), msg.header.stamp.to_sec()
        if stamp <= 0:
            return
        with self.lock:
            previous = self.poses.get(source)
            if previous and previous[1] == stamp:
                return
            if previous and stamp < previous[1] and source == self.source:
                self.clear_motion()
            self.poses[source] = (now, stamp, point, q)
            nav = self.poses.get("navigation")
            if source == "fastlio" and nav and now - nav[0] <= POSE_FRESH:
                return
            if not self.valid_alignment(now):
                return
            if self.source is not None and self.source != source:
                # Start a fresh trail when the physical reference point changes.
                # A plan may have arrived before navigation odometry.
                self.pose = None
                self.trail.clear()
                self.epoch = uuid.uuid4().hex
            self.source = source
            map_q, offset = self.solution
            world = rotation(map_q) @ point + offset
            body = multiply(map_q, q)
            self.pose = [stamp, *world.tolist(), *body.tolist()]
            self.aligned = True
            if not self.viewing_observer and (not self.trail or now - self.trail[-1][1] >= .095):
                self.trail.append((self.next_id(), now, world.tolist()))

    def on_cloud(self, msg):
        if msg.header.frame_id != "drone_odom":
            return
        now = time.monotonic()
        with self.lock:
            if now - self.cloud_at < .24 or not self.valid_alignment(now):
                return
            self.cloud_at = now
            epoch, solution = self.epoch, self.solution
        try:
            cloud = sample(cloud_xyz(msg), [.07, .07, .06], 6500)
            q, offset = solution
            encoded = floats(cloud @ rotation(q).T + offset)
        except (ValueError, TypeError) as exc:
            rospy.logwarn_throttle(5, "monitor cloud: %s", exc)
            return
        with self.lock:
            if self.epoch == epoch:
                self.cloud = (self.next_id(), now, encoded, len(cloud))

    def on_executor(self, msg):
        try:
            value = json.loads(msg.data)
            with self.lock:
                self.executor = time.monotonic(), value
        except ValueError:
            return

    def on_legacy_plan(self, msg):
        self.on_plan(msg, False, 0, False)

    def on_path_plan(self, msg):
        if msg.identity.trajectory_id == msg.spline.traj_id:
            self.on_plan(msg.spline, True, msg.identity.path_seq, msg.pending_start)

    def on_plan(self, msg, path, sequence, pending):
        now = time.monotonic()
        with self.lock:
            if not self.valid_alignment(now):
                return
            solution = self.solution
        try:
            spline = Spline(msg.order, msg.knots, [xyz(p) for p in msg.pos_pts],
                            msg.start_time.to_sec(), msg.traj_id)
            count = max(2, min(1200, math.ceil(spline.duration * 25) + 1))
            positions = np.array([spline.position(t) for t in np.linspace(0, spline.duration, count)])
            q, offset = solution
            encoded = floats(positions @ rotation(q).T + offset)
        except (ValueError, TypeError) as exc:
            rospy.logwarn_throttle(5, "monitor B-spline: %s", exc)
            return
        with self.lock:
            if self.solution is not solution or not self.valid_alignment(time.monotonic()):
                return
            key = (path, sequence, msg.traj_id, pending)
            self.plans[key] = dict(id=self.next_id(), points=encoded, pending=pending)
            self.plans.move_to_end(key)
            while len(self.plans) > 20:
                self.plans.popitem(last=False)

    def snapshot(self, query, local_map=False):
        epoch = query.get("epoch", [""])[0]
        cursors = [int(query.get(key, ["0"])[0]) for key in ("trail", "cloud", "plan")]
        now = time.monotonic()
        with self.lock:
            aligned = self.valid_alignment(now)
            if not aligned and (self.aligned or self.cloud or self.plans):
                self.clear_motion()
            dll_pose = dll_reason = ywl_pose = ywl_reason = None
            if aligned and self.source == "fastlio":
                dll_pose, dll_reason = self.dll_view(now)
                if not dll_pose:
                    ywl_pose, ywl_reason = self.ywl_view(now)
            observer = "dll" if dll_pose else "ywl" if ywl_pose else None
            observed = dll_pose or ywl_pose
            if self.viewing_observer != observer:
                self.viewing_observer = observer
                self.epoch = uuid.uuid4().hex
                self.trail.clear()
            if observed and (not self.trail or self.trail[-1][1] != observed[0]):
                self.trail.append((self.next_id(), observed[0], observed[1:4]))
            same_epoch = epoch == self.epoch
            trail_id, cloud_id, plan_id = cursors if same_epoch else (0, 0, 0)
            fresh_pose = bool(self.pose and self.source in self.poses
                              and now - self.poses[self.source][0] <= POSE_FRESH)
            cloud = self.cloud if aligned and self.cloud and now - self.cloud[1] <= CLOUD_FRESH else None
            exec_at, executor = self.executor
            state = executor.get("state", "") if now - exec_at <= PLAN_FRESH else ""
            plan = None
            if aligned and state in {"RUNNING", "HOLD", "ALIGNING_HEADING"}:
                generation = executor.get("generation", 0)
                trajectory = executor.get("trajectory_id")
                if isinstance(generation, int) and isinstance(trajectory, int):
                    path = generation >= PATH_GENERATION
                    seq = generation - PATH_GENERATION if path else 0
                    plan = self.plans.get((path, seq, trajectory, state == "ALIGNING_HEADING"))
            trail_reset = bool(trail_id and self.trail and trail_id < self.trail[0][0])
            result = dict(
                epoch=self.epoch, localMap=local_map, mapKey=self.map_key, mapId=self.map_path.parent.name if self.map_path else None,
                mapError=self.map_error, relocation=self.relocation[2] if now - self.relocation[0] <= 2.5 else "WAITING",
                aligned=aligned, pose=observed or (self.pose if aligned and fresh_pose else None),
                poseSource=observer or (self.source if aligned and fresh_pose else None),
                dllReason=dll_reason, ywlReason=ywl_reason,
                executor=state, cloudId=cloud[0] if cloud else 0,
                cloudCount=cloud[3] if cloud else 0,
                planId=plan["id"] if plan else 0, pending=plan["pending"] if plan else False,
                trailId=self.trail[-1][0] if self.trail else 0, trailReset=trail_reset,
                trail=[item[2] for item in self.trail if item[0] > trail_id],
            )
            if cloud and cloud[0] != cloud_id:
                result["cloud"] = cloud[2]
            if plan and plan["id"] != plan_id:
                result["plan"] = plan["points"]
            return result

    def map_payload(self, key):
        with self.map_lock:
            with self.lock:
                if not self.map_key or self.map_key != key:
                    return None
                path = self.map_path
            if self.map_cache and self.map_cache[0] == key:
                return self.map_cache[1]
            metadata, raw = display_map(path)
            header = json.dumps(metadata, separators=(",", ":")).encode()
            payload = len(header).to_bytes(4, "little") + header + raw
            self.map_cache = key, payload
            return payload


def start_status_bridge():
    # Reuse the same checks as the Ubuntu status service in this ROS process.
    source = Path.home() / "mid360s_ws/docker/ros-noetic-rviz/local_view/scripts/drone_status.py"
    try:
        spec = importlib.util.spec_from_file_location("warehouse_drone_status", source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        bridge = module.Bridge(os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311"))
        bridge.run_id = rospy.get_param("/run_id", None)
        bridge.subscribe()
        threading.Thread(target=bridge.poll_loop, daemon=True).start()
        return bridge
    except (ImportError, OSError, AttributeError, rospy.ROSException) as error:
        rospy.logwarn("warehouse monitor: board status unavailable: %s", error)
        return None


def server(port, monitor, frontend, telemetry_only=False):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, payload, kind, code=200):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            host = self.headers.get("Host", "").split(":")[0].lower()
            if host not in {"localhost", "127.0.0.1"}:
                self.send_error(403)
                return
            url = urlsplit(self.path)
            query = parse_qs(url.query)
            static = {"/": ("index.html", "text/html; charset=utf-8"),
                      "/monitor.js": ("monitor.js", "text/javascript; charset=utf-8"),
                      "/monitor.css": ("monitor.css", "text/css; charset=utf-8")}
            try:
                if not telemetry_only and url.path in static:
                    name, kind = static[url.path]
                    self.reply((frontend / name).read_bytes(), kind)
                elif url.path == "/telemetry":
                    self.reply(json.dumps(monitor.snapshot(query, telemetry_only), separators=(",", ":"), allow_nan=False).encode(),
                               "application/json")
                elif telemetry_only and url.path == "/status":
                    status = self.server.drone_status
                    if status is None:
                        self.send_error(503)
                        return
                    self.reply(json.dumps(status.snapshot(), ensure_ascii=False, allow_nan=False).encode(),
                               "application/json; charset=utf-8")
                elif not telemetry_only and url.path == "/map":
                    payload = monitor.map_payload(query.get("key", [""])[0])
                    if payload is None:
                        self.send_error(404)
                    else:
                        self.reply(payload, "application/octet-stream")
                else:
                    self.send_error(404)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except (ValueError, OSError, KeyError) as exc:
                rospy.logwarn_throttle(5, "monitor HTTP %s: %s", url.path, exc)
                self.send_error(400 if url.path == "/telemetry" else 500)

        def log_message(self, *_):
            pass

    http = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    http.drone_status = None
    return http


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--maps", type=Path, default=Path.home() / "test_data/mapping")
    parser.add_argument("--telemetry-only", action="store_true")
    parser.add_argument("--board-status", action="store_true")
    parser.add_argument("--stop-on-stdin-close", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    monitor = Monitor(args.maps)
    with server(args.port, monitor, Path(__file__).resolve().parent / "web", args.telemetry_only) as http:
        rospy.init_node("warehouse_monitor", anonymous=True, disable_rosout=True, disable_rostime=True)
        monitor.subscribe()
        if args.board_status:
            http.drone_status = start_status_bridge()
        if args.stop_on_stdin_close:
            def watch_stdin():
                while sys.stdin.buffer.read(8192):
                    pass
                rospy.signal_shutdown("desktop connection closed")

            threading.Thread(target=watch_stdin, daemon=True).start()
        http.timeout = .5
        print("只读仓库监看 http://127.0.0.1:%d/" % args.port, flush=True)
        while not rospy.is_shutdown():
            http.handle_request()


if __name__ == "__main__":
    main()
