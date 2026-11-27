"""One camera action shared by all shelf waypoints in a mission."""
from collections import Counter, defaultdict
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_matrix
from drone_navigation.core import finite, check_goal_bounds, rotation
from front_marker_control.localization import LocalPoseBuffer
from nav_forward import path_of
from nav_square import Stopped
from .geometry import STANDOFF, scan_target, shortest_order, paper_axes, fit_observation
from .vision import FRAME, SIZE, ShelfVision, load_calibration


class ShelfScan:
    def __init__(self, motion, args):
        self.motion = motion
        self.points = args.mission['waypoints']
        self.ceiling = args.mission['flight']['max_control_z']
        self.out = Path(args.archive)
        self.capture_waypoints = args.mission['recording']['capture_waypoints']
        self.cam_translation, self.cam_rotation, self.layout, self.config_paths = load_calibration()
        self.lever = finite(motion.cfg['lever_body'])
        self.vision = ShelfVision()
        self.pose_lock = threading.Lock()
        self.poses = LocalPoseBuffer(400)
        self.odom = rospy.Subscriber('/drone_nav/odom', Odometry, self.on_odom, queue_size=25)
        self.attempted = set()
        self.discovered = set()
        self.counts = Counter()
        self.axes = None
        self.yaw = None
        self.timer = rospy.Timer(rospy.Duration(.01), self.forward_pending)

    def on_odom(self, msg):
        with self.pose_lock:
            if msg.header.frame_id != 'drone_odom' or msg.child_frame_id != 'drone_nav_base':
                self.poses.clear()
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.poses.accept(msg.header.stamp.to_sec(), msg.header.frame_id,
                              (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def pose_at(self, stamp):
        with self.pose_lock:
            return self.poses.lookup(stamp.to_sec(), .20)

    def stopped(self):
        self.motion.guard(stopped=True)

    def capture(self, frame, index):
        if frame is None:
            return dict(error='no_fresh_full_resolution_frame')
        path = self.out / ('%02d_%s.png' % (index + 1, self.points[index]['name']))
        temp = path.with_suffix('.tmp.png')
        try:
            img = np.frombuffer(frame.data, np.uint8).reshape(SIZE[1], frame.step)[:, :SIZE[0]]
            if path.exists() or not cv2.imwrite(str(temp), img, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                raise OSError('cannot save observation photograph: ' + path.name)
            self.stopped()
            temp.replace(path)
        finally:
            temp.unlink(missing_ok=True)
        photo = dict(file=path.name, stamp_ns=frame.header.stamp.to_nsec(), frame_id=FRAME)
        nav = self.pose_at(frame.header.stamp)
        if nav is not None:
            r, t = self.motion.reference
            control = r @ nav.position + t
            camera = control + r @ rotation(nav.quaternion) @ (self.cam_translation - self.lever)
            photo.update(control_xyz_map=control.tolist(), camera_xyz_map=camera.tolist(),
                         body_quat_odom_xyzw=list(nav.quaternion))
        else:
            photo['pose_error'] = 'no time-aligned odometry for photo'
        return photo

    def measure(self, tag, pose):
        info = self.vision.small_info
        if info is None or pose is None or self.motion.reference is None:
            raise ValueError('missing calibration or aligned odometry')
        return fit_observation([(p.x,p.y) for p in tag.corners], pose, self.motion.reference,
                               self.cam_rotation, self.cam_translation - self.lever, self.axes,
                               self.layout['target_marker_size_m'], np.asarray(info.K).reshape(3,3), np.asarray(info.D))

    def forward_pending(self, _):
        vision = self.vision
        with vision.lock:
            if not vision.pending:
                return
            since, header, tag = vision.pending[0]
            stamp = header.stamp
            if (since != vision.since or vision.target is None or (rospy.Time.now()-stamp).to_sec() > .5):
                vision.pending.popleft()
                return
            pose = self.pose_at(stamp)
            if pose is None:
                # Do not let a frame older than the available odometry starve newer frames.
                with self.pose_lock:
                    newest = self.poses.newest_stamp
                if newest is not None and newest >= stamp.to_sec():
                    vision.pending.popleft()
                return
            vision.pending.popleft()
        try:
            center, orientation, _, _, rms = self.measure(tag, pose)
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            return
        if rms > 2.:
            return
        transform = np.eye(4)
        transform[:3,:3] = orientation
        msg = PoseStamped(header=header, pose=Pose(Point(*center), Quaternion(*quaternion_from_matrix(transform))))
        with vision.lock:
            if since == vision.since and vision.target == ('tag36h11',tag.id):
                vision.forwarded.append(msg)
                vision.pose_pub.publish(msg)

    def observe(self, frames, history):
        samples = defaultdict(dict)
        for frame in frames:
            pose = self.pose_at(frame.header.stamp)
            ids = Counter(tag.id for tag in frame.tags)
            for tag in frame.tags:
                key = (frame.family, tag.id)
                self.discovered.add(key)
                stamp = frame.header.stamp.to_nsec()
                pixels = [[p.x, p.y] for p in tag.corners]
                item = dict(family=frame.family, id=tag.id, stamp_ns=stamp,
                            corners_px=pixels if np.isfinite(pixels).all() else None)
                history.append(item)
                try:
                    if ids[tag.id] != 1:
                        raise ValueError('duplicate ID in frame')
                    center, _, point, camera, rms = self.measure(tag, pose)
                    item.update(center_camera=center.tolist(), xyz_map=point.tolist(), camera_map=camera.tolist(),
                                reprojection_rms_px=rms)
                    if rms > 2.:
                        raise ValueError('fixed-orientation reprojection RMS > 2 px')
                    samples[key][stamp] = point
                except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                    item['error'] = str(error)
        return samples

    def target(self, samples):
        if len(samples) < 3:
            raise ValueError('fewer than 3 time-aligned observations')
        points = np.array(list(samples.values()))
        center = np.median(points,axis=0)
        if np.linalg.norm(points-center,axis=1).max() > .10:
            raise ValueError('tag positions vary by more than 0.10 m from the median')
        g, odom, angle, camera, paper = scan_target(center,self.axes,self.layout['label'],self.motion.reference,
                                                   self.cam_translation-self.lever, self.yaw)
        return g, odom, angle, dict(tag_xyz_map=center.tolist(), goal_map=g.tolist(), goal_odom=odom.tolist(),
                                    goal_height_map_m=float(g[2]), yaw_map_deg=math.degrees(angle),
                                    camera_map=camera.tolist(), paper_map=paper.tolist(), standoff_m=STANDOFF)

    def precheck(self, g, odom, current=None):
        self.stopped()
        if g[2] > self.ceiling:
            raise ValueError('above mission height limit')
        check_goal_bounds([odom], self.motion.cfg, ['scan goal'])
        points = [current, odom] if current is not None and np.linalg.norm(current - odom) < .20 else [odom]
        response = self.motion.check(path_of(points))
        if not response.valid:
            raise ValueError(response.reason)
        self.stopped()

    def report(self, index, state, key=None, route=None):
        self.motion.events.update(waypoint_index=index + 1, observation=self.points[index]['name'], phase='ROUTE', action=state,
                                  tagId=key[1] if key else None, discovered=len(self.discovered),
                                  success=self.counts['ok'], failed=self.counts['failed'], **({'route': route} if route is not None else {}))

    def run(self, i):
        point = self.points[i]
        self.vision.select(None)
        with self.vision.lock:
            self.vision.frames.clear()
        self.yaw = math.radians(point['yaw_deg'])
        self.axes = paper_axes(self.yaw)
        self.report(i, '识别标签')
        self.motion.pause(1., stopped=True)
        start = rospy.Time.now()
        end = start + rospy.Duration(point['hold_s'] - 1.)
        frame = self.vision.take_photo(start, self.stopped) if self.capture_waypoints else None
        self.motion.pause(max(0., (end - rospy.Time.now()).to_sec()) + .20, stopped=True)
        batch = dict(observation=i + 1, yaw_deg=point['yaw_deg'], axes=self.axes.tolist(), name=self.points[i]['name'],
                     photo=self.capture(frame, i) if self.capture_waypoints else dict(status='disabled'),
                     observations=[], candidates=[], order=[])
        self.motion.record['batches'].append(batch)
        frames = self.vision.window(start, end)
        batch['frames'] = len(frames)
        if len({frame.header.stamp.to_nsec() for frame in frames}) < 3:
            self.motion.save()
            raise Stopped('观察窗口内 AprilTag 相机数据断流')
        samples = self.observe(frames, batch['observations'])
        observed = {(item['family'], item['id']) for item in batch['observations']}
        targets = {}
        for key in sorted(observed):
            entry = dict(family=key[0], id=key[1])
            batch['candidates'].append(entry)
            if key in self.attempted:
                entry['status'] = 'already_attempted'
                continue
            try:
                goal, odom, angle, details = self.target(samples.get(key, {}))
                entry.update(details)
                self.precheck(goal, odom)
            except ValueError as error:
                entry.update(status='skipped', reason=str(error))
                continue
            entry['status'] = 'planned'
            targets[key] = goal, odom, angle, entry
        r, t = self.motion.reference
        next_index = min(i + 1, len(self.points) - 1)
        finish = np.asarray(self.points[next_index]['xyz'])
        start_map = r @ self.motion.s.pose()[0] + t
        order, distance = shortest_order(start_map, {k: v[0] for k, v in targets.items()}, finish)
        batch.update(order=[k[1] for k in order], distance_m=distance,
                     start_map=start_map.tolist(), finish_map=finish.tolist())
        displayed = [start_map.tolist(), *[targets[k][0].tolist() for k in order], finish.tolist()]
        self.motion.save()
        self.report(i, '扫描规划完成' if order else '无可扫描的新标签', route=dict(points=displayed, ids=[key[1] for key in order]))
        moved = False
        for key in order:
            g, odom, angle, entry = targets[key]
            try:
                self.precheck(g, odom, self.motion.s.pose()[0])
            except ValueError as error:
                entry.update(status='skipped', reason=str(error))
                self.motion.save()
                continue
            self.report(i, '前往读码点', key)
            self.motion.fly_extra('Tag %d 读码点' % key[1], g, odom, angle)
            moved = True
            self.attempted.add(key)
            self.report(i, '读取 QR', key)
            result = dict(family=key[0], id=key[1], observation=i + 1,
                          goal_map=g.tolist(), started_ns=rospy.Time.now().to_nsec(), status='running')
            self.motion.record['scans'].append(result)
            self.motion.save()
            try:
                result.update(self.decode_once(key))
            except BaseException as error:
                result.update(status='aborted', reason=str(error))
                raise
            finally:
                self.motion.save()
            self.counts['ok' if result['status'] == 'success' else 'failed'] += 1
            self.report(i, '读码成功' if result['status'] == 'success' else '读码失败', key)
        if next_index == i and moved:
            self.report(i, '返回末观察点')
            self.motion.fly_extra('返回' + point['name'] + '观察点', finish, r.T @ (finish - t), self.yaw)

    def decode_once(self, key):
        done = threading.Event()
        result = []

        def decode():
            try:
                result.append(self.vision.decode())
            except rospy.ServiceException as error:
                result.append(error)
            finally:
                done.set()

        self.vision.select(key)
        worker = None
        try:
            # Give the downscaled stream one frame to populate the target pose.
            self.motion.pause(.25, stopped=True)
            start = rospy.Time.now()
            worker = threading.Thread(target=decode, daemon=True)
            worker.start()
            deadline = time.monotonic() + 6.
            while not done.wait(.025):
                self.stopped()
                if time.monotonic() >= deadline:
                    raise Stopped('读码服务超时')
            self.stopped()
            if not result or isinstance(result[0], Exception):
                return dict(status='failed', reason='decode_service_error: ' + str(result[0] if result else 'no response'))
            response = result[0]
            record = dict(frame_ns=response.header.stamp.to_nsec(), elapsed_s=response.elapsed_s,
                          decoder_report=response.report, session_dir=response.session_dir)
            if response.success and response.used_tag_roi and response.header.frame_id == FRAME and response.header.stamp >= start:
                pose = self.vision.matched_pose(response.header.stamp)
                if len(response.codes) == 1 and pose and self.code_on_label(response.codes[0], pose):
                    code = response.codes[0]
                    return dict(record, status='success', qr=code.code, raw_hex=code.raw_hex,
                                decode_strategy=code.source)
            return dict(record, status='failed', reason=response.error or 'QR not associated with requested tag')
        finally:
            self.vision.select(None)
            if worker is not None and not done.is_set():
                cancel = threading.Thread(target=self.cancel_decode, daemon=True)
                cancel.start()
                cancel.join(timeout=.3)

    def cancel_decode(self):
        try:
            self.vision.cancel()
        except rospy.ServiceException as error:
            say('读码取消服务异常：' + str(error))

    def code_on_label(self, code, pose):
        info = self.vision.info
        if code.format != 'QRCode' or info is None:
            return False
        p, q = pose.pose.position, pose.pose.orientation
        rvec = cv2.Rodrigues(rotation([q.x, q.y, q.z, q.w]))[0]
        tvec = np.array([p.x, p.y, p.z])
        label = self.layout['label']
        w = label['width_meters'] / 2.
        h = label['height_meters'] / 2.
        x, y = label['tag_offset_x_meters'], label['tag_offset_y_meters']
        # Check the code against the physical paper above the tag, not the
        # larger padded ROI the decoder uses for detection.
        bottom, top = self.layout['target_marker_size_m'] / 2., h - y
        corners = np.array([[-w-x, bottom, 0], [w-x, bottom, 0], [w-x, top, 0], [-w-x, top, 0]])
        area, _ = cv2.projectPoints(corners, rvec, tvec, np.asarray(info.K).reshape(3, 3), np.asarray(info.D))
        pixels = np.asarray(code.corners).reshape(4, 2)
        return (np.isfinite(pixels).all() and
                all(cv2.pointPolygonTest(area.astype(np.float32), tuple(p), True) >= -3. for p in pixels)
                and abs(cv2.contourArea(pixels.astype(np.float32))) > 64.)
