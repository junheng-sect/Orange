"""Execute map-frame segments and arrival actions with the navigation gateway."""
import math
import time

import actionlib
import numpy as np
import rospy
from geometry_msgs.msg import Point
from drone_navigation.msg import ExecutePathAction, ExecutePathGoal
from drone_navigation.core import heading
from nav_square import SquareRunner, Stopped
from .config import segments


RETURN_NAME = '返回起飞点上方'


class MissionRoute(SquareRunner):
    goals_in_map = True
    side_m = None
    repeat_laps = False
    test_name = record_prefix = 'inspection'
    route_name = '巡检任务'

    def __init__(self, args, cfg, state, buf, prepare, check, client):
        super().__init__(args, cfg, state, buf, prepare, check, client)
        for key, value in args.mission['navigation'].items():
            if cfg.get(key) != value:
                raise Stopped('导航实际配置不匹配：' + key)
        self.manual_fc = None
        if self.real and not args.mission['flight']['auto_takeoff']:
            from nav_land import Fc
            self.manual_fc = Fc(state)
        self.points = args.mission['waypoints']
        self.profile = args.mission['flight']
        self.events = args.events
        self.hold_seconds = 0.
        self.ground_map = None
        self.owner = None
        self.completed = False
        self.returning = False
        self.ceiling = self.profile['max_control_z']
        self.groups = {g[-1]: g for g in segments(self.points)}
        self.path_client = None
        if any(len(g) > 1 for g in self.groups.values()):
            self.path_client = actionlib.SimpleActionClient('/drone_nav/execute_path', ExecutePathAction)
            if not self.path_client.wait_for_server(rospy.Duration(10)):
                raise Stopped('连续航段 action 未就绪')
        self.shelf = None
        if any(p['action'] == 'scan_shelf' for p in self.points):
            from .shelf import ShelfScan
            self.shelf = ShelfScan(self, args)

    def camera_ready(self):
        if self.shelf:
            from .vision import wait_until
            v = self.shelf.vision
            if wait_until(lambda: v.latest, 2., lambda: None, rospy.Time.now()) is None or v.ready_info() is None:
                raise Stopped('OV13855 标定或角点流未就绪')

    def guard(self, **opts):
        if self.owner:
            self.owner.guard(**opts)
        else:
            super().guard(**opts)
        if self.manual_fc and (not self.manual_fc.ext_fresh() or self.manual_fc.landed != 2):
            raise Stopped('手飞接入需要新鲜的在空中确认')
        if self.reference is not None:
            r, t = self.reference
            pos = self.s.pose()[0]
            if np.any(pos < self.lo) or np.any(pos > self.hi):
                raise Stopped('控制点超出 drone_odom 边界')
            z = (r @ pos + t)[2]
            if z > self.ceiling:
                raise Stopped('超过巡检地图限高')
            if self.returning and abs(z - self.points[-1]['xyz'][2]) > .15:
                raise Stopped('等高返航偏离目标高度超过 0.15 m')
            if self.shelf:
                frame = self.shelf.vision.latest
                if frame is None or not -.02 <= (rospy.Time.now() - frame.header.stamp).to_sec() <= .5:
                    raise Stopped('OV13855 角点流中断')

    def make_route_map(self):
        r, t = self.reference
        here, yaw, _ = self.s.pose()
        yaw = heading(r, yaw)
        xyz = [p['xyz'] for p in self.points]
        angles = [math.radians(p['yaw_deg']) if 'yaw_deg' in p else yaw for p in self.points]
        names = [p['name'] for p in self.points]
        if self.profile['return_home']:
            home = self.ground_map if self.ground_map is not None else r @ here + t
            xyz.append([*home[:2], xyz[-1][2]])
            angles.append(angles[-1])
            names.append(RETURN_NAME)
        return np.array(xyz), np.array(angles), tuple(names)

    def prepare_preview(self):
        # READY inputs can precede the gateway's first aligned LIO/PX4 pose pair.
        until = time.monotonic() + 10.
        while True:
            reason = self.inputs_problem()
            if reason or self.s.get('gateway_status') == 'FAULT':
                raise Stopped(reason or str(self.s.get('gateway_status', 'reason')))
            reply = self.prepare()
            if (reply.success or reply.message != 'alignment, inputs or exclusive ownership not ready'
                    or time.monotonic() >= until or rospy.is_shutdown()):
                return reply
            time.sleep(.15)

    def freeze(self):
        if self.manual_fc and self.s.pose()[2] > .1:
            raise Stopped('手飞接入时未停稳')
        self.camera_ready()
        values = super().freeze()
        self.record.update(batches=[], scans=[])
        self.events.update(phase='ROUTE')
        return values

    def run_lap(self):
        self.completed = super().run_lap()
        self.events.update(phase='DONE' if self.completed else 'ABORTED',
                           end='SUCCEEDED' if self.completed else 'ABORTED',
                           reason=(None if self.completed else getattr(self, 'abort_reason', self.s.issue)))
        return self.completed

    def skip_leg(self, i, goal, yaw):
        return i < len(self.points) and i not in self.groups

    def save(self):
        if self.record is not None and self.shelf:
            s = self.shelf
            self.record['tag_counts'] = dict(discovered=len(s.discovered), read_ok=s.counts['ok'])
        super().save()
        if self.record is not None:
            self.events.scans(self.record)

    def fly_extra(self, name, g, odom, angle):
        i = len(self.names)
        self.names += (name,)
        self.reprepare()
        self.send_point(i, odom, g, angle)

    def send_point(self, i, goal, gm, yaw):
        self.check_segments = np.linalg.norm(self.s.pose()[0] - goal) < .20
        super().send_leg(i, goal, gm, yaw)

    def path(self, group):
        self.guard()
        points = [self.points[j]['xyz'] for j in group]
        leg = dict(direction=self.names[group[-1]], path_map=points, end='RUNNING')
        self.record['legs'].append(leg)
        self.save()
        client = self.path_client
        deadline = time.monotonic() + 95.
        feedback = [0]
        def on_progress(msg):
            feedback[0] = msg.point_index
        try:
            client.send_goal(ExecutePathGoal(map_id=self.cfg['map_id'], points=[Point(*p) for p in points]),
                             feedback_cb=on_progress)
            previous = None
            while client.get_result() is None:
                self.guard(stopped=True)
                if client.get_state() == 9 or time.monotonic() >= deadline:
                    raise Stopped('连续航段断开或超时')
                idx = feedback[0]
                if 0 <= idx < len(group) and previous != idx:
                    self.events.update(phase='ROUTE', waypoint_index=group[idx] + 1, observation=self.names[group[idx]],
                                       action='连续通行', route=None, tagId=None)
                    previous = idx
                time.sleep(.05)
            self.guard(stopped=True)
            result = client.get_result()
            if client.get_state() != 3 or not result.success:
                raise Stopped('连续航段失败：' + result.reason)
        except BaseException:
            client.cancel_goal()
            raise
        leg.update(end='SUCCEEDED')
        self.save()

    def send_leg(self, i, goal, gm, yaw):
        if i < len(self.points):
            group = self.groups[i]
            if len(group) > 1:
                self.path(group)
                if self.points[i]['action'] == 'pass':
                    return
                self.reprepare()
            self.events.update(phase='ROUTE', waypoint_index=i + 1, observation=self.names[i], action='前往航点', tagId=None, route=None)
        else:
            self.returning = True
            self.events.update(phase='RETURN', observation=RETURN_NAME, action='等高返航')
        if (i < len(self.points) and self.points[i]['action'] == 'pass'
                or self.returning and self.points[-1]['action'] == 'pass'):
            # A single point cannot form an ExecutePath; hold its departure heading.
            r, _ = self.reference
            yaw_odom = self.s.pose()[1]
            yaw = heading(r, yaw_odom)
            self.record['goal_yaw_map_deg'][i] = math.degrees(yaw)
            self.record['goal_yaw_odom_deg'][i] = math.degrees(yaw_odom)
            if self.owner:
                self.owner.yaws[i] = yaw_odom
        try:
            self.send_point(i, goal, gm, yaw)
        finally:
            self.returning = False

    def hold_for(self, i):
        if i < len(self.points):
            return self.points[i].get('hold_s', 0.)
        return 2. if self.profile['return_home'] and i == len(self.points) else 1.

    def hold_at_point(self, i, leg, seconds):
        if i < len(self.points) and self.points[i]['action'] == 'scan_shelf':
            self.shelf.run(i)
        elif self.returning and self.owner:
            until = time.monotonic() + seconds
            while True:
                self.guard(stopped=True)
                self.owner.check_return()
                if time.monotonic() >= until:
                    break
                time.sleep(.05)
        else:
            self.pause(seconds, stopped=True)
