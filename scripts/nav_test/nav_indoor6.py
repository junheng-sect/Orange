#!/usr/bin/env python3
"""Indoor6: ground-triggered takeoff, indoor5 shelf inspection, return and slow landing."""
import argparse
from copy import copy
import math
import time
from pathlib import Path

import numpy as np
import rospy

from drone_navigation.core import check_goal_bounds, heading
from nav_forward import say
from nav_global import serve
from nav_indoor4 import probe, wait_until
from nav_indoor5 import Indoor5Runner, load_indoor5_route
from nav_outdoor_full import FullRunner, run_full
from nav_square import Stopped

RETURN_NAME = '返回起飞点上方'


def return_point(ground_map, route):
    return np.array([*ground_map[:2], route['points'][-1]['xyz'][2]], dtype=float)


class Indoor6Survey(Indoor5Runner):
    test_name = record_prefix = 'test_indoor6'
    route_name = '室内 test6 四点扫码与返航'
    test_label = 'test6'
    event_prefix = 'INDOOR6'
    flight = None
    ground_map = None

    def make_route_map(self):
        points, yaws, names = super().make_route_map()
        ground = self.ground_map
        if ground is None:  # In preview, the operator marks home by starting here.
            r, t = self.reference
            ground = r @ self.s.pose()[0] + t
        return np.vstack([points, return_point(ground, self.route)]), np.append(yaws, yaws[-1]), names + (RETURN_NAME,)

    def guard(self, **kwargs):
        if self.flight is None:
            super().guard(**kwargs)
        else:
            self.flight.guard(**kwargs)

    def hold_for(self, i):
        return 2. if self.names[i] == RETURN_NAME else super().hold_for(i)

    def hold_at_point(self, i, leg, seconds):
        if self.names[i] == RETURN_NAME and self.flight is not None:
            until = time.monotonic() + seconds
            while True:
                self.flight.guard(stopped=True)
                self.flight.check_return()
                remaining = until - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(.05, remaining))
        else:
            super().hold_at_point(i, leg, seconds)


class Indoor6Flight(FullRunner):
    test_name = record_prefix = 'test_indoor6'
    route_name = Indoor6Survey.route_name

    def __init__(self, args, cfg, state, buf, prepare, check, client, fc, *services):
        self.route = args.points
        self.names = tuple(point['name'] for point in args.route['points'])
        self.holds = (args.route['hold_s'],) * len(self.route)
        super().__init__(copy(args), cfg, state, buf, prepare, check, client, fc, *services)
        # The relative-height tolerance can be reached while still climbing into the odom goal envelope.
        self.takeoff_min_odom_z = self.lo[2] + float(cfg.get('goal_boundary_margin', 0.))
        self.survey = Indoor6Survey(args, cfg, state, buf, self.prepare, self.scan_check, client)
        self.survey.flight = self
        self.check_service = check
        self.ground_start = None
        self.returning = False

    def scan_check(self, path):
        # The scanner also checks candidate goals while the previous goal is in its normal STOPPED state.
        return self.service_call('check_trajectory', self.check_service, path, stopped=True)

    def verify_profile(self):
        route = self.survey.route
        if (self.cfg.get('takeoff_height') != .5 or self.cfg.get('max_vel_z') != .3
                or self.cfg.get('max_acc_z') != .3 or self.cfg.get('allow_short_goals') is not True
                or self.cfg.get('allow_unknown_above') is not False
                or Path(self.cfg['map_id']).name != route['map_id']):
            raise Stopped('室内 test6 起飞、航线或地图配置不匹配')
        vision = self.survey.vision
        if wait_until(lambda: vision.latest, 2., self.guard, rospy.Time.now()) is None or vision.ready_info() is None:
            raise Stopped('地面预检未收到 OV13855 新鲜角点或标定')

    def guard(self, **kwargs):
        super().guard(**kwargs)
        if self.s.phase in ('COUNTDOWN', 'OFFBOARD_ENTRY'):
            frame = self.survey.vision.latest
            if frame is None or not -.02 <= (rospy.Time.now() - frame.header.stamp).to_sec() <= .5:
                raise Stopped('起飞前 OV13855 角点流中断')
        if self.reference is not None:
            r, t = self.reference
            map_z = float((r @ self.s.pose()[0] + t)[2])
            if self.returning:
                error = abs(map_z - self.record['return_map'][2])
                self.flight['return_max_z_error_m'] = max(error, self.flight.get('return_max_z_error_m', 0.))
                if error > .15:
                    raise Stopped('等高返航偏离右下点高度超过 0.15 m')
            if map_z > self.survey.route['max_control_z']:
                raise Stopped('test6 超过巡检地图限高')

    def ground_geometry(self):
        p, _, speed = self.s.pose()
        if speed > .1:
            raise Stopped('起飞前机体未静止')
        if self.ground_start is None:
            self.ground_start = p.copy()
        elif np.linalg.norm(p - self.ground_start) > .05:
            raise Stopped('地面预检后起飞位置移动超过 5 cm')
        return p

    def preflight_targets(self):
        r, t = self.reference
        takeoff = self.ground_start + np.array([0., 0., self.cfg['takeoff_height']])
        if (r @ takeoff + t)[2] > self.survey.route['max_control_z']:
            raise Stopped('预计起飞终点超过室内地图限高')
        ground_map = r @ self.ground_start + t
        home = return_point(ground_map, self.survey.route)
        if home[2] <= ground_map[2]:
            raise Stopped('预计返航点不高于起飞地面，拒绝此起飞位置')
        return np.vstack([self.goals, r.T @ (home - t)]), self.names + (RETURN_NAME,)

    def wait_arm(self):
        super().wait_arm()
        r, t = self.reference
        p = self.ground_geometry()
        self.survey.ground_map = r @ p + t
        self.survey.reference = self.reference
        self.flight.update(home_odom=p.tolist(), home_map=self.survey.ground_map.tolist(),
                           ground_fcu_z=self.ground_z, takeoff_height=self.cfg['takeoff_height'],
                           takeoff_min_odom_z=self.takeoff_min_odom_z)
        self.save()

    def expected_takeoff_map_z(self):
        r, _ = self.reference
        return self.survey.ground_map[2] + r[2, 2] * self.cfg['takeoff_height']

    def fly_route(self):
        self.flight['takeoff_stable_ns'] = rospy.Time.now().to_nsec()
        self.phase('ROUTE')
        scan = self.survey
        scan.record, scan.record_path = self.record, self.record_path
        scan.record_scan_setup()
        goals_map, yaw_map, scan.names = scan.make_route_map()
        r, t = self.reference
        self.goals = (goals_map - t) @ r
        self.yaws = np.array([heading(r.T, yaw) for yaw in yaw_map])
        pos = self.s.pose()[0]
        takeoff_xy_error = np.linalg.norm(pos[:2] - np.array(self.flight['home_odom'][:2]))
        if takeoff_xy_error > .15:
            raise Stopped('原地起飞水平偏差超过 0.15 m')
        check_goal_bounds(np.vstack([pos, self.goals]), self.cfg, ('起飞停稳点',) + scan.names)
        self.record.update(goals_map=goals_map.tolist(), goals_odom=self.goals.tolist(),
                           goal_yaw_map_deg=np.degrees(yaw_map).tolist(), goal_yaw_odom_deg=np.degrees(self.yaws).tolist(),
                           hold_s=[*self.holds, scan.hold_for(len(self.holds))],
                           home_map=scan.ground_map.tolist(), return_map=goals_map[-1].tolist())
        self.save()
        try:
            for i, (goal, gm, yaw) in enumerate(zip(self.goals, goals_map, yaw_map)):
                if i:
                    scan.reprepare()
                if scan.names[i] == RETURN_NAME:
                    self.returning = True
                    here = r @ self.s.pose()[0] + t
                    scan.report(len(scan.route['points']) - 1, '等高返回起飞点',
                                route=dict(points=[here.tolist(), gm.tolist()], ids=[]))
                scan.send_leg(i, goal, gm, yaw)
        finally:
            # The full-flight owner cancels the outstanding action on abort.
            self.active = scan.active
            self.returning = False
        self.check_return()
        self.flight['return_error_m'] = float(np.linalg.norm(self.s.pose()[0] - self.goals[-1]))
        self.save()

    def route_finished(self):
        legs = self.record['legs']
        return (len(self.record.get('batches', [])) == len(self.survey.route['points'])
                and bool(legs) and legs[-1]['direction'] == RETURN_NAME
                and all(leg['end'] == 'SUCCEEDED' for leg in legs))

    def land_once(self):
        super().land_once()
        self.record['end'] = 'SUCCEEDED'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--probe-camera', action='store_true')
    parser.add_argument('--capture-waypoints', action='store_true')
    parser.add_argument('--preview', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])
    if args.probe_camera:
        return probe(args.out, Indoor6Survey.vision_type, 'indoor6_camera_probe')
    args.route = load_indoor5_route()
    args.points = [(*point['xyz'], args.route['yaw_deg']) for point in args.route['points']]
    args.hold, args.delay = args.route['hold_s'], 3.
    args.max_start_offset, args.check_segments = math.inf, False
    args.out = str(args.out.resolve())
    if args.preview:
        return serve('nav_indoor6_preview', args, Indoor6Survey)
    return run_full(args, Indoor6Flight, 'test_indoor6')


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (Stopped, rospy.ROSException, ValueError) as error:
        say('test_indoor6 终止：' + str(error))
        raise SystemExit(1)
