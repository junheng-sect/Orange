"""Ground-triggered lifecycle backed by the existing guarded navigation services."""
from copy import copy
import numpy as np
import rospy
from drone_navigation.core import check_goal_bounds, heading
from nav_outdoor_full import FullRunner
from nav_square import Stopped
from .route import MissionRoute, RETURN_NAME

class FlightLifecycle(FullRunner):
    test_name = record_prefix = 'inspection'
    route_name = '巡检任务'

    validate_points = staticmethod(list)

    def __init__(self, args, cfg, state, buf, prepare, check, client, fc, *services):
        self.route = args.points
        self.names = tuple(point['name'] for point in args.mission['waypoints'])
        self.holds = tuple(point.get('hold_s', 0.) for point in args.mission['waypoints'])
        super().__init__(copy(args), cfg, state, buf, prepare, check, client, fc, *services)
        # The relative-height tolerance can be reached while still climbing into the odom goal envelope.
        self.takeoff_min_odom_z = self.lo[2] + float(cfg.get('goal_boundary_margin', 0.))
        self.motion = MissionRoute(args, cfg, state, buf, self.prepare, self.scan_check, client)
        self.motion.owner = self
        self.check_service = check
        self.ground_start = None
        self.returning = False

    def scan_check(self, path):
        # The scanner also checks candidate goals while the previous goal is in its normal STOPPED state.
        return self.service_call('check_trajectory', self.check_service, path, stopped=True)

    def verify_profile(self):
        self.motion.camera_ready()

    def run_once(self):
        result = super().run_once()
        self.a.events.update(end=self.flight['end'], phase='DONE' if result else 'ABORTED',
                             reason=self.flight.get('reason'))
        return result

    def phase(self, name):
        super().phase(name)
        self.a.events.update(phase=name)

    def guard(self, **kwargs):
        super().guard(**kwargs)
        if self.motion.shelf and self.s.phase in ('COUNTDOWN', 'OFFBOARD_ENTRY'):
            frame = self.motion.shelf.vision.latest
            if frame is None or not -.02 <= (rospy.Time.now() - frame.header.stamp).to_sec() <= .5:
                raise Stopped('起飞前 OV13855 角点流中断')
        if self.reference is not None:
            r, t = self.reference
            pos = self.s.pose()[0]
            if np.any(pos[:2] < self.lo[:2]) or np.any(pos > self.hi):
                raise Stopped('控制点超出导航边界')
            map_z = float((r @ pos + t)[2])
            if self.returning:
                error = abs(map_z - self.record['return_map'][2])
                self.flight['return_max_z_error_m'] = max(error, self.flight.get('return_max_z_error_m', 0.))
                if error > .15:
                    raise Stopped('等高返航偏离末点高度超过 0.15 m')
            if map_z > self.motion.ceiling:
                raise Stopped('超过巡检地图限高')

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
        if (r @ takeoff + t)[2] > self.motion.ceiling:
            raise Stopped('预计起飞终点超过室内地图限高')
        ground_map = r @ self.ground_start + t
        if not self.motion.profile['return_home']:
            return self.goals, self.names
        home = np.array([*ground_map[:2], self.route[-1][2]])
        if home[2] <= ground_map[2]:
            raise Stopped('预计返航点不高于起飞地面，拒绝此起飞位置')
        return np.vstack([self.goals, r.T @ (home - t)]), self.names + (RETURN_NAME,)

    def wait_arm(self):
        super().wait_arm()
        r, t = self.reference
        p = self.ground_geometry()
        self.motion.ground_map = r @ p + t
        self.motion.reference = self.reference
        self.flight.update(home_odom=p.tolist(), home_map=self.motion.ground_map.tolist(),
                           ground_fcu_z=self.ground_z, takeoff_height=self.cfg['takeoff_height'],
                           takeoff_min_odom_z=self.takeoff_min_odom_z)
        self.save()

    def expected_takeoff_map_z(self):
        r, _ = self.reference
        return self.motion.ground_map[2] + r[2, 2] * self.cfg['takeoff_height']

    def fly_route(self):
        self.flight['takeoff_stable_ns'] = rospy.Time.now().to_nsec()
        self.phase('ROUTE')
        scan = self.motion
        scan.record, scan.record_path = self.record, self.record_path
        self.record.update(batches=[], scans=[])
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
                           hold_s=[scan.hold_for(i) for i in range(len(goals_map))],
                           home_map=scan.ground_map.tolist(), return_map=goals_map[-1].tolist() if scan.profile['return_home'] else None)
        self.save()
        try:
            for i, (goal, gm, yaw) in enumerate(zip(self.goals, goals_map, yaw_map)):
                if scan.skip_leg(i, goal, self.yaws[i]):
                    continue
                if self.record['legs']:
                    scan.reprepare()
                self.returning = scan.profile['return_home'] and i == len(self.route)
                scan.send_leg(i, goal, gm, yaw)
        finally:
            self.active = scan.active
            self.returning = False
        if scan.profile['return_home']:
            self.check_return()
            self.flight['return_error_m'] = float(np.linalg.norm(self.s.pose()[0] - self.goals[-1]))
        self.save()

    def route_finished(self):
        scans = sum(p['action'] == 'scan_shelf' for p in self.motion.points)
        legs = self.record['legs']
        return (len(self.record.get('batches', [])) == scans and bool(legs)
                and legs[-1]['direction'] == RETURN_NAME and all(leg['end'] == 'SUCCEEDED' for leg in legs))

    def land_once(self):
        if self.motion.profile['auto_land']:
            super().land_once()
        self.record['end'] = 'SUCCEEDED'
