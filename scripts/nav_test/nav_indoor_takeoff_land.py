#!/usr/bin/env python3
"""Single, pilot-armed vertical flight above the measured indoor takeoff point."""
import argparse
import math
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from mavros_msgs.srv import ParamGet
from std_msgs.msg import Bool

from drone_navigation.core import wrap
from nav_outdoor_full import FullRunner, Stopped, run_full, say
from vertical_profile import MAP, MAX_MAP_Z, PARAMETERS, TASK, arguments, validate


class VerticalFlight(FullRunner):
    test_name = record_prefix = TASK
    arm_prompt = '起飞点预检通过。通道 6 开录包，在 POSCTL 地面人工解锁，本次只原地起降一趟。'
    route = names = holds = ()
    validate_points = staticmethod(tuple)  # This runner has no route or action client.

    def __init__(self, *args):
        super().__init__(*args)
        self.home = None
        self.home_yaw = None
        self.takeoff_min_odom_z = self.lo[2] + self.cfg['goal_boundary_margin']
        self.permit = rospy.Publisher('/drone_nav/vertical_test_permit', Bool, queue_size=1)

    def create_records(self):
        super().create_records()
        self.flight.update(parameters={key: getattr(self.a, key) for key in PARAMETERS},
                           max_horizontal_drift_m=0., MPC_TKO_RAMP_T=None)
        self.save()

    def verify_profile(self):
        if (Path(self.cfg['map_id']).name != MAP or self.cfg.get('vertical_test_guard') is not True
                or self.cfg.get('takeoff_height') != self.a.height
                or self.s.get('gateway_status', 'vertical_test_height') != self.a.height
                or self.cfg.get('max_vel_z') != .3 or self.cfg.get('max_acc_z') != .3
                or self.cfg.get('allow_unknown_above') is not False):
            raise Stopped('室内地图、网关起飞高度或竖直限制与本趟配置不一致')
        # Parameter sampling is diagnostic only. Even a wedged MAVROS call must not block preflight.
        values = []

        def sample():
            try:
                response = rospy.ServiceProxy('/mavros/param/get', ParamGet)(param_id='MPC_TKO_RAMP_T')
                if response.success and math.isfinite(response.value.real):
                    values.append(response.value.real)
            except rospy.ServiceException:
                pass

        worker = threading.Thread(target=sample, daemon=True)
        worker.start()
        worker.join(1.)
        self.flight['MPC_TKO_RAMP_T'] = values[0] if values else None
        self.save()

    def ground_geometry(self):
        p, yaw, speed = self.s.pose()
        if speed > .1:
            raise Stopped('地面未静止')
        if self.home is None:
            self.home, self.home_yaw = p.copy(), yaw
            self.flight.update(home_odom=p.tolist(), home_map=(self.reference[0] @ p + self.reference[1]).tolist(),
                               home_yaw_rad=yaw)
        elif np.linalg.norm(p - self.home) > .05:
            raise Stopped('地面预检后起飞位置移动超过 5 cm')
        return p

    def expected_takeoff_map_z(self):
        return self.flight['home_map'][2] + self.reference[0][2, 2] * self.a.height

    def preflight_targets(self):
        if self.expected_takeoff_map_z() > MAX_MAP_Z:
            raise Stopped('预计起飞终点超过室内地图限高 1.70 m')
        return self.goals, self.names  # FullRunner adds the mandatory takeoff endpoint.

    def guard(self, **kwargs):
        super().guard(**kwargs)
        if self.home is None:
            return
        p, _, _ = self.s.pose()
        if self.s.phase in ('COUNTDOWN', 'OFFBOARD_ENTRY'):
            self.ground_geometry()
        if self.s.phase in ('TAKEOFF', 'HOVER', 'LAND_REQUEST', 'LANDING'):
            drift = float(np.linalg.norm(p[:2] - self.home[:2]))
            self.flight['max_horizontal_drift_m'] = max(drift, self.flight['max_horizontal_drift_m'])
            if drift > .15:
                raise Stopped('原地起降水平偏移 %.3f m，超过 0.15 m' % drift)
            if (self.reference[0] @ p + self.reference[1])[2] > MAX_MAP_Z:
                raise Stopped('超过室内地图限高 1.70 m')
        self.permit.publish(True)

    def check_return(self):
        p, yaw, speed = self.s.pose()
        if (abs(p[2] - self.home[2] - self.a.height) > .10 or speed > .1
                or abs(wrap(yaw - self.home_yaw)) > math.radians(5)):
            raise Stopped('原地悬停高度、速度或航向未稳定，取消慢降')

    def fly_route(self):
        self.phase('HOVER')
        self.flight['hover_start_wall'] = time.time()
        try:
            self.pause(self.a.hover)
            self.check_return()
            self.flight['hover_completed'] = True
        finally:
            self.flight['hover_end_wall'] = time.time()
            self.save()

    def route_finished(self):
        return self.flight.get('hover_completed') is True

    def prepare_for_landing(self):
        if self.s.get('gateway_status') != 'READY':
            raise Stopped('悬停后网关未 READY，取消慢降')

    def land_once(self):
        super().land_once()
        self.record.update(end='SUCCEEDED', end_wall=time.time())

    def run_once(self):
        try:
            return super().run_once()
        finally:
            if self.flight and self.flight['end'] != 'SUCCEEDED':
                self.permit.publish(False)
                say('原地起降中止：' + self.flight.get('reason', '流程中断'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    arguments(parser)
    parser.add_argument('--out', required=True)
    a = parser.parse_args(rospy.myargv()[1:])
    validate({key: getattr(a, key) for key in PARAMETERS})
    a.preview = False
    return run_full(a, VerticalFlight, TASK)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (Stopped, ValueError, rospy.ROSException) as error:
        say('原地起降中止：' + str(error))
        raise SystemExit(1)
