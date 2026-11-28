"""Exercise the vertical runner over loopback ROS with a mock FCU and no hardware."""
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import rospy
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamGet, ParamGetResponse
from std_msgs.msg import Bool
from traj_utils.srv import CheckTrajectoryResponse
from integration_outdoor_full import Fixture, nav
from nav_indoor_takeoff_land import VerticalFlight
from vertical_profile import MAP, TASK


class VerticalFixture(Fixture):
    climb_height = vertical_height = .65

    def publish(self):
        phase = self.runner.s.phase if self.runner else None
        if self.scenario == 'drift' and phase == 'TAKEOFF' and self.landed == 2:
            self.position[0] += .20
        if self.scenario == 'landing_drift' and phase == 'LANDING':
            self.position[1] += .20
        if self.scenario == 'takeover' and phase == 'HOVER':
            self.mode = 'POSCTL'
        super().publish()

    def check(self, req):
        if self.scenario == 'early_arm':
            self.armed = True
            time.sleep(.15)
        if self.scenario == 'map_occupied':
            return CheckTrajectoryResponse(valid=False, reason='occupied footprint')
        return super().check(req)


def main(out, scenario):
    assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11452'
    rospy.init_node('vertical_fixture')
    fixture = VerticalFixture()
    fixture.reset(scenario)
    fixture.map_id = MAP
    fixture.position[:2] = [.38, -.12]
    fixture.yaw = .35
    height = 1.8 if scenario == 'height_limit' else fixture.climb_height
    fixture.vertical_height = height
    cfg = rospy.get_param('/drone_nav/config')
    cfg.update(map_id=MAP, takeoff_height=height, vertical_test_guard=True)
    rospy.set_param('/drone_nav/config', cfg)
    if scenario == 'normal':
        rospy.Service('/mavros/param/get', ParamGet,
                      lambda req: ParamGetResponse(req.param_id == 'MPC_TKO_RAMP_T', ParamValue(real=1.2)))
    permits = []
    subscription = rospy.Subscriber('/drone_nav/vertical_test_permit', Bool, lambda m: permits.append(m.data))
    args = SimpleNamespace(preview=False, out=str(out), height=height, delay=1., hover=0. if scenario=='zero' else .3)

    class Runner(VerticalFlight):
        def __init__(self, *services):
            super().__init__(*services)
            fixture.runner = self

    try:
        with patch.object(rospy, 'init_node'), patch.object(nav, 'PREFLIGHT_MAP_TIMEOUT', 1.5):
            code = nav.run_full(args, Runner, TASK)
        flight = json.loads(next(out.glob('flight_*.json')).read_text())
        goal = json.loads(next(out.glob('goal_*.json')).read_text())
        success = scenario in ('normal','zero')
        assert (code == 0) == success, (scenario, flight.get('reason'))
        assert flight['end'] == goal['end'] == ('SUCCEEDED' if success else 'ABORTED')
        assert not goal['legs'] and not fixture.goals
        assert flight['parameters'] == dict(height=height, hover=args.hover, delay=1.)
        assert flight['MPC_TKO_RAMP_T'] == (1.2 if scenario == 'normal' else None)
        if success:
            assert [p['phase'] for p in flight['phases']] == [
                'PREFLIGHT','WAIT_ARM','COUNTDOWN','OFFBOARD_ENTRY','TAKEOFF','HOVER','LAND_REQUEST','LANDING','DONE']
            assert flight['hover_end_wall'] - flight['hover_start_wall'] >= args.hover
            assert len(flight['preflight_checks']) == 1 and flight['landing_states']
            assert flight['home_odom'][:2] == [.38,-.12]
            assert flight['takeoff_request_wall'] < flight['hover_start_wall'] < flight['land_request_wall']
        else:
            deadline = time.monotonic()+1.
            while False not in permits and time.monotonic()<deadline:
                time.sleep(.02)
            assert False in permits
            why = dict(early_arm='解锁', map_unknown='预检等待', map_occupied='occupied', height_limit='限高',
                       drift='水平偏移', takeover='模式', stale_landing='断流', landing_drift='水平偏移')[scenario]
            assert why in flight['reason'], flight['reason']
        airborne = success or scenario in ('drift','takeover','stale_landing','landing_drift')
        assert fixture.takeoff_count == int(airborne)
        assert fixture.land_count == int(success or scenario in ('stale_landing','landing_drift'))
        assert len(fixture.requests) == int(airborne)
        print('PASS vertical ' + scenario, flush=True)
    finally:
        subscription.unregister()
        fixture.close()
        fixture.server.action_server.stop()
        rospy.signal_shutdown('vertical scenario complete')


if __name__ == '__main__':
    main(Path(sys.argv[1]), sys.argv[2])
