"""One indoor6 scenario per process on the loopback-only, hardware-free test master."""
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rospy

from integration_outdoor_full import Fixture, nav
from integration_indoor5 import CornerCamera
from nav_indoor6 import RETURN_NAME, Indoor6Flight, load_indoor5_route


class IndoorFixture(Fixture):
    climb_height = .5
    map_yaw = 0.
    final_clear_wall = None

    @property
    def ground_level(self):
        # The synthetic FCU must touch the same floor that it sampled before liftoff.
        return -.073 if self.scenario in ('clearance_recovers', 'clearance_low') else 0.

    def publish(self):
        if self.scenario in ('clearance_recovers', 'clearance_low'):
            if self.climb_timer is None:
                self.position[2] = -.073
            elif self.runner.s.phase == 'TAKEOFF':
                elapsed = time.monotonic() - self.climb_timer
                # Relative climb already within 0.50 +/- 0.10 m, odom floor still unmet.
                z = .386
                if self.scenario == 'clearance_recovers':
                    if 2.8 <= elapsed < 3.8:
                        z = .410
                    elif elapsed >= 4.6:
                        z = .425
                        if self.final_clear_wall is None:
                            self.final_clear_wall = time.time()
                self.position[2] = z
        super().publish()

    @property
    def p(self):
        return self.position

    @property
    def a(self):
        return self.yaw

    def check(self, req):
        p = req.path.poses[-1].pose.position
        if self.scenario == 'return_blocked' and self.goals and np.linalg.norm([p.x-.36, p.y-.20]) < .02:
            from traj_utils.srv import CheckTrajectoryResponse
            return CheckTrajectoryResponse(valid=False, reason='occupied footprint at home')
        return super().check(req)

    def execute(self, goal):
        p = goal.waypoints[0].pose.position
        if self.scenario == 'return_z_excursion' and np.linalg.norm([p.x-.36, p.y-.20]) < .02:
            with self.lock:
                self.position[2] += .25
        if self.scenario == 'return_takeover' and np.linalg.norm([p.x-.36, p.y-.20]) < .02:
            with self.lock:
                self.mode = 'POSCTL'
        return super().execute(goal)

    def prepare_takeoff(self, request):
        result = super().prepare_takeoff(request)
        if self.scenario == 'countdown_disarm':
            with self.lock:
                self.arm_timer = float('inf')
                self.armed = False
        return result


class Camera(CornerCamera):
    def configure(self):
        super().configure()
        if self.flight.scenario == 'miss_qr':
            self.missing_key = 3
        elif self.flight.scenario == 'empty_final':
            del self.coords[3]
            self.tag_axes.pop(3)

    def publish(self, ev):
        # Other labels already scanned can still be visible at the final point.
        if (self.flight.scenario == 'empty_final' and np.linalg.norm(self.flight.p - self.route[3]['xyz']) < .04):
            from std_msgs.msg import Header
            from ov13855_apriltag_vision.msg import TagCornerArray
            from nav_indoor4 import FRAME
            self.tags.publish(TagCornerArray(header=Header(stamp=rospy.Time.now(), frame_id=FRAME),
                                            family='tag36h11', width=1280, height=960, tags=[]))
            return
        super().publish(ev)


def warm_startup(out, share, mission, fixture):
    """Real subprocess entrypoints, latched RC edges and per-run supervisor barriers."""
    import json
    import subprocess
    import rosgraph
    env = dict(os.environ, INSPECTION_ARCHIVE=str(out), INSPECTION_LIVE=str(out),
               INSPECTION_WORKSPACE=str(share.parents[2]), INSPECTION_PREVIEW='0',
               INSPECTION_CONFIG=json.dumps(mission))
    cmd = [sys.executable, '-B', '-u', '-m', 'drone_inspection.runtime']
    probe = subprocess.run(cmd + ['probe'], env=env, capture_output=True, text=True, timeout=25)
    assert probe.returncode == 0, probe.stdout + probe.stderr
    assert '相机就绪' in probe.stdout
    print('PASS production camera probe', flush=True)
    master = rosgraph.Master('/inspection_warm_fixture')
    for early_arm in (False, True):
        run = out / ('arming' if early_arm else 'normal')
        run.mkdir()
        env.update(INSPECTION_ARCHIVE=str(run), INSPECTION_LIVE=str(run))
        release = run / 'release'
        previous = set(dict(master.getSystemState()[1]).get('/mavros/state', []))
        with (run / 'runner.log').open('w') as log:
            proc = subprocess.Popen(cmd + ['warm', str(release)], env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 10
                while True:
                    subscribers = dict(master.getSystemState()[1]).get('/mavros/state', [])
                    if any(n.startswith('/inspection_') and n not in previous for n in subscribers):
                        break
                    assert proc.poll() is None and time.monotonic() < deadline
                    time.sleep(.05)
                time.sleep(.4)
                if early_arm:
                    with fixture.lock:
                        fixture.armed = True
                    time.sleep(.35)
                    with fixture.lock:
                        fixture.armed = False
                (run/'gateway_started').touch()
                time.sleep(3.)
                snapshot = run/'mission_state.json'
                if not early_arm:
                    assert json.loads(snapshot.read_text())['phase'] == 'BRINGUP'
                    assert fixture.check_count == 0
                release.touch()
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline:
                    state = json.loads(snapshot.read_text())
                    if state['phase'] in ('WAIT_ARM','ABORTED'): break
                    assert proc.poll() is None, (run/'runner.log').read_text()
                    time.sleep(.05)
                if early_arm:
                    assert state['phase'] == 'ABORTED' and '解锁' in state['reason'], state
                else:
                    assert state['phase'] == 'WAIT_ARM', (run/'runner.log').read_text()
                assert not fixture.requests and not fixture.takeoff_count and not fixture.goals
                print('PASS warm runner', state['phase'], flush=True)
            finally:
                if proc.poll() is None:
                    proc.terminate()
                proc.wait(timeout=5)


def main(out, scenario, port=11448):
    assert port in (11448, 11450) and os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:%d' % port
    inspection = port == 11450
    runner_type = Indoor6Flight
    route = load_indoor5_route()
    rospy.init_node('indoor6_fixture')
    fixture = IndoorFixture()
    fixture.reset(scenario)
    fixture.position[:2] = [.36, .20]
    fixture.map_id = route['map_id']
    cfg = rospy.get_param('/drone_nav/config')
    cfg.update(takeoff_height=.5, allow_short_goals=True, map_id=fixture.map_id)
    rospy.set_param('/drone_nav/config', cfg)
    if inspection:
        Camera.target_topic = '/drone_inspection/target_tag_pose'
    camera = Camera(fixture, cfg, route['points'])
    out.mkdir(parents=True)
    args = SimpleNamespace(preview=False, out=str(out), route=route, points=[(*p['xyz'], route['yaw_deg']) for p in route['points']],
                           hold=route['hold_s'], delay=3., max_start_offset=float('inf'), check_segments=False)
    # run_full owns init_node in production. Here ROS is initialized by the fixture.
    import json
    from unittest.mock import patch
    if inspection:
        from drone_inspection.lifecycle import FlightLifecycle
        from drone_inspection.config import load
        from drone_inspection.records import Records
        from drone_inspection.runtime import arguments
        import rospkg
        share = Path(rospkg.RosPack().get_path('drone_inspection'))
        mission = load(share / 'missions/indoor_shelf.yaml')
        mission['navigation'] = {k: cfg[k] for k in ('map_id', 'takeoff_height', 'allow_short_goals')}
        if scenario == 'auto_hold':
            mission['flight'].update(auto_land=False, return_home=False)
            mission['waypoints'] = [dict(name='交接点', xyz=[.8,.2,.7], yaw_deg=30., hold_s=.1, action='hover')]
        args = arguments(mission, out, out, Records(out, out.name, mission), False)
        runner_type = FlightLifecycle

    if inspection and scenario == 'warm_startup':
        try:
            warm_startup(out, share, mission, fixture)
        finally:
            camera.close()
            fixture.close()
            rospy.signal_shutdown('warm startup fixture complete')
        return

    class Runner(runner_type):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            fixture.runner = self
    try:
        with patch.object(rospy, 'init_node'), patch.object(nav,'TAKEOFF_TIMEOUT', 5. if scenario == 'done_low' else nav.TAKEOFF_TIMEOUT), \
                patch.object(nav,'LAND_TIMEOUT', 8. if scenario=='land_no_disarm' else 40.):
            try:
                code = nav.run_full(args, Runner, 'test_indoor6')
            except nav.Stopped:
                assert scenario == 'startup_armed'
                code = 1
        if scenario == 'auto_hold':
            assert inspection and code == 0 and fixture.armed and fixture.landed == 2
            assert fixture.takeoff_count == 1 and fixture.land_count == 0
            np.testing.assert_allclose(fixture.goals, [[.8,.2,.7]])
            assert json.loads((out/'mission_state.json').read_text())['end'] == 'SUCCEEDED'
        elif scenario == 'startup_armed':
            assert code and not fixture.requests and not fixture.goals
        else:
            flight = json.loads(next(out.glob('flight_*.json')).read_text())
            record = json.loads(next(out.glob('goal_*.json')).read_text())
            success = scenario in ('normal','miss_qr','empty_final','clearance_recovers')
            assert (code == 0) == success, (scenario,flight.get('reason'))
            assert (flight['end']=='SUCCEEDED') == success
            assert (record['end']=='SUCCEEDED') == success
            if success:
                assert not fixture.armed and fixture.landed == 1 and fixture.land_count == 1
                assert record['legs'][-1]['direction'] == RETURN_NAME
                np.testing.assert_allclose(fixture.goals[-1], [.36,.20,.48], atol=1e-6)
                assert len(record['batches']) == 4
                if scenario == 'normal':
                    assert [b['order'] for b in record['batches']] == [[5], [2,1], [4,0], [3]]
                assert record['tag_counts']['read_ok'] == (5 if scenario in ('miss_qr', 'empty_final') else 6), record['tag_counts']
                if scenario == 'clearance_recovers':
                    assert flight['takeoff_stable']['odom_position'][2] == .425
                    assert flight['takeoff_min_odom_z'] == .4
                    assert flight['takeoff_stable_ns'] / 1e9 - fixture.final_clear_wall >= nav.STABLE_SECONDS
                    assert fixture.takeoff_count == 1
                else:
                    assert flight['takeoff_stable']['fc_position'][2] == .5
                assert flight['return_error_m'] < .01
                assert flight['return_max_z_error_m'] < .01
                assert flight['takeoff_stable_ns'] > 0
                if scenario != 'empty_final':
                    np.testing.assert_allclose(fixture.goals[-2],route['points'][-1]['xyz'])
                    assert abs(fixture.goals[-3][2]-.48)>.05
            elif scenario == 'land_no_disarm':
                assert fixture.land_count == 1 and fixture.armed
            else:
                assert fixture.land_count == 0
                expected_reason = {'countdown_disarm':'上锁', 'done_low':'超时',
                                   'clearance_low':'z=0.386 m，最低 0.400 m',
                                   'return_blocked':'occupied footprint', 'return_takeover':'模式',
                                   'return_z_excursion':'等高返航偏离'}
                assert expected_reason[scenario] in flight['reason'], flight['reason']
                if scenario in ('countdown_disarm','done_low','clearance_low'):
                    assert not fixture.goals
                if scenario == 'clearance_low':
                    assert '超时' in flight['reason'] and fixture.takeoff_count == 1
                    assert flight['phases'][-1]['phase'] == 'TAKEOFF'
                    request = next(call for call in flight['services'] if call['service'] == 'takeoff')
                    assert nav.TAKEOFF_TIMEOUT - .1 <= flight['end_wall'] - request['wall'] < nav.TAKEOFF_TIMEOUT + 1.
                if scenario == 'countdown_disarm':
                    assert not fixture.requests and not fixture.takeoff_count
        print('PASS indoor6',scenario, flush=True)
    finally:
        camera.close()
        fixture.close()
        fixture.server.action_server.stop()
        rospy.signal_shutdown('indoor6 isolated scenario complete')


if __name__ == '__main__':
    if '--out' in sys.argv:
        from integration_indoor4 import main as preview
        from nav_indoor6 import Indoor6Survey

        def verify(runner, camera, messages, route, fixture, out):
            assert runner.record['tag_counts']['read_ok'] == 6
            assert [b['order'] for b in runner.record['batches']] == [[5],[2,1],[4,0],[3]]
            np.testing.assert_allclose(runner.record['legs'][-1]['goal_map'], [.45,.34,.48], atol=.001)
            assert runner.record['legs'][-1]['direction'] == RETURN_NAME
            print('PASS preview return to marked XY at the last observation height', flush=True)

        preview(CornerCamera, Indoor6Survey, load_indoor5_route, verify, 11449)
    else:
        main(Path(sys.argv[1]), sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 11448)
