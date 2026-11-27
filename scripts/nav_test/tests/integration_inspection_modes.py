"""Exercise the real mission worker over ROS against a loopback-only mock vehicle."""
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch
from nav_msgs.msg import Odometry
from drone_navigation.executor import Executor

import actionlib
import numpy as np
import rospkg
import rospy

from integration_indoor6 import IndoorFixture, Camera
from drone_inspection.config import load
from drone_inspection.runtime import main
from drone_navigation.core import rz, SETPOINT_INPUTS
from drone_navigation.msg import ExecutePathAction, ExecutePathResult


class ModeFixture(IndoorFixture):
    def execute(self, goal):
        if self.scenario == 'planner_loss':
            started = time.monotonic()
            time.sleep(.35)
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            def fail(reason):
                with self.lock:
                    self.exec_state, self.exec_reason = 'FAULT', reason
            anchor = object()
            Executor.tick(SimpleNamespace(lock=self.lock, active=True, odom=odom, odom_at=time.monotonic(),
                odom_age=.5, plan='EXEC_TRAJ', path_mode=False, spline=anchor, spline_goal=anchor,
                goal=anchor, goal_at=started, plan_at=started, fail=fail))
            time.sleep(.2)
            self.server.set_aborted()
            return
        super().execute(goal)

    def path(self, goal):
        self.paths.append([[p.x,p.y,p.z] for p in goal.points])
        with self.lock:
            self.position = np.array(self.paths[-1][-1])
            self.gw, self.reason, self.exec_state = 'FAULT', 'task stopped; prepare before next route', 'IDLE'
        self.path_server.set_succeeded(ExecutePathResult(success=True, reason='mock path settled'))


class SecondFace(Camera):
    target_topic = '/drone_inspection/target_tag_pose'

    def configure(self):
        super().configure()
        turn = rz(math.pi / 2)
        anchor = np.array(self.route[-1]['xyz'])
        self.coords[3] = anchor + turn @ (self.coords[3] - anchor)
        self.tag_axes[3] = turn @ self.tag_axes[3]


def run(out, case):
    assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11451'
    rospy.init_node('inspection_mode_fixture')
    share = Path(rospkg.RosPack().get_path('drone_inspection'))
    cfg = load(share / 'missions/indoor_shelf.yaml')
    cfg['navigation'] = rospy.get_param('/drone_nav/config')
    cfg['navigation']['map_id'] = 'synthetic'
    cfg['navigation']['allow_short_goals'] = True
    rospy.set_param('/drone_nav/config', cfg['navigation'])
    preview = case != 'manual_hover'
    cfg['flight'].update(auto_takeoff=False, auto_land=False, return_home=False)
    cfg['flight']['takeoff_delay_s'] = 1.
    fixture = ModeFixture()
    camera = None
    try:
        with fixture.lock:
            fixture.reset(case)
            fixture.map_id = 'synthetic'
            fixture.real_output = not preview
            fixture.position = np.array([.36,.20,.50])
        fixture.paths = []
        fixture.path_server = actionlib.SimpleActionServer('/drone_nav/execute_path', ExecutePathAction,
                                      execute_cb=fixture.path, auto_start=False)
        fixture.path_server.start()
        if case == 'multi_yaw':
            cfg['waypoints'][-1]['yaw_deg'] += 90.
            camera = SecondFace(fixture, cfg['navigation'], cfg['waypoints'])
        else:
            cfg['waypoints'] = [dict(name='门口', xyz=[.8,.20,.5], action='pass'),
                                dict(name='出口', xyz=[1.3,.20,.5], action='pass')]
            if case != 'plain_pass':
                cfg['waypoints'][-1].update(action='hover', yaw_deg=35., hold_s=.1)
            if case in ('manual_hover','planner_loss'):
                cfg['waypoints'] = cfg['waypoints'][-1:]
        if case == 'manual_hover':
            def pilot():
                while fixture.ready_timer is None and not fixture.closed:
                    time.sleep(.02)
                time.sleep(.15)
                with fixture.lock:
                    fixture.armed, fixture.mode, fixture.landed = True, 'OFFBOARD', 2
                    fixture.ready_timer, fixture.gw = None, 'READY'
            threading.Thread(target=pilot, daemon=True).start()
        out.mkdir(parents=True)
        env = dict(INSPECTION_LIVE=str(out), INSPECTION_ARCHIVE=str(out),
                   INSPECTION_WORKSPACE=str(share.parents[2]), INSPECTION_CONFIG=json.dumps(cfg),
                   INSPECTION_PREVIEW=str(int(preview)))
        with patch.dict(os.environ, env), patch.object(sys,'argv',['runtime','run']), patch.object(rospy,'init_node'):
            code = main()
        state = json.loads((out/'mission_state.json').read_text())
        success = case != 'planner_loss'
        assert (code == 0) == success and (state['end'] == 'SUCCEEDED') == success, state
        assert fixture.takeoff_count == fixture.land_count == 0 and not fixture.requests
        assert not (SETPOINT_INPUTS & {topic for topic, _ in rospy.get_published_topics()})
        if case in ('path_preview','plain_pass'):
            assert fixture.paths == [[[.8,.20,.5],[1.3,.20,.5]]]
            assert len(fixture.goals) == (1 if case == 'path_preview' else 0)
        if case in ('manual_hover','path_preview'):
            assert abs(math.degrees(fixture.yaw)-35) < .01
        if case == 'multi_yaw':
            scan = json.loads((out/'scan_results.json').read_text())
            assert [b['yaw_deg'] for b in scan['batches']] == [-89.5]*3 + [.5]
            assert set(camera.ids) == set(range(6)) and len(camera.ids) == 6, camera.ids
            assert all(item['status']=='success' for item in scan['scans']), scan['scans']
        else:
            assert not any('ov13855' in topic for topic, _ in rospy.get_published_topics())
        if not success:
            assert 'planner heartbeat lost during execution' in state['reason'], state
        print('PASS inspection mode',case, flush=True)
    finally:
        if camera:
            camera.close()
        fixture.close()
        rospy.signal_shutdown('fixture complete')


if __name__ == '__main__':
    run(Path(sys.argv[1]), sys.argv[2])
