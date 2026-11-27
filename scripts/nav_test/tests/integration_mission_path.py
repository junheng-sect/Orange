"""A continuous guide point and terminal yaw through the real output-disabled navigation stack."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np
import rospkg
import rospy
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import ExecutePathActionGoal
from drone_navigation.core import SETPOINT_INPUTS
from integration_indoor4 import Fixture, wait_for
from drone_inspection.config import load
from drone_inspection.runtime import main


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    out = parser.parse_args().out
    assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11453'
    rospy.init_node('inspection_path_fixture')
    cfg = load(Path(rospkg.RosPack().get_path('drone_inspection')) / 'missions/indoor_shelf.yaml')
    cfg['navigation'] = rospy.get_param('/drone_nav/config')
    cfg['flight'].update(return_home=False, auto_land=False)
    cfg['waypoints'] = [dict(name='通行', xyz=[.65,0.,1.], action='pass'),
                        dict(name='停靠', xyz=[1.35,0.,1.], action='hover', hold_s=.2, yaw_deg=35.)]
    fixture = Fixture(out)
    fixture.a = 0.
    messages = []
    subscriber = rospy.Subscriber('/drone_nav/execute_path/goal', ExecutePathActionGoal, messages.append)
    try:
        from nav_forward import path_of
        check = rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory)
        rospy.wait_for_service('/drone_nav/check_trajectory', 15)
        wait_for(lambda: check(path_of([np.array(p['xyz']) for p in cfg['waypoints']])).valid, 20, 'grid not ready')
        env = dict(INSPECTION_CONFIG=json.dumps(cfg), INSPECTION_LIVE=str(out), INSPECTION_ARCHIVE=str(out),
                   INSPECTION_WORKSPACE=str(Path(__file__).resolve().parents[3]), INSPECTION_PREVIEW='1')
        with patch.dict(os.environ, env), patch.object(rospy,'init_node'), patch.object(sys,'argv',['runtime','run']):
            assert main() == 0, (out/'mission_state.json').read_text()
        assert len(messages) == 1 and len(messages[0].goal.points) == 2
        np.testing.assert_allclose(fixture.p, cfg['waypoints'][-1]['xyz'], atol=.05)
        assert abs(math.degrees(fixture.a)-35.) < 5.
        assert not (SETPOINT_INPUTS & {topic for topic, _ in rospy.get_published_topics()})
        print('PASS real EGO continuous pass, endpoint yaw, no camera or FCU output', flush=True)
    finally:
        fixture.close()
        rospy.signal_shutdown('fixture complete')
