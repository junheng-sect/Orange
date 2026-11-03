#!/usr/bin/env python3
"""Exercise actual runtime callbacks with ROS transports/timers mocked; no FCU I/O."""
import copy
import math
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, EstimatorStatus
from std_msgs.msg import String
from traj_utils.msg import Bspline
from drone_navigation.gateway import Gateway
from drone_navigation.executor import Executor
from drone_navigation.ros_common import put, xyz


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.wall = patch('time.monotonic', side_effect=lambda: self.now)
        self.clock = patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.now))
        self.wall.start(); self.clock.start()
        self.addCleanup(self.wall.stop); self.addCleanup(self.clock.stop)

    def build(self, cls, module):
        with ExitStack() as stack:
            stack.enter_context(patch(module+'.config', return_value={'max_vel': .5, 'max_acc': .5}))
            stack.enter_context(patch(module+'.loop'))
            for name in ('Publisher', 'Subscriber', 'Service', 'ServiceProxy'):
                stack.enter_context(patch.object(rospy, name, side_effect=lambda *a, **k: Mock()))
            stack.enter_context(patch.object(rospy, 'get_param', side_effect=lambda key, default=None: default))
            if cls is Gateway:
                stack.enter_context(patch(module+'.message_filters.Subscriber'))
                stack.enter_context(patch(module+'.message_filters.ApproximateTimeSynchronizer'))
            return cls()

    def odom(self, p=(0., 0., 1.), yaw=0.):
        o = Odometry(); o.header.stamp = rospy.Time.now()
        put(o.pose.pose.position, p)
        o.pose.pose.orientation.z = math.sin(yaw/2)
        o.pose.pose.orientation.w = math.cos(yaw/2)
        return o

    def gateway(self):
        g = self.build(Gateway, 'drone_navigation.gateway')
        # Exercise real-output gating but retain the mocked output transport.
        g.real = True; g.output = Mock(); g.px = self.odom(); g.px_at = self.now
        g.alignment.r = np.eye(3); g.alignment.t = np.zeros(3)
        g.input_ok = g.owners_ok = True; g.input_at = g.owners_at = self.now
        est = EstimatorStatus(); est.header.stamp = rospy.Time.now()
        for name in ('attitude_status_flag', 'velocity_horiz_status_flag',
                     'velocity_vert_status_flag', 'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag'):
            setattr(est, name, True)
        g.on_est(est)
        g.on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.assertTrue(g.prepare(None).success)
        return g

    def test_prepare_move_one_metre_then_offboard_freezes_switch_pose(self):
        for switch_p in ((1., 0., 1.), (0., 0., 2.)):
            with self.subTest(switch_p=switch_p):
                g = self.gateway(); initial = xyz(g.px.pose.pose.position)
                # Pre-stream follows measured position and yaw on every tick.
                midpoint = (initial+np.array(switch_p))*.5
                g.px = self.odom(midpoint, .2); g.tick()
                before = g.output.publish.call_args[0][0]
                np.testing.assert_allclose(xyz(before.position), midpoint)
                np.testing.assert_allclose(xyz(before.velocity), 0.)
                self.assertAlmostEqual(before.yaw, .2)
                # Move again between ticks: the OFFBOARD edge must capture this pose.
                g.px = self.odom(switch_p, .4); g.px.twist.twist.linear.x = .3
                g.on_state(State(connected=True, armed=True, mode='OFFBOARD')); g.tick()
                first = g.output.publish.call_args[0][0]
                np.testing.assert_allclose(xyz(first.position), switch_p)
                np.testing.assert_allclose(xyz(first.velocity), 0.)
                self.assertAlmostEqual(first.yaw, .4)
                g.px = self.odom(np.array(switch_p)+[.1, 0, 0], .5); g.tick()
                frozen = g.output.publish.call_args[0][0]
                np.testing.assert_allclose(xyz(frozen.position), switch_p)
                self.assertAlmostEqual(frozen.yaw, .4)

    def goal(self, seq=1, yaw=0.):
        g = PoseStamped(); g.header.stamp = rospy.Time.from_sec(self.now-.1)
        g.header.seq = seq; g.header.frame_id = 'drone_odom'
        g.pose.orientation.z = math.sin(yaw/2); g.pose.orientation.w = math.cos(yaw/2)
        return g

    def spline(self, seq, x=0.):
        b = Bspline(); b.order = 3; b.knots = list(range(8))
        b.pos_pts = [Point(x=x, z=1.) for _ in range(4)]
        b.start_time = rospy.Time.now(); b.traj_id = seq
        return b

    def executor(self):
        e = self.build(Executor, 'drone_navigation.executor')
        e.on_odom(self.odom()); e.on_plan(String('4')); e.on_goal(self.goal())
        e.check.return_value = SimpleNamespace(valid=True, reason='clear')
        e.on_spline(self.spline(1)); e.validate(); e.tick()
        self.assertTrue(e.active); self.assertEqual(e.spline.id, 1)
        return e

    def refresh(self, e):
        e.on_odom(self.odom()); e.on_plan(String('4'))

    def test_replan_keeps_approved_stream_until_candidate_passes(self):
        e = self.executor(); e.on_spline(self.spline(2, .02))
        e.pub.publish.reset_mock(); e.tick()
        np.testing.assert_allclose(xyz(e.pub.publish.call_args[0][0].position), [0, 0, 1])
        calls = []
        def check(path):
            calls.append(path)
            if len(calls) == 2:
                # While the new candidate's RPC is outstanding, ticks use the old one.
                e.tick(); self.assertEqual(e.spline.id, 1)
                np.testing.assert_allclose(xyz(e.pub.publish.call_args[0][0].position), [0, 0, 1])
            return SimpleNamespace(valid=True, reason='clear')
        e.check.side_effect = check; e.validate(); e.tick()
        self.assertEqual(len(calls), 2); self.assertEqual(e.spline.id, 2)
        np.testing.assert_allclose(xyz(e.pub.publish.call_args[0][0].position), [.02, 0, 1])

    def test_rejected_candidate_preserves_still_clear_current_spline(self):
        e = self.executor(); e.on_spline(self.spline(2, .02))
        e.check.side_effect = [SimpleNamespace(valid=True, reason='clear'),
                               SimpleNamespace(valid=False, reason='unknown footprint')]
        e.validate(); e.pub.publish.reset_mock(); e.tick()
        self.assertTrue(e.active); self.assertEqual(e.spline.id, 1)
        e.pub.publish.assert_called_once()

    def test_waypoint_handoff_stream_survives_over_gateway_stale_window(self):
        e = self.executor(); self.now += 1.1; self.refresh(e); e.validate(); e.tick()
        e.on_goal(self.goal(seq=2, yaw=1.))
        for _ in range(5):
            self.now += .1; self.refresh(e); e.validate(); e.tick()
            self.assertTrue(e.active)
            cmd = e.pub.publish.call_args[0][0]
            self.assertEqual(cmd.generation, 2); self.assertAlmostEqual(cmd.yaw, 0.)
            np.testing.assert_allclose(xyz(cmd.position), [0, 0, 1])
            np.testing.assert_allclose(xyz(cmd.velocity), 0.)
        e.on_spline(self.spline(2, .02)); e.validate(); e.tick()
        self.assertIs(e.spline_goal, e.goal); self.assertEqual(e.spline.id, 2)

    def test_wait_target_after_missed_fault_stops_unfinished_trajectory(self):
        e = self.executor(); e.on_plan(String('FAULT')); e.on_plan(String('1'))
        e.pub.publish.reset_mock(); e.tick()
        self.assertFalse(e.active); self.assertIn('WAIT_TARGET', e.fault)
        e.pub.publish.assert_not_called(); e.cancel.publish.assert_called_once()

    def test_wait_target_at_normal_endpoint_and_initial_planning_is_allowed(self):
        e = self.executor(); self.now += .995; self.refresh(e); e.validate()
        e.on_plan(String('1')); e.tick(); self.assertTrue(e.active)
        self.now += .01; self.refresh(e); e.validate(); e.on_plan(String('1')); e.tick()
        self.assertTrue(e.pub.publish.call_args[0][0].terminal)
        e.on_cancel(None); e.on_goal(self.goal(seq=2)); e.on_plan(String('1')); e.tick()
        self.assertTrue(e.active); self.assertIsNone(e.spline)

    def test_pending_candidate_does_not_bypass_expired_map_validation(self):
        e = self.executor(); e.on_spline(self.spline(2))
        self.now += .31; self.refresh(e); e.pub.publish.reset_mock(); e.tick()
        e.pub.publish.assert_not_called()

    def test_current_collision_cancels_both_current_and_pending(self):
        e = self.executor(); e.on_spline(self.spline(2))
        e.check.return_value = SimpleNamespace(valid=False, reason='occupied')
        e.validate(); self.assertFalse(e.active)
        self.assertIsNone(e.spline); self.assertIsNone(e.pending_spline)

    def test_cancel_during_validation_cannot_activate_old_candidate(self):
        e = self.executor(); e.on_spline(self.spline(2))
        def check(path):
            e.on_cancel(None)
            return SimpleNamespace(valid=True, reason='clear')
        e.check.side_effect = check; e.validate(); e.pub.publish.reset_mock(); e.tick()
        self.assertFalse(e.active); self.assertIsNone(e.spline)
        e.pub.publish.assert_not_called()

    def test_new_candidate_keeps_discontinuity_guard(self):
        e = self.executor(); e.on_spline(self.spline(2, 1.)); e.validate(); e.tick()
        self.assertFalse(e.active); self.assertIn('discontinuous', e.fault)


if __name__ == '__main__':
    unittest.main()
