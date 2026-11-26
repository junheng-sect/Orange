#!/usr/bin/env python3
"""Exercise actual runtime callbacks with ROS transports/timers mocked; no FCU I/O."""
import copy
import json
import math
import threading
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget, State, EstimatorStatus
from std_msgs.msg import String
from traj_utils.msg import Bspline, PathStart, PathTrajectory
from drone_navigation.core import rz
from drone_navigation.gateway import Gateway, STOPPED
from drone_navigation.msg import NavigationCommand
from drone_navigation.executor import Executor
from drone_navigation.mission import Mission
from drone_navigation.msg import ExecuteRouteGoal
from drone_navigation.ros_common import PATH_GENERATION, path_of, put, xyz


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

    def running_gateway(self, r, t):
        g = self.gateway(); g.alignment.r = r; g.alignment.t = t
        g.on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        # READY needs 1 s of pre-stream after prepare; refresh the inputs that age meanwhile.
        self.now += 1.; g.px = self.odom(); g.px_at = g.input_at = g.owners_at = self.now
        g.on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        g.on_goal(self.goal(seq=1))
        return g

    def command(self, p, v=(0., 0., 0.), acc=(0., 0., 0.), terminal=False):
        c = NavigationCommand(); c.header.stamp = rospy.Time.now(); c.header.frame_id = 'drone_odom'
        put(c.position, p); put(c.velocity, v); put(c.acceleration, acc)
        c.generation = 1; c.terminal = terminal
        return c

    def test_trajectory_stream_sends_acceleration_feedforward(self):
        r, t = rz(math.pi/2), np.array([2., 3., .3])
        g = self.running_gateway(r, t)
        g.on_cmd(self.command((.5, 0., 1.), (.2, 0., 0.), (-.3, .1, 0.))); g.tick()
        m = g.output.publish.call_args[0][0]
        self.assertEqual(m.type_mask, PositionTarget.IGNORE_YAW_RATE)
        np.testing.assert_allclose(xyz(m.position), r@[.5, 0., 1.]+t)
        np.testing.assert_allclose(xyz(m.acceleration_or_force), r@[-.3, .1, 0.])
        # Hold / brake stream keeps the position + velocity mask.
        g.on_cancel(None); g.tick()
        self.assertEqual(g.output.publish.call_args[0][0].type_mask, 2496)
        g = self.running_gateway(r, t); g.on_cmd(self.command((.5, 0., 1.), acc=(.6, 0., 0.)))
        self.assertEqual(g.fault, 'invalid navigation command')

    def test_normal_arrival_holds_final_trajectory_point(self):
        r, t, end = rz(math.pi/2), np.array([2., 3., .3]), np.array([1., 0., 1.])
        for order in ('cancel first', 'arrived first'):
            with self.subTest(order=order):
                g = self.running_gateway(r, t)
                g.on_cmd(self.command(end, terminal=True)); g.tick()
                # The settle check fires while the vehicle is still 5 cm past the endpoint.
                g.px = self.odom(r@end+t+[.05, 0., 0.]); g.px_at = self.now
                events = (g.on_cancel, g.on_arrived) if order == 'cancel first' else (g.on_arrived, g.on_cancel)
                for event in events: event(None)
                g.tick(); m = g.output.publish.call_args[0][0]
                self.assertEqual(g.fault, STOPPED); self.assertEqual(m.type_mask, 2496)
                np.testing.assert_allclose(xyz(m.position), r@end+t)
                np.testing.assert_allclose(xyz(m.velocity), 0.)
                self.assertAlmostEqual(m.yaw, math.pi/2)

    def test_other_stops_still_brake_in_place(self):
        end = np.array([1., 0., 1.]); here = end+[.05, 0., 0.]
        def fault(g):
            g.on_executor(String(json.dumps({'state': 'FAULT'}))); g.on_arrived(None); g.on_cancel(None)
        def stale(g):
            g.on_arrived(None); self.now += .4; g.px = self.odom(here); g.px_at = self.now; g.on_cancel(None)
        cases = {'cancel without arrival': (True, lambda g: g.on_cancel(None)),
                 'arrival after a fault': (True, fault),
                 'stale arrival': (True, stale),
                 'arrival mid-trajectory': (False, lambda g: (g.on_arrived(None), g.on_cancel(None)))}
        for name, (terminal, stop) in cases.items():
            with self.subTest(case=name):
                g = self.running_gateway(np.eye(3), np.zeros(3))
                g.on_cmd(self.command(end, (0. if terminal else .2, 0., 0.), terminal=terminal)); g.tick()
                g.px = self.odom(here); g.px_at = self.now
                stop(g); g.tick()
                np.testing.assert_allclose(xyz(g.output.publish.call_args[0][0].position), here)

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

    def test_stationary_spline_before_goal_is_buffered_then_map_validated(self):
        e=self.build(Executor, 'drone_navigation.executor')
        e.on_odom(self.odom());e.on_plan(String('4'))
        e.on_spline(self.spline(1))
        self.assertIsNone(e.spline)
        self.assertEqual(e.last_id,-1)
        e.on_goal(self.goal())
        self.assertIsNotNone(e.pending_spline)
        self.assertIsNone(e.spline)
        e.check.return_value=SimpleNamespace(valid=True,reason='clear')
        e.validate();e.tick()
        self.assertEqual(e.spline.id,1)

    def test_early_spline_expiry_old_goal_and_cancel_do_not_resume(self):
        for case in ('expired','old_goal','cancelled'):
            e=self.build(Executor, 'drone_navigation.executor')
            e.on_spline(self.spline(1))
            if case=='expired':self.now+=.31
            elif case=='old_goal':self.now+=.2
            else:e.on_cancel(None)
            goal=self.goal()
            if case=='old_goal':goal.header.stamp=rospy.Time.from_sec(self.now)
            e.on_goal(goal)
            self.assertIsNone(e.pending_spline,case)

    def test_mission_rejects_short_translation_before_publishing_goal(self):
        for target,anchor,speed in (((.1,0,1),None,0),((.1,0,1),(0,0,1),0),
                                    ((0,0,1),(0,0,1),.11),((.18,0,1),(.18,0,1),0)):
            m=self.short_mission(anchor,speed)
            goal=PoseStamped();goal.header.frame_id='drone_map';put(goal.pose.position,target);goal.pose.orientation.w=1.
            m.execute(ExecuteRouteGoal(map_id='synthetic',waypoints=[goal]))
            m.goal_pub.publish.assert_not_called()
            self.assertIn('below 0.2m',m.abort.call_args[0][0])

    def short_mission(self, anchor, speed=0.):
        m=Mission.__new__(Mission)
        m.task_lock=threading.Lock()
        m.c=dict(map_id='synthetic',bounds_min=[-5,-5,.4],bounds_max=[5,5,2.5])
        m.ready=True;m.gw_at=self.now;m.map_ok=lambda:True
        m.odom=self.odom();m.odom.twist.twist.linear.x=speed;m.odom_at=self.now
        m.last_completed_goal=None if anchor is None else np.array(anchor)
        tr=TransformStamped();tr.header.stamp=rospy.Time.now();tr.transform.rotation.w=1.
        m.buf=Mock();m.buf.lookup_transform.return_value=tr
        m.check=Mock(return_value=SimpleNamespace(valid=True))
        m.server=Mock();m.server.is_preempt_requested.return_value=True
        m.abort=Mock();m.goal_pub=Mock()
        return m

    def test_mission_allows_turn_at_completed_identical_anchor(self):
        m=self.short_mission((0,0,1))
        goal=PoseStamped();goal.header.frame_id='drone_map';put(goal.pose.position,(0,0,1))
        goal.pose.orientation.z=math.sin(math.pi/4);goal.pose.orientation.w=math.cos(math.pi/4)
        m.execute(ExecuteRouteGoal(map_id='synthetic',waypoints=[goal]))
        m.goal_pub.publish.assert_called_once()
        self.assertEqual(m.abort.call_args[0][0],'cancelled by client')

    def test_command_carries_spline_acceleration_and_zero_at_end(self):
        e = self.build(Executor, 'drone_navigation.executor')
        e.on_odom(self.odom()); e.on_plan(String('4')); e.on_goal(self.goal())
        e.check.return_value = SimpleNamespace(valid=True, reason='clear')
        b = self.spline(1); b.pos_pts = [Point(x=x, z=1.) for x in (0., 0., .1, .3)]
        e.on_spline(b); e.validate()
        self.now += .5; self.refresh(e); e.validate(); e.tick()
        acc = e.spline.state(.5)[2]; self.assertGreater(np.linalg.norm(acc), .05)
        np.testing.assert_allclose(xyz(e.pub.publish.call_args[0][0].acceleration), acc)
        self.now += .6; self.refresh(e); e.validate(); e.tick()
        cmd = e.pub.publish.call_args[0][0]
        self.assertTrue(cmd.terminal)
        np.testing.assert_allclose(xyz(cmd.velocity), 0.); np.testing.assert_allclose(xyz(cmd.acceleration), 0.)

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

    def test_cancel_before_path_motion_rejects_delayed_path_and_command(self):
        g = self.running_gateway(np.eye(3), np.zeros(3))
        self.now += .02
        path = path_of([[.3, 0, 1], [.6, 0, 1]])
        path.header.seq = 1
        g.on_path(path)
        self.assertEqual(g.expected_gen, PATH_GENERATION+1)
        g.on_cancel(None)
        g.on_path(path)
        command = self.command((.02, 0, 1), (.1, 0, 0))
        command.generation = PATH_GENERATION+1
        g.on_cmd(command)
        self.assertFalse(g.running)
        self.assertIsNone(g.cmd)

    def test_path_candidate_waits_for_alignment_and_validates_session(self):
        e = self.build(Executor, 'drone_navigation.executor')
        e.on_odom(self.odom())
        e.on_plan(String('4'))
        path = path_of([[.3, 0, 1], [.6, 0, 1]])
        path.header.seq = 2
        e.on_path(path)
        spline = self.spline(8)
        spline.pos_pts = [Point(x=x, z=1.) for x in (0., .1, .2, .3)]
        ident = PathStart(path_seq=1, path_stamp=path.header.stamp, trajectory_id=8)
        message = PathTrajectory(identity=ident, pending_start=True, spline=spline)
        e.on_path_trajectory(message)
        self.assertIsNone(e.path_candidate)
        message.identity.path_seq = 2
        e.on_path_trajectory(message)
        e.check.return_value = SimpleNamespace(valid=False, reason='unknown footprint')
        e.validate()
        e.tick()
        e.pub.publish.assert_not_called()
        e.check.return_value = SimpleNamespace(valid=True, reason='clear')
        for _ in range(12):
            self.now += .1
            self.refresh(e)
            e.validate()
            e.tick()
        self.assertEqual(e.gen, PATH_GENERATION + 2)
        self.assertIsNone(e.spline)
        self.assertEqual(e.start_pub.publish.call_args[0][0].trajectory_id, 8)
        self.assertFalse(e.pub.publish.call_args[0][0].terminal)
        np.testing.assert_allclose(xyz(e.pub.publish.call_args[0][0].velocity), 0.)
        e.on_cancel(None)
        message.pending_start = False
        e.on_path_trajectory(message)
        e.on_path(path)
        self.assertFalse(e.active)
        self.assertIsNone(e.pending_spline)

    def test_tangent_wrap_and_low_speed_heading_hold(self):
        e = self.build(Executor, 'drone_navigation.executor')
        e.on_odom(self.odom())
        e.yaw = math.radians(179.92)
        e.heading_at = self.now
        target = math.radians(-179.92)
        self.assertTrue(e.path_heading(.2*np.array([math.cos(target), math.sin(target), 0]), self.now+.02)[0])
        self.assertAlmostEqual(e.yaw, target)
        self.assertTrue(e.path_heading(np.array([0., 0., .2]), self.now+.04)[0])
        self.assertAlmostEqual(e.yaw, target)

    def test_sustained_measured_sideways_motion_cancels_path(self):
        e = self.build(Executor, 'drone_navigation.executor')
        o = self.odom(yaw=math.pi/2)
        o.twist.twist.linear.y = .2
        e.on_odom(o)
        e.active = True
        v = np.zeros(3)
        ok, angle = e.path_heading(v, self.now)
        self.assertTrue(ok)
        self.assertAlmostEqual(angle, math.pi/2)
        self.now += .51
        self.assertFalse(e.path_heading(v, self.now)[0])
        self.assertFalse(e.active)
        e.cancel.publish.assert_called_once()

    def test_new_candidate_keeps_discontinuity_guard(self):
        e = self.executor(); e.on_spline(self.spline(2, 1.)); e.validate(); e.tick()
        self.assertFalse(e.active); self.assertIn('discontinuous', e.fault)


if __name__ == '__main__':
    unittest.main()
