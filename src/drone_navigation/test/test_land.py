#!/usr/bin/env python3
"""Gateway slow landing: descent profile, land_start()/tick() with ROS transports mocked; no FCU I/O."""
import math
import unittest
from contextlib import ExitStack
from unittest.mock import Mock, patch
import numpy as np
import rospy
from mavros_msgs.msg import EstimatorStatus, ExtendedState, PositionTarget, State
from nav_msgs.msg import Odometry
from drone_navigation.core import descend_profile
from drone_navigation.gateway import Gateway, LAND_PRESS
from drone_navigation.msg import NavigationCommand
from drone_navigation.ros_common import put, xyz

CONFIG = {'max_vel': 1., 'max_acc': 1., 'max_vel_z': .3, 'max_acc_z': .3, 'takeoff_height': 1.}
GROUND, AIR = ExtendedState.LANDED_STATE_ON_GROUND, ExtendedState.LANDED_STATE_IN_AIR


class DescendProfileTests(unittest.TestCase):
    def test_ramps_to_land_speed_then_holds_it(self):
        for v, a in ((.1, .3), (.5, .3), (.05, 1.)):
            t1 = v / a
            self.assertEqual(descend_profile(0., v, a), (0., 0.))
            np.testing.assert_allclose(descend_profile(t1, v, a), (v*v/(2*a), v))
            s, ds = descend_profile(t1 + 10., v, a)
            self.assertAlmostEqual(ds, v)
            self.assertAlmostEqual(s, v*v/(2*a) + v*10.)
            ts = np.linspace(0., 20., 2001)
            out = np.array([descend_profile(t, v, a) for t in ts])
            self.assertTrue(np.all(np.diff(out[:, 0]) >= -1e-12))
            self.assertLessEqual(out[:, 1].max(), v + 1e-9)
            np.testing.assert_allclose(np.gradient(out[:, 0], ts)[5:-5], out[:, 1][5:-5], atol=5e-3)

    def test_rejects_degenerate_profile(self):
        for args in ((0., .3), (.1, 0.), (.1, -1.)):
            with self.assertRaises(ValueError):
                descend_profile(0., *args)


class GatewayLandTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        for p in (patch('time.monotonic', side_effect=lambda: self.now),
                  patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.now))):
            p.start()
            self.addCleanup(p.stop)

    def odom(self, p=(1., 2., 1.05), yaw=.3):
        o = Odometry(); o.header.stamp = rospy.Time.now(); put(o.pose.pose.position, p)
        o.pose.pose.orientation.z = math.sin(yaw / 2); o.pose.pose.orientation.w = math.cos(yaw / 2)
        return o

    def gateway(self, config=CONFIG, params=None):
        params = dict(params or {})
        with ExitStack() as stack:
            stack.enter_context(patch('drone_navigation.gateway.config', return_value=dict(config)))
            stack.enter_context(patch('drone_navigation.gateway.loop'))
            for name in ('Publisher', 'Subscriber', 'Service', 'ServiceProxy'):
                stack.enter_context(patch.object(rospy, name, side_effect=lambda *a, **k: Mock()))
            stack.enter_context(patch.object(rospy, 'get_param',
                                             side_effect=lambda key, default=None: params.get(key, default)))
            stack.enter_context(patch('drone_navigation.gateway.message_filters.Subscriber'))
            stack.enter_context(patch('drone_navigation.gateway.message_filters.ApproximateTimeSynchronizer'))
            g = Gateway()
        g.real = True; g.output = Mock(); g.alignment.r = np.eye(3); g.alignment.t = np.zeros(3)
        g.input_ok = g.owners_ok = True; g.input_at = g.owners_at = self.now
        g.on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.refresh(g, p=(1., 2., .05), airborne=False)
        return g

    def refresh(self, g, p=(1., 2., 1.05), airborne=True, state=True, at_rest=False):
        """New paired PX4 sample, input heartbeat, estimator status and landed state."""
        g.px = self.odom(p); g.px_at = g.input_at = g.owners_at = self.now
        g.input_ok = g.owners_ok = True
        if state and g.state is not None and self.now - g.state_at >= 1.:
            g.on_state(State(connected=True, armed=g.state.armed, mode=g.state.mode))
        est = EstimatorStatus(); est.header.stamp = rospy.Time.now()
        for name in ('attitude_status_flag', 'velocity_horiz_status_flag', 'velocity_vert_status_flag',
                     'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag'):
            setattr(est, name, True)
        est.const_pos_mode_status_flag = at_rest or not airborne
        g.on_est(est)
        g.on_ext(ExtendedState(landed_state=AIR if airborne else GROUND))

    def sent(self, g):
        return g.output.publish.call_args[0][0]

    def airborne(self, z=1.05):
        """On the ground: record the ground reference with prepare_takeoff, then fly to z and re-prepare."""
        g = self.gateway()
        self.assertTrue(g.prepare_takeoff(None).success)
        self.assertAlmostEqual(g.ground_z, .05)
        g.on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.now += 1.
        self.refresh(g, p=(1., 2., z), airborne=True)
        self.assertTrue(g.prepare(None).success)
        self.now += 1.
        self.refresh(g, p=(1., 2., z), airborne=True)
        g.tick()
        self.assertTrue(g.ready, g.fault)
        return g

    def test_land_start_requires_ready_and_a_ground_reference(self):
        g = self.gateway()  # never prepared a takeoff, so the ground reference is unknown
        self.assertTrue(g.prepare(None).success)
        g.on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.now += 1.; self.refresh(g); g.tick()
        self.assertTrue(g.ready, g.fault)
        self.assertIn('ground reference', g.land_start(None).message)
        self.assertEqual(g.land_state, '')
        g = self.airborne()
        g.on_state(State(connected=True, armed=True, mode='POSCTL')); g.tick()
        self.assertFalse(g.land_start(None).success)  # not READY outside OFFBOARD
        g = self.airborne()
        g.hold_p[2] = g.ground_z - LAND_PRESS + .04
        self.assertFalse(g.land_start(None).success)  # already at the ground reference

    def test_descent_follows_profile_and_holds_at_ground(self):
        g = self.airborne()
        reply = g.land_start(None)
        self.assertTrue(reply.success, reply.message)
        self.assertEqual(g.land_state, 'DESCENDING')
        self.assertFalse(g.land_start(None).success)  # no second descent
        z1, t0, seen = g.ground_z - LAND_PRESS, self.now, []
        while g.land_state == 'DESCENDING':
            self.now += .02
            self.refresh(g, p=(1., 2., max(g.hold_p[2], 0.)), airborne=True)
            g.tick()
            seen.append((self.now - t0, self.sent(g)))
            self.assertLess(self.now - t0, 20., 'descent never finished')
        for t, m in seen[:-1]:
            s, ds = descend_profile(t, g.land_v, g.takeoff_a)
            self.assertAlmostEqual(m.position.z, 1.05 - s, places=6)
            self.assertAlmostEqual(m.velocity.z, -ds, places=6)
            self.assertEqual((m.position.x, m.position.y), (1., 2.))
            self.assertEqual(m.type_mask, PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                                            PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)
        self.assertEqual(g.land_state, 'TOUCHED'); self.assertIsNone(g.land)
        last = self.sent(g)
        self.assertAlmostEqual(last.position.z, z1, places=9)
        np.testing.assert_allclose(xyz(last.velocity), 0.)
        # 1 m at 0.1 m/s, not PX4's AUTO.LAND speed.
        self.assertGreater(seen[-1][0], 9.)
        self.assertLess(seen[-1][0], 12.)

    def test_offboard_exit_stops_the_descent_without_resuming(self):
        g = self.airborne()
        self.assertTrue(g.land_start(None).success)
        for _ in range(100):  # 2 s of descent
            self.now += .02; self.refresh(g, p=(1., 2., max(g.hold_p[2], 0.))); g.tick()
        z = self.sent(g).position.z
        g.on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.assertIsNone(g.land); self.assertEqual(g.land_state, 'STOPPED')
        g.on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.now += 1.; self.refresh(g, p=(1., 2., z)); g.tick()
        self.assertLess(abs(self.sent(g).position.z - z), .02)  # holds the last descent point
        self.assertIsNone(g.land)

    def test_navigation_command_is_ignored_during_the_descent(self):
        g = self.airborne()
        self.assertTrue(g.land_start(None).success)
        c = NavigationCommand(); c.header.stamp = rospy.Time.now(); c.header.frame_id = 'drone_odom'
        put(c.position, (2., 2., 1.)); put(c.velocity, (0., 0., 0.)); put(c.acceleration, (0., 0., 0.))
        c.generation = 0; g.goal_stamp = rospy.Time.from_sec(self.now); g.expected_gen = 0
        g.on_cmd(c)
        self.assertFalse(g.running)

    def test_const_pos_in_the_last_centimetres_keeps_the_descent_ready(self):
        g = self.airborne()
        self.assertTrue(g.land_start(None).success)
        self.now += .02; self.refresh(g, p=(1., 2., .95), at_rest=True); g.tick()
        self.assertEqual(g.fault, 'PX4 local estimate unavailable')  # still a metre up: a real fault
        g = self.airborne()
        self.assertTrue(g.land_start(None).success)
        self.now += .02; self.refresh(g, p=(1., 2., .08), at_rest=True); g.tick()
        self.assertEqual(g.fault, '')
        self.assertEqual(g.land_state, 'DESCENDING')
        self.assertTrue(g.ready)

    def test_land_vel_param_is_validated(self):
        for bad in (0., -.1, .6):
            with self.assertRaises(ValueError):
                self.gateway(params={'~land_vel': bad})
        g = self.gateway(params={'~land_vel': .2})
        self.assertAlmostEqual(g.land_v, .2)


if __name__ == '__main__':
    unittest.main()
