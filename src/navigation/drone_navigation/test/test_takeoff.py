#!/usr/bin/env python3
"""Gateway ground takeoff (test6): climb profile and the gateway callbacks with ROS transports mocked; no FCU I/O."""
import math
import unittest
from contextlib import ExitStack
from unittest.mock import Mock, patch
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from mavros_msgs.msg import State, EstimatorStatus, ExtendedState, PositionTarget
from drone_navigation.core import climb_profile
from drone_navigation.gateway import Gateway, GROUND_DROP
from drone_navigation.ros_common import put, xyz

CONFIG = {'max_vel': 1., 'max_acc': 1., 'max_vel_z': .3, 'max_acc_z': .3, 'takeoff_height': 1.}


class ClimbProfileTests(unittest.TestCase):
    def test_trapezoid_reaches_height_at_rest_within_limits(self):
        for h, v, a in ((1., .3, .3), (.3, .3, .3), (.05, .3, .3), (2.5, 1., 1.)):
            T = climb_profile(0., h, v, a)[3]
            ts = np.linspace(0., T, 2001)
            s, ds, dds = np.array([climb_profile(t, h, v, a)[:3] for t in ts]).T
            self.assertAlmostEqual(s[0], 0.); self.assertAlmostEqual(ds[0], 0.)
            self.assertAlmostEqual(s[-1], h); self.assertAlmostEqual(ds[-1], 0.); self.assertEqual(dds[-1], 0.)
            self.assertTrue(np.all(np.diff(s) >= -1e-12) and ds.max() <= v + 1e-9 and np.abs(dds).max() <= a + 1e-9)
            np.testing.assert_allclose(np.gradient(s, ts)[5:-5], ds[5:-5], atol=2e-3)
            self.assertEqual(climb_profile(T + 5., h, v, a)[:3], (h, 0., 0.))
        self.assertAlmostEqual(climb_profile(0., 1., .3, .3)[3], 1. / .3 + 1.)  # 1 m: 1 s up, 2.33 s cruise, 1 s down

    def test_rejects_degenerate_profile(self):
        for args in ((0., .3, .3), (1., 0., .3), (1., .3, -1.)):
            with self.assertRaises(ValueError):
                climb_profile(0., *args)


class GatewayTakeoffTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        for p in (patch('time.monotonic', side_effect=lambda: self.now),
                  patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.now))):
            p.start(); self.addCleanup(p.stop)

    def odom(self, p=(1., 2., .05), yaw=.3):
        o = Odometry(); o.header.stamp = rospy.Time.now(); put(o.pose.pose.position, p)
        o.pose.pose.orientation.z = math.sin(yaw / 2); o.pose.pose.orientation.w = math.cos(yaw / 2)
        return o

    def landed(self, g, state=ExtendedState.LANDED_STATE_ON_GROUND):
        g.on_ext(ExtendedState(landed_state=state))

    def gateway(self, config=CONFIG):
        with ExitStack() as stack:
            stack.enter_context(patch('drone_navigation.gateway.config', return_value=dict(config)))
            stack.enter_context(patch('drone_navigation.gateway.loop'))
            for name in ('Publisher', 'Subscriber', 'Service', 'ServiceProxy'):
                stack.enter_context(patch.object(rospy, name, side_effect=lambda *a, **k: Mock()))
            stack.enter_context(patch.object(rospy, 'get_param', side_effect=lambda key, default=None: default))
            stack.enter_context(patch('drone_navigation.gateway.message_filters.Subscriber'))
            stack.enter_context(patch('drone_navigation.gateway.message_filters.ApproximateTimeSynchronizer'))
            g = Gateway()
        g.real = True; g.output = Mock(); g.alignment.r = np.eye(3); g.alignment.t = np.zeros(3)
        self.refresh(g, at_rest=True)
        g.on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.landed(g)
        return g

    def refresh(self, g, p=(1., 2., .05), at_rest=False, state=True):
        """New paired PX4 sample, input heartbeat and estimator status; at_rest raises const_pos as PX4 does while still.
        /mavros/state follows the 1 Hz FCU heartbeat unless state=False."""
        g.px = self.odom(p); g.px_at = g.input_at = g.owners_at = self.now; g.input_ok = g.owners_ok = True
        if state and g.state is not None and self.now - g.state_at >= 1.:
            g.on_state(State(connected=True, armed=g.state.armed, mode=g.state.mode))
        est = EstimatorStatus(); est.header.stamp = rospy.Time.now()
        for name in ('attitude_status_flag', 'velocity_horiz_status_flag', 'velocity_vert_status_flag',
                     'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag'):
            setattr(est, name, True)
        est.const_pos_mode_status_flag = at_rest
        g.on_est(est)

    def sent(self, g):
        return g.output.publish.call_args[0][0]

    def offboard(self, g, at_rest=True):
        """1 s of ground pre-stream, then the OFFBOARD edge; returns the READY gateway."""
        self.now += 1.; self.refresh(g, at_rest=at_rest); self.landed(g)
        g.on_state(State(connected=True, armed=True, mode='OFFBOARD')); g.tick()
        self.assertTrue(g.ready, g.fault)
        return g

    def test_const_pos_on_ground_only_accepted_when_preparing_takeoff(self):
        g = self.gateway()
        self.assertTrue(g.prepare(None).success); g.tick()
        self.assertEqual(g.fault, 'PX4 local estimate unavailable')  # ordinary prepare keeps rejecting a resting EKF
        g = self.gateway()
        self.assertTrue(g.prepare_takeoff(None).success); g.tick()
        self.assertEqual(g.fault, ''); self.assertEqual(g.takeoff_state, 'GROUND')
        # The same flag in the air (not landed) is not excused.
        self.landed(g, ExtendedState.LANDED_STATE_IN_AIR); g.tick()
        self.assertEqual(g.fault, 'PX4 local estimate unavailable')

    def test_prepare_takeoff_requires_landed_armed_outside_offboard(self):
        g = self.gateway(); self.landed(g, ExtendedState.LANDED_STATE_IN_AIR)
        self.assertFalse(g.prepare_takeoff(None).success)
        g = self.gateway(); self.now += 1.1; self.refresh(g, at_rest=True)  # extended_state older than 1 s
        self.assertFalse(g.prepare_takeoff(None).success)
        g = self.gateway(); g.on_state(State(connected=True, armed=False, mode='POSCTL'))
        self.assertFalse(g.prepare_takeoff(None).success)
        g = self.gateway(); g.alignment.r = None
        self.assertFalse(g.prepare_takeoff(None).success)

    def test_ground_hold_streams_below_vehicle_until_climb(self):
        g = self.gateway(); self.assertTrue(g.prepare_takeoff(None).success); g.tick()
        np.testing.assert_allclose(xyz(self.sent(g).position), [1., 2., .05 - GROUND_DROP])
        self.offboard(g)
        m = self.sent(g)
        np.testing.assert_allclose(xyz(m.position), [1., 2., .05 - GROUND_DROP]); np.testing.assert_allclose(xyz(m.velocity), 0.)
        self.assertTrue(m.type_mask & PositionTarget.IGNORE_AFZ)

    def test_takeoff_refused_before_offboard_or_twice(self):
        g = self.gateway(); self.assertTrue(g.prepare_takeoff(None).success); g.tick()
        self.assertFalse(g.takeoff(None).success)  # not READY: still in position mode
        self.offboard(g)
        self.assertTrue(g.takeoff(None).success)
        self.assertFalse(g.takeoff(None).success)

    def climb(self, g, airborne_at=1.5):
        """Fly the whole profile at 50 Hz, the vehicle tracking the setpoint; PX4 reports in air after airborne_at s."""
        T = climb_profile(0., g.takeoff_h, g.takeoff_v, g.takeoff_a)[3]; t0 = self.now; out = []
        while self.now - t0 < T + .5:
            self.now += .02
            z = .05 + climb_profile(self.now - t0 - .3, g.takeoff_h, g.takeoff_v, g.takeoff_a)[0]
            self.refresh(g, p=(1., 2., z), at_rest=self.now - t0 < .5)
            self.landed(g, ExtendedState.LANDED_STATE_ON_GROUND if self.now - t0 < airborne_at else ExtendedState.LANDED_STATE_IN_AIR)
            g.tick(); out.append((self.now - t0, self.sent(g), g.takeoff_state))
        return out

    def test_climb_follows_profile_then_holds_and_finishes_when_airborne(self):
        g = self.gateway(); self.assertTrue(g.prepare_takeoff(None).success); g.tick(); self.offboard(g)
        self.assertTrue(g.takeoff(None).success)
        out = self.climb(g)
        T = climb_profile(0., 1., .3, .3)[3]
        for t, m, st in out:
            if t < T - .03:
                s, ds, dds, _ = climb_profile(t, 1., .3, .3)
                self.assertAlmostEqual(m.position.z, .05 + s, places=6); self.assertAlmostEqual(m.velocity.z, ds, places=6)
                self.assertAlmostEqual(m.acceleration_or_force.z, dds, places=6)
                self.assertEqual((m.position.x, m.position.y), (1., 2.)); self.assertAlmostEqual(m.yaw, .3)
                self.assertEqual(m.type_mask, PositionTarget.IGNORE_YAW_RATE)  # P + V + A
        last = out[-1][1]
        np.testing.assert_allclose(xyz(last.position), [1., 2., 1.05]); np.testing.assert_allclose(xyz(last.velocity), 0.)
        self.assertEqual(g.takeoff_state, 'DONE'); self.assertFalse(g.takeoff_mode); self.assertEqual(g.fault, '')
        # Once airborne the resting exemption is gone with takeoff_mode.
        self.refresh(g, p=(1., 2., 1.05), at_rest=True); g.tick()
        self.assertEqual(g.fault, 'PX4 local estimate unavailable')

    def test_never_lifting_off_faults_and_returns_ground_hold(self):
        g = self.gateway(); self.assertTrue(g.prepare_takeoff(None).success); g.tick(); self.offboard(g)
        self.assertTrue(g.takeoff(None).success)
        self.climb(g, airborne_at=1e9)
        self.assertEqual(g.takeoff_state, 'HOLD')
        for _ in range(170):  # 3.4 s more, still landed at ground height
            self.now += .02; self.refresh(g, at_rest=True); self.landed(g); g.tick()
        self.assertEqual(g.fault, 'takeoff not detected by PX4'); self.assertEqual(g.takeoff_state, 'STOPPED')
        self.assertAlmostEqual(self.sent(g).position.z, .05 - GROUND_DROP)

    def test_offboard_exit_mid_climb_brakes_from_last_climb_setpoint(self):
        g = self.gateway(); self.assertTrue(g.prepare_takeoff(None).success); g.tick(); self.offboard(g)
        self.assertTrue(g.takeoff(None).success)
        for _ in range(100):  # 2 s into the climb
            self.now += .02; self.refresh(g, p=(1., 2., .3)); self.landed(g, ExtendedState.LANDED_STATE_IN_AIR); g.tick()
        g.on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.assertIsNone(g.climb); self.assertEqual(g.takeoff_state, 'STOPPED'); self.assertFalse(g.prepared)

    def test_stale_fcu_state_stops_climb_without_resuming(self):
        g = self.gateway(); self.assertTrue(g.prepare_takeoff(None).success); g.tick(); self.offboard(g)
        self.assertTrue(g.takeoff(None).success)
        for _ in range(50):
            self.now += .02; self.refresh(g, p=(1., 2., .2)); self.landed(g, ExtendedState.LANDED_STATE_IN_AIR); g.tick()
        z = self.sent(g).position.z
        g.state_at = self.now - 1.  # last heartbeat 1 s ago, then /mavros/state stays silent
        self.now += 1.1; self.refresh(g, p=(1., 2., .2), state=False); self.landed(g, ExtendedState.LANDED_STATE_IN_AIR); g.tick()
        self.assertIsNone(g.climb); self.assertEqual(g.takeoff_state, 'STOPPED')
        g.on_state(State(connected=True, armed=True, mode='OFFBOARD')); g.tick()
        self.assertLess(abs(self.sent(g).position.z - z), .02)  # brakes from the last climb point, no jump ahead

    def test_vertical_permit_stops_climb_and_landing(self):
        for landing in (False, True):
            with self.subTest(landing=landing):
                g = self.gateway(dict(CONFIG, vertical_test_guard=True))
                self.assertFalse(g.prepare_takeoff(None).success)
                g.vertical_permit(Bool(True))
                self.assertTrue(g.prepare_takeoff(None).success)
                g.tick()
                self.now += 1.
                self.refresh(g, at_rest=True)
                self.landed(g)
                g.vertical_permit(Bool(True))
                g.on_state(State(connected=True, armed=True, mode='OFFBOARD'))
                g.tick()
                self.assertTrue(g.takeoff(None).success)
                self.landed(g, ExtendedState.LANDED_STATE_IN_AIR)
                self.refresh(g, p=(1., 2., .5))
                if landing:
                    g.climb = None
                    g.capture_hold()
                    self.assertTrue(g.land_start(None).success)
                    g.vertical_permit(Bool(False))
                else:
                    self.now += .51
                    self.refresh(g, p=(1., 2., .5))
                g.tick()
                self.assertTrue(g.vertical_aborted)
                self.assertIsNone(g.climb)
                self.assertIsNone(g.land)
                self.assertFalse(g.land_velocity_only)
                self.assertFalse(g.ready)
                self.assertAlmostEqual(self.sent(g).velocity.z, 0.)
                g.vertical_permit(Bool(True))
                self.assertFalse(g.prepare(None).success)
                self.assertFalse(g.takeoff(None).success)
                self.assertFalse(g.land_start(None).success)

    def test_invalid_takeoff_config_rejected_at_start(self):
        for bad in ({'takeoff_height': .1}, {'takeoff_height': 3.}, {'max_vel_z': 0.}, {'max_acc_z': 2.}):
            with self.assertRaises(ValueError):
                self.gateway(dict(CONFIG, **bad))


if __name__ == '__main__':
    unittest.main()
