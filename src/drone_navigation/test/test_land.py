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
from drone_navigation.gateway import (Gateway, LAND_PRESS, LAND_CONFIRM_GRACE, LAND_PRESS_V,
                                      LAND_PRESS_TIMEOUT, LAND_CANDIDATE_HOLD, LAND_DISARM_TIMEOUT)
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
        self.workers=[]
        def thread(target,args=(),**kwargs):
            return Mock(start=lambda:self.workers.append((target,args)))
        worker_patch=patch('drone_navigation.gateway.threading.Thread',side_effect=thread)
        worker_patch.start();self.addCleanup(worker_patch.stop)
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
        g.real = True; g.output = Mock(); g.disarm_service=Mock(return_value=Mock(success=True)); g.alignment.r = np.eye(3); g.alignment.t = np.zeros(3)
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

    def test_prepare_invalidates_cached_ready_before_next_land_request(self):
        g=self.airborne();self.assertTrue(g.ready)
        self.assertTrue(g.prepare(None).success)
        self.assertFalse(g.ready)
        self.assertFalse(g.land_start(None).success)
        self.now+=1.1;self.refresh(g);g.tick()
        self.assertTrue(g.ready,g.fault)
        self.assertTrue(g.land_start(None).success)

    def test_descent_continues_past_old_endpoint_until_real_touchdown(self):
        g = self.airborne()
        reply = g.land_start(None)
        self.assertTrue(reply.success, reply.message)
        self.assertEqual(g.land_state, 'DESCENDING')
        self.assertFalse(g.land_start(None).success)  # no second descent
        z1, t0, seen = g.ground_z - LAND_PRESS, self.now, []
        floor, contact_at = g.ground_z-.02, None
        while g.land_state == 'DESCENDING':
            self.now += .02
            # Return surface is 2 cm lower; actual tracking lags the target by 2 cm.
            # The old ground-2cm endpoint would stop 2 cm above this surface.
            actual=min(1.05,max(floor,g.hold_p[2]+.02))
            if actual <= floor+1e-8:contact_at=contact_at or self.now
            confirmed=contact_at is not None and self.now-contact_at >= .6
            self.refresh(g, p=(1., 2., actual), airborne=not confirmed)
            g.tick()
            seen.append((self.now - t0, self.sent(g)))
            self.assertLess(self.now - t0, 20., 'descent never finished')
        for t, m in seen[:-1]:
            s, ds = descend_profile(t, g.land_v, g.takeoff_a)
            self.assertAlmostEqual(m.position.z, max(z1,1.05-s), places=6)
            self.assertAlmostEqual(m.velocity.z, -ds, places=6)
            self.assertEqual((m.position.x, m.position.y), (1., 2.))
            expected=PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE
            if m.position.z<=z1+1e-9:expected|=PositionTarget.IGNORE_PZ
            self.assertEqual(m.type_mask,expected)
        self.assertEqual(g.land_state, 'TOUCHED'); self.assertIsNone(g.land)
        last = self.sent(g)
        self.assertTrue(g.on_ground())
        self.assertGreaterEqual(last.position.z,z1)
        self.assertLess(last.position.z,g.ground_z-.02)
        np.testing.assert_allclose(xyz(last.velocity), 0.)
        frozen=xyz(last.position).copy()
        self.now+=.02;self.refresh(g,p=(1.,2.,floor),airborne=False);g.tick()
        np.testing.assert_allclose(xyz(self.sent(g).position),frozen)
        self.assertFalse(g.land_start(None).success)
        # 1 m at 0.1 m/s, not PX4's AUTO.LAND speed.
        self.assertGreater(seen[-1][0], 9.)
        self.assertLess(seen[-1][0], 13.)

    def advance(self,g,seconds,p=(1.,2.,.07),airborne=True,velocity=(0.,0.,0.)):
        until=self.now+seconds
        while self.now<until-1e-8:
            self.now=min(until,self.now+.02);self.refresh(g,p=p,airborne=airborne)
            put(g.px.twist.twist.linear,velocity);g.tick()

    def run_workers(self):
        while self.workers:
            target,args=self.workers.pop(0);target(*args)

    def test_low_slow_stall_uses_bounded_velocity_press_without_false_touchdown(self):
        g=self.airborne();self.assertTrue(g.land_start(None).success)
        self.advance(g,LAND_CANDIDATE_HOLD+.1)
        self.assertEqual(g.land_state,'PRESSING')
        m=self.sent(g)
        self.assertTrue(m.type_mask & PositionTarget.IGNORE_PZ)
        self.assertFalse(m.type_mask & (PositionTarget.IGNORE_PX|PositionTarget.IGNORE_PY|PositionTarget.IGNORE_VZ))
        self.assertAlmostEqual(m.velocity.z,-LAND_PRESS_V)
        self.assertEqual((m.position.x,m.position.y,m.yaw),(1.,2.,.3))
        self.assertEqual(g.land_disarm_count,0)
        self.advance(g,LAND_PRESS_TIMEOUT+.1)
        self.assertEqual(g.land_state,'STOPPED')
        self.assertIn('landing not detected',g.fault)
        self.assertAlmostEqual(self.sent(g).position.z,.07)
        np.testing.assert_allclose(xyz(self.sent(g).velocity),0.)
        self.assertFalse(self.sent(g).type_mask & PositionTarget.IGNORE_PZ)
        self.assertFalse(g.land_start(None).success)

    def test_position_endpoint_keeps_velocity_and_cannot_claim_contact_high_up(self):
        g=self.airborne();self.assertTrue(g.land_start(None).success)
        while not g.land_limit_reached:
            self.advance(g,.02,p=(1.,2.,1.))
            self.assertGreaterEqual(self.sent(g).position.z,g.land_limit_z)
        self.assertEqual(g.land_state,'DESCENDING')
        self.assertAlmostEqual(self.sent(g).velocity.z,-g.land_v)
        self.assertTrue(self.sent(g).type_mask & PositionTarget.IGNORE_PZ)
        self.advance(g,LAND_CONFIRM_GRACE+.1,p=(1.,2.,1.))
        self.assertEqual(g.land_state,'STOPPED')
        self.assertIn('before near-ground',g.fault)
        self.assertEqual(g.land_disarm_count,0)

    def test_candidate_timer_resets_and_never_requests_disarm_in_air(self):
        g=self.airborne();self.assertTrue(g.land_start(None).success)
        self.advance(g,2.)
        self.advance(g,.1,p=(1.,2.,.4))
        self.advance(g,2.)
        self.assertEqual(g.land_state,'DESCENDING')
        self.assertEqual(g.land_disarm_count,0)
        self.advance(g,1.1)
        self.assertEqual(g.land_state,'PRESSING')
        self.assertEqual(g.land_disarm_count,0)

    def test_velocity_press_stops_at_measured_ground_minus_ten_centimetres(self):
        g=self.airborne();self.assertTrue(g.land_start(None).success)
        self.advance(g,3.1)
        self.advance(g,.02,p=(1.,2.,g.land_limit_z))
        self.assertEqual(g.land_state,'STOPPED')
        self.assertIn('measured 0.10 m',g.fault)
        self.assertEqual(self.sent(g).velocity.z,0.)
        self.assertEqual(g.land_disarm_count,0)

    def test_pressing_aborts_on_motion_or_mode_takeover_and_never_disarms(self):
        for fault in ('motion','mode'):
            g=self.airborne();self.assertTrue(g.land_start(None).success)
            self.advance(g,3.1)
            if fault=='motion':self.advance(g,.02,velocity=(0.,0.,.5))
            else:g.on_state(State(connected=True,armed=True,mode='POSCTL'))
            self.assertEqual(g.land_state,'STOPPED')
            self.assertEqual(g.land_disarm_count,0)

    def test_ordinary_disarm_is_ground_gated_rate_limited_and_awaits_actual_state(self):
        g=self.airborne();self.assertTrue(g.land_start(None).success)
        self.advance(g,3.1)
        self.advance(g,.02,airborne=False)
        self.assertEqual(g.land_state,'TOUCHED')
        self.assertEqual(g.land_disarm_count,1)
        self.assertEqual(self.sent(g).velocity.z,0.)
        self.run_workers();g.disarm_service.assert_called_once_with(value=False)
        self.advance(g,.5,airborne=False)
        self.assertEqual(g.land_disarm_count,1)
        self.advance(g,.6,airborne=False);self.run_workers()
        self.assertEqual(g.land_disarm_count,2)
        self.assertTrue(g.state.armed) # service acknowledgement is not a state transition
        g.on_state(State(connected=True,armed=False,mode='OFFBOARD'))
        self.assertEqual(g.land_state,'DONE')
        self.assertFalse(g.prepared)
        self.assertFalse(g.fault)

    def test_px4_auto_disarm_before_next_tick_requires_fresh_ground(self):
        for stale in (False,True):
            g=self.airborne();self.assertTrue(g.land_start(None).success)
            self.refresh(g,airborne=False)
            if stale:g.ext_at=self.now-1.1
            g.on_state(State(connected=True,armed=False,mode="OFFBOARD"))
            self.assertEqual(g.land_state,"STOPPED" if stale else "DONE")
            self.assertFalse(g.prepared)
            self.assertEqual(g.land_disarm_count,0)
            self.assertEqual(bool(g.fault),stale)

    def test_ground_without_disarm_times_out_with_distinct_reason(self):
        g=self.airborne();self.assertTrue(g.land_start(None).success)
        self.advance(g,.02,airborne=False);self.run_workers()
        self.advance(g,LAND_DISARM_TIMEOUT+.1,airborne=False)
        self.assertEqual(g.land_state,'STOPPED')
        self.assertIn('landed but not disarmed',g.fault)
        count=g.disarm_service.call_count
        self.run_workers()
        self.assertEqual(g.disarm_service.call_count,count) # queued work cancelled on stop

    def test_pending_disarm_does_not_block_ticks_and_takeover_cancels_worker(self):
        g=self.airborne();self.assertTrue(g.land_start(None).success)
        self.advance(g,.02,airborne=False)
        n=g.output.publish.call_count
        self.advance(g,1.1,airborne=False)
        self.assertGreater(g.output.publish.call_count,n+40)
        self.assertEqual(g.land_disarm_count,1) # no overlapping RPC while the first is pending
        g.on_state(State(connected=True,armed=True,mode='POSCTL'))
        self.run_workers();g.disarm_service.assert_not_called()

    def test_delayed_touchdown_during_limit_grace_is_accepted(self):
        g=self.airborne();self.assertTrue(g.land_start(None).success)
        while not g.land_limit_reached:
            self.now+=.02;self.refresh(g,p=(1.,2.,.05),airborne=True);g.land_candidate_at=None;g.tick()
        self.now+=1.;self.refresh(g,p=(1.,2.,.05),airborne=False);g.tick()
        self.assertEqual(g.land_state,'TOUCHED')
        self.assertEqual(g.fault,'')
        self.assertEqual(self.sent(g).velocity.z,0.)

    def test_stale_landed_feedback_never_marks_touched(self):
        g=self.airborne();self.assertTrue(g.land_start(None).success)
        self.now+=.02;self.refresh(g,airborne=False)
        old,at=g.ext,g.ext_at
        self.now+=1.1;self.refresh(g,airborne=True)
        g.ext,g.ext_at=old,at;g.tick()
        self.assertEqual(g.land_state,'STOPPED')
        self.assertIn('landed status stale',g.fault)

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
        self.now+=.02;self.refresh(g,p=(1.,2.,.05),airborne=False);g.tick()
        self.assertEqual(g.land_state,'TOUCHED')
        c.header.stamp=rospy.Time.now();g.on_cmd(c)
        self.assertFalse(g.running) # landing retains ownership while waiting for auto-disarm

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
