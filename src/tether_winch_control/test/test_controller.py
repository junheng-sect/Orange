#!/usr/bin/env python3
from dataclasses import replace
import math
import unittest

from tether_winch_control.controller import Config, Controller, attachment_position
from tether_winch_control.transport import make_transport


class Harness:
    def __init__(self, config=None, radius=2.0):
        self.controller = Controller(config or Config(low_duty=0.2, high_duty=0.4))
        self.now = 1.0
        self.radius = radius

    def sample(self, speed=0.0, dt=0.02, **overrides):
        self.now += dt
        self.radius += speed * dt
        values = dict(stamp=1000 + self.now, position=(self.radius, 0, 0),
                      orientation=(0, 0, 0, 1), frame_id='map', child_frame_id='base_link',
                      now=self.now, source_now=1000 + self.now)
        values.update(overrides)
        return self.controller.observe(**values)

    def samples(self, count=30, **kwargs):
        return [self.sample(**kwargs) for _ in range(count)]

    def enable(self):
        return self.controller.enable(self.now, 1000 + self.now)

    def ready(self):
        self.samples()
        assert self.enable()[0]
        return self


class ConfigTests(unittest.TestCase):
    def test_uncalibrated_default_cannot_enable(self):
        h = Harness(Config())
        h.samples()
        self.assertFalse(h.enable()[0])
        self.assertIsNone(h.controller.status().target_duty)

    def test_invalid_configuration(self):
        for parameters in [dict(low_duty=0.2), dict(low_duty=-0.1, high_duty=0.3),
                           dict(low_duty=0.4, high_duty=0.4), dict(low_duty=0.5, high_duty=0.2),
                           dict(low_duty=0.2, high_duty=1.01),
                           dict(low_duty=math.nan, high_duty=0.5),
                           dict(low_duty=True, high_duty=0.5),
                           dict(window_sec=0), dict(window_sec=0.5), dict(timeout_sec=math.inf),
                           dict(min_samples=2), dict(min_samples=3.5), dict(max_samples=2),
                           dict(max_samples=3, min_samples=4),
                           dict(enter_speed_mps=0), dict(exit_speed_mps=-0.1),
                           dict(enter_hold_sec=-1), dict(anchor=(0, math.nan, 0)),
                           dict(attachment_offset=(0, 0)), dict(frame_id=''),
                           dict(child_frame_id=True), dict(near_anchor_m=-1)]:
            with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                Config(**parameters)

    def test_geometry_rotates_body_offset_and_normalizes_quaternion(self):
        q = (0, 0, math.sqrt(2), math.sqrt(2))  # non-unit +90 degrees yaw
        p = attachment_position((2, 3, 4), q, (1, 0, 0))
        for actual, expected in zip(p, (2, 4, 4)):
            self.assertAlmostEqual(actual, expected)
        h = Harness(Config(anchor=(2, 4, 3), attachment_offset=(1, 0, 0)))
        state = h.sample(position=(2, 3, 4), orientation=q)
        self.assertAlmostEqual(state.distance_m, 1.0)


class MotionTests(unittest.TestCase):
    def test_starts_disabled_and_needs_fresh_samples(self):
        h = Harness()
        self.assertEqual(h.controller.status().mode, 'DISABLED')
        self.assertFalse(h.enable()[0])
        h.samples(2)
        self.assertFalse(h.enable()[0])
        h.sample()
        self.assertTrue(h.enable()[0])
        self.assertEqual(h.controller.status().mode, 'LOW')

    def test_stationary_and_outward_are_low(self):
        h = Harness().ready()
        for state in h.samples(60) + h.samples(60, speed=0.3):
            self.assertEqual(state.mode, 'LOW')
            self.assertEqual(state.target_duty, 0.2)

    def test_approach_high_then_stop_and_reverse_low(self):
        h = Harness().ready()
        states = h.samples(50, speed=-0.3)
        self.assertEqual(states[-1].mode, 'HIGH')
        self.assertEqual(states[-1].target_duty, 0.4)
        self.assertAlmostEqual(states[-1].radial_velocity_mps, -0.3, places=8)
        stopped = h.samples(20)
        first_low = next(index for index, state in enumerate(stopped) if state.mode == 'LOW')
        self.assertLessEqual((first_low + 1) * 0.02, 0.24)
        h.samples(40, speed=-0.3)
        self.assertEqual(h.controller.status().mode, 'HIGH')
        self.assertEqual(h.samples(20, speed=0.3)[-1].mode, 'LOW')

    def test_equal_radius_movement_never_high(self):
        h = Harness().ready()
        for i in range(100):
            angle = i * 0.01
            state = h.sample(position=(2 * math.cos(angle), 2 * math.sin(angle), 0))
            self.assertEqual(state.mode, 'LOW')
            self.assertAlmostEqual(state.radial_velocity_mps, 0, places=8)

    def test_three_dimensional_approach(self):
        h = Harness()
        for _ in range(20):
            h.sample(position=(0, 0, 2))
        self.assertTrue(h.enable()[0])
        for i in range(40):
            state = h.sample(position=(0, 0, 2 - 0.006 * (i + 1)))
        self.assertEqual(state.mode, 'HIGH')

    def test_small_stationary_noise_does_not_switch(self):
        h = Harness().ready()
        for i in range(200):
            state = h.sample(position=(2 + 0.001 * math.sin(i * 0.7), 0, 0))
            self.assertEqual(state.mode, 'LOW')

    def test_hysteresis_and_hold(self):
        h = Harness().ready()
        states = h.samples(5, speed=-0.1)
        self.assertTrue(all(s.mode == 'LOW' for s in states))
        h.samples(40, speed=-0.1)
        self.assertEqual(h.controller.status().mode, 'HIGH')
        # A speed between thresholds preserves HIGH, but does not enter it from LOW.
        self.assertEqual(h.samples(40, speed=-0.035)[-1].mode, 'HIGH')
        h.samples(30)
        self.assertEqual(h.samples(40, speed=-0.035)[-1].mode, 'LOW')

    def test_near_anchor_inhibits_high(self):
        h = Harness(radius=0.09).ready()
        self.assertTrue(all(s.mode == 'LOW' for s in h.samples(30, speed=-0.1)))

    def test_repeated_enable_is_idempotent_and_disable_has_no_command(self):
        h = Harness().ready()
        h.samples(40, speed=-0.2)
        self.assertTrue(h.enable()[0])
        self.assertEqual(h.controller.status().mode, 'HIGH')
        state = h.controller.disable()
        self.assertEqual(state.mode, 'DISABLED')
        self.assertIsNone(state.target_duty)
        self.assertTrue(all(s.target_duty is None for s in h.samples(20, speed=-0.2)))

    def test_epoch_timestamp_regression(self):
        h = Harness().ready()
        # Separate session to avoid intentionally tripping a large source-clock gap.
        c = Controller(h.controller.config)
        for i in range(30):
            stamp = 1800000000.0 + i * 0.02
            state = c.observe(stamp, (2 - 0.1 * i * 0.02, 0, 0),
                              (0, 0, 0, 1), 'map', 'base_link', 1 + i * 0.02, stamp)
        self.assertAlmostEqual(state.radial_velocity_mps, -0.1, places=5)


class FaultTests(unittest.TestCase):
    def test_invalid_localization_latches_fault(self):
        cases = [dict(stamp=0), dict(stamp=math.nan), dict(stamp=1000),
                 dict(stamp=10000), dict(position=(math.nan, 0, 0)),
                 dict(position=(math.inf, 0, 0)), dict(position=(1e308, 1e308, 1e308)),
                 dict(orientation=(0, 0, 0, 0)), dict(orientation=(0, math.nan, 0, 1)),
                 dict(frame_id='odom'), dict(child_frame_id='body'), dict(position=(3, 0, 0))]
        for bad in cases:
            with self.subTest(bad=bad):
                h = Harness().ready()
                h.samples(30, speed=-0.2)
                state = h.sample(**bad)
                self.assertEqual(state.mode, 'FAULT')
                self.assertIsNone(state.target_duty)
                self.assertFalse(state.input_valid)
                recovered = h.samples(30)[-1]
                self.assertTrue(recovered.input_valid)
                self.assertEqual(recovered.mode, 'FAULT')
                self.assertIsNone(recovered.target_duty)
                self.assertTrue(h.enable()[0])
                self.assertEqual(h.controller.status().mode, 'LOW')

    def test_duplicate_and_out_of_order_samples(self):
        for delta in (0, -0.01):
            h = Harness().ready()
            old = h.now + 1000
            self.assertEqual(h.sample(stamp=old + delta).mode, 'FAULT')

    def test_timeout_uses_wall_clock_even_with_frozen_source_clock(self):
        h = Harness().ready()
        h.samples(30, speed=-0.2)
        state = h.controller.tick(h.now + 0.51, 1000 + h.now)
        self.assertEqual(state.mode, 'FAULT')
        self.assertIsNone(state.target_duty)
        self.assertFalse(h.controller.enable(h.now + 0.52, 1000 + h.now)[0])

    def test_gap_is_detected_even_without_watchdog_tick(self):
        h = Harness().ready()
        self.assertEqual(h.sample(dt=0.6).mode, 'FAULT')

    def test_fresh_arrival_of_stale_data_is_rejected(self):
        h = Harness().ready()
        self.assertEqual(h.sample(stamp=1000 + h.now - 0.6).mode, 'FAULT')

    def test_clock_rewind_recovery_requires_explicit_enable(self):
        h = Harness().ready()
        state = h.controller.tick(h.now + 0.01, 500)
        self.assertEqual(state.mode, 'FAULT')
        for i in range(20):
            source = 500 + 0.02 * (i + 1)
            state = h.controller.observe(source, (2, 0, 0), (0, 0, 0, 1),
                                         'map', 'base_link', h.now + 0.02 * (i + 1), source)
        self.assertTrue(state.input_valid)
        self.assertEqual(state.mode, 'FAULT')
        self.assertTrue(h.controller.enable(h.now + 0.4, 500.4)[0])

    def test_bad_clocks_and_forward_source_jump(self):
        for now, source in [(math.nan, 1002), (0, 1002), (2, 0), (2, math.inf), (2, 5000)]:
            with self.subTest(now=now, source=source):
                h = Harness().ready()
                self.assertEqual(h.controller.tick(now, source).mode, 'FAULT')

    def test_sample_rate_loss_does_not_hold_high(self):
        h = Harness().ready()
        h.samples(30, speed=-0.2)
        self.assertEqual(h.controller.status().mode, 'HIGH')
        # Within timeout, but no longer enough samples within the regression window.
        self.assertEqual(h.sample(dt=0.3).mode, 'FAULT')

    def test_disable_does_not_clear_latched_fault(self):
        h = Harness().ready()
        h.sample(frame_id='wrong')
        self.assertEqual(h.controller.disable().mode, 'FAULT')
        self.assertFalse(h.enable()[0])


class TransportTests(unittest.TestCase):
    def test_real_backends_are_rejected(self):
        for name in ('udp', 'http', 'esp32', '', None):
            with self.assertRaises(ValueError):
                make_transport(name)

    def test_mock_does_not_report_applied_output_and_invalidates_on_fault(self):
        transport = make_transport('mock')
        h = Harness().ready()
        transport.update(h.controller.status(), h.now)
        self.assertEqual(transport.last_command.duty, 0.2)
        self.assertFalse(transport.hardware_feedback_valid)
        h.sample(frame_id='wrong')
        transport.update(h.controller.status(), h.now)
        self.assertIsNone(transport.last_command)
        self.assertEqual(len(transport.commands), 1)

    def test_bounded_mock_history_and_invalid_duty(self):
        transport = make_transport('mock')
        h = Harness().ready()
        for i in range(1200):
            transport.update(h.controller.status(), h.now + i * 0.02)
        self.assertEqual(len(transport.commands), 1000)
        with self.assertRaises(ValueError):
            transport.update(replace(h.controller.status(), target_duty=2.0), h.now)


if __name__ == '__main__':
    unittest.main()
