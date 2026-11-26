#!/usr/bin/env python3
import unittest

from barcode_rc_trigger.policy import RcPulseTrigger

LOW, MID, HIGH = 1000, 1650, 1900
DT = 0.02  # 50 Hz RC samples


class Feeder:
    def __init__(self, trigger):
        self.trigger = trigger
        self.now = 0.0

    def hold(self, pwm, seconds):
        """Feed `pwm` for `seconds` at 50 Hz; return how many samples fired."""
        fired = 0
        for _ in range(int(round(seconds / DT))):
            fired += self.trigger.update(self.now, pwm)
            self.now += DT
        return fired


class RcPulseTriggerTest(unittest.TestCase):
    def setUp(self):
        self.feed = Feeder(RcPulseTrigger(1600, 1700, 0.05))

    def test_initial_high_never_fires(self):
        self.assertEqual(self.feed.hold(HIGH, 2.0), 0)
        self.assertEqual(self.feed.hold(LOW, 0.2), 0)
        self.assertEqual(self.feed.hold(HIGH, 0.2), 1)

    def test_one_fire_per_pulse_even_when_held(self):
        self.feed.hold(LOW, 0.2)
        self.assertEqual(self.feed.hold(HIGH, 3.0), 1)

    def test_rearm_after_low(self):
        self.feed.hold(LOW, 0.2)
        fired = 0
        for _ in range(5):
            fired += self.feed.hold(HIGH, 0.2)
            fired += self.feed.hold(LOW, 0.2)
        self.assertEqual(fired, 5)

    def test_bounce_shorter_than_hold_ignored(self):
        self.feed.hold(LOW, 0.2)
        self.assertEqual(self.feed.hold(HIGH, 0.04), 0)
        self.assertEqual(self.feed.hold(LOW, 0.1), 0)
        self.assertEqual(self.feed.hold(HIGH, 0.1), 1)

    def test_fires_after_hold_elapsed(self):
        trigger = RcPulseTrigger(1600, 1700, 0.05)
        trigger.update(0.0, LOW)
        self.assertFalse(trigger.update(1.00, HIGH))
        self.assertFalse(trigger.update(1.04, HIGH))
        self.assertTrue(trigger.update(1.05, HIGH))

    def test_mid_band_neither_arms_nor_fires(self):
        self.feed.hold(MID, 0.2)
        self.assertEqual(self.feed.hold(HIGH, 0.2), 0)
        # Armed from LOW, passing through MID keeps it armed but restarts the hold timer.
        self.feed.hold(LOW, 0.2)
        self.feed.hold(MID, 0.2)
        self.assertEqual(self.feed.hold(HIGH, 0.2), 1)
        # After firing, MID does not re-arm.
        self.feed.hold(MID, 0.2)
        self.assertEqual(self.feed.hold(HIGH, 0.2), 0)

    def test_invalid_sample_disarms(self):
        self.feed.hold(LOW, 0.2)
        self.assertTrue(self.feed.trigger.armed)
        self.feed.hold(None, 0.1)
        self.assertFalse(self.feed.trigger.armed)
        self.assertEqual(self.feed.hold(HIGH, 0.2), 0)

    def test_invalid_during_hold_cancels(self):
        self.feed.hold(LOW, 0.2)
        self.feed.hold(HIGH, 0.02)
        self.feed.hold(None, 0.02)
        self.assertEqual(self.feed.hold(HIGH, 0.2), 0)

    def test_reset_disarms(self):
        self.feed.hold(LOW, 0.2)
        self.feed.trigger.reset()
        self.assertEqual(self.feed.hold(HIGH, 0.2), 0)

    def test_zero_hold_fires_on_first_high_sample(self):
        trigger = RcPulseTrigger(1600, 1700, 0.0)
        trigger.update(0.0, LOW)
        self.assertTrue(trigger.update(0.02, HIGH))

    def test_threshold_edges(self):
        trigger = RcPulseTrigger(1600, 1700, 0.0)
        trigger.update(0.0, 1600)            # not below arm_below
        self.assertFalse(trigger.armed)
        trigger.update(0.0, 1599)
        self.assertFalse(trigger.update(0.1, 1700))  # not above trigger_above
        self.assertTrue(trigger.update(0.2, 1701))

    def test_rejects_bad_config(self):
        with self.assertRaises(ValueError):
            RcPulseTrigger(1700, 1700, 0.05)
        with self.assertRaises(ValueError):
            RcPulseTrigger(1600, 1700, -0.1)
        with self.assertRaises(ValueError):
            RcPulseTrigger(1600, 1700, float('nan'))


if __name__ == '__main__':
    unittest.main()
