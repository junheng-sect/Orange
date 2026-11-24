#!/usr/bin/env python3
"""Deterministic unit tests for the OFFBOARD bag-recording policy.

Covers the open/close decisions and health semantics behind the recorder node:
a mid-OFFBOARD (re)start or link-drop reconnect must never open a half-length
bag, open/write failures are fatal, and a COMPLETE/ABORT session stays healthy
through its tail even while the vehicle holds in OFFBOARD.

Run with:  python3 test/test_recorder_session.py
"""

import importlib.util
import os
import unittest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_PATH = os.path.join(TEST_DIR, os.pardir, 'scripts',
                           'recorder_session.py')

_spec = importlib.util.spec_from_file_location('recorder_session', MODULE_PATH)
recorder_session = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(recorder_session)

RecorderSession = recorder_session.RecorderSession


class RecorderPolicyTests(unittest.TestCase):
    def test_healthy_exit_arms_and_offboard_edge_opens(self):
        session = RecorderSession()
        # Idle POSCTL (disarmed) arms the recorder.
        self.assertIsNone(session.on_state(True, False, False))
        self.assertTrue(session.healthy())
        # Operator enters OFFBOARD: the edge requests a bag.
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        self.assertTrue(session.recording())
        self.assertTrue(session.healthy())

    def test_mid_offboard_start_never_opens_until_a_real_exit(self):
        session = RecorderSession()
        # Recorder respawns while the vehicle is already in OFFBOARD: the first
        # state it ever sees is active, so no bag may be opened.
        self.assertIsNone(session.on_state(True, True, True))
        self.assertTrue(session.offboard_active)
        self.assertFalse(session.healthy())  # in OFFBOARD, no bag, not finished
        self.assertFalse(session.recording())
        # A link drop must not arm it either.
        self.assertIsNone(session.on_state(False, False, False))
        # Reconnect lands straight back in OFFBOARD: still refused.
        self.assertIsNone(session.on_state(True, True, True))
        self.assertFalse(session.healthy())
        # A real exit to a healthy non-OFFBOARD mode arms the next edge.
        self.assertIsNone(session.on_state(True, False, True))
        self.assertTrue(session.healthy())
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        self.assertTrue(session.recording())
        self.assertTrue(session.healthy())

    def test_disarm_in_offboard_closes_and_rearm_opens_fresh_bag(self):
        session = RecorderSession()
        session.on_state(True, False, False)  # healthy exit prime: armed
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        # Disarm while the mode string still reads OFFBOARD closes the bag.
        self.assertEqual(session.on_state(True, True, False), 'close')
        session.bag_closed()
        self.assertFalse(session.recording())
        self.assertTrue(session.healthy())
        # Re-arm while still in OFFBOARD mode: like the controller, a disarm is
        # a real interruption, so a fresh rising edge opens a new bag.
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        self.assertTrue(session.recording())
        self.assertTrue(session.healthy())

    def test_open_failure_is_fatal_and_unhealthy(self):
        session = RecorderSession()
        session.on_state(True, False, False)
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.fail()  # open raised
        self.assertFalse(session.healthy())
        self.assertFalse(session.recording())
        # Recovery comes from a process restart (the node signal_shutdowns and
        # roslaunch respawns it fresh). Within this instance the failure stays
        # fatal: even a genuine exit + re-entry must not quietly open.
        session.on_state(True, False, False)
        self.assertFalse(session.healthy())
        self.assertFalse(session.recording())
        self.assertIsNone(session.on_state(True, True, True))
        self.assertFalse(session.recording())

    def test_fresh_instance_recovers_after_restart(self):
        # A respawned recorder (new instance) starts healthy and idle.
        session = RecorderSession()
        session.on_state(True, False, False)
        self.assertTrue(session.healthy())
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        self.assertTrue(session.recording())
        self.assertTrue(session.healthy())

    def test_write_failure_closes_bag_and_never_quietly_recovers(self):
        session = RecorderSession()
        session.on_state(True, False, False)
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        self.assertTrue(session.recording())
        session.fail()  # write raised
        self.assertFalse(session.recording())
        self.assertFalse(session.healthy())
        # A new genuine session still must not open while the recorder is fatal.
        session.on_state(True, False, False)
        session.on_state(True, False, False)
        self.assertIsNone(session.on_state(True, True, True))
        self.assertFalse(session.recording())

    def test_close_failure_is_fatal_not_closed(self):
        # bag.close() raising must leave the recorder unhealthy: the half
        # written bag is not a clean recording, and the next session must not
        # open without a respawn (which the controller enforces via the
        # health interlock).
        session = RecorderSession()
        session.on_state(True, False, False)
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        self.assertTrue(session.recording())
        # Close path fails: the node calls fail() instead of bag_closed().
        session.fail()
        self.assertFalse(session.recording())
        self.assertFalse(session.healthy())
        session.on_state(True, False, False)
        session.on_state(True, False, False)
        self.assertIsNone(session.on_state(True, True, True))
        self.assertFalse(session.recording())

    def test_terminal_status_keeps_session_healthy_after_tail_close(self):
        session = RecorderSession()
        session.on_state(True, False, False)
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        session.note_terminal()  # COMPLETE seen while still in OFFBOARD
        # The 1 s tail elapses and the bag closes even though the vehicle is
        # still holding in OFFBOARD.
        session.bag_closed()
        self.assertTrue(session.healthy())
        self.assertFalse(session.recording())
        # A real exit clears the finished flag; a fresh edge opens a new bag.
        self.assertIsNone(session.on_state(True, False, False))  # no bag open
        self.assertTrue(session.healthy())
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        self.assertTrue(session.recording())
        self.assertTrue(session.healthy())

    def test_full_two_session_lifecycle(self):
        session = RecorderSession()
        session.on_state(True, False, False)
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        session.on_state(True, True, True)  # steady state
        self.assertTrue(session.healthy())
        # Falling edge closes and re-arms for the next session.
        self.assertEqual(session.on_state(True, False, False), 'close')
        session.bag_closed()
        self.assertFalse(session.recording())
        self.assertTrue(session.healthy())
        self.assertEqual(session.on_state(True, True, True), 'open')
        session.bag_opened()
        self.assertTrue(session.recording())


if __name__ == '__main__':
    unittest.main(verbosity=2)
