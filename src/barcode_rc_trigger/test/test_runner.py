#!/usr/bin/env python3
import collections
import unittest

from barcode_rc_trigger.runner import (StepResult, format_step, parse_mode, run_sequence,
                                       step_ok)

CameraRes = collections.namedtuple('CameraRes', 'success partial error elapsed_s report codes')
SerialRes = collections.namedtuple('SerialRes', 'success code error elapsed_s')
Code = collections.namedtuple('Code', 'format code')


class RunSequenceTest(unittest.TestCase):
    def setUp(self):
        self.order = []
        self.calls = {
            'camera': lambda: self.order.append('camera') or 'cam',
            'serial': lambda: self.order.append('serial') or 'ser',
        }

    def test_modes(self):
        self.assertEqual(parse_mode('camera'), ('camera',))
        self.assertEqual(parse_mode('serial'), ('serial',))
        self.assertEqual(parse_mode('both'), ('camera', 'serial'))
        for bad in ('', 'Both', 'serial_then_camera', None):
            with self.assertRaises(ValueError):
                parse_mode(bad)

    def test_both_runs_camera_then_serial(self):
        results = run_sequence(parse_mode('both'), self.calls)
        self.assertEqual(self.order, ['camera', 'serial'])
        self.assertEqual(results, [StepResult('camera', 'cam', ''),
                                   StepResult('serial', 'ser', '')])

    def test_clock_records_each_step_start(self):
        ticks = iter([10.0, 11.5])
        results = run_sequence(parse_mode('both'), self.calls, clock=lambda: next(ticks))
        self.assertEqual([result.started for result in results], [10.0, 11.5])
        # 不给 clock 时为 None（CH9 节点不需要）。
        self.assertIsNone(run_sequence(parse_mode('camera'), self.calls)[0].started)

    def test_single_modes_call_only_one(self):
        run_sequence(parse_mode('camera'), self.calls)
        self.assertEqual(self.order, ['camera'])
        self.order.clear()
        run_sequence(parse_mode('serial'), self.calls)
        self.assertEqual(self.order, ['serial'])

    def test_failure_does_not_stop_next_step(self):
        def broken():
            self.order.append('camera')
            raise RuntimeError('service /barcode_reader_camera/decode unavailable')
        self.calls['camera'] = broken
        results = run_sequence(parse_mode('both'), self.calls)
        self.assertEqual(self.order, ['camera', 'serial'])
        self.assertIsNone(results[0].response)
        self.assertIn('unavailable', results[0].error)
        self.assertEqual(results[1].response, 'ser')


class FormatTest(unittest.TestCase):
    def test_camera_ok(self):
        res = CameraRes(True, False, '', 0.4804, 'QR=1/1 C128=1/1',
                        [Code('QRCode', 'A20260609001'), Code('Code128', 'NEXT9017324')])
        text = format_step(StepResult('camera', res, ''))
        self.assertEqual(text, 'camera: ok report="QR=1/1 C128=1/1" [QRCode]"A20260609001" '
                               '[Code128]"NEXT9017324" 480ms')
        self.assertTrue(step_ok(StepResult('camera', res, '')))

    def test_camera_partial_and_fail(self):
        partial = CameraRes(False, True, 'partial', 1.2, 'C128=1/2', [Code('Code128', 'x')])
        self.assertEqual(format_step(StepResult('camera', partial, '')),
                         'camera: partial report="C128=1/2" [Code128]"x" 1200ms')
        fail = CameraRes(False, False, 'no_tag', 0.012, '', [])
        self.assertEqual(format_step(StepResult('camera', fail, '')),
                         'camera: fail error=no_tag 12ms')
        self.assertFalse(step_ok(StepResult('camera', fail, '')))

    def test_serial(self):
        ok = SerialRes(True, 'NEXT"9\r', '', 0.72)
        self.assertEqual(format_step(StepResult('serial', ok, '')),
                         'serial: ok "NEXT\\"9\\x0d" 720ms')
        fail = SerialRes(False, '', 'timeout', 1.5)
        self.assertEqual(format_step(StepResult('serial', fail, '')),
                         'serial: fail error=timeout 1500ms')

    def test_unavailable(self):
        result = StepResult('serial', None, 'timeout exceeded')
        self.assertEqual(format_step(result), 'serial: unavailable (timeout exceeded)')
        self.assertFalse(step_ok(result))


if __name__ == '__main__':
    unittest.main()
