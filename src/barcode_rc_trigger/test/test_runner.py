#!/usr/bin/env python3
import collections
import unittest

from barcode_rc_trigger.runner import (Raw, StepResult, format_block, parse_mode, run_sequence,
                                       step_fields, step_ok, summary_is_warning)

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
        result = StepResult('camera', res, '')
        self.assertEqual(step_fields(result), [
            ('ok', True), ('report', 'QR=1/1 C128=1/1'),
            ('codes', ['QRCode:A20260609001', 'Code128:NEXT9017324']), ('ms', 480)])
        self.assertTrue(step_ok(result))

    def test_camera_partial_and_fail(self):
        partial = CameraRes(False, True, 'partial', 1.2, 'C128=1/2', [Code('Code128', 'x')])
        self.assertEqual(step_fields(StepResult('camera', partial, '')), [
            ('ok', False), ('error', 'partial'), ('report', 'C128=1/2'),
            ('codes', ['Code128:x']), ('ms', 1200)])
        fail = CameraRes(False, False, 'no_tag', 0.012, '', [])
        self.assertEqual(step_fields(StepResult('camera', fail, '')),
                         [('ok', False), ('error', 'no_tag'), ('ms', 12)])
        self.assertFalse(step_ok(StepResult('camera', fail, '')))

    def test_serial(self):
        ok = SerialRes(True, 'NEXT"9\r', '', 0.72)
        self.assertEqual(step_fields(StepResult('serial', ok, '')),
                         [('ok', True), ('code', 'NEXT"9\r'), ('ms', 720)])
        fail = SerialRes(False, '', 'no_read', 3.0204)
        self.assertEqual(step_fields(StepResult('serial', fail, '')),
                         [('ok', False), ('error', 'no_read'), ('ms', 3020)])

    def test_unavailable(self):
        result = StepResult('serial', None, 'timeout exceeded')
        self.assertEqual(step_fields(result), [('ok', False), ('error', 'unavailable'),
                                               ('detail', 'timeout exceeded')])
        self.assertFalse(step_ok(result))

    def test_block(self):
        cam = CameraRes(True, False, '', 0.037, 'QR=1/1', [Code('QRCode', '12345678')])
        ser = SerialRes(False, '', 'no_read', 3.017)
        results = [StepResult('camera', cam, ''), StepResult('serial', ser, '')]
        sections = [('tag', [('source', Raw('img')), ('x', Raw('-0.180'))])]
        sections += [(r.name, step_fields(r)) for r in results]
        self.assertEqual(format_block('pulse#90', sections), '\n'.join([
            'pulse#90',
            'tag:', '  source: img', '  x: -0.180',
            'camera:', '  ok: true', '  report: "QR=1/1"', '  codes:', '    - "QRCode:12345678"',
            '  ms: 37',
            'serial:', '  ok: false', '  error: no_read', '  ms: 3017',
            '---']))

    def test_block_none_empty_and_escape(self):
        text = format_block('arrival#1', [('tag', None), ('x', [('codes', []),
                                                                ('code', 'a"b\r')])])
        self.assertEqual(text, 'arrival#1\ntag: none\nx:\n  codes: []\n  code: "a\\"b\\x0d"\n---')

    def test_warning_only_when_unavailable(self):
        fail = SerialRes(False, '', 'no_read', 3.0)
        self.assertFalse(summary_is_warning([StepResult('serial', fail, '')]))
        self.assertTrue(summary_is_warning([StepResult('camera', None, 'down'),
                                            StepResult('serial', fail, '')]))


if __name__ == '__main__':
    unittest.main()
