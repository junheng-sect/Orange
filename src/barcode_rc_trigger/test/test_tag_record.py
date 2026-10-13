#!/usr/bin/env python3
import collections
import os
import shutil
import tempfile
import time
import unittest

from barcode_rc_trigger.record import ARRIVAL_HEADER, HEADER, CsvRecorder, csv_path, csv_row
from barcode_rc_trigger.runner import StepResult
from barcode_rc_trigger.tag_pose import (TagHistory, pick_tag, sample_from_matrix,
                                         tag_fields)

CameraRes = collections.namedtuple('CameraRes', 'success partial error elapsed_s report codes')
SerialRes = collections.namedtuple('SerialRes', 'success code error elapsed_s')
Code = collections.namedtuple('Code', 'format code')

# 正对相机、tag 在相机前方 0.5 m：tag Z 与光轴相反。
FACING = [1, 0, 0, 0.01,
          0, -1, 0, -0.02,
          0, 0, -1, 0.5,
          0, 0, 0, 1]


class TagPoseTest(unittest.TestCase):
    def test_sample_from_matrix(self):
        sample = sample_from_matrix(10.0, 1.0, 3, FACING)
        self.assertEqual((sample.marker_id, sample.x, sample.y, sample.z), (3, 0.01, -0.02, 0.5))
        self.assertAlmostEqual(sample.tilt_deg, 0.0)
        # 绕相机 y 轴转 60 度：tag Z 的相机 z 分量为 -cos60。
        tilted = list(FACING)
        tilted[10] = -0.5
        self.assertAlmostEqual(sample_from_matrix(0, 0, 0, tilted).tilt_deg, 60.0)

    def test_latest_respects_age(self):
        history = TagHistory()
        self.assertIsNone(history.latest(5.0, 0.5))
        history.add(sample_from_matrix(10.0, 4.8, 0, FACING))
        self.assertIsNotNone(history.latest(5.0, 0.5))
        self.assertIsNone(history.latest(5.4, 0.5))

    def test_pick_prefers_image_stamp(self):
        history = TagHistory()
        old = sample_from_matrix(10.0, 1.0, 1, FACING)
        new = sample_from_matrix(10.1, 1.1, 2, FACING)
        history.add(old)
        history.add(new)
        self.assertEqual(pick_tag(history, 10.0, new), (old, 'img'))
        self.assertEqual(pick_tag(history, 99.0, new), (new, 'pulse'))
        self.assertEqual(pick_tag(history, 0.0, new), (new, 'pulse'))
        self.assertEqual(pick_tag(history, 0.0, None), (None, ''))

    def test_format(self):
        sample = sample_from_matrix(10.0, 1.0, 0, FACING)
        self.assertEqual([(k, str(v)) for k, v in tag_fields(sample, 'img')],
                         [('source', 'img'), ('id', '0'), ('x', '0.010'), ('y', '-0.020'),
                          ('z', '0.500'), ('dist', '0.500'), ('tilt_deg', '0.0')])
        self.assertIsNone(tag_fields(None, ''))


class RecordTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def row(self, *args):
        return dict(zip(HEADER, csv_row(*args)))

    def test_both_with_tag(self):
        tag = sample_from_matrix(10.0, 1.0, 0, FACING)
        results = [
            StepResult('camera', CameraRes(True, False, '', 0.48, 'QR=1/1',
                                           [Code('QRCode', 'A1'), Code('Code128', 'B2')]), ''),
            StepResult('serial', SerialRes(False, '', 'no_read', 3.5), ''),
        ]
        row = self.row(1.0, 7, tag, 'img', results)
        self.assertEqual((row['pulse'], row['tag_src'], row['z'], row['dist']), (7, 'img', '0.5000', '0.5005'))
        self.assertEqual((row['camera_ok'], row['camera_codes'], row['camera_ms']),
                         (1, 'QRCode:A1;Code128:B2', '480'))
        self.assertEqual((row['serial_ok'], row['serial_error'], row['serial_ms']), (0, 'no_read', '3500'))

    def test_no_tag_and_unavailable(self):
        row = self.row(1.0, 1, None, '', [StepResult('serial', None, 'timeout')])
        self.assertEqual((row['x'], row['camera_ok'], row['serial_ok'], row['serial_error']),
                         ('', '', 0, 'unavailable'))

    def test_recorder_appends_with_single_header(self):
        path = os.path.join(self.tmp, 'sub', 'log.csv')
        for _ in range(2):
            recorder = CsvRecorder(path)
            recorder.write(csv_row(1.0, 1, None, '', []))
            recorder.close()
        with open(path) as handle:
            lines = handle.read().splitlines()
        self.assertEqual(len(lines), 3)
        self.assertEqual(lines[0], ','.join(HEADER))

    def test_control_mode_column(self):
        row = self.row(1.0, 3, None, '', [], {'control_mode': 'POS_HOLD'})
        self.assertEqual((row['pulse'], row['control_mode']), (3, 'POS_HOLD'))
        self.assertEqual(self.row(1.0, 4, None, '', [])['control_mode'], '')  # control not running

    def test_arrival_row_shares_result_columns(self):
        tag = sample_from_matrix(10.0, 1.0, 0, FACING)
        results = [StepResult('camera', CameraRes(True, False, '', 0.2, 'QR=1/1', [Code('QRCode', 'A1')]), '')]
        row = dict(zip(ARRIVAL_HEADER, csv_row(2.0, 5, tag, 'img', results, {'hold_lost': 1}, ARRIVAL_HEADER)))
        self.assertEqual((row['arrival'], row['hold_lost'], row['z'], row['camera_codes']),
                         (5, 1, '0.5000', 'QRCode:A1'))
        self.assertEqual(ARRIVAL_HEADER[3:], HEADER[3:])  # same analysis scripts for both files

    def test_csv_goes_into_camera_session(self):
        session = '/data/barcode/20261013_150000_qr_top_tag_bottom'
        self.assertEqual(csv_path(session, '/data/barcode', 0.0, 'both'), session + '/rc_read.csv')
        self.assertEqual(csv_path(session, '/data/barcode', 0.0, 'camera', 'arrival_read.csv'),
                         session + '/arrival_read.csv')

    def test_csv_without_camera_session_gets_own_folder(self):
        start = time.mktime((2026, 10, 13, 15, 0, 5, 0, 0, -1))  # local time, like the folder name
        self.assertEqual(csv_path('', '/data/barcode', start, 'serial'),
                         '/data/barcode/20261013_150005_serial/rc_read.csv')


if __name__ == '__main__':
    unittest.main()
