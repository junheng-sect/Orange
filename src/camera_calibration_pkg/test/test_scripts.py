#!/usr/bin/env python3
"""标定辅助脚本的离线测试（不需要相机）。"""

import io
import os
import subprocess
import sys
import tarfile
import tempfile
import unittest

import yaml

SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')
sys.path.insert(0, SCRIPTS)
import auto_save_calibrator  # noqa: E402
import generate_checkerboard  # noqa: E402
import recalibrate  # noqa: E402
import save_calibration  # noqa: E402

OST = """image_width: %d
image_height: %d
camera_name: ov13855_2560x1440
camera_matrix:
  rows: 3
  cols: 3
  data: [2000.0, 0.0, 1280.0, 0.0, 2000.0, 720.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, -0.2, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [2000.0, 0.0, 1280.0, 0.0, 0.0, 2000.0, 720.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""


def make_tar(path, width=2560, height=1440, samples=3):
    with tarfile.open(path, 'w:gz') as archive:
        files = [('left-%04d.png' % i, b'png') for i in range(samples)] + [
            ('ost.yaml', (OST % (width, height)).encode())]
        for name, data in files:
            info = tarfile.TarInfo(name)
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))


class SaveCalibrationTest(unittest.TestCase):
    def test_saves_timestamped_yaml_and_tar(self):
        with tempfile.TemporaryDirectory() as tmp:
            tar = os.path.join(tmp, 'calibrationdata.tar.gz')
            make_tar(tar)
            out = os.path.join(tmp, 'out')
            self.assertEqual(save_calibration.main(['--tar', tar, '--out-dir', out, '--reference', '']), 0)
            saved = sorted(os.listdir(out))
            self.assertEqual(len(saved), 2)
            self.assertTrue(saved[0].startswith('ov13855_2560x1440_') and saved[0].endswith('.tar.gz'))
            with open(os.path.join(out, saved[1])) as f:
                self.assertEqual(yaml.safe_load(f)['camera_matrix']['data'][0], 2000.0)

    def test_rejects_wrong_resolution(self):
        with tempfile.TemporaryDirectory() as tmp:
            tar = os.path.join(tmp, 'calibrationdata.tar.gz')
            make_tar(tar, width=1920, height=1080)
            out = os.path.join(tmp, 'out')
            self.assertEqual(save_calibration.main(['--tar', tar, '--out-dir', out]), 1)
            self.assertFalse(os.path.exists(out))

    def test_missing_tar(self):
        self.assertEqual(save_calibration.main(['--tar', '/nonexistent/calibrationdata.tar.gz']), 1)


class CheckerboardTest(unittest.TestCase):
    def test_layout_picks_smallest_paper(self):
        self.assertEqual(generate_checkerboard.layout(8, 6, 25.0, 'auto')[0], 'A4')
        # 7x5 内角点 41 mm：8x6 格 328x246 mm，A4 放不下。
        self.assertEqual(generate_checkerboard.layout(7, 5, 41.0, 'auto')[0], 'A3')
        self.assertIsNone(generate_checkerboard.layout(7, 5, 41.0, 'A4'))

    def test_writes_pdf_with_exact_page_size(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, 'board.pdf')
            self.assertEqual(generate_checkerboard.main(['--cols', '6', '--rows', '4', '--square', '0.025',
                                                         '--out', out]), 0)
            info = subprocess.check_output(['pdfinfo', out]).decode()
            size = [l for l in info.splitlines() if l.startswith('Page size')][0].split()
            # A4 竖版 210x297 mm = 595.28x841.89 pt
            self.assertAlmostEqual(float(size[2]), 595.28, delta=1)
            self.assertAlmostEqual(float(size[4]), 841.89, delta=1)


def synthetic_views(k, cols=7, rows=5, square=0.041):
    """无畸变相机下棋盘格的 2560x1440 合成图。"""
    import cv2
    import numpy as np
    texture = np.full(((rows + 3) * 100, (cols + 3) * 100), 255, np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                texture[(r + 1) * 100:(r + 2) * 100, (c + 1) * 100:(c + 2) * 100] = 0
    to_board = np.array([[square / 100, 0, -2 * square], [0, square / 100, -2 * square], [0, 0, 1]])
    views = []
    for rx, ry, tz in [(0.1, 0.2, 0.6), (-0.2, 0.1, 0.7), (0.3, -0.2, 0.8), (-0.1, -0.3, 0.65)]:
        rotation, _ = cv2.Rodrigues(np.array([rx, ry, 0.0]))
        h = k @ np.column_stack([rotation[:, 0], rotation[:, 1], [-0.15, -0.1, tz]]) @ to_board
        views.append(cv2.warpPerspective(texture, h, (2560, 1440), borderValue=128))
    return views


class AutoSaveTest(unittest.TestCase):
    def test_calibrate_saves_yaml_with_rms_note(self):
        import numpy as np
        from unittest import mock
        from camera_calibration.calibrator import ChessboardInfo, MonoCalibrator
        k = np.array([[1900.0, 0, 1280], [0, 1420.0, 720], [0, 0, 1]])
        calibrator = MonoCalibrator([ChessboardInfo('chessboard', 7, 5, 0.041)], name='ov13855_2560x1440')
        calibrator.db = [(None, image) for image in synthetic_views(k)]
        with tempfile.TemporaryDirectory() as tmp:
            tar = os.path.join(tmp, 'calibrationdata.tar.gz')
            out = os.path.join(tmp, 'out')

            def do_save():
                with tarfile.open(tar, 'w:gz') as archive:
                    calibrator.do_tarfile_save(archive)
            calibrator.do_save = do_save
            params = {'~out_dir': out, '~reference': ''}
            with mock.patch.object(auto_save_calibrator, 'TAR', tar), \
                    mock.patch('rospy.get_param', lambda name, default=None: params.get(name, default)):
                auto_save_calibrator.do_calibration_and_save(calibrator)
            self.assertTrue(calibrator.calibrated)
            saved = [n for n in os.listdir(out) if n.endswith('.yaml')]
            self.assertEqual(len(saved), 1)
            with open(os.path.join(out, saved[0])) as f:
                text = f.read()
            self.assertIn('样本图 4 张', text)
            self.assertIn('重投影 RMS', text)
            calib = yaml.safe_load(text)
            self.assertAlmostEqual(calib['camera_matrix']['data'][0], 1900.0, delta=20)
            self.assertAlmostEqual(calib['camera_matrix']['data'][4], 1420.0, delta=20)
            _, total = auto_save_calibrator.reprojection_rms(calibrator)
            self.assertLess(total, 0.5)


class RecalibrateTest(unittest.TestCase):
    def test_recovers_intrinsics_from_tar(self):
        import cv2
        import numpy as np
        k = np.array([[1900.0, 0, 1280], [0, 1420.0, 720], [0, 0, 1]])
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, 'out')
            os.makedirs(out)
            tar = os.path.join(out, 'ov13855_2560x1440_20260101_000000.tar.gz')
            with tarfile.open(tar, 'w:gz') as archive:
                for i, image in enumerate(synthetic_views(k)):
                    data = cv2.imencode('.png', image)[1].tobytes()
                    info = tarfile.TarInfo('left-%04d.png' % i)
                    info.size = len(data)
                    archive.addfile(info, io.BytesIO(data))
            self.assertEqual(recalibrate.main(['--size', '7x5', '--square', '0.041', '--k', '2',
                                               '--min-images', '3', '--out-dir', out, '--reference', '']), 0)
            saved = [n for n in os.listdir(out) if n.endswith('_k2.yaml')]
            self.assertEqual(len(saved), 1)
            with open(os.path.join(out, saved[0])) as f:
                text = f.read()
            self.assertIn('使用 4/4 张', text)
            calib = yaml.safe_load(text)
            self.assertAlmostEqual(calib['camera_matrix']['data'][0], 1900.0, delta=20)
            self.assertEqual(calib['distortion_coefficients']['data'][4], 0.0)

    def test_too_few_images(self):
        with tempfile.TemporaryDirectory() as tmp:
            tar = os.path.join(tmp, 'calibrationdata.tar.gz')
            make_tar(tar)
            self.assertEqual(recalibrate.main(['--size', '7x5', '--square', '0.041', '--tar', tar,
                                               '--out-dir', tmp]), 1)


if __name__ == '__main__':
    unittest.main()
