#!/usr/bin/env python3
"""官方 cameracalibrator.py 加自动保存：单目 CALIBRATE 完成后打印逐图重投影误差，并自动 SAVE + 另存 yaml。

另外在新位置因移动过快未采样时给出提示。

命令行参数与官方工具相同；私有参数 ~out_dir、~reference 控制另存目录和对比用的当前标定。
"""

import os
import runpy
import sys
import time
import traceback

import cv2
import numpy
import roslib.packages
import rospy
from camera_calibration.calibrator import CAMERA_MODEL, MonoCalibrator

# devel 空间经 catkin 中继脚本执行，sys.path[0] 是中继目录；按源文件目录导入同包脚本。
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import save_calibration  # noqa: E402

_do_calibration = MonoCalibrator.do_calibration
_is_slow_moving = MonoCalibrator.is_slow_moving
_last_speed_log = [0.0]
TAR = '/tmp/calibrationdata.tar.gz'  # 官方 do_save 的固定输出路径


def reprojection_rms(calibrator):
    """逐图 (索引, RMS 像素) 与总体 RMS；物方点与官方标定一致（单位格），不影响像素误差。"""
    per_image, squared, count = [], 0.0, 0
    for index, (corners, _, board) in enumerate(calibrator.good_corners):
        opts = calibrator.mk_object_points([board])[0]
        _, rvec, tvec = cv2.solvePnP(opts, corners, calibrator.intrinsics, calibrator.distortion)
        projected, _ = cv2.projectPoints(opts, rvec, tvec, calibrator.intrinsics, calibrator.distortion)
        residual = numpy.sum((projected - corners) ** 2, axis=2)
        per_image.append((index, float(numpy.sqrt(residual.mean()))))
        squared += float(residual.sum())
        count += residual.size
    return per_image, (squared / count) ** 0.5


def is_slow_moving_logged(self, corners, ids, last_frame_corners, last_frame_ids):
    """官方只在新位姿时检查速度且被拒时无提示；这里每秒最多提示一次实际位移。"""
    slow = _is_slow_moving(self, corners, ids, last_frame_corners, last_frame_ids)
    if not slow and ids is None and last_frame_corners is not None and time.time() - _last_speed_log[0] > 1.0:
        motion = numpy.linalg.norm((corners - last_frame_corners).reshape(-1, 2), axis=1).mean()
        print('新位置未采样：标定板移动 %.1f px/帧，超过上限 %.1f，到位后停稳片刻' % (
            motion, self.max_chessboard_speed))
        _last_speed_log[0] = time.time()
    return slow


def do_calibration_and_save(self, dump=False):
    _do_calibration(self, dump)
    try:
        notes = []
        board = self._boards[0]
        notes.append('棋盘内角点 %dx%d，边长 %.4f m' % (board.n_cols, board.n_rows, board.dim))
        if self.camera_model == CAMERA_MODEL.PINHOLE:
            per_image, total = reprojection_rms(self)
            worst = sorted(per_image, key=lambda item: -item[1])[:5]
            print('重投影 RMS：总体 %.3f px；最差：%s' % (
                total, '，'.join('left-%04d %.3f' % item for item in worst)))
            notes.append('重投影 RMS %.3f px' % total)
        self.do_save()
        save_calibration.save(TAR, self.name,
                              os.path.expanduser(rospy.get_param('~out_dir', '~/.ros/camera_info')),
                              self.size[0], self.size[1],
                              rospy.get_param('~reference', save_calibration.DEFAULT_REFERENCE), notes)
    except Exception:  # GUI 回调里不能抛出，否则标定窗口失去响应
        traceback.print_exc()
        print('自动保存失败，可在窗口点 SAVE 后运行 save_calibration.py 手动保存')


if __name__ == '__main__':
    MonoCalibrator.do_calibration = do_calibration_and_save
    MonoCalibrator.is_slow_moving = is_slow_moving_logged
    runpy.run_path(roslib.packages.find_node('camera_calibration', 'cameracalibrator.py')[0], run_name='__main__')
