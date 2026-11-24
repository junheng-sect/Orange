#!/usr/bin/env python3
"""从 cameracalibrator 的 SAVE 结果中取出 ost.yaml，带时间戳另存并与当前标定对比。

auto_save_calibrator.py 已在 CALIBRATE 后自动调用 save()；本脚本用于手动补存或重算逐图误差。
"""

import argparse
import datetime
import os
from pathlib import Path
import re
import shutil
import sys
import tarfile

import yaml

DEFAULT_REFERENCE = str(Path(__file__).resolve().parents[3] / 'drivers/ov13855_cam/config/ov13855_4224x3136.yaml')


def load_result(tar_path):
    with tarfile.open(tar_path, 'r') as archive:
        names = archive.getnames()
        if 'ost.yaml' not in names:
            raise ValueError('%s 中没有 ost.yaml，确认已点 CALIBRATE 再点 SAVE' % tar_path)
        text = archive.extractfile('ost.yaml').read().decode('utf-8')
    samples = sorted(n for n in names if re.match(r'left-\d+\.png$', n))
    return text, yaml.safe_load(text), samples


def intrinsics(calib):
    k = calib['camera_matrix']['data']
    return k[0], k[4], k[2], k[5]


def reprojection_errors(tar_path, calib, size, square):
    """用结果内参对每张样本图 solvePnP，返回 (文件名, RMS 像素) 列表。"""
    import cv2
    import numpy as np
    cols, rows = (int(v) for v in size.lower().split('x'))
    obj = np.zeros((cols * rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    k = np.array(calib['camera_matrix']['data'], np.float64).reshape(3, 3)
    d = np.array(calib['distortion_coefficients']['data'], np.float64)
    errors = []
    with tarfile.open(tar_path, 'r') as archive:
        for name in sorted(n for n in archive.getnames() if re.match(r'left-\d+\.png$', n)):
            buf = np.frombuffer(archive.extractfile(name).read(), np.uint8)
            gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
            found, corners = cv2.findChessboardCorners(gray, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH)
            if not found:
                errors.append((name, None))
                continue
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
            _, rvec, tvec = cv2.solvePnP(obj, corners, k, d)
            projected, _ = cv2.projectPoints(obj, rvec, tvec, k, d)
            errors.append((name, float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=2))))))
    return errors


def save(tar, camera_name, out_dir, width, height, reference=DEFAULT_REFERENCE, notes=()):
    """另存 ost.yaml（notes 写成文件头注释）和原始 tar，打印新旧内参对比；返回 (yaml 路径, 标定)。"""
    text, calib, samples = load_result(tar)
    if (calib['image_width'], calib['image_height']) != (width, height):
        raise ValueError('标定尺寸 %dx%d 与期望 %dx%d 不符' % (
            calib['image_width'], calib['image_height'], width, height))
    if calib.get('camera_name') != camera_name:
        print('警告：ost.yaml camera_name=%s，期望 %s' % (calib.get('camera_name'), camera_name))

    os.makedirs(out_dir, exist_ok=True)
    now = datetime.datetime.now()
    stem = '%s_%s' % (camera_name, now.strftime('%Y%m%d_%H%M%S'))
    yaml_path = os.path.join(out_dir, stem + '.yaml')
    header = ['# 棋盘格标定 %s，样本图 %d 张' % (now.strftime('%Y-%m-%d %H:%M'), len(samples))] + [
        '# ' + n for n in notes]
    with open(yaml_path, 'w') as f:
        f.write('\n'.join(header) + '\n' + text)
    shutil.copy2(tar, os.path.join(out_dir, stem + '.tar.gz'))

    fx, fy, cx, cy = intrinsics(calib)
    print('已保存：%s（样本图 %d 张，原始包另存为 %s.tar.gz）' % (yaml_path, len(samples), stem))
    print('新标定  fx=%.3f fy=%.3f cx=%.3f cy=%.3f' % (fx, fy, cx, cy))
    print('畸变 %s D=%s' % (calib['distortion_model'], calib['distortion_coefficients']['data']))
    if reference and os.path.isfile(reference):
        with open(reference) as f:
            rfx, rfy, rcx, rcy = intrinsics(yaml.safe_load(f))
        print('当前包内 fx=%.3f fy=%.3f cx=%.3f cy=%.3f（%s）' % (rfx, rfy, rcx, rcy, reference))
    return yaml_path, calib


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tar', default='/tmp/calibrationdata.tar.gz')
    parser.add_argument('--camera-name', default='ov13855_4224x3136')
    parser.add_argument('--out-dir', default=os.path.expanduser('~/.ros/camera_info'))
    parser.add_argument('--width', type=int, default=4224)
    parser.add_argument('--height', type=int, default=3136)
    parser.add_argument('--reference', default=DEFAULT_REFERENCE, help='用于对比的当前标定 yaml')
    parser.add_argument('--size', help='内角点数如 7x5；与 --square 同时给出时逐图计算重投影误差')
    parser.add_argument('--square', type=float, help='单格边长（米）')
    args = parser.parse_args(argv)

    try:
        _, calib = save(args.tar, args.camera_name, args.out_dir, args.width, args.height, args.reference)
    except (OSError, tarfile.TarError, ValueError) as error:
        print('错误：%s' % error, file=sys.stderr)
        return 1

    if args.size and args.square:
        errors = reprojection_errors(args.tar, calib, args.size, args.square)
        valid = [e for _, e in errors if e is not None]
        for name, error in errors:
            print('  %s %s' % (name, '未检出角点' if error is None else '%.3f px' % error))
        if valid:
            print('逐图重投影 RMS：平均 %.3f px，最大 %.3f px' % (sum(valid) / len(valid), max(valid)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
