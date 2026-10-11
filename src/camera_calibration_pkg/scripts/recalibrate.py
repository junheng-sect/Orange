#!/usr/bin/env python3
"""对已保存的标定 tar 离线重算内参：原图重检角点，可选畸变项数，逐张剔除误差过大的样本后输出 yaml。"""

import argparse
import datetime
import glob
import os
import re
import sys
import tarfile

import cv2
import numpy
from camera_calibration.calibrator import CAMERA_MODEL, MonoCalibrator

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import save_calibration  # noqa: E402

RADIAL_FIX = [cv2.CALIB_FIX_K1, cv2.CALIB_FIX_K2, cv2.CALIB_FIX_K3]


def latest_tar(out_dir, camera_name):
    tars = sorted(glob.glob(os.path.join(out_dir, camera_name + '_*.tar.gz')))
    return tars[-1] if tars else None


def detect(tar_path, cols, rows):
    """返回 [(文件名, 角点)] 与图像尺寸；检不出整板的图跳过并提示。"""
    samples, size = [], None
    with tarfile.open(tar_path, 'r') as archive:
        for name in sorted(n for n in archive.getnames() if re.match(r'left-\d+\.png$', n)):
            gray = cv2.imdecode(numpy.frombuffer(archive.extractfile(name).read(), numpy.uint8),
                                cv2.IMREAD_GRAYSCALE)
            if gray is None:
                print('  %s 无法解码，跳过' % name)
                continue
            size = (gray.shape[1], gray.shape[0])
            found, corners = cv2.findChessboardCorners(gray, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH)
            if not found:
                print('  %s 未检出 %dx%d 角点，跳过' % (name, cols, rows))
                continue
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
            samples.append((name, corners))
    return samples, size


def calibrate(samples, objp, size, flags):
    rms, k, d, _, _, std, _, per_view = cv2.calibrateCameraExtended(
        [objp] * len(samples), [c for _, c in samples], size, None, None, flags=flags)
    return rms, k, d, std.ravel(), per_view.ravel()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--size', required=True, help='内角点数，如 7x5')
    parser.add_argument('--square', type=float, required=True, help='单格边长（米），只写入记录，不影响内参')
    parser.add_argument('--tar', help='默认取 --out-dir 下最新的 <camera_name>_*.tar.gz')
    parser.add_argument('--k', type=int, default=3, choices=[2, 3], help='径向畸变项数')
    parser.add_argument('--max-image-error', type=float, default=0.5, help='单张 RMS 上限（像素）')
    parser.add_argument('--min-images', type=int, default=15)
    parser.add_argument('--camera-name', default='ov13855_2560x1440')
    parser.add_argument('--out-dir', default=os.path.expanduser('~/.ros/camera_info'))
    parser.add_argument('--reference', default=save_calibration.DEFAULT_REFERENCE)
    args = parser.parse_args(argv)

    tar = args.tar or latest_tar(args.out_dir, args.camera_name)
    if not tar or not os.path.isfile(tar):
        print('错误：找不到标定 tar，用 --tar 指定', file=sys.stderr)
        return 1
    cols, rows = (int(v) for v in args.size.lower().split('x'))
    objp = numpy.zeros((cols * rows, 3), numpy.float32)
    objp[:, :2] = numpy.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square
    flags = 0
    for fix in RADIAL_FIX[args.k:]:
        flags |= fix

    print('读取 %s' % tar)
    samples, size = detect(tar, cols, rows)
    total = len(samples)
    if total < args.min_images:
        print('错误：有效样本 %d 张，少于 --min-images %d' % (total, args.min_images), file=sys.stderr)
        return 1
    rejected = []
    while True:
        rms, k, d, std, per_view = calibrate(samples, objp, size, flags)
        worst = int(numpy.argmax(per_view))
        if per_view[worst] <= args.max_image_error or len(samples) <= args.min_images:
            break
        print('  剔除 %s（%.2f px）' % (samples[worst][0], per_view[worst]))
        rejected.append(samples.pop(worst)[0])
    if per_view.max() > args.max_image_error:
        print('警告：已到 --min-images，仍有样本误差 %.2f px，建议重采' % per_view.max())

    print('使用 %d/%d 张，%d 项径向畸变，RMS %.3f px，单张最大 %.2f px' % (
        len(samples), total, args.k, rms, per_view.max()))
    print('fx %.2f±%.2f  fy %.2f±%.2f  cx %.2f±%.2f  cy %.2f±%.2f' % (
        k[0, 0], std[0], k[1, 1], std[1], k[0, 2], std[2], k[1, 2], std[3]))
    print('D = %s' % numpy.round(d.ravel()[:5], 6).tolist())

    # P 与官方工具一致：alpha=0 的最优新相机矩阵
    new_k, _ = cv2.getOptimalNewCameraMatrix(k, d, size, 0.0)
    p = numpy.zeros((3, 4))
    p[:, :3] = new_k
    text = MonoCalibrator.lryaml(args.camera_name, d.ravel()[:5].reshape(-1, 1), k, numpy.eye(3), p, size,
                                 CAMERA_MODEL.PINHOLE)
    now = datetime.datetime.now()
    header = [
        '# 棋盘格标定 %s，离线重算自 %s' % (now.strftime('%Y-%m-%d %H:%M'), os.path.basename(tar)),
        '# 棋盘内角点 %dx%d，边长 %.4f m；%d 项径向畸变；使用 %d/%d 张，RMS %.3f px' % (
            cols, rows, args.square, args.k, len(samples), total, rms),
    ]
    if rejected:
        header.append('# 剔除：' + ', '.join(rejected))
    os.makedirs(args.out_dir, exist_ok=True)
    yaml_path = os.path.join(args.out_dir, '%s_%s_k%d.yaml' % (
        args.camera_name, now.strftime('%Y%m%d_%H%M%S'), args.k))
    with open(yaml_path, 'w') as f:
        f.write('\n'.join(header) + '\n' + text)
    print('已保存：%s' % yaml_path)
    if args.reference and os.path.isfile(args.reference):
        with open(args.reference) as f:
            rfx, rfy, rcx, rcy = save_calibration.intrinsics(save_calibration.yaml.safe_load(f))
        print('当前包内 fx=%.2f fy=%.2f cx=%.2f cy=%.2f（%s）' % (rfx, rfy, rcx, rcy, args.reference))
    return 0


if __name__ == '__main__':
    sys.exit(main())
