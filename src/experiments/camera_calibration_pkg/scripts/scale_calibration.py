#!/usr/bin/env python3
"""把标定按纯缩放换算到同一画幅的另一个输出分辨率，不重新标定。

OV13855 的 ISP 主通路、自通路各档输出都是整幅 4224x3136 缩放、不裁剪（crop 均为 (0,0)/4224x3136），
所以同一镜头在不同分辨率下只差横/纵两个缩放比例：K、P 的前两行分别乘 sx、sy，
主点按像素中心约定 c' = (c+0.5)*s-0.5；畸变系数定义在归一化坐标上，不变。
两种分辨率视场不同（有裁剪）时不能用。
"""

import argparse
import datetime
import os
import sys

import numpy
import yaml


def scale_matrix(m, sx, sy):
    """K（3x3）或 P（3x4）换算到新尺寸：第一行乘 sx、第二行乘 sy，再补主点的像素中心偏移。"""
    out = numpy.array(m, numpy.float64)
    out[0, :] *= sx
    out[1, :] *= sy
    out[0, 2] += 0.5 * sx - 0.5
    out[1, 2] += 0.5 * sy - 0.5
    return out


def scale(calib, width, height):
    """返回 (K, D, R, P, sx, sy)，均为 numpy 数组，D 为列向量。"""
    sx = float(width) / calib['image_width']
    sy = float(height) / calib['image_height']
    k = scale_matrix(numpy.reshape(calib['camera_matrix']['data'], (3, 3)), sx, sy)
    p = scale_matrix(numpy.reshape(calib['projection_matrix']['data'], (3, 4)), sx, sy)
    d = numpy.array(calib['distortion_coefficients']['data'], numpy.float64).reshape(-1, 1)
    r = numpy.array(calib['rectification_matrix']['data'], numpy.float64).reshape(3, 3)
    return k, d, r, p, sx, sy


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', help='源标定 yaml')
    parser.add_argument('--width', type=int, required=True)
    parser.add_argument('--height', type=int, required=True)
    parser.add_argument('--camera-name', required=True, help='写入的 camera_name，须与相机配置的 camera_name 一致')
    parser.add_argument('--out', required=True, help='输出 yaml 路径')
    args = parser.parse_args(argv)

    from camera_calibration.calibrator import CAMERA_MODEL, MonoCalibrator
    with open(args.source) as f:
        source_text = f.read()
    calib = yaml.safe_load(source_text)
    if calib.get('distortion_model') != 'plumb_bob':
        print('错误：只支持 plumb_bob，源文件为 %s' % calib.get('distortion_model'), file=sys.stderr)
        return 1
    k, d, r, p, sx, sy = scale(calib, args.width, args.height)
    text = MonoCalibrator.lryaml(args.camera_name, d, k, r, p, (args.width, args.height), CAMERA_MODEL.PINHOLE)
    header = [
        '# 由 %s（%dx%d）按纯缩放换算到 %dx%d，%s；未重新标定' % (
            os.path.basename(args.source), calib['image_width'], calib['image_height'],
            args.width, args.height, datetime.datetime.now().strftime('%Y-%m-%d %H:%M')),
        '# 横向 x%.7f、纵向 x%.7f；主点按像素中心 (c+0.5)*s-0.5；畸变系数不变（camera_calibration_pkg/scale_calibration.py）' % (
            sx, sy),
        '# 只在两种分辨率视场相同（ISP 整幅缩放、不裁剪）时成立。源文件注释：',
    ] + ['#   ' + line.lstrip('# ') for line in source_text.splitlines() if line.startswith('#')]
    with open(args.out, 'w') as f:
        f.write('\n'.join(header) + '\n' + text)
    print('已保存：%s' % args.out)
    print('fx %.2f  fy %.2f  cx %.2f  cy %.2f  （源 fx %.2f fy %.2f cx %.2f cy %.2f）' % (
        k[0, 0], k[1, 1], k[0, 2], k[1, 2], calib['camera_matrix']['data'][0], calib['camera_matrix']['data'][4],
        calib['camera_matrix']['data'][2], calib['camera_matrix']['data'][5]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
