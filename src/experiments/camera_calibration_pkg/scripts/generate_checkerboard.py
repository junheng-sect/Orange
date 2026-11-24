#!/usr/bin/env python3
"""生成 1:1 物理尺寸的棋盘格 PDF；--cols/--rows 为内角点数，格子数各多 1。"""

import argparse
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

PAPERS = {'A4': (210.0, 297.0), 'A3': (297.0, 420.0)}
MARGIN_MM = 10.0
FOOTER_MM = 12.0


def layout(cols, rows, square_mm, paper):
    """返回 (纸名, 纸宽, 纸高)；放不下返回 None。auto 按 A4→A3 取最小可用纸。"""
    board_w, board_h = (cols + 1) * square_mm, (rows + 1) * square_mm
    for name in (PAPERS if paper == 'auto' else [paper]):
        short, long_ = PAPERS[name]
        for page_w, page_h in ((short, long_), (long_, short)):
            if board_w + 2 * MARGIN_MM <= page_w and board_h + 2 * MARGIN_MM + FOOTER_MM <= page_h:
                return name, page_w, page_h
    return None


def generate(cols, rows, square, paper, out):
    square_mm = square * 1000.0
    fit = layout(cols, rows, square_mm, paper)
    if fit is None:
        raise ValueError('%d x %d 格、%.1f mm 放不进 %s，换更大纸或减小 --square' % (
            cols + 1, rows + 1, square_mm, paper))
    name, page_w, page_h = fit
    board_w, board_h = (cols + 1) * square_mm, (rows + 1) * square_mm
    x0 = (page_w - board_w) / 2.0
    y0 = FOOTER_MM + (page_h - FOOTER_MM - board_h) / 2.0
    fig = plt.figure(figsize=(page_w / 25.4, page_h / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, page_w)
    ax.set_ylim(0, page_h)
    ax.axis('off')
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                ax.add_patch(Rectangle((x0 + c * square_mm, y0 + r * square_mm), square_mm, square_mm,
                                       facecolor='black', edgecolor='none'))
    ax.text(page_w / 2.0, MARGIN_MM / 2.0 + 2.0,
            # 页脚用英文，避免 matplotlib 3.1 嵌入 CJK 字体集合的问题。
            'inner corners %dx%d   squares %dx%d   square %.1f mm   %s   print at 100%% / actual size' % (
                cols, rows, cols + 1, rows + 1, square_mm, name),
            ha='center', va='center', fontsize=8)
    fig.savefig(out, format='pdf')
    plt.close(fig)
    return name


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cols', type=int, default=7, help='横向内角点数')
    parser.add_argument('--rows', type=int, default=5, help='纵向内角点数')
    parser.add_argument('--square', type=float, default=0.041, help='单格边长（米）')
    parser.add_argument('--paper', choices=['auto', 'A4', 'A3'], default='auto')
    parser.add_argument('--out', help='输出 PDF，默认 checkerboard_<cols>x<rows>_<mm>mm_<纸>.pdf')
    args = parser.parse_args(argv)
    if args.cols < 2 or args.rows < 2 or args.square <= 0:
        parser.error('内角点数至少 2，边长须为正')
    mm = ('%.1f' % (args.square * 1000)).rstrip('0').rstrip('.')
    paper = (layout(args.cols, args.rows, args.square * 1000, args.paper) or [args.paper])[0]
    out = args.out or 'checkerboard_%dx%d_%smm_%s.pdf' % (args.cols, args.rows, mm, paper)
    try:
        generate(args.cols, args.rows, args.square, args.paper, out)
    except ValueError as error:
        print('错误：%s' % error, file=sys.stderr)
        return 1
    print('已生成：%s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
