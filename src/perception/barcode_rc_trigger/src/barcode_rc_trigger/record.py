"""每次读码一行 CSV：tag 位置 + 相机/读码器结果，用于统计在什么位置能读成功（不依赖 ROS）。

CH9 拨杆写 rc_read.csv（HEADER），到位自动读码（barcode_arrival_trigger）写 arrival_read.csv
（ARRIVAL_HEADER）；两者只有前三列不同，后面的 tag 位置和读码结果列相同，可以用同一套脚本分析。
"""
import csv
import os
import time

from .runner import step_ok
from .tag_pose import distance

RESULT_COLUMNS = ['tag_src', 'marker_id', 'x', 'y', 'z', 'dist', 'tilt_deg',
                  'camera_ok', 'camera_error', 'camera_report', 'camera_codes', 'camera_ms',
                  'serial_ok', 'serial_error', 'serial_code', 'serial_ms']
# control_mode：拨杆时刻 front_marker_control 的控制模式（没在跑时为空），看是否在 POS_HOLD 时读的。
HEADER = ['time', 'pulse', 'control_mode'] + RESULT_COLUMNS
# hold_lost：读码期间退出了 POS_HOLD。
ARRIVAL_HEADER = ['time', 'arrival', 'hold_lost'] + RESULT_COLUMNS


def csv_path(session_dir, csv_dir, start_time, mode, name='rc_read.csv'):
    """本次运行的 CSV：相机读码节点返回了会话目录就写进去（和这次的图片、attempts.csv 同一个文件夹）；
    否则（只跑 serial，或相机没开记录）在 csv_dir 下按节点启动时间建一个文件夹。"""
    if session_dir:
        return os.path.join(session_dir, name)
    folder = '%s_%s' % (time.strftime('%Y%m%d_%H%M%S', time.localtime(start_time)), mode)
    return os.path.join(csv_dir, folder, name)


def _ms(response):
    return '%.0f' % (response.elapsed_s * 1000.0)


def csv_row(wall_time, seq, tag, tag_source, results, extra=None, header=HEADER):
    """未调用的那一路留空；服务不在线时 ok=0、error=unavailable。

    header[1] 是序号列（pulse / arrival）；extra 填其余专有列，如 {'control_mode': 'POS_HOLD'}。
    """
    row = dict.fromkeys(header, '')
    row['time'] = '%.3f' % wall_time
    row[header[1]] = seq
    row.update(extra or {})
    if tag is not None:
        row.update(tag_src=tag_source, marker_id=tag.marker_id, x='%.4f' % tag.x,
                   y='%.4f' % tag.y, z='%.4f' % tag.z, dist='%.4f' % distance(tag),
                   tilt_deg='%.1f' % tag.tilt_deg)
    for result in results:
        prefix = result.name
        row[prefix + '_ok'] = int(step_ok(result))
        response = result.response
        if response is None:
            row[prefix + '_error'] = 'unavailable'
            continue
        row[prefix + '_ms'] = _ms(response)
        if prefix == 'camera':
            row['camera_error'] = response.error
            row['camera_report'] = response.report
            row['camera_codes'] = ';'.join('%s:%s' % (c.format, c.code) for c in response.codes)
        else:
            row['serial_error'] = response.error
            row['serial_code'] = response.code
    return [row[key] for key in header]


class CsvRecorder(object):
    """path 所在目录不存在时自动创建；每行写完立即 flush，节点被杀也不丢已有记录。"""

    def __init__(self, path, header=HEADER):
        self.path = path
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        new_file = not os.path.exists(path) or os.path.getsize(path) == 0
        self.file = open(path, 'a', newline='')
        self.writer = csv.writer(self.file)
        if new_file:
            self.writer.writerow(header)
            self.file.flush()

    def write(self, row):
        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        self.file.close()
