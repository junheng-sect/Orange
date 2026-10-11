"""每次拨杆一行 CSV：tag 位置 + 相机/读码器结果，用于统计在什么位置能读成功（不依赖 ROS）。"""
import csv
import os

from .runner import step_ok
from .tag_pose import distance

HEADER = ['time', 'pulse', 'tag_src', 'marker_id', 'x', 'y', 'z', 'dist', 'tilt_deg',
          'camera_ok', 'camera_error', 'camera_report', 'camera_codes', 'camera_ms',
          'serial_ok', 'serial_error', 'serial_code', 'serial_ms']


def _ms(response):
    return '%.0f' % (response.elapsed_s * 1000.0)


def csv_row(wall_time, seq, tag, tag_source, results):
    """未调用的那一路留空；服务不在线时 ok=0、error=unavailable。"""
    row = dict.fromkeys(HEADER, '')
    row['time'] = '%.3f' % wall_time
    row['pulse'] = seq
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
    return [row[key] for key in HEADER]


class CsvRecorder(object):
    """path 所在目录不存在时自动创建；每行写完立即 flush，节点被杀也不丢已有记录。"""

    def __init__(self, path):
        self.path = path
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        new_file = not os.path.exists(path) or os.path.getsize(path) == 0
        self.file = open(path, 'a', newline='')
        self.writer = csv.writer(self.file)
        if new_file:
            self.writer.writerow(HEADER)
            self.file.flush()

    def write(self, row):
        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        self.file.close()
