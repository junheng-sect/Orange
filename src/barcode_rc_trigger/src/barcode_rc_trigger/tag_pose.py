"""读码时刻的 AprilTag 位姿（不依赖 ROS）：从 HomogeneousTransform 取位置，按图像时间戳或最新一帧匹配。

位置是 tag 中心在相机光学系下的坐标（x 右、y 下、z 前，m）；tilt 是 tag 法向与相机光轴的夹角，
正对相机为 0。
"""
import collections
import math

from .runner import Raw

TagSample = collections.namedtuple('TagSample', ['stamp', 'received', 'marker_id',
                                                 'x', 'y', 'z', 'tilt_deg'])


def sample_from_matrix(stamp, received, marker_id, matrix):
    """matrix 为行主序 ^camera T_marker；stamp 为图像时间（s），received 为本地单调时钟收到时刻。"""
    x, y, z = float(matrix[3]), float(matrix[7]), float(matrix[11])
    # tag Z 轴在相机系的 z 分量为 matrix[10]；正对相机时 tag Z 与光轴相反。
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(matrix[10])))))
    return TagSample(stamp, received, int(marker_id), x, y, z, tilt)


class TagHistory(object):
    """最近若干帧 tag 位姿。"""

    def __init__(self, maxlen=60):
        self.samples = collections.deque(maxlen=maxlen)

    def add(self, sample):
        self.samples.append(sample)

    def at(self, stamp, tolerance_s=0.01):
        """与图像同一传感器帧的位姿；相机读码用的就是这一帧的位姿。

        位姿来自 1280x960 自通路，读码图像来自 4224x3136 主通路，同一帧两路时间戳差约 0.01 ms
        （帧间隔 66.7 ms），取 tolerance_s 以内最近的一帧。
        """
        best, best_dt = None, tolerance_s
        for sample in self.samples:
            dt = abs(sample.stamp - stamp)
            if dt <= best_dt:
                best, best_dt = sample, dt
        return best

    def latest(self, now, max_age_s):
        """now 时刻 max_age_s 以内收到的最新一帧，没有则 None。"""
        if not self.samples:
            return None
        sample = self.samples[-1]
        return sample if now - sample.received <= max_age_s else None


def pick_tag(history, image_stamp, pulse_sample):
    """优先取相机所解图像那一帧的位姿（img），否则用拨杆时刻的最新位姿（pulse）。"""
    if image_stamp:
        sample = history.at(image_stamp)
        if sample is not None:
            return sample, 'img'
    if pulse_sample is not None:
        return pulse_sample, 'pulse'
    return None, ''


def distance(sample):
    return math.sqrt(sample.x ** 2 + sample.y ** 2 + sample.z ** 2)


def tag_fields(sample, source):
    """汇总里的 tag 段（runner.format_block 的 fields）；没有 tag 时为 None，输出 `tag: none`。"""
    if sample is None:
        return None
    return [('source', Raw(source)), ('id', sample.marker_id),
            ('x', Raw('%.3f' % sample.x)), ('y', Raw('%.3f' % sample.y)),
            ('z', Raw('%.3f' % sample.z)), ('dist', Raw('%.3f' % distance(sample))),
            ('tilt_deg', Raw('%.1f' % sample.tilt_deg))]
