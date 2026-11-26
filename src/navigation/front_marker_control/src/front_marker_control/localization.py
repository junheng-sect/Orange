"""Time-aligned local-frame pose caching for the Odometry input.

Pure-python module without ROS imports so the caching and interpolation
logic stays unit-testable; the node converts ROS stamps to float seconds
before calling in.
"""

import bisect
import math
from collections import deque, namedtuple

from front_marker_control.geometry import (
    finite_vector3,
    normalize_quaternion_values,
    yaw_from_quaternion,
)

LocalPose = namedtuple('LocalPose', ('stamp', 'position', 'quaternion'))


def slerp_quaternion(first, second, alpha):
    """Spherical interpolation between two normalized quaternions."""
    first = normalize_quaternion_values(first)
    second = normalize_quaternion_values(second)
    alpha = float(alpha)
    if not math.isfinite(alpha) or not 0.0 <= alpha <= 1.0:
        raise ValueError('slerp alpha must be finite within [0, 1]')

    dot = sum(first[index] * second[index] for index in range(4))
    if dot < 0.0:
        # q and -q describe the same rotation; take the shortest path.
        second = tuple(-value for value in second)
        dot = -dot
    if dot > 1.0 - 1e-10:
        # Nearly identical rotations: linear interpolation plus renorm.
        blended = tuple(
            (1.0 - alpha) * first[index] + alpha * second[index]
            for index in range(4)
        )
    else:
        theta = math.acos(min(1.0, dot))
        sine = math.sin(theta)
        weight_first = math.sin((1.0 - alpha) * theta) / sine
        weight_second = math.sin(alpha * theta) / sine
        blended = tuple(
            weight_first * first[index] + weight_second * second[index]
            for index in range(4)
        )
    return normalize_quaternion_values(blended)


def interpolate_local_pose(first, second, stamp):
    """Interpolate position linearly and orientation by slerp."""
    stamp = float(stamp)
    if not math.isfinite(stamp):
        raise ValueError('interpolation stamp must be finite')
    gap = float(second.stamp) - float(first.stamp)
    if not math.isfinite(gap) or gap <= 0.0:
        raise ValueError('interpolation requires increasing stamps')
    if not first.stamp <= stamp <= second.stamp:
        raise ValueError('interpolation stamp must lie between the samples')
    alpha = (stamp - first.stamp) / gap
    position = tuple(
        first.position[index]
        + alpha * (second.position[index] - first.position[index])
        for index in range(3)
    )
    quaternion = slerp_quaternion(first.quaternion, second.quaternion, alpha)
    return LocalPose(stamp, position, quaternion)


class LocalPoseBuffer(object):
    """Bounded cache of body poses in one fixed local frame.

    Samples are keyed by source stamp; ``accept`` validates and stores, and
    ``lookup`` interpolates at an arbitrary stamp without extrapolating.
    Invalid values and a frame change clear the whole cache.
    """

    def __init__(self, capacity=100):
        capacity = int(capacity)
        if capacity < 2:
            raise ValueError('capacity must be at least 2')
        self.capacity = capacity
        self.clear()

    def clear(self):
        self.frame_id = None
        self._samples = deque(maxlen=self.capacity)

    def __len__(self):
        return len(self._samples)

    @property
    def newest_stamp(self):
        return self._samples[-1].stamp if self._samples else None

    def accept(self, stamp, frame_id, position, quaternion):
        """Validate and store one Odometry pose.

        Returns ``None`` when stored, otherwise a rejection reason.  Invalid
        positions or quaternions and a frame change clear the cache.
        """
        stamp = float(stamp)
        if not math.isfinite(stamp):
            return '拒绝非有限定位时间戳'
        frame_id = str(frame_id)
        if not frame_id:
            return '拒绝空定位坐标系'
        try:
            position = finite_vector3(position, 'odometry position')
            quaternion = normalize_quaternion_values(quaternion)
        except ValueError as exception:
            self.clear()
            return '拒绝非法 Odometry: %s' % exception
        if self.frame_id is None:
            self.frame_id = frame_id
        elif frame_id != self.frame_id:
            previous = self.frame_id
            self.clear()
            return '定位坐标系变化 %s -> %s，已清除缓存' % (previous, frame_id)
        if self._samples and stamp <= self._samples[-1].stamp:
            return '定位时间戳未递增，丢弃'
        self._samples.append(LocalPose(stamp, position, quaternion))
        return None

    def lookup(self, stamp, max_gap):
        """Return the pose at ``stamp`` or ``None``.

        Requires two bracketing samples within ``max_gap``; never
        extrapolates beyond the cached range.
        """
        stamp = float(stamp)
        max_gap = float(max_gap)
        if not math.isfinite(stamp):
            raise ValueError('lookup stamp must be finite')
        if not math.isfinite(max_gap) or max_gap <= 0.0:
            raise ValueError('max_gap must be finite and positive')
        samples = list(self._samples)
        if not samples:
            return None
        if stamp < samples[0].stamp or stamp > samples[-1].stamp:
            return None
        stamps = [sample.stamp for sample in samples]
        index = bisect.bisect_right(stamps, stamp) - 1
        if samples[index].stamp == stamp:
            return samples[index]
        if index + 1 >= len(samples):
            return None
        first = samples[index]
        second = samples[index + 1]
        gap = second.stamp - first.stamp
        if gap > max_gap:
            return None
        return interpolate_local_pose(first, second, stamp)


def yaw_pose(stamp, position, yaw):
    """Build a level LocalPose from a yaw angle (radians)."""
    half = 0.5 * float(yaw)
    return LocalPose(
        float(stamp),
        finite_vector3(position, 'position'),
        (0.0, 0.0, math.sin(half), math.cos(half)),
    )


def pose_yaw(pose):
    """Return the yaw of a LocalPose (radians)."""
    return yaw_from_quaternion(pose.quaternion)
