"""Two-level winch decision logic, independent of ROS and networking.

Source stamps and source_now share the localization clock; now is a monotonic
receipt clock. All mutation must be serialized by the caller.
"""
from collections import deque
from dataclasses import dataclass
import math
from typing import Optional


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('%s must be a finite number' % name)
    if not math.isfinite(value):
        raise ValueError('%s must be a finite number' % name)
    return float(value)


def validate_duty(value, name='duty'):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 500:
        raise ValueError('%s must be an integer in [0, 500]' % name)
    return value


def vector(values, size, name):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError('%s must contain %d numbers' % (name, size))
    return tuple(finite_number(v, name) for v in values)


@dataclass(frozen=True)
class Config:
    anchor: tuple = (0.0, 0.0, 0.0)
    attachment_offset: tuple = (0.0, 0.0, 0.0)
    frame_id: str = 'drone_odom'
    child_frame_id: str = 'drone_base'
    low_duty: int = 100
    high_duty: int = 300
    fallback_duty: int = 100
    window_sec: float = 0.2
    min_samples: int = 3
    enter_speed_mps: float = -0.05
    exit_speed_mps: float = -0.02
    enter_hold_sec: float = 0.15
    timeout_sec: float = 0.5
    future_tolerance_sec: float = 0.05
    near_anchor_m: float = 0.1
    max_position_speed_mps: float = 3.0
    position_jump_slack_m: float = 0.05
    max_samples: int = 1000

    def __post_init__(self):
        object.__setattr__(self, 'anchor', vector(self.anchor, 3, 'anchor'))
        object.__setattr__(self, 'attachment_offset',
                           vector(self.attachment_offset, 3, 'attachment_offset'))
        for name in ('frame_id', 'child_frame_id'):
            value = getattr(self, name)
            if not isinstance(value, str) or not value or value.strip() != value:
                raise ValueError('%s must be a nonempty frame name' % name)
        for name in ('window_sec', 'timeout_sec', 'max_position_speed_mps'):
            if finite_number(getattr(self, name), name) <= 0:
                raise ValueError('%s must be positive' % name)
        for name in ('enter_hold_sec', 'future_tolerance_sec',
                     'near_anchor_m', 'position_jump_slack_m'):
            if finite_number(getattr(self, name), name) < 0:
                raise ValueError('%s must be nonnegative' % name)
        if self.window_sec >= self.timeout_sec:
            raise ValueError('window_sec must be smaller than timeout_sec')
        for name in ('min_samples', 'max_samples'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 3:
                raise ValueError('%s must be an integer >= 3' % name)
        if self.max_samples < self.min_samples:
            raise ValueError('max_samples must be >= min_samples')
        enter = finite_number(self.enter_speed_mps, 'enter_speed_mps')
        leave = finite_number(self.exit_speed_mps, 'exit_speed_mps')
        if not enter < leave <= 0:
            raise ValueError('require enter_speed_mps < exit_speed_mps <= 0')
        for name in ('low_duty', 'high_duty', 'fallback_duty'):
            validate_duty(getattr(self, name), name)
        if self.low_duty >= self.high_duty:
            raise ValueError('require low_duty < high_duty')



@dataclass(frozen=True)
class Status:
    mode: str
    enabled: bool
    input_valid: bool
    reason: str
    distance_m: Optional[float]
    radial_velocity_mps: Optional[float]
    target_duty: Optional[float]


def attachment_position(position, quaternion, offset):
    """Rotate body-frame offset using ROS quaternion order (x,y,z,w)."""
    p = vector(position, 3, 'position')
    q = vector(quaternion, 4, 'orientation')
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid orientation norm')
    x, y, z, w = (v / norm for v in q)
    ox, oy, oz = offset
    # R(q) * offset = offset + 2*w*(q_xyz x offset) + 2*q_xyz x (q_xyz x offset).
    cx, cy, cz = y * oz - z * oy, z * ox - x * oz, x * oy - y * ox
    result = (
        p[0] + ox + 2 * (w * cx + y * cz - z * cy),
        p[1] + oy + 2 * (w * cy + z * cx - x * cz),
        p[2] + oz + 2 * (w * cz + x * cy - y * cx),
    )
    return vector(result, 3, 'attachment position')


class Controller:
    def __init__(self, config):
        self.config = config
        self.mode = 'DISABLED'
        self.reason = 'automatic control disabled'
        self._fault = False
        self._last_now = None
        self._last_source_now = None
        self._clear_history()

    def _clear_history(self):
        self._samples = deque(maxlen=self.config.max_samples)
        self._last_stamp = None
        self._last_receive = None
        self._last_position = None
        self._candidate_since = None
        self._distance = None
        self._velocity = None

    def trip(self, reason):
        self._trip(reason)
        return self.status()

    def _trip(self, reason):
        self.mode = 'FAULT'
        self.reason = reason
        self._fault = True
        self._clear_history()

    def _check_clocks(self, now, source_now):
        now = finite_number(now, 'monotonic time')
        source_now = finite_number(source_now, 'source clock')
        if now < 0 or source_now <= 0:
            raise ValueError('clock not initialized')
        if self._last_now is not None and now < self._last_now:
            raise ValueError('monotonic clock moved backwards')
        if self._last_source_now is not None and source_now < self._last_source_now:
            # Rebaseline the clock for recovery, but invalidate the current session.
            self._last_source_now = source_now
            raise ValueError('source clock moved backwards')
        self._last_now = now
        self._last_source_now = source_now

    def _is_fresh(self, now, source_now):
        return (self._last_receive is not None
                and now - self._last_receive <= self.config.timeout_sec
                and -self.config.future_tolerance_sec
                <= source_now - self._last_stamp <= self.config.timeout_sec)

    def tick(self, now, source_now):
        """Call periodically using wall/monotonic time, even if ROS time stops."""
        try:
            self._check_clocks(now, source_now)
            if self._last_receive is not None and not self._is_fresh(now, source_now):
                self._trip('localization timeout or source clock discontinuity')
        except ValueError as error:
            self._trip(str(error))
        return self.status()

    def observe(self, stamp, position, orientation, frame_id, child_frame_id,
                now, source_now):
        try:
            self._check_clocks(now, source_now)
            stamp = finite_number(stamp, 'source stamp')
            if stamp <= 0:
                raise ValueError('zero or negative source stamp')
            if not (-self.config.future_tolerance_sec
                    <= source_now - stamp <= self.config.timeout_sec):
                raise ValueError('stale or future source stamp')
            if frame_id != self.config.frame_id or child_frame_id != self.config.child_frame_id:
                raise ValueError('unexpected localization frame')
            p = vector(position, 3, 'position')
            attached = attachment_position(p, orientation, self.config.attachment_offset)
            distance = math.hypot(*(a - b for a, b in zip(attached, self.config.anchor)))
            finite_number(distance, 'distance')
            if self._last_stamp is not None:
                dt = stamp - self._last_stamp
                if dt <= 0:
                    raise ValueError('duplicate or out-of-order source stamp')
                if dt > self.config.timeout_sec or not self._is_fresh(now, source_now):
                    raise ValueError('localization gap')
                displacement = math.hypot(*(a - b for a, b in zip(p, self._last_position)))
                limit = self.config.position_jump_slack_m + self.config.max_position_speed_mps * dt
                if displacement > limit:
                    raise ValueError('localization position jump')
            self._last_stamp, self._last_receive, self._last_position = stamp, now, p
            self._distance = distance
            self._samples.append((stamp, distance))
            while self._samples and stamp - self._samples[0][0] > self.config.window_sec + 1e-9:
                self._samples.popleft()
            self._velocity = self._slope()
            if self.mode in ('LOW', 'HIGH'):
                if self._velocity is None:
                    # A rate drop must not keep a previous HIGH command alive.
                    self._trip('insufficient localization samples while enabled')
                else:
                    self._select_level(stamp)
        except (ValueError, TypeError, OverflowError) as error:
            self._trip(str(error))
        return self.status()

    def _slope(self):
        if len(self._samples) < self.config.min_samples:
            return None
        # Subtract the first stamp before regression to avoid epoch cancellation.
        start = self._samples[0][0]
        times = [s[0] - start for s in self._samples]
        mean_t = sum(times) / len(times)
        mean_d = sum(s[1] for s in self._samples) / len(times)
        denominator = sum((t - mean_t) ** 2 for t in times)
        if denominator < 1e-12:
            return None
        slope = sum((t - mean_t) * (s[1] - mean_d)
                    for t, s in zip(times, self._samples)) / denominator
        return finite_number(slope, 'radial velocity')

    def _select_level(self, stamp):
        if self._distance <= self.config.near_anchor_m:
            self.mode, self.reason = 'LOW', 'near anchor: high level inhibited'
            self._candidate_since = None
        elif self.mode == 'HIGH':
            if self._velocity >= self.config.exit_speed_mps:
                self.mode, self.reason = 'LOW', 'approach ended'
                self._candidate_since = None
        elif self._velocity < self.config.enter_speed_mps:
            if self._candidate_since is None:
                self._candidate_since = stamp
            if stamp - self._candidate_since + 1e-9 >= self.config.enter_hold_sec:
                self.mode, self.reason = 'HIGH', 'sustained approach'
        else:
            self._candidate_since = None
            self.reason = 'stationary, departing, or within deadband'

    def enable(self, now, source_now):
        self.tick(now, source_now)
        if self.mode in ('LOW', 'HIGH'):
            return True, 'already enabled'
        if self._velocity is None or not self._is_fresh(now, source_now):
            return False, 'need enough fresh localization samples'
        self._fault = False
        self.mode, self.reason = 'LOW', 'enabled at low level'
        self._candidate_since = None
        return True, self.reason

    def disable(self):
        self._candidate_since = None
        if not self._fault:
            self.mode, self.reason = 'DISABLED', 'automatic control disabled'
        return self.status()

    def status(self):
        enabled = self.mode in ('LOW', 'HIGH')
        duty = None
        if enabled:
            duty = self.config.high_duty if self.mode == 'HIGH' else self.config.low_duty
        return Status(self.mode, enabled, self._velocity is not None,
                      self.reason, self._distance, self._velocity, duty)
