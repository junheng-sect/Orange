"""Source-time paired final-pose refinement. Pure Python; caller serializes access."""
import math
from collections import deque

from front_marker_control.control import wrap_angle
from front_marker_control.geometry import (
    rotate_vector_by_quaternion, rotate_vector_by_yaw, yaw_from_quaternion,
)

DEFAULTS = {
    'yaw_sampling_duration_sec': 3.0,
    'yaw_sampling_min_samples': 60,
    'yaw_sampling_outlier_deg': 15.0,
    'yaw_sampling_min_inlier_ratio': 0.80,
    'yaw_sampling_max_std_deg': 6.0,
    'yaw_refinement_max_correction_deg': 20.0,
    'yaw_sample_pending_timeout_sec': 0.50,
    'refine_speed_gate_mps': 0.05,
    'refine_stable_hold_sec': 1.0,
    'pose_transition_speed_mps': 0.10,
    'pose_transition_accel_mps2': 0.50,
    'pose_transition_yaw_rate_deg_s': 10.0,
    'pose_settle_yaw_tolerance_deg': 2.0,
    'yaw_acceptance_tolerance_deg': 3.0,
}


def validate_parameters(parameters):
    for name in DEFAULTS:
        value = parameters[name]
        if not math.isfinite(value) or value <= 0:
            raise ValueError('%s must be finite and positive' % name)
    count = parameters['yaw_sampling_min_samples']
    if count != int(count) or count < 2:
        raise ValueError('yaw_sampling_min_samples must be an integer >= 2')
    if parameters['yaw_sampling_min_inlier_ratio'] > 1:
        raise ValueError('yaw_sampling_min_inlier_ratio must be <= 1')
    for name in ('yaw_sampling_outlier_deg', 'yaw_sampling_max_std_deg',
                 'yaw_refinement_max_correction_deg', 'yaw_acceptance_tolerance_deg',
                 'pose_settle_yaw_tolerance_deg'):
        if parameters[name] >= 180:
            raise ValueError('%s must be below 180 degrees' % name)


def robust_angles(angles, parameters):
    """Circular L1 medoid, fixed-angle trimming, then circular mean/std.

    Return indices as well so position and heading use exactly the same samples.
    """
    if not angles:
        return None, float('inf'), [], 'empty window'
    median = min(angles, key=lambda a: sum(abs(wrap_angle(b - a)) for b in angles))
    limit = math.radians(parameters['yaw_sampling_outlier_deg'])
    indices = [i for i, a in enumerate(angles) if abs(wrap_angle(a - median)) <= limit]
    cosine = sum(math.cos(angles[i]) for i in indices) / len(indices)
    sine = sum(math.sin(angles[i]) for i in indices) / len(indices)
    length = min(1.0, math.hypot(cosine, sine))
    deviation = math.sqrt(-2 * math.log(max(length, 1e-15)))
    mean = math.atan2(sine, cosine)
    reason = ''
    if len(indices) < parameters['yaw_sampling_min_samples']:
        reason = 'insufficient inliers'
    elif len(indices) / len(angles) < parameters['yaw_sampling_min_inlier_ratio']:
        reason = 'inlier ratio'
    elif deviation > math.radians(parameters['yaw_sampling_max_std_deg']):
        reason = 'circular std'
    return mean, deviation, indices, reason


class PoseTransition:
    """Straight-line rest-to-rest trapezoid and independent shortest-path yaw slew."""
    def __init__(self, position, yaw, target, parameters):
        self.start = tuple(position)
        self.yaw_start = yaw
        self.target = target
        self.delta = tuple(b - a for a, b in zip(position, target[0]))
        self.distance = math.sqrt(sum(v*v for v in self.delta))
        self.accel = parameters['pose_transition_accel_mps2']
        self.speed = min(parameters['pose_transition_speed_mps'],
                         math.sqrt(self.distance * self.accel))
        self.ramp = self.speed / self.accel
        self.cruise = max(0.0, self.distance / self.speed - self.ramp) if self.speed else 0.0
        self.duration = 2*self.ramp + self.cruise
        self.yaw_delta = wrap_angle(target[1] - yaw)
        self.yaw_rate = math.radians(parameters['pose_transition_yaw_rate_deg_s'])
        self.time = 0.0
        self.done = False
        self.command = (self.start, yaw)

    def step(self, dt):
        self.time += dt
        t = min(self.time, self.duration)
        if t < self.ramp:
            distance = 0.5*self.accel*t*t
        elif t < self.ramp + self.cruise:
            distance = 0.5*self.speed*self.ramp + self.speed*(t-self.ramp)
        else:
            distance = self.distance - 0.5*self.accel*(self.duration-t)**2
        fraction = min(1.0, max(0.0, distance/self.distance)) if self.distance else 1.0
        position = tuple(a + fraction*d for a, d in zip(self.start, self.delta))
        yaw_step = min(abs(self.yaw_delta), self.yaw_rate*self.time)
        yaw = wrap_angle(self.yaw_start + math.copysign(yaw_step, self.yaw_delta))
        self.done = self.time >= self.duration and yaw_step >= abs(self.yaw_delta)
        self.command = (position, yaw)
        return self.command


class FinalPoseRefiner:
    def __init__(self, parameters, offset, tolerances, max_gap, pose_timeout):
        self.p = dict(parameters)
        validate_parameters(self.p)
        self.offset = tuple(offset)
        self.tolerances = tuple(tolerances)
        self.max_gap = max_gap
        self.pose_timeout = pose_timeout
        self.reason = ''
        self.stats = 'samples=0 span=0 std=n/a correction=n/a residual=n/a'
        self.generation = 0
        self.reset('initial')

    def reset(self, reason):
        self.mode = 'ALIGN'
        self.target = None
        self.target_stamp = None
        self.transition = None
        self.hold_yaw = None
        self.stable_since = None
        self.last_tick = None
        self.clear_window(reason)

    def clear_window(self, reason='', epoch=float('inf')):
        self.pending = deque()
        self.samples = []
        self.last_marker_stamp = None
        self.last_pair_stamp = None
        self.last_pair_receive = None
        self.epoch = epoch
        self.reason = reason

    def visual_interruption(self, reason):
        if self.mode in ('ALIGN', 'YAW_SAMPLING'):
            self.reset(reason)
        else:
            # Local positioning remains valid, but vision has not verified it.
            self.clear_window(reason)
            self.stable_since = None

    def within(self, error):
        return all(abs(v) <= limit for v, limit in zip(error, self.tolerances))

    def marker(self, stamp, received, position_body, yaw_error):
        if stamp < self.epoch or received < self.epoch:
            return
        if self.last_marker_stamp is not None and stamp <= self.last_marker_stamp:
            return
        self.last_marker_stamp = stamp
        self.pending.append((stamp, received, tuple(position_body), yaw_error))
        # Bounded even with a stalled odometry producer / unusually high input rate.
        if len(self.pending) > 256:
            self.visual_interruption('pending queue overflow')

    def process_pairs(self, now, buffer):
        while self.pending:
            stamp, received, body, error = self.pending[0]
            if now - received > self.p['yaw_sample_pending_timeout_sec']:
                self.visual_interruption('pair timeout')
                return
            pose = buffer.lookup(stamp, self.max_gap)
            if pose is None:
                if buffer.newest_stamp is not None and buffer.newest_stamp >= stamp:
                    self.visual_interruption('source interpolation unavailable')
                return
            self.pending.popleft()
            if self.last_pair_stamp is not None and stamp-self.last_pair_stamp > self.pose_timeout + 1e-9:
                self.visual_interruption('source pair gap')
                return
            self.last_pair_stamp = stamp
            self.last_pair_receive = now
            if self.mode == 'YAW_SAMPLING':
                angle = wrap_angle(yaw_from_quaternion(pose.quaternion) + error)
                rotated = rotate_vector_by_quaternion(pose.quaternion, body)
                marker_local = tuple(a+b for a, b in zip(pose.position, rotated))
            else:
                angle, marker_local = error, None
            self.samples.append((stamp, angle, marker_local))
            if self.samples[-1][0] - self.samples[0][0] >= self.p['yaw_sampling_duration_sec'] - 1e-9:
                self.evaluate(now)
                return  # clear_window discards pre-epoch queued samples

    def evaluate(self, now):
        angles = [s[1] for s in self.samples]
        mean, deviation, indices, reason = robust_angles(angles, self.p)
        correction = (wrap_angle(mean-self.hold_yaw)
                      if self.mode == 'YAW_SAMPLING' else mean)
        self.stats = 'samples=%d inliers=%d span=%.3f std=%.2f correction=%s residual=%s' % (
            len(angles), len(indices), self.samples[-1][0]-self.samples[0][0],
            math.degrees(deviation),
            '%.2f' % math.degrees(correction) if self.mode == 'YAW_SAMPLING' else 'n/a',
            '%.2f' % math.degrees(mean) if self.mode != 'YAW_SAMPLING' else 'n/a')
        if self.mode == 'YAW_SAMPLING':
            if not reason and abs(correction) > math.radians(self.p['yaw_refinement_max_correction_deg']):
                reason = 'correction limit'
            if reason:
                self.reset(reason)
                return
            marker = tuple(sum(self.samples[i][2][axis] for i in indices)/len(indices)
                           for axis in range(3))
            offset = rotate_vector_by_yaw(self.offset, mean)
            self.target = (tuple(a-b for a, b in zip(marker, offset)), mean)
            self.target_stamp = self.samples[-1][0]
            self.mode = 'POSE_ALIGNING'
            self.stable_since = None
            self.generation += 1
            self.clear_window('target accepted')
        elif reason:
            # A noisy window is inconclusive, never a successful verification.
            self.clear_window(reason, epoch=now)
        elif abs(mean) > math.radians(self.p['yaw_acceptance_tolerance_deg']):
            self.reset('visual yaw residual exceeds tolerance')
        else:
            self.mode = 'POS_HOLD'
            self.clear_window('visual yaw verified', epoch=now)

    def actual_pose_within(self, local_pose, speed):
        position_error = tuple(a-b for a, b in zip(self.target[0], local_pose.position))
        position_error = rotate_vector_by_yaw(position_error, -self.target[1])
        return (self.within(position_error) and speed < self.p['refine_speed_gate_mps']
                and abs(wrap_angle(self.target[1]-yaw_from_quaternion(local_pose.quaternion)))
                <= math.radians(self.p['pose_settle_yaw_tolerance_deg']))

    def tick(self, now, dt, pose_fresh, local_pose, speed, error, buffer):
        if self.last_tick is not None and now < self.last_tick:
            self.reset('clock reversal')
        self.last_tick = now
        if local_pose is None:
            self.reset('localization unavailable')
            return
        if not pose_fresh:
            self.visual_interruption('vision unavailable')
        if self.mode == 'ALIGN':
            if not pose_fresh or not self.within(error) or speed >= self.p['refine_speed_gate_mps']:
                self.stable_since = None
                return
            if self.stable_since is None:
                self.stable_since = now
            if now - self.stable_since >= self.p['refine_stable_hold_sec']:
                self.mode = 'YAW_SAMPLING'
                self.hold_yaw = yaw_from_quaternion(local_pose.quaternion)
                self.clear_window('sampling', epoch=now)
            return
        if self.mode == 'YAW_SAMPLING':
            if not self.within(error) or speed >= self.p['refine_speed_gate_mps']:
                self.reset('sampling stability lost')
                return
        if self.mode == 'POSE_ALIGNING':
            if self.transition is None:
                self.transition = PoseTransition(local_pose.position, self.hold_yaw, self.target, self.p)
            else:
                self.transition.step(dt)
            settled = (pose_fresh and self.transition.done
                       and self.actual_pose_within(local_pose, speed))
            if not settled:
                self.stable_since = None
                self.clear_window('waiting for actual pose')
            else:
                if self.stable_since is None:
                    self.stable_since = now
                if now-self.stable_since >= self.p['refine_stable_hold_sec'] and math.isinf(self.epoch):
                    self.clear_window('verifying', epoch=now)
        if self.mode == 'POS_HOLD' and pose_fresh and math.isinf(self.epoch):
            self.clear_window('monitoring', epoch=now)
        if pose_fresh:
            if (self.last_pair_receive is not None and not self.pending
                    and now-self.last_pair_receive > self.pose_timeout):
                self.visual_interruption('pair stream interrupted')
                return
            self.process_pairs(now, buffer)
            # The committing tick must already emit a position setpoint.
            if self.mode == 'POSE_ALIGNING' and self.transition is None:
                self.transition = PoseTransition(local_pose.position, self.hold_yaw, self.target, self.p)

    def status(self):
        span = self.samples[-1][0]-self.samples[0][0] if self.samples else 0.0
        return 'refine=%s n=%d span=%.3f | %s | reject=%s' % (
            self.mode, len(self.samples), span, self.stats, self.reason)
