"""Proportional control and command limiting primitives."""

import math


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    angle = float(angle)
    if not math.isfinite(angle):
        raise ValueError('angle must be finite')
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class FirstOrderAngleLowPass(object):
    """Sample-driven first-order low-pass using the shortest angular delta."""

    def __init__(self, tau):
        self.tau = float(tau)
        if not math.isfinite(self.tau) or self.tau < 0.0:
            raise ValueError('low-pass tau must be finite and non-negative')
        self.reset()

    def reset(self):
        self.value = None

    def update(self, angle, dt):
        angle = wrap_angle(angle)
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('low-pass requires finite positive dt')
        if self.value is None or self.tau == 0.0:
            self.value = angle
        else:
            alpha = dt / (self.tau + dt)
            self.value = wrap_angle(
                self.value + alpha * wrap_angle(angle - self.value)
            )
        return self.value


class FirstOrderLowPass(object):
    """Sample-driven first-order low-pass for finite scalar values."""

    def __init__(self, tau):
        self.tau = float(tau)
        if not math.isfinite(self.tau) or self.tau < 0.0:
            raise ValueError('low-pass tau must be finite and non-negative')
        self.reset()

    def reset(self):
        self.value = None

    def update(self, value, dt):
        value = float(value)
        dt = float(dt)
        if not math.isfinite(value):
            raise ValueError('low-pass value must be finite')
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('low-pass requires finite positive dt')
        if self.value is None or self.tau == 0.0:
            self.value = value
        else:
            alpha = dt / (self.tau + dt)
            self.value += alpha * (value - self.value)
        return self.value


def apply_deadband(value, deadband):
    return 0.0 if abs(float(value)) < abs(float(deadband)) else float(value)


class HysteresisDeadband(object):
    """Stateful deadband with separate thresholds for stopping and starting."""

    def __init__(self, enter_threshold, exit_threshold):
        self.enter_threshold = float(enter_threshold)
        self.exit_threshold = float(exit_threshold)
        if (
            not math.isfinite(self.enter_threshold)
            or not math.isfinite(self.exit_threshold)
        ):
            raise ValueError('deadband thresholds must be finite')
        if self.enter_threshold < 0.0 or self.exit_threshold < 0.0:
            raise ValueError('deadband thresholds must be non-negative')
        if self.enter_threshold > self.exit_threshold:
            raise ValueError(
                'deadband enter threshold must not exceed exit threshold'
            )
        self.reset()

    def reset(self):
        self.active = False

    def update(self, value):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('deadband value must be finite')

        magnitude = abs(value)
        if self.active:
            if magnitude <= self.enter_threshold:
                self.active = False
        elif magnitude >= self.exit_threshold:
            self.active = True
        return value if self.active else 0.0


def limit_vector2(first, second, limit):
    norm = math.hypot(float(first), float(second))
    limit = abs(float(limit))
    if norm > limit and norm > 1e-12:
        scale = limit / norm
        return float(first) * scale, float(second) * scale
    return float(first), float(second)


class Vector3SlewRateLimiter(object):
    """Limit the Euclidean rate of change of a three-dimensional vector."""

    def __init__(self):
        self.reset()

    @staticmethod
    def _vector3(value, name):
        if len(value) != 3:
            raise ValueError('%s must contain three values' % name)
        vector = tuple(float(component) for component in value)
        if not all(math.isfinite(component) for component in vector):
            raise ValueError('%s must contain finite values' % name)
        return vector

    def reset(self, value=(0.0, 0.0, 0.0)):
        self.value = self._vector3(value, 'reset value')
        return self.value

    def update(self, target, max_rate, dt):
        target = self._vector3(target, 'target')
        max_rate = float(max_rate)
        dt = float(dt)
        if not math.isfinite(max_rate) or max_rate <= 0.0:
            raise ValueError('max_rate must be finite and positive')
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('slew-rate limiter requires finite positive dt')

        delta = tuple(target[index] - self.value[index] for index in range(3))
        delta_norm = math.sqrt(sum(component * component for component in delta))
        max_delta = max_rate * dt
        if delta_norm <= max_delta or delta_norm <= 1e-12:
            self.value = target
        else:
            scale = max_delta / delta_norm
            self.value = tuple(
                self.value[index] + delta[index] * scale for index in range(3)
            )
        return self.value


class BodyVelocityController(object):
    """Sample-driven FLU position/yaw P controller producing a bounded velocity command."""

    def __init__(
        self,
        kp_x,
        kp_y,
        kp_z,
        kp_yaw,
        vx_limit,
        vyz_limit,
        yaw_rate_limit,
        velocity_deadband,
        z_error_deadband,
        yaw_error_deadband,
        yaw_lpf_tau,
        enable_yaw_control=True,
        velocity_deadband_enter=0.02,
        position_lpf_tau_x=0.05,
        position_lpf_tau_yz=0.10,
    ):
        self.kp_x = self._proportional_gain(kp_x, 'kp_x')
        self.kp_y = self._proportional_gain(kp_y, 'kp_y')
        self.kp_z = self._proportional_gain(kp_z, 'kp_z')
        self.kp_yaw = self._proportional_gain(kp_yaw, 'kp_yaw')
        self.vx_limit = abs(float(vx_limit))
        self.vyz_limit = abs(float(vyz_limit))
        self.yaw_rate_limit = abs(float(yaw_rate_limit))
        self.velocity_deadband = float(velocity_deadband)
        self.velocity_deadband_enter = float(velocity_deadband_enter)
        self.z_error_deadband = abs(float(z_error_deadband))
        self.yaw_error_deadband = abs(float(yaw_error_deadband))
        self.enable_yaw_control = bool(enable_yaw_control)
        self.position_filters = (
            FirstOrderLowPass(position_lpf_tau_x),
            FirstOrderLowPass(position_lpf_tau_yz),
            FirstOrderLowPass(position_lpf_tau_yz),
        )
        self.yaw_filter = FirstOrderAngleLowPass(yaw_lpf_tau)
        self.velocity_deadband_filters = tuple(
            HysteresisDeadband(
                self.velocity_deadband_enter,
                self.velocity_deadband,
            )
            for _ in range(3)
        )

    @staticmethod
    def _proportional_gain(value, name):
        gain = float(value)
        if not math.isfinite(gain) or gain < 0.0:
            raise ValueError('%s must be finite and non-negative' % name)
        return gain

    def reset(self):
        for position_filter in self.position_filters:
            position_filter.reset()
        self.yaw_filter.reset()
        for deadband_filter in self.velocity_deadband_filters:
            deadband_filter.reset()

    def update(self, position_error_body, yaw_error, dt):
        if len(position_error_body) != 3:
            raise ValueError('position_error_body must contain three values')
        error_x, error_y, error_z = (
            float(value) for value in position_error_body
        )
        if not all(math.isfinite(value) for value in (error_x, error_y, error_z)):
            raise ValueError('position_error_body must contain finite values')
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('controller requires finite positive dt')

        filtered_error_x, filtered_error_y, filtered_error_z = tuple(
            position_filter.update(value, dt)
            for position_filter, value in zip(
                self.position_filters,
                (error_x, error_y, error_z),
            )
        )

        vx = max(
            -self.vx_limit,
            min(self.vx_limit, self.kp_x * filtered_error_x),
        )
        raw_vy = self.kp_y * filtered_error_y
        raw_vz = (
            0.0
            if abs(filtered_error_z) < self.z_error_deadband
            else self.kp_z * filtered_error_z
        )
        vy, vz = limit_vector2(raw_vy, raw_vz, self.vyz_limit)
        body_velocity = tuple(
            deadband_filter.update(value)
            for deadband_filter, value in zip(
                self.velocity_deadband_filters,
                (vx, vy, vz),
            )
        )

        filtered_yaw_error = self.yaw_filter.update(yaw_error, dt)
        if not self.enable_yaw_control:
            yaw_rate = 0.0
        elif abs(filtered_yaw_error) < self.yaw_error_deadband:
            yaw_rate = 0.0
        else:
            yaw_rate = max(
                -self.yaw_rate_limit,
                min(self.yaw_rate_limit, self.kp_yaw * filtered_yaw_error),
            )

        return body_velocity, yaw_rate, filtered_yaw_error
