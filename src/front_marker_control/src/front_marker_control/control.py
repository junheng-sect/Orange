"""PID and command limiting primitives."""

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


class PIDController(object):
    """PID with explicit dt, conditional integration and output clamping."""

    def __init__(self, kp, ki, kd, output_limit=None, integral_limit=1.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = None if output_limit is None else abs(float(output_limit))
        self.integral_limit = float(integral_limit)
        if not math.isfinite(self.integral_limit) or self.integral_limit < 0.0:
            raise ValueError('integral limit must be finite and non-negative')
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error, dt):
        error = float(error)
        dt = float(dt)
        if not math.isfinite(error) or not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('PID requires finite error and positive dt')
        previous_integral = self.integral
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = 0.0 if self.previous_error is None else (
            error - self.previous_error
        ) / dt
        raw_output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = raw_output
        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))
            if output != raw_output and error * raw_output > 0.0:
                self.integral = previous_integral
        self.previous_error = error
        return output


def apply_deadband(value, deadband):
    return 0.0 if abs(float(value)) < abs(float(deadband)) else float(value)


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
    """Sample-driven FLU position/yaw PID producing a bounded velocity command."""

    def __init__(
        self,
        kp_x,
        ki_x,
        kd_x,
        kp_yz,
        ki_yz,
        kd_yz,
        kp_yaw,
        ki_yaw,
        kd_yaw,
        vx_limit,
        vyz_limit,
        yaw_rate_limit,
        velocity_deadband,
        z_error_deadband,
        yaw_error_deadband,
        yaw_lpf_tau,
        enable_yaw_control=True,
        integral_limit_x=1.0,
        integral_limit_yz=1.0,
        integral_limit_yaw=1.0,
    ):
        self.vyz_limit = abs(float(vyz_limit))
        self.velocity_deadband = abs(float(velocity_deadband))
        self.z_error_deadband = abs(float(z_error_deadband))
        self.yaw_error_deadband = abs(float(yaw_error_deadband))
        self.enable_yaw_control = bool(enable_yaw_control)
        self.pid_x = PIDController(
            kp_x,
            ki_x,
            kd_x,
            output_limit=abs(float(vx_limit)),
            integral_limit=integral_limit_x,
        )
        self.pid_y = PIDController(
            kp_yz, ki_yz, kd_yz, integral_limit=integral_limit_yz
        )
        self.pid_z = PIDController(
            kp_yz, ki_yz, kd_yz, integral_limit=integral_limit_yz
        )
        self.pid_yaw = PIDController(
            kp_yaw,
            ki_yaw,
            kd_yaw,
            output_limit=abs(float(yaw_rate_limit)),
            integral_limit=integral_limit_yaw,
        )
        self.yaw_filter = FirstOrderAngleLowPass(yaw_lpf_tau)

    def reset(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()
        self.yaw_filter.reset()

    def update(self, position_error_body, yaw_error, dt):
        if len(position_error_body) != 3:
            raise ValueError('position_error_body must contain three values')

        vx = self.pid_x.update(float(position_error_body[0]), dt)
        error_y = float(position_error_body[1])
        error_z = float(position_error_body[2])
        previous_integral_y = self.pid_y.integral
        previous_integral_z = self.pid_z.integral
        raw_vy = self.pid_y.update(error_y, dt)

        if abs(error_z) < self.z_error_deadband:
            self.pid_z.reset()
            raw_vz = 0.0
        else:
            raw_vz = self.pid_z.update(error_z, dt)

        vy, vz = limit_vector2(raw_vy, raw_vz, self.vyz_limit)
        if math.hypot(raw_vy, raw_vz) > self.vyz_limit:
            if error_y * raw_vy > 0.0:
                self.pid_y.integral = previous_integral_y
            if error_z * raw_vz > 0.0:
                self.pid_z.integral = previous_integral_z
        body_velocity = (
            apply_deadband(vx, self.velocity_deadband),
            apply_deadband(vy, self.velocity_deadband),
            apply_deadband(vz, self.velocity_deadband),
        )

        filtered_yaw_error = self.yaw_filter.update(yaw_error, dt)
        if not self.enable_yaw_control:
            self.pid_yaw.reset()
            yaw_rate = 0.0
        elif abs(filtered_yaw_error) < self.yaw_error_deadband:
            self.pid_yaw.reset()
            yaw_rate = 0.0
        else:
            yaw_rate = self.pid_yaw.update(filtered_yaw_error, dt)

        return body_velocity, yaw_rate, filtered_yaw_error
