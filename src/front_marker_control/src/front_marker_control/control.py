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
    """PID with explicit dt, integral clamp and optional output clamp."""

    def __init__(self, kp, ki, kd, output_limit=None, integral_limit=1.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = None if output_limit is None else abs(float(output_limit))
        self.integral_limit = abs(float(integral_limit))
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error, dt):
        error = float(error)
        dt = float(dt)
        if not math.isfinite(error) or not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('PID requires finite error and positive dt')
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = 0.0 if self.previous_error is None else (
            error - self.previous_error
        ) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))
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
        v_limit,
        yaw_rate_limit,
        velocity_deadband,
        z_error_deadband,
        yaw_error_deadband,
        yaw_lpf_tau,
        enable_yaw_control=True,
    ):
        self.v_limit = abs(float(v_limit))
        self.velocity_deadband = abs(float(velocity_deadband))
        self.z_error_deadband = abs(float(z_error_deadband))
        self.yaw_error_deadband = abs(float(yaw_error_deadband))
        self.enable_yaw_control = bool(enable_yaw_control)
        self.pid_x = PIDController(
            kp_x, ki_x, kd_x, output_limit=abs(float(vx_limit))
        )
        self.pid_y = PIDController(kp_yz, ki_yz, kd_yz)
        self.pid_z = PIDController(kp_yz, ki_yz, kd_yz)
        self.pid_yaw = PIDController(
            kp_yaw,
            ki_yaw,
            kd_yaw,
            output_limit=abs(float(yaw_rate_limit)),
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
        vy = self.pid_y.update(float(position_error_body[1]), dt)

        if abs(float(position_error_body[2])) < self.z_error_deadband:
            self.pid_z.reset()
            vz = 0.0
        else:
            vz = self.pid_z.update(float(position_error_body[2]), dt)

        vy, vz = limit_vector2(vy, vz, self.v_limit)
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
