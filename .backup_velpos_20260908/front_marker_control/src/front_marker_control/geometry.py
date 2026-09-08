"""Validation and geometry helpers for standard PoseStamped input."""

import math

from front_marker_control.control import wrap_angle


def finite_vector3(values, name='vector'):
    values = tuple(float(value) for value in values)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('%s must contain three finite values' % name)
    return values


def normalize_quaternion_values(values):
    """Normalize a four-value quaternion sequence; raise on invalid input."""
    values = tuple(float(value) for value in values)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion contains a non-finite value')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError('quaternion has zero length')
    return tuple(value / norm for value in values)


def normalize_quaternion(quaternion):
    return normalize_quaternion_values(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    )


def yaw_from_quaternion(values):
    """Return the rotation about z of a quaternion sequence, in [-pi, pi)."""
    x, y, z, w = normalize_quaternion_values(values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotate_vector_by_quaternion(values, vector):
    """Rotate a three-vector by a quaternion sequence (both body->local)."""
    x, y, z, w = normalize_quaternion_values(values)
    vx, vy, vz = finite_vector3(vector, 'vector')
    # v' = q v q^-1 with t = 2 * q_vec x v: v' = v + w*t + q_vec x t.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def rotate_vector_by_yaw(vector, yaw):
    """Rotate a three-vector about the z-axis by ``yaw``."""
    vx, vy, vz = finite_vector3(vector, 'vector')
    yaw = float(yaw)
    if not math.isfinite(yaw):
        raise ValueError('yaw must be finite')
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (vx * cosine - vy * sine, vx * sine + vy * cosine, vz)


def position_hold_target(
    p_body,
    quaternion_body,
    p_marker_body,
    marker_yaw_error,
    target_xyz_body,
    apply_yaw_correction=True,
):
    """Compute the locked local-frame hold position and yaw.

    ``p_body``/``quaternion_body``: body pose in the local ENU frame.
    ``p_marker_body``: Marker position in the body frame.
    ``marker_yaw_error``: body-frame heading error of the Marker normal.
    ``target_xyz_body``: hold offset from the Marker, expressed in the body
    frame at the target yaw.

    Returns ``(p_target, yaw_target)`` where ``p_target = p_marker -
    Rz(yaw_target) * target_xyz_body`` and ``yaw_target`` wraps the local yaw
    plus the Marker heading error (plain local yaw when yaw correction is
    disabled).
    """
    p_body = finite_vector3(p_body, 'p_body')
    p_marker_body = finite_vector3(p_marker_body, 'p_marker_body')
    target_xyz_body = finite_vector3(target_xyz_body, 'target_xyz_body')
    marker_yaw_error = float(marker_yaw_error)
    if not math.isfinite(marker_yaw_error):
        raise ValueError('marker_yaw_error must be finite')
    quaternion_body = normalize_quaternion_values(quaternion_body)

    yaw_target = yaw_from_quaternion(quaternion_body)
    if apply_yaw_correction:
        yaw_target = wrap_angle(yaw_target + marker_yaw_error)

    marker_offset_local = rotate_vector_by_quaternion(
        quaternion_body, p_marker_body
    )
    p_marker = tuple(
        p_body[index] + marker_offset_local[index] for index in range(3)
    )
    hold_offset_local = rotate_vector_by_yaw(target_xyz_body, yaw_target)
    p_target = tuple(
        p_marker[index] - hold_offset_local[index] for index in range(3)
    )
    return p_target, yaw_target


def marker_heading_body(quaternion):
    """Return heading of the direction from body to the Marker in body FLU."""
    x, y, z, w = normalize_quaternion(quaternion)
    marker_normal_x = 2.0 * (x * z + w * y)
    marker_normal_y = 2.0 * (y * z - w * x)
    direction_x = -marker_normal_x
    direction_y = -marker_normal_y
    if math.hypot(direction_x, direction_y) <= 1e-12:
        raise ValueError('marker normal has no horizontal projection')
    return math.atan2(direction_y, direction_x)


def position_error_body(pose, target_xyz_body):
    target = finite_vector3(target_xyz_body, 'target_xyz_body')
    observed = finite_vector3(
        (pose.position.x, pose.position.y, pose.position.z),
        'marker position',
    )
    return tuple(observed[index] - target[index] for index in range(3))
