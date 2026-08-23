"""Validation and geometry helpers for standard PoseStamped input."""

import math


def finite_vector3(values, name='vector'):
    values = tuple(float(value) for value in values)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('%s must contain three finite values' % name)
    return values


def normalize_quaternion(quaternion):
    values = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('pose quaternion contains a non-finite value')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError('pose quaternion has zero length')
    return tuple(value / norm for value in values)


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
