"""Rigid-transform helpers for front-camera tracking."""

import math

import numpy as np


# Upright forward-facing USB camera optical frame expressed in FLU.
# This is the fixed REP-103 axis permutation, not a measured cam-IMU extrinsic.
DEFAULT_BODY_R_CAMERA_OPTICAL = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)

# The default alignment deliberately ignores the measured camera lever arm.
DEFAULT_BODY_T_CAMERA_OPTICAL = np.zeros(3, dtype=np.float64)


# Retained for regression coverage of the explicit legacy stereo configs.
CALIBRATED_CAM1_BODY_R_CAMERA_OPTICAL = np.array(
    [
        [-0.01394793768528143, 0.00009780706708639623, 0.99990271800216],
        [-0.9998798106485599, -0.006771017692188367, -0.013946955826041775],
        [0.006768994883215791, -0.99997707161374, 0.00019223704455495892],
    ],
    dtype=np.float64,
)
CALIBRATED_CAM1_BODY_T_CAMERA_OPTICAL = np.array(
    [0.03671416066693577, -0.03152979437303069, -0.0032290058260840346],
    dtype=np.float64,
)


def validate_vector3(values, name='vector'):
    vector = np.asarray(list(values), dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError('%s must contain three finite values' % name)
    return vector


def validate_rotation(rotation):
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError('rotation must be a finite 3x3 matrix')
    if not np.allclose(matrix.T.dot(matrix), np.eye(3), atol=2e-3):
        raise ValueError('rotation is not orthonormal')
    if not math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=2e-3):
        raise ValueError('rotation determinant is not +1')
    return matrix


def validate_homogeneous(matrix):
    transform = np.asarray(matrix, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError('homogeneous transform must be 4x4')
    if not np.all(np.isfinite(transform)):
        raise ValueError('homogeneous transform contains non-finite values')
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError('homogeneous transform has an invalid last row')
    validate_rotation(transform[:3, :3])
    return transform


def row_major_matrix(values):
    array = np.asarray(list(values), dtype=np.float64)
    if array.size != 16:
        raise ValueError('matrix message must contain 16 values')
    return validate_homogeneous(array.reshape(4, 4))


def body_target_to_camera(
    target_xyz_body,
    body_r_camera_optical,
    body_t_camera_optical,
):
    """Express a desired Marker position from body FLU in camera optical."""
    target_body = validate_vector3(target_xyz_body, 'target_xyz_body')
    rotation = validate_rotation(body_r_camera_optical)
    translation = validate_vector3(
        body_t_camera_optical, 'body_t_camera_optical'
    )
    return rotation.T.dot(target_body - translation)


def body_direction_to_camera(body_direction, body_r_camera_optical):
    """Rotate a free direction vector from body FLU into camera optical."""
    direction = validate_vector3(body_direction, 'body_direction')
    rotation = validate_rotation(body_r_camera_optical)
    return rotation.T.dot(direction)


def camera_marker_to_body(
    camera_t_marker,
    body_r_camera_optical,
    body_t_camera_optical,
):
    """Compose ^body T_camera with ^camera T_marker."""
    camera_marker = validate_homogeneous(camera_t_marker)
    rotation = validate_rotation(body_r_camera_optical)
    translation = validate_vector3(
        body_t_camera_optical, 'body_t_camera_optical'
    )
    body_camera = np.eye(4, dtype=np.float64)
    body_camera[:3, :3] = rotation
    body_camera[:3, 3] = translation
    return validate_homogeneous(body_camera.dot(camera_marker))


def tracking_errors(
    camera_t_marker,
    target_xyz_camera=(0.0, 0.0, 1.0),
    target_facing_direction_camera=(0.0, 0.0, 1.0),
):
    transform = validate_homogeneous(camera_t_marker)
    target = validate_vector3(target_xyz_camera, 'target_xyz_camera')
    target_direction = validate_vector3(
        target_facing_direction_camera, 'target_facing_direction_camera'
    )
    if math.hypot(float(target_direction[0]), float(target_direction[2])) <= 1e-12:
        raise ValueError(
            'target_facing_direction_camera must have a non-zero x-z projection'
        )
    position_error = transform[:3, 3] - target
    marker_normal_camera = transform[:3, :3].dot(np.array([0.0, 0.0, 1.0]))
    desired_direction_camera = -marker_normal_camera
    observed_yaw_camera = math.atan2(
        float(desired_direction_camera[0]),
        float(desired_direction_camera[2]),
    )
    target_yaw_camera = math.atan2(
        float(target_direction[0]),
        float(target_direction[2]),
    )
    yaw_delta = target_yaw_camera - observed_yaw_camera
    yaw_error = math.atan2(math.sin(yaw_delta), math.cos(yaw_delta))
    return position_error, yaw_error, desired_direction_camera


def camera_velocity_to_body(camera_velocity, body_r_camera_optical):
    velocity = validate_vector3(camera_velocity, 'camera_velocity')
    rotation = validate_rotation(body_r_camera_optical)
    return rotation.dot(velocity)


def rotation_to_quaternion(rotation):
    matrix = validate_rotation(rotation)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError('rotation produced a zero-length quaternion')
    quaternion /= norm
    return tuple(float(value) for value in quaternion)
