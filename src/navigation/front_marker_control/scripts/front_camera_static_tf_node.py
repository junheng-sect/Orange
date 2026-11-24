#!/usr/bin/env python3
"""Publish the static camera TF from YAML parameters.

Replaces the launch-command-line static_transform_publisher so the transform
values live in config/front_camera_tf.yaml. Publishes the same TF edge and
transform as before; invalid configuration (non-finite values, non-unit
quaternion, empty frames) is rejected at startup.
"""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def _finite_vector(name, values):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError('{} must contain exactly three values'.format(name))
    vector = [float(value) for value in values]
    if not all(math.isfinite(value) for value in vector):
        raise ValueError('{} values must be finite'.format(name))
    return vector


def _unit_quaternion(values):
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        raise ValueError('rotation_quaternion must contain exactly four values')
    quaternion = [float(value) for value in values]
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError('rotation_quaternion values must be finite')
    norm_squared = sum(value * value for value in quaternion)
    if not math.isfinite(norm_squared) or abs(norm_squared - 1.0) > 1.0e-3:
        raise ValueError(
            'rotation_quaternion must be a unit quaternion (norm_squared={:.6f})'.format(
                norm_squared
            )
        )
    return quaternion


def main():
    rospy.init_node('front_camera_static_transform')

    base_frame = str(rospy.get_param('~base_frame', 'drone_base')).strip()
    camera_frame = str(
        rospy.get_param('~camera_frame', 'usb_camera_optical_frame')
    ).strip()
    if not base_frame or not camera_frame:
        raise ValueError('base_frame and camera_frame must not be empty')
    if base_frame == camera_frame:
        raise ValueError('base_frame and camera_frame must differ')

    translation = _finite_vector(
        '~translation', rospy.get_param('~translation', [0.18, 0.0, 0.0])
    )
    rotation = _unit_quaternion(rospy.get_param('~rotation_quaternion', [-0.5, 0.5, -0.5, 0.5]))

    transform = TransformStamped()
    transform.header.frame_id = base_frame
    transform.child_frame_id = camera_frame
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = rotation[0]
    transform.transform.rotation.y = rotation[1]
    transform.transform.rotation.z = rotation[2]
    transform.transform.rotation.w = rotation[3]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transform)
    rospy.loginfo(
        'front_camera_static_transform: %s -> %s | translation=(%.4f, %.4f, %.4f) '
        '| quaternion=(%.4f, %.4f, %.4f, %.4f)',
        base_frame,
        camera_frame,
        translation[0],
        translation[1],
        translation[2],
        rotation[0],
        rotation[1],
        rotation[2],
        rotation[3],
    )
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except (KeyError, TypeError, ValueError) as exception:
        rospy.logfatal('front_camera_static_transform failed: %s', exception)
        raise
