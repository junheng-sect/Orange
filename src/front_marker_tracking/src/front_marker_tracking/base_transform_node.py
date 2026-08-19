"""Convert camera-frame Marker observations into a configured FLU frame."""

import threading

from front_marker_tracking.msg import HomogeneousTransform
from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
from std_msgs.msg import String

from front_marker_tracking.transforms import (
    DEFAULT_BODY_R_CAMERA_OPTICAL,
    DEFAULT_BODY_T_CAMERA_OPTICAL,
    camera_marker_to_body,
    rotation_to_quaternion,
    row_major_matrix,
    validate_rotation,
    validate_vector3,
)


class FrontMarkerBaseTransformNode(object):
    """Publish a validated ^base_link T_marker without any control output."""

    def __init__(self):
        self.lock = threading.RLock()
        self._read_parameters()

        self.transform_pub = rospy.Publisher(
            self.output_transform_topic,
            HomogeneousTransform,
            queue_size=10,
        )
        self.pose_pub = rospy.Publisher(
            self.output_pose_topic,
            PoseStamped,
            queue_size=10,
        )
        self.status_pub = rospy.Publisher(
            self.status_topic,
            String,
            queue_size=10,
            latch=True,
        )
        self.transform_sub = rospy.Subscriber(
            self.input_transform_topic,
            HomogeneousTransform,
            self.transform_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        self.publish_status(
            'WAIT_MARKER: waiting for %s' % self.input_transform_topic
        )
        rospy.loginfo(
            'front_marker_base_transform_node started | input=%s camera=%s | '
            'base=%s output=%s | body_t_camera=(%.6f, %.6f, %.6f)m',
            self.input_transform_topic,
            self.expected_camera_frame,
            self.base_frame,
            self.output_transform_topic,
            self.body_t_camera_optical[0],
            self.body_t_camera_optical[1],
            self.body_t_camera_optical[2],
        )

    def _read_parameters(self):
        get = rospy.get_param
        self.input_transform_topic = str(
            get('~input_transform_topic', '/front_marker_tracking/transform')
        )
        self.output_transform_topic = str(
            get(
                '~output_transform_topic',
                '/front_marker_tracking/right/transform_base_marker',
            )
        )
        self.output_pose_topic = str(
            get(
                '~output_pose_topic',
                '/front_marker_tracking/right/marker_pose_base',
            )
        )
        self.status_topic = str(
            get(
                '~status_topic',
                '/front_marker_tracking/right/base_transform_status',
            )
        )
        self.expected_camera_frame = str(
            get('~expected_camera_frame', 'usb_camera_optical_frame')
        )
        self.base_frame = str(get('~base_frame', 'base_link'))
        self.target_marker_id = int(get('~target_marker_id', 33))

        rotation_values = get(
            '~body_R_camera_optical',
            DEFAULT_BODY_R_CAMERA_OPTICAL.reshape(9).tolist(),
        )
        if len(rotation_values) != 9:
            raise ValueError('body_R_camera_optical must contain 9 row-major values')
        self.body_r_camera_optical = validate_rotation(
            np.asarray(rotation_values, dtype=np.float64).reshape(3, 3)
        )
        self.body_t_camera_optical = validate_vector3(
            get(
                '~body_t_camera_optical',
                DEFAULT_BODY_T_CAMERA_OPTICAL.tolist(),
            ),
            'body_t_camera_optical',
        )

        if not self.input_transform_topic or not self.output_transform_topic:
            raise ValueError('transform topics must not be empty')
        if not self.output_pose_topic or not self.status_topic:
            raise ValueError('pose and status topics must not be empty')
        if not self.expected_camera_frame or not self.base_frame:
            raise ValueError('camera and base frame names must not be empty')
        if self.target_marker_id < 0:
            raise ValueError('target_marker_id must be non-negative')

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def transform_callback(self, message):
        with self.lock:
            if message.marker_id != self.target_marker_id:
                self.publish_status(
                    'ID_MISMATCH: received ID%d, expected ID%d'
                    % (message.marker_id, self.target_marker_id)
                )
                return
            if message.header.frame_id != self.expected_camera_frame:
                self.publish_status(
                    "FRAME_MISMATCH: received '%s', expected '%s'"
                    % (message.header.frame_id, self.expected_camera_frame)
                )
                return

            try:
                camera_t_marker = row_major_matrix(message.matrix)
                base_t_marker = camera_marker_to_body(
                    camera_t_marker,
                    self.body_r_camera_optical,
                    self.body_t_camera_optical,
                )
            except ValueError as exception:
                self.publish_status('INVALID_TRANSFORM: %s' % exception)
                return

            output = HomogeneousTransform()
            output.header.seq = message.header.seq
            output.header.stamp = message.header.stamp
            output.header.frame_id = self.base_frame
            output.marker_id = message.marker_id
            output.matrix = base_t_marker.reshape(16).tolist()
            self.transform_pub.publish(output)

            quaternion = rotation_to_quaternion(base_t_marker[:3, :3])
            pose = PoseStamped()
            pose.header.seq = output.header.seq
            pose.header.stamp = output.header.stamp
            pose.header.frame_id = output.header.frame_id
            pose.pose.position.x = float(base_t_marker[0, 3])
            pose.pose.position.y = float(base_t_marker[1, 3])
            pose.pose.position.z = float(base_t_marker[2, 3])
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = quaternion
            self.pose_pub.publish(pose)

            self.publish_status(
                'TRANSFORMED: ID%d %s xyz_base=(%.3f,%.3f,%.3f)m'
                % (
                    message.marker_id,
                    self.expected_camera_frame,
                    base_t_marker[0, 3],
                    base_t_marker[1, 3],
                    base_t_marker[2, 3],
                )
            )


def main():
    rospy.init_node('front_marker_base_transform_node')
    FrontMarkerBaseTransformNode()
    rospy.spin()


if __name__ == '__main__':
    main()
