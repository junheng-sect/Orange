#!/usr/bin/env python3
"""Write one standard rosbag for each armed OFFBOARD session."""

import os
import threading
from datetime import datetime

import rosbag
import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vz_chirp_control.msg import VzChirpStatus


class OffboardBagRecorder:
    def __init__(self):
        self._lock = threading.RLock()
        self._bag = None
        self._bag_path = ''
        self._offboard_active = False

        self._output_directory = os.path.abspath(
            rospy.get_param('~output_directory',
                            '/home/vslam/seeker/logs/vz_chirp')
        )
        self._state_topic = rospy.get_param('~state_topic', '/mavros/state')
        self._topics = [
            (
                rospy.get_param('~setpoint_topic',
                                '/mavros/setpoint_raw/local'),
                PositionTarget,
            ),
            (
                rospy.get_param('~target_topic',
                                '/mavros/setpoint_raw/target_local'),
                PositionTarget,
            ),
            (
                rospy.get_param('~velocity_topic',
                                '/mavros/local_position/velocity_local'),
                TwistStamped,
            ),
            (
                rospy.get_param('~odom_topic',
                                '/mavros/local_position/odom'),
                Odometry,
            ),
            (
                rospy.get_param('~status_topic', '/vz_chirp/status'),
                VzChirpStatus,
            ),
        ]
        bag_path_topic = rospy.get_param('~bag_path_topic',
                                         '/vz_chirp/bag_path')
        self._path_publisher = rospy.Publisher(
            bag_path_topic, String, queue_size=1, latch=True
        )

        # Data subscribers are connected before the State subscriber. Once the
        # OFFBOARD edge arrives, every subsequent data callback can be written.
        self._subscribers = [
            rospy.Subscriber(
                topic,
                message_type,
                self._data_callback,
                callback_args=topic,
                queue_size=500,
                tcp_nodelay=True,
            )
            for topic, message_type in self._topics
        ]
        self._state_subscriber = rospy.Subscriber(
            self._state_topic,
            State,
            self._state_callback,
            queue_size=50,
            tcp_nodelay=True,
        )
        rospy.on_shutdown(self.close)
        rospy.loginfo('OFFBOARD bag recorder ready; output=%s',
                      self._output_directory)

    @staticmethod
    def _is_offboard(message):
        return bool(
            message.connected
            and message.armed
            and message.mode == 'OFFBOARD'
        )

    def _state_callback(self, message):
        now = rospy.Time.now()
        active = self._is_offboard(message)
        with self._lock:
            if active and not self._offboard_active:
                self._open_locked()
                self._write_locked(self._state_topic, message, now)
            elif not active and self._offboard_active:
                # Include the exact state message that ended the recording.
                self._write_locked(self._state_topic, message, now)
                self._close_locked()
            elif active:
                self._write_locked(self._state_topic, message, now)
            self._offboard_active = active

    def _data_callback(self, message, topic):
        with self._lock:
            if self._offboard_active and self._bag is not None:
                self._write_locked(topic, message, rospy.Time.now())

    def _unique_path(self):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = os.path.join(self._output_directory,
                            '{}_vz_chirp_offboard'.format(stamp))
        candidate = base + '.bag'
        suffix = 1
        while os.path.exists(candidate) or os.path.exists(candidate + '.active'):
            candidate = '{}_{:02d}.bag'.format(base, suffix)
            suffix += 1
        return candidate

    def _open_locked(self):
        if self._bag is not None:
            return
        try:
            os.makedirs(self._output_directory, exist_ok=True)
            self._bag_path = self._unique_path()
            self._bag = rosbag.Bag(self._bag_path, mode='w')
        except Exception as exception:
            self._bag = None
            self._bag_path = ''
            rospy.logfatal('cannot open OFFBOARD rosbag: %s', exception)
            rospy.signal_shutdown('cannot open OFFBOARD rosbag')
            return
        self._path_publisher.publish(String(data='OPEN ' + self._bag_path))
        rospy.logwarn('OFFBOARD entered: recording %s', self._bag_path)

    def _write_locked(self, topic, message, stamp):
        if self._bag is None:
            return
        try:
            self._bag.write(topic, message, t=stamp)
        except Exception as exception:
            rospy.logerr('rosbag write failed for %s: %s', topic, exception)
            self._close_locked()
            rospy.signal_shutdown('OFFBOARD rosbag write failed')

    def _close_locked(self):
        if self._bag is None:
            return
        path = self._bag_path
        try:
            self._bag.close()
        except Exception as exception:
            rospy.logerr('failed to close rosbag %s cleanly: %s', path, exception)
        finally:
            self._bag = None
            self._bag_path = ''
        self._path_publisher.publish(String(data='CLOSED ' + path))
        rospy.logwarn('OFFBOARD ended: closed %s', path)

    def close(self):
        with self._lock:
            self._close_locked()
            self._offboard_active = False


def main():
    rospy.init_node('vz_chirp_bag_recorder')
    OffboardBagRecorder()
    rospy.spin()


if __name__ == '__main__':
    main()
