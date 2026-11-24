#!/usr/bin/env python3
"""Write one standard rosbag for each armed OFFBOARD session.

Recording opens on the OFFBOARD rising edge; leaving OFFBOARD or disarming
closes the bag immediately. Bags are named with a timestamp prefix and made
unique against existing files.

The recorder is respawned by roslaunch (never required) and reports its health
on /vz_chirp/recorder_health at ~2 Hz. The chirp controller treats a running
sweep as unsafe (ABORT + position hold) whenever the recorder is unhealthy or
not recording, so a chirp is never left unrecorded. The recorder itself only
opens a bag on an OFFBOARD edge it observed coming from a healthy
non-OFFBOARD state, so a respawn that lands mid-OFFBOARD waits for a real exit
and re-entry instead of manufacturing a half-length bag.

The ROS-free decision/health policy lives in RecorderSession so it can be
driven deterministically in unit tests.
"""

import os
import sys
import threading
from datetime import datetime

import rosbag
import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vz_chirp_control.msg import VzChirpStatus, VzRecorderHealth

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from recorder_session import RecorderSession  # noqa: E402

HEALTH_PERIOD_SEC = 0.5


class OffboardBagRecorder:
    def __init__(self):
        self._lock = threading.RLock()
        self._bag = None
        self._bag_path = ''
        self._session = RecorderSession()

        self._output_directory = os.path.abspath(
            rospy.get_param('~bag_directory',
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
        health_topic = rospy.get_param('~recorder_health_topic',
                                       '/vz_chirp/recorder_health')
        self._path_publisher = rospy.Publisher(
            bag_path_topic, String, queue_size=1, latch=True
        )
        self._health_publisher = rospy.Publisher(
            health_topic, VzRecorderHealth, queue_size=1, latch=True
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
        self._health_timer = rospy.Timer(
            rospy.Duration(HEALTH_PERIOD_SEC), self._health_tick
        )
        rospy.on_shutdown(self.close)
        rospy.loginfo('OFFBOARD bag recorder ready; output=%s health=%s',
                      self._output_directory, health_topic)
        self._publish_health()

    @staticmethod
    def _is_offboard(message):
        return bool(
            message.connected
            and message.armed
            and message.mode == 'OFFBOARD'
        )

    def _state_callback(self, message):
        now = rospy.Time.now()
        with self._lock:
            action = self._session.on_state(
                message.connected, message.mode == 'OFFBOARD',
                message.armed)
            if action == 'open':
                self._open_locked()
                if self._bag is not None:
                    # Include the exact state message that started the session.
                    self._write_locked(self._state_topic, message, now)
            elif action == 'close':
                # Include the exact state message that ended the recording.
                self._write_locked(self._state_topic, message, now)
                self._close_locked()
            elif self._session.offboard_active:
                self._write_locked(self._state_topic, message, now)

    def _data_callback(self, message, topic):
        with self._lock:
            if self._session.offboard_active and self._bag is not None:
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
            self._session.bag_opened()
        except Exception as exception:
            self._bag = None
            self._bag_path = ''
            self._session.fail()
            self._publish_health()
            rospy.logfatal('cannot open OFFBOARD rosbag: %s', exception)
            # Exit so roslaunch respawns a clean recorder; the controller
            # independently holds position via the health interlock.
            rospy.signal_shutdown('cannot open OFFBOARD rosbag')
            return
        self._path_publisher.publish(String(data='OPEN ' + self._bag_path))
        rospy.logwarn('OFFBOARD entered: recording %s', self._bag_path)
        self._publish_health()

    def _write_locked(self, topic, message, stamp):
        if self._bag is None:
            return
        try:
            self._bag.write(topic, message, t=stamp)
        except Exception as exception:
            rospy.logerr('rosbag write failed for %s: %s', topic, exception)
            path = self._bag_path
            try:
                self._bag.close()
            except Exception as close_exception:
                rospy.logerr('also failed to close %s cleanly: %s', path,
                             close_exception)
            self._bag = None
            self._bag_path = ''
            self._session.fail()
            self._publish_health()
            rospy.signal_shutdown('OFFBOARD rosbag write failed')

    def _close_locked(self):
        if self._bag is None:
            return
        path = self._bag_path
        close_failed = False
        try:
            self._bag.close()
        except Exception as exception:
            close_failed = True
            rospy.logerr('failed to close rosbag %s cleanly: %s', path,
                         exception)
        finally:
            self._bag = None
            self._bag_path = ''
        if close_failed:
            # A half-closed bag is not a clean recording: mark the recorder
            # unhealthy (not CLOSED) so the controller never treats this
            # recorder as able to record the next session without a respawn.
            self._session.fail()
            self._publish_health()
            self._path_publisher.publish(
                String(data='CLOSE_FAILED ' + path))
            return
        self._session.bag_closed()
        self._path_publisher.publish(String(data='CLOSED ' + path))
        rospy.logwarn('OFFBOARD ended: closed %s', path)
        self._publish_health()

    def _health_tick(self, _event):
        with self._lock:
            self._publish_health()

    def _publish_health(self):
        message = VzRecorderHealth()
        message.header.stamp = rospy.Time.now()
        message.healthy = self._session.healthy()
        message.recording = self._session.recording()
        message.bag_path = self._bag_path or ''
        self._health_publisher.publish(message)

    def close(self):
        with self._lock:
            self._close_locked()


def main():
    rospy.init_node('vz_chirp_bag_recorder')
    OffboardBagRecorder()
    rospy.spin()


if __name__ == '__main__':
    main()
