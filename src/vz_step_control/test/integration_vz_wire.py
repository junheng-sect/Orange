#!/usr/bin/env python3
"""Live wire-level integration for the vz_step_control_node (no FCU).

Brings up a roscore, a fake MAVROS driver and the real vz_step_control_node,
then drives one fast OFFBOARD test to COMPLETE while staying in OFFBOARD. It
asserts on the actual published PositionTarget.type_mask (IGNORE_AFZ and friends
present in both position and velocity modes) and on the /mavros/set_message_interval
service calls (100 Hz on entry, 30/10 Hz restored after COMPLETE while the
vehicle is still holding in OFFBOARD).

Run with a sourced devel workspace:
    cd /home/vslam/seeker
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    python3 src/vz_step_control/test/integration_vz_wire.py
Exit code 0 = all assertions passed.
"""

import os
import signal
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import MessageInterval, MessageIntervalResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from vz_step_control.msg import VzStepStatus

SETPOINT_TOPIC = '/mavros/setpoint_raw/local'
STATE_TOPIC = '/mavros/state'
STATUS_TOPIC = '/vz_step/status'

FAILURES = []


def check(label, condition, detail=''):
    status = 'PASS' if condition else 'FAIL'
    print('[{}] {} {}'.format(status, label, detail))
    if not condition:
        FAILURES.append(label)


class FakeMavros:
    def __init__(self):
        self.interval_requests = []  # (message_id, message_rate)
        self.setpoint_samples = []   # (type_mask, velocity_z, stamp)
        self.complete_seen = False
        self.x, self.y, self.z, self.yaw = 0.5, -0.25, 1.2, 0.3
        self.vz = 0.0
        self.entered = time.time()

        self.state_pub = rospy.Publisher(STATE_TOPIC, State, queue_size=10)
        self.odom_pub = rospy.Publisher(
            '/mavros/local_position/odom', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher(
            '/mavros/local_position/pose', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher(
            '/mavros/local_position/velocity_local', TwistStamped,
            queue_size=10)
        self.target_pub = rospy.Publisher(
            '/mavros/setpoint_raw/target_local', PositionTarget,
            queue_size=10)
        rospy.Subscriber(SETPOINT_TOPIC, PositionTarget,
                         self.setpoint_callback, queue_size=200)
        rospy.Subscriber(STATUS_TOPIC, VzStepStatus, self.status_callback,
                         queue_size=50)
        self.interval_srv = rospy.Service(
            '/mavros/set_message_interval', MessageInterval,
            self.interval_callback)

    def interval_callback(self, request):
        self.interval_requests.append(
            (request.message_id, float(request.message_rate)))
        response = MessageIntervalResponse()
        response.success = True
        return response

    def setpoint_callback(self, message):
        self.setpoint_samples.append(
            (message.type_mask, message.velocity.z, rospy.Time.now().to_sec()))
        if message.type_mask & PositionTarget.IGNORE_PZ:
            self.vz = message.velocity.z
        else:
            self.vz = 0.0
        self.x = message.position.x
        self.y = message.position.y
        self.yaw = message.yaw

    def status_callback(self, message):
        if message.state == VzStepStatus.COMPLETE:
            self.complete_seen = True

    def _offboard(self):
        elapsed = time.time() - self.entered
        if elapsed < 1.0:
            return False               # POSCTL (disengaged) prime
        if self.complete_seen and elapsed > 8.0:
            return False               # exit only well after the 1 s tail
        return True

    def publish_state(self, _event):
        message = State()
        message.header = Header()
        message.header.stamp = rospy.Time.now()
        message.connected = True
        active = self._offboard()
        # Armed during the POSCTL prime too, so the rising edge is mode-driven.
        message.armed = True
        message.mode = 'OFFBOARD' if active else 'POSCTL'
        self.state_pub.publish(message)

    def publish_data(self, _event):
        self.z += self.vz * 0.01
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.z = self.vz
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = stamp
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        velocity = TwistStamped()
        velocity.header = Header()
        velocity.header.stamp = stamp
        velocity.twist.linear.z = self.vz
        self.velocity_pub.publish(velocity)

        target = PositionTarget()
        target.header = Header()
        target.header.stamp = stamp
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.target_pub.publish(target)


def _ensure_roscore(proc):
    for _ in range(60):
        result = subprocess.run(['rosnode', 'list'], capture_output=True)
        if result.returncode == 0:
            return
        time.sleep(0.5)
    raise RuntimeError('roscore did not become ready')


def _wait_until(predicate, timeout, label):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    print('TIMEOUT waiting for {}'.format(label))
    return False


def main():
    existing_master = (subprocess.run(['rosnode', 'list'],
                                      capture_output=True).returncode == 0)
    if existing_master:
        roscore_proc = None
    else:
        roscore_proc = subprocess.Popen(['roscore'],
                                        stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL,
                                        preexec_fn=os.setsid)
        time.sleep(2.0)
    _ensure_roscore(roscore_proc)

    rospy.init_node('vz_wire_integration', anonymous=True)
    driver = FakeMavros()
    rospy.Timer(rospy.Duration(0.1), driver.publish_state)
    rospy.Timer(rospy.Duration(0.01), driver.publish_data)

    node_log = open('/tmp/vz_wire_node.log', 'wb')
    node_proc = subprocess.Popen([
        'rosrun', 'vz_step_control', 'vz_step_control_node',
        '_step_speed_mps:=0.5',
        '_step_duration_sec:=0.3',
        '_stabilize_duration_sec:=2.5',
        '_hold_duration_sec:=0.4',
        '_cycle_count:=1',
        '_setpoint_rate_hz:=100',
        '_enable_setpoint_output:=true',
        '_recorder_required:=false',
        '_requested_telemetry_rate_hz:=100',
        '_minimum_data_rate_hz:=90',
        '_rate_window_sec:=2.0',
        '_recorder_health_timeout_sec:=1.0',
        '_telemetry_tail_sec:=1.0',
    ], stdout=node_log, stderr=subprocess.STDOUT,
       preexec_fn=os.setsid)

    try:
        # Wait for the fast test to reach COMPLETE, then a little more so the
        # post-end telemetry restore (tail = 1 s) fires while still in OFFBOARD.
        if not _wait_until(lambda: driver.complete_seen, 15.0, 'COMPLETE'):
            check('reached COMPLETE', False)
        else:
            check('reached COMPLETE', True)
            time.sleep(1.8)  # stay OFFBOARD past the 1 s tail

        # --- type_mask assertions (issue #6) ------------------------------
        pos_modes = [s for s in driver.setpoint_samples
                     if not (s[0] & PositionTarget.IGNORE_PZ)]
        vel_modes = [s for s in driver.setpoint_samples
                     if s[0] & PositionTarget.IGNORE_PZ]
        check('saw position-mode setpoints', bool(pos_modes))
        check('saw velocity-mode setpoints', bool(vel_modes))
        if pos_modes:
            mask = pos_modes[-1][0]
            check('position mask ignores AFZ',
                  bool(mask & PositionTarget.IGNORE_AFZ))
            check('position mask ignores VZ',
                  bool(mask & PositionTarget.IGNORE_VZ))
            check('position mask keeps PZ (not ignored)',
                  not (mask & PositionTarget.IGNORE_PZ))
            check('position mask ignores AFX/AFY',
                  bool(mask & PositionTarget.IGNORE_AFX)
                  and bool(mask & PositionTarget.IGNORE_AFY))
        if vel_modes:
            mask = vel_modes[0][0]
            check('velocity mask ignores AFZ',
                  bool(mask & PositionTarget.IGNORE_AFZ))
            check('velocity mask ignores PZ',
                  bool(mask & PositionTarget.IGNORE_PZ))
            check('velocity mask ignores AFX/AFY',
                  bool(mask & PositionTarget.IGNORE_AFX)
                  and bool(mask & PositionTarget.IGNORE_AFY))
            check('velocity mask does NOT ignore VZ',
                  not (mask & PositionTarget.IGNORE_VZ))

        # --- telemetry rate assertions (issue #3) -------------------------
        boost = [(mid, rate) for (mid, rate) in driver.interval_requests
                 if rate >= 90.0]
        restore = [(mid, rate) for (mid, rate) in driver.interval_requests
                   if rate < 90.0]
        check('requested 100 Hz for msg 32 on entry',
              (32, 100.0) in boost or any(
                  mid == 32 and rate >= 90.0 for mid, rate in boost))
        check('requested 100 Hz for msg 85 on entry',
              any(mid == 85 and rate >= 90.0 for mid, rate in boost))
        check('restored msg 32 to normal rate after end',
              any(mid == 32 and rate < 90.0 for mid, rate in restore))
        check('restored msg 85 to normal rate after end',
              any(mid == 85 and rate < 90.0 for mid, rate in restore))
        print('[note] interval service calls: {}'.format(
            driver.interval_requests))
    finally:
        os.killpg(os.getpgid(node_proc.pid), signal.SIGTERM)
        time.sleep(0.5)
        node_log.flush()
        node_log.close()
        if roscore_proc is not None:
            os.killpg(os.getpgid(roscore_proc.pid), signal.SIGTERM)

    if FAILURES:
        try:
            print('----- controller node log tail -----')
            print(open('/tmp/vz_wire_node.log').read()[-3000:])
        except OSError:
            pass
    print('RESULT: {}'.format('FAIL' if FAILURES else 'PASS'))
    sys.exit(1 if FAILURES else 0)


if __name__ == '__main__':
    main()
