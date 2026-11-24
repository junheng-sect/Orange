#!/usr/bin/env python3
"""Live integration: recorder failure must abort the test, never run unrecorded.

Brings up roscore, a fake MAVROS driver, and the real vz_step_control.launch
(record=true so the controller demands recorder health). It drives one OFFBOARD
session and kills the bag recorder mid-STABILIZE. Expected behaviour:

  * the recorder opened a bag for the session before it was killed,
  * the controller ABORTs at the STABILIZE->UP_STEP gate (recorder health lost)
    and keeps publishing position holds (no velocity step ever ran),
  * the 100 Hz MAVLink streams are restored to 30/10 Hz after the ABORT while
    still in OFFBOARD, and are not re-boosted.

Run with a sourced devel workspace:
    cd /home/vslam/seeker
    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    python3 src/experiments/vz_step_control/test/integration_recorder_abort.py /tmp/vz_live_bags
Exit code 0 = all assertions passed.
"""

import os
import signal
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import MessageInterval, MessageIntervalResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from vz_step_control.msg import VzRecorderHealth, VzStepStatus

FAILURES = []


def check(label, condition, detail=''):
    print('[{}] {} {}'.format('PASS' if condition else 'FAIL', label, detail))
    if not condition:
        FAILURES.append(label)


def _wait_until(predicate, timeout, label):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            if predicate():
                return True
        except rospy.ROSInterruptException:
            return False
        time.sleep(0.05)
    print('TIMEOUT waiting for {}'.format(label))
    return False


class FakeMavros:
    def __init__(self):
        self.interval_requests = []
        self.setpoints = []            # (type_mask, stamp)
        self.phase = None
        self.abort_reason = ''
        self.health_samples = []       # (healthy, recording, stamp)
        self.statuses = []             # (state, reason, stamp)
        self.go = False
        self.go_time = None
        self._stop = False

        self.state_pub = rospy.Publisher('/mavros/state', State, queue_size=10)
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
        rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget,
                         self.setpoint_callback, queue_size=200)
        rospy.Subscriber('/vz_step/status', VzStepStatus,
                         self.status_callback, queue_size=50)
        rospy.Subscriber('/vz_step/recorder_health', VzRecorderHealth,
                         self.health_callback, queue_size=50)
        self.interval_srv = rospy.Service(
            '/mavros/set_message_interval', MessageInterval,
            self.interval_callback)

    def interval_callback(self, request):
        self.interval_requests.append(
            (request.message_id, float(request.message_rate), time.time()))
        return MessageIntervalResponse(success=True)

    def setpoint_callback(self, message):
        self.setpoints.append((message.type_mask, time.time()))

    def status_callback(self, message):
        if not self.statuses or self.statuses[-1][0] != message.state:
            self.statuses.append((message.state, message.reason, time.time()))
        self.phase = message.state
        if message.state == VzStepStatus.ABORT:
            self.abort_reason = message.reason

    def health_callback(self, message):
        self.health_samples.append(
            (message.healthy, message.recording, time.time()))

    def _offboard(self):
        if not self.go or self.go_time is None:
            return False
        return time.time() < self.go_time + 7.0

    def publish_state(self, _event=None):
        if self._stop:
            return
        message = State()
        message.header = Header()
        message.header.stamp = rospy.Time.now()
        message.connected = True
        message.armed = True
        message.mode = 'OFFBOARD' if self._offboard() else 'POSCTL'
        self.state_pub.publish(message)

    def publish_data(self, _event=None):
        if self._stop:
            return
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 0.5
        odom.pose.pose.position.y = -0.25
        odom.pose.pose.position.z = 1.2
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = stamp
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        velocity = TwistStamped()
        velocity.header = Header()
        velocity.header.stamp = stamp
        self.velocity_pub.publish(velocity)

        target = PositionTarget()
        target.header = Header()
        target.header.stamp = stamp
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.target_pub.publish(target)


def main():
    bag_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/vz_live_bags'
    # Always start a private master: the real board runs the seeker.service
    # bringup (front_marker_control) on the default master, and its
    # /mavros/setpoint_raw/local publisher would trip this node's duplicate
    # setpoint publisher monitor and abort the session under test.
    import random
    port = random.randint(12000, 18000)
    env = dict(os.environ, ROS_MASTER_URI='http://localhost:%d' % port)
    roscore_proc = subprocess.Popen(['roscore', '-p', str(port)],
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL,
                                    preexec_fn=os.setsid)
    os.environ['ROS_MASTER_URI'] = 'http://localhost:%d' % port
    time.sleep(2.0)

    rospy.init_node('vz_recorder_abort_integration', anonymous=True)
    driver = FakeMavros()
    rospy.Timer(rospy.Duration(0.1), driver.publish_state)
    rospy.Timer(rospy.Duration(0.01), driver.publish_data)
    threading.Thread(target=rospy.spin, daemon=True).start()

    subprocess.run(['rm', '-rf', bag_dir])
    os.makedirs(bag_dir)

    launch_log = open('/tmp/vz_abort_launch.log', 'wb')
    # 阶跃参数已统一到 YAML；测试通过临时 config_file 覆盖，避免改动包内配置。
    test_config = '/tmp/vz_abort_test_config.yaml'
    with open(test_config, 'w') as f:
        f.write(
            'enable_setpoint_output: true\n'
            'bag_directory: {}\n'
            'step_speed_mps: 0.5\n'
            'step_duration_sec: 0.3\n'
            'stabilize_duration_sec: 2.5\n'
            'hold_duration_sec: 0.5\n'
            'cycle_count: 1\n'.format(bag_dir)
        )
    launch = subprocess.Popen([
        'roslaunch', 'vz_step_control', 'vz_step_control.launch',
        'config_file:={}'.format(test_config),
        'record:=true',
    ], stdout=launch_log, stderr=subprocess.STDOUT,
       preexec_fn=os.setsid)

    try:
        # Wait until the controller is streaming (PRESTREAM) and the recorder
        # reports healthy while idle, then raise OFFBOARD. This avoids the
        # startup race where the controller would refuse entry before the
        # recorder was up.
        ok = _wait_until(
            lambda: driver.phase == VzStepStatus.PRESTREAM
            and any(h and not r for (h, r, _) in driver.health_samples),
            20.0, 'controller PRESTREAM + recorder healthy idle')
        check('controller reached PRESTREAM with healthy recorder', ok)
        if not ok:
            return

        driver.go = True
        driver.go_time = time.time()

        if not _wait_until(lambda: driver.phase == VzStepStatus.STABILIZE,
                           6.0, 'STABILIZE'):
            check('entered STABILIZE', False)
            print('[diag] statuses: {}'.format(
                [(s[0], s[1][:60]) for s in driver.statuses[-8:]]))
            print('[diag] interval requests: {}'.format(
                [(m, r) for (m, r, _) in driver.interval_requests]))
            print('[diag] last health: {}'.format(
                driver.health_samples[-3:]))
            return
        check('entered STABILIZE', True)

        # Recorder must be healthy and recording shortly after the edge.
        ok = _wait_until(
            lambda: any(h and r for (h, r, _) in driver.health_samples),
            3.0, 'recorder healthy+recording')
        check('recorder reported healthy and recording', ok)

        # Kill the recorder mid-STABILIZE.
        subprocess.run(['rosnode', 'kill', '/vz_step_bag_recorder'],
                       capture_output=True)
        ok = _wait_until(lambda: driver.phase == VzStepStatus.ABORT,
                         6.0, 'ABORT after recorder loss')
        check('controller aborted after recorder loss', ok)
        check('abort reason mentions recorder',
              'recorder' in driver.abort_reason, driver.abort_reason)
        check('no velocity step ever ran unrecorded',
              not any(mask & PositionTarget.IGNORE_PZ
                      for (mask, _) in driver.setpoints))

        # Position hold keeps flowing after ABORT while still in OFFBOARD.
        abort_mark = time.time() + 0.2
        ok = _wait_until(
            lambda: any(stamp > abort_mark
                        and not (mask & PositionTarget.IGNORE_PZ)
                        for (mask, stamp) in driver.setpoints),
            4.0, 'position holds after abort')
        check('kept publishing position holds after abort', ok)

        # Telemetry: entry boosted to 100 Hz, restored to 30/10 after ABORT
        # (1 s tail), and NOT re-boosted while the hold continues.  The two
        # streams (32 and 85) are restored independently, so wait for BOTH
        # low-rate restores before asserting, not just any one of them.
        ok = _wait_until(
            lambda: (any(m == 32 and r < 90.0
                         for (m, r, _t) in driver.interval_requests)
                     and any(m == 85 and r < 90.0
                             for (m, r, _t) in driver.interval_requests)),
            4.0, 'telemetry restore after ABORT (both 32 and 85)')
        check('telemetry restored to normal rates after ABORT', ok)
        boost = [(m, r) for (m, r, _) in driver.interval_requests if r >= 90.0]
        restore = [(m, r) for (m, r, _) in driver.interval_requests if r < 90.0]
        check('entry requested 100 Hz for 32 and 85',
              any(m == 32 for (m, r) in boost) and
              any(m == 85 for (m, r) in boost))
        check('restored 32 and 85 after ABORT',
              any(m == 32 for (m, r) in restore) and
              any(m == 85 for (m, r) in restore))
        check('no 100 Hz re-request after restore',
              boost.count((32, 100.0)) <= 1 and boost.count((85, 100.0)) <= 1)
        print('[note] interval service calls: {}'.format(
            [(m, r) for (m, r, _) in driver.interval_requests]))

        # A bag was opened for the session before the kill.
        bags = [f for f in os.listdir(bag_dir) if f.endswith('.bag')]
        check('at least one bag was opened before the kill',
              len(bags) >= 1, str(bags))
    finally:
        driver._stop = True
        time.sleep(0.3)
        if launch.poll() is None:
            os.killpg(os.getpgid(launch.pid), signal.SIGTERM)
        time.sleep(1.0)
        os.killpg(os.getpgid(roscore_proc.pid), signal.SIGTERM)
        launch_log.flush()
        launch_log.close()

    if FAILURES:
        try:
            print('----- roslaunch console tail -----')
            print(open('/tmp/vz_abort_launch.log').read()[-4000:])
        except OSError:
            pass

    print('RESULT: {}'.format('FAIL' if FAILURES else 'PASS'))
    sys.exit(1 if FAILURES else 0)


if __name__ == '__main__':
    main()
