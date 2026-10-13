"""ROS adapter: RC switch pulse on /mavros/rc/in -> one camera decode and/or serial read.

每次拨杆同时记下 tag 位置（相机光学系），写进汇总日志和 CSV，用来看在什么位置能读成功。
"""
import os
import threading
import time

import rospy
from mavros_msgs.msg import RCIn
from ov13855_apriltag_vision.msg import HomogeneousTransform
from std_msgs.msg import UInt32

from .policy import RcPulseTrigger
from .readers import ReaderCalls, log_summary
from .record import CsvRecorder, csv_row
from .runner import parse_mode, run_sequence
from .tag_pose import TagHistory, pick_tag, sample_from_matrix, tag_fields

# PWM outside this band is treated as RC invalid (PX4 reports unused channels as 0/65535).
PWM_VALID_MIN = 800
PWM_VALID_MAX = 2200


class BarcodeRcTriggerNode:
    def __init__(self):
        self.rc_channel = int(rospy.get_param('~rc_channel', 9))  # 1-based, 9 = CH9
        if self.rc_channel < 1:
            raise ValueError('rc_channel is 1-based and must be >= 1')
        self.policy = RcPulseTrigger(int(rospy.get_param('~arm_below', 1600)),
                                     int(rospy.get_param('~trigger_above', 1700)),
                                     float(rospy.get_param('~hold_s', 0.05)))
        self.rc_timeout_s = float(rospy.get_param('~rc_timeout_s', 1.0))
        self.steps = parse_mode(rospy.get_param('~mode', 'camera'))
        self.camera_service = rospy.get_param('~camera_service', '/barcode_reader_camera/decode')
        self.serial_service = rospy.get_param('~serial_service', '/barcode_reader_serial/read')
        self.readers = ReaderCalls(self.camera_service, self.serial_service,
                                   float(rospy.get_param('~serial_timeout_s', 0.0)),
                                   float(rospy.get_param('~service_wait_s', 0.5)))
        rc_topic = rospy.get_param('~rc_topic', '/mavros/rc/in')
        transform_topic = str(rospy.get_param(
            '~transform_topic', '/ov13855_apriltag_vision/debug/homogeneous_transform'))
        self.tag_timeout_s = float(rospy.get_param('~tag_timeout_s', 0.5))
        csv_dir = os.path.expanduser(str(rospy.get_param('~csv_dir', '~/barcode_rc_logs')))

        self.lock = threading.Lock()
        self.busy = False
        self.pulse_seq = 0
        self.last_rc = None      # monotonic receive time of the last valid sample
        self.rc_online = False
        self.started = time.monotonic()
        self.tags = TagHistory()
        self.recorder = None
        if csv_dir:
            self.recorder = CsvRecorder(os.path.join(
                csv_dir, time.strftime('rc_read_%Y%m%d_%H%M%S.csv')))
            rospy.on_shutdown(self.recorder.close)

        self.pulse_pub = rospy.Publisher('~pulse', UInt32, queue_size=10)
        self.rc_sub = rospy.Subscriber(rc_topic, RCIn, self.on_rc, queue_size=50)
        if transform_topic:
            self.tag_sub = rospy.Subscriber(transform_topic, HomogeneousTransform,
                                            self.on_transform, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.on_timer)
        # Only the services this mode calls, so camera mode prints nothing about the serial reader.
        services = {'camera': self.camera_service, 'serial': self.serial_service}
        rospy.loginfo('barcode_rc_trigger: %s CH%d arm<%d fire>%d hold=%.2fs mode=%s %s', rc_topic,
                      self.rc_channel, self.policy.arm_below, self.policy.trigger_above,
                      self.policy.hold_s, '+'.join(self.steps),
                      ' '.join('%s=%s' % (name, services[name]) for name in self.steps))
        rospy.loginfo('tag: %s (%.1fs) | csv: %s', transform_topic or 'off', self.tag_timeout_s,
                      self.recorder.path if self.recorder else 'off')

    def on_transform(self, message):
        sample = sample_from_matrix(message.header.stamp.to_sec(), time.monotonic(),
                                    message.marker_id, message.matrix)
        with self.lock:
            self.tags.add(sample)

    def on_rc(self, message):
        now = time.monotonic()
        index = self.rc_channel - 1
        pwm = int(message.channels[index]) if index < len(message.channels) else None
        if pwm is not None and not PWM_VALID_MIN <= pwm <= PWM_VALID_MAX:
            pwm = None
        if pwm is None:
            rospy.logwarn_throttle(5.0, 'CH%d invalid in RC message (%d channels)',
                                   self.rc_channel, len(message.channels))
        with self.lock:
            fired = self.policy.update(now, pwm)
            if pwm is not None:
                self.last_rc = now
                if not self.rc_online:
                    self.rc_online = True
                    rospy.loginfo('RC online: CH%d=%d', self.rc_channel, pwm)
            if fired:
                self._start_pulse_locked(pwm)

    def on_timer(self, _event):
        now = time.monotonic()
        with self.lock:
            if self.last_rc is None:
                if now - self.started > 5.0:
                    rospy.logwarn_throttle(10.0, 'No valid CH%d on RC topic yet '
                                           '(MAVROS running? transmitter on?)', self.rc_channel)
                return
            if self.rc_online and now - self.last_rc > self.rc_timeout_s:
                self.rc_online = False
                self.policy.reset()
                rospy.logwarn('RC lost for %.1fs: trigger disarmed until CH%d is seen low again',
                              now - self.last_rc, self.rc_channel)

    def _start_pulse_locked(self, pwm):
        self.pulse_seq += 1
        seq = self.pulse_seq
        if self.busy:
            rospy.logwarn('pulse#%d ignored: previous read still running', seq)
            return
        self.busy = True
        rospy.loginfo('pulse#%d CH%d=%d -> %s', seq, self.rc_channel, pwm, '+'.join(self.steps))
        self.pulse_pub.publish(UInt32(seq))
        pulse_tag = self.tags.latest(time.monotonic(), self.tag_timeout_s)
        threading.Thread(target=self._run, args=(seq, pulse_tag),
                         name='barcode-rc-read-%d' % seq, daemon=True).start()

    def _run(self, seq, pulse_tag):
        try:
            results = run_sequence(self.steps, self.readers.calls)
            image_stamp = 0.0
            for result in results:
                if result.name == 'camera' and result.response is not None:
                    image_stamp = result.response.header.stamp.to_sec()
            with self.lock:
                tag, source = pick_tag(self.tags, image_stamp, pulse_tag)
            log_summary('pulse#%d' % seq, results, [('tag', tag_fields(tag, source))])
            if self.recorder is not None:
                self.recorder.write(csv_row(time.time(), seq, tag, source, results))
        except Exception as error:
            if not rospy.is_shutdown():
                rospy.logerr('pulse#%d 记录失败: %s', seq, error)
        finally:
            with self.lock:
                self.busy = False


def main():
    rospy.init_node('barcode_rc_trigger')
    BarcodeRcTriggerNode()
    rospy.spin()
