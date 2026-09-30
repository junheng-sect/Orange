"""ROS adapter: RC switch pulse on /mavros/rc/in -> one camera decode and/or serial read."""
import threading
import time

import rospy
from mavros_msgs.msg import RCIn
from std_msgs.msg import UInt32
from barcode_reader_camera.srv import DecodeBarcodes
from barcode_reader_serial.srv import ReadBarcode

from .policy import RcPulseTrigger
from .runner import format_step, parse_mode, run_sequence, step_ok

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
        self.steps = parse_mode(rospy.get_param('~mode', 'both'))
        self.camera_service = rospy.get_param('~camera_service', '/barcode_reader_camera/decode')
        self.serial_service = rospy.get_param('~serial_service', '/barcode_reader_serial/read')
        self.serial_timeout_s = float(rospy.get_param('~serial_timeout_s', 0.0))
        self.service_wait_s = float(rospy.get_param('~service_wait_s', 0.5))
        rc_topic = rospy.get_param('~rc_topic', '/mavros/rc/in')

        self.camera_proxy = rospy.ServiceProxy(self.camera_service, DecodeBarcodes)
        self.serial_proxy = rospy.ServiceProxy(self.serial_service, ReadBarcode)
        self.calls = {'camera': self._call_camera, 'serial': self._call_serial}

        self.lock = threading.Lock()
        self.busy = False
        self.pulse_seq = 0
        self.last_rc = None      # monotonic receive time of the last valid sample
        self.rc_online = False
        self.started = time.monotonic()

        self.pulse_pub = rospy.Publisher('~pulse', UInt32, queue_size=10)
        self.rc_sub = rospy.Subscriber(rc_topic, RCIn, self.on_rc, queue_size=50)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.on_timer)
        rospy.loginfo('barcode_rc_trigger: %s CH%d arm<%d fire>%d hold=%.2fs mode=%s '
                      'camera=%s serial=%s', rc_topic, self.rc_channel, self.policy.arm_below,
                      self.policy.trigger_above, self.policy.hold_s, '+'.join(self.steps),
                      self.camera_service, self.serial_service)

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
        threading.Thread(target=self._run, args=(seq,), name='barcode-rc-read-%d' % seq,
                         daemon=True).start()

    def _run(self, seq):
        try:
            results = run_sequence(self.steps, self.calls)
            summary = 'pulse#%d %s' % (seq, ' | '.join(format_step(r) for r in results))
            if all(step_ok(r) for r in results):
                rospy.loginfo(summary)
            else:
                rospy.logwarn(summary)
        finally:
            with self.lock:
                self.busy = False

    def _call_camera(self):
        rospy.wait_for_service(self.camera_service, self.service_wait_s)
        return self.camera_proxy()

    def _call_serial(self):
        rospy.wait_for_service(self.serial_service, self.service_wait_s)
        return self.serial_proxy(self.serial_timeout_s)


def main():
    rospy.init_node('barcode_rc_trigger')
    BarcodeRcTriggerNode()
    rospy.spin()
