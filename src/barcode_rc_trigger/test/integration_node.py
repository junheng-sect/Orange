#!/usr/bin/env python3
"""ROS integration with a fake RC stream and fake reader services; run with rostest node.test."""
import threading
import time
import unittest

import rospy
import rostest
from mavros_msgs.msg import RCIn
from std_msgs.msg import UInt32
from barcode_reader_camera.msg import Barcode
from barcode_reader_camera.srv import DecodeBarcodes, DecodeBarcodesResponse
from barcode_reader_serial.srv import ReadBarcode, ReadBarcodeResponse

LOW, HIGH = 1000, 1900
CH9_INDEX = 8


class Integration(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.calls = []
        self.serial_timeouts = []
        self.pulses = []
        self.camera_delay = 0.0
        self.pwm = None  # None = RC stream stopped
        self.stop = False
        self.camera_srv = rospy.Service('/fake_camera/decode', DecodeBarcodes, self.on_decode)
        self.serial_srv = rospy.Service('/fake_serial/read', ReadBarcode, self.on_read)
        self.rc_pub = rospy.Publisher('/test_rc/in', RCIn, queue_size=50)
        self.pulse_sub = rospy.Subscriber('/barcode_rc_trigger/pulse', UInt32,
                                          lambda m: self.pulses.append(m.data))
        self.assertTrue(self.wait_for(lambda: self.rc_pub.get_num_connections() > 0
                                      and self.pulse_sub.get_num_connections() > 0, 20))
        self.rc_thread = threading.Thread(target=self.publish_rc, daemon=True)
        self.rc_thread.start()

    def tearDown(self):
        self.stop = True
        self.rc_thread.join(1)
        self.pulse_sub.unregister()
        self.camera_srv.shutdown()
        self.serial_srv.shutdown()

    def publish_rc(self):
        while not self.stop and not rospy.is_shutdown():
            pwm = self.pwm
            if pwm is not None:
                channels = [1500] * 16
                channels[CH9_INDEX] = pwm
                self.rc_pub.publish(RCIn(channels=channels))
            time.sleep(0.02)

    def on_decode(self, _request):
        with self.lock:
            self.calls.append('camera')
        time.sleep(self.camera_delay)
        return DecodeBarcodesResponse(success=True, report='QR=1/1', elapsed_s=0.01,
                                      codes=[Barcode(format='QRCode', code='ABC')])

    def on_read(self, request):
        with self.lock:
            self.calls.append('serial')
            self.serial_timeouts.append(request.timeout_s)
        return ReadBarcodeResponse(success=True, code='NEXT9017324', elapsed_s=0.01)

    @staticmethod
    def wait_for(condition, timeout):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True

    def level(self, pwm, seconds):
        self.pwm = pwm
        time.sleep(seconds)

    def flick(self):
        self.level(LOW, 0.3)
        self.level(HIGH, 0.3)
        self.level(LOW, 0.3)

    def expect(self, calls, pulses, timeout=3.0):
        self.wait_for(lambda: self.calls == calls and self.pulses == pulses, timeout)
        time.sleep(0.3)  # nothing extra shows up afterwards
        self.assertEqual(self.calls, calls)
        self.assertEqual(self.pulses, pulses)

    def test_rc_pulses(self):
        # Switch already high when RC appears: must not fire.
        self.level(HIGH, 0.6)
        self.expect([], [])

        # One flick -> camera then serial, pulse #1.
        self.flick()
        self.expect(['camera', 'serial'], [1])

        # Second flick while the first read is still running is ignored (#3 not published).
        self.camera_delay = 1.5
        self.level(LOW, 0.2)
        self.level(HIGH, 0.2)
        self.level(LOW, 0.2)
        self.level(HIGH, 0.2)
        self.level(LOW, 0.2)
        self.expect(['camera', 'serial'] * 2, [1, 2], timeout=4.0)
        self.camera_delay = 0.0
        self.flick()
        self.expect(['camera', 'serial'] * 3, [1, 2, 4])

        # RC lost longer than rc_timeout_s disarms: coming back already high must not fire.
        self.level(LOW, 0.3)
        self.level(None, 1.0)
        self.level(HIGH, 0.6)
        self.expect(['camera', 'serial'] * 3, [1, 2, 4])
        self.flick()
        self.expect(['camera', 'serial'] * 4, [1, 2, 4, 5])

        # Serial service offline: camera still runs, node stays usable.
        self.serial_srv.shutdown()
        self.flick()
        self.expect(['camera', 'serial'] * 4 + ['camera'], [1, 2, 4, 5, 6])
        self.level(LOW, 1.0)  # let service_wait_s expire
        self.flick()
        self.expect(['camera', 'serial'] * 4 + ['camera'] * 2, [1, 2, 4, 5, 6, 7])

        self.assertEqual(self.serial_timeouts, [0.0] * 4)


if __name__ == '__main__':
    rospy.init_node('barcode_rc_trigger_integration')
    rostest.rosrun('barcode_rc_trigger', 'barcode_rc_trigger_node', Integration)
