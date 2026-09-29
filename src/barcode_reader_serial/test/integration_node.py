#!/usr/bin/env python3
"""ROS integration against a pty reader; run with rostest node.test."""
import threading
import time
import unittest

import rospy
import rostest
from std_srvs.srv import Trigger
from barcode_reader_serial.msg import BarcodeResult
from barcode_reader_serial.srv import ReadBarcode
from pty_device import PtyReaderDevice


class Integration(unittest.TestCase):
    def setUp(self):
        # The node started first and failed to open this link; ~read must reopen it.
        self.device = PtyReaderDevice(link=rospy.get_param('~tty_link'))
        rospy.wait_for_service('/barcode_reader_serial/read', 20)
        self.read = rospy.ServiceProxy('/barcode_reader_serial/read', ReadBarcode)
        self.cancel = rospy.ServiceProxy('/barcode_reader_serial/cancel', Trigger)
        self.results = []
        self.subscriber = rospy.Subscriber('/barcode_reader_serial/result', BarcodeResult,
                                           self.results.append)
        deadline = time.monotonic() + 5
        while self.subscriber.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)

    def tearDown(self):
        self.subscriber.unregister()
        self.device.close()

    def test_read_timeout_busy_cancel(self):
        self.device.reply = b'NEXT9017324\r'
        response = self.read(0.0)
        self.assertTrue(response.success, response)
        self.assertEqual(response.code, 'NEXT9017324')
        self.assertEqual(response.raw_hex, b'NEXT9017324'.hex(' '))

        self.device.reply = None
        response = self.read(0.3)
        self.assertEqual(response.error, 'timeout')

        pending = []
        worker = threading.Thread(target=lambda: pending.append(self.read(10.0)))
        worker.start()
        time.sleep(0.3)
        self.assertEqual(self.read(0.0).error, 'busy')
        self.assertTrue(self.cancel().success)
        worker.join(3)
        self.assertEqual(pending[0].error, 'cancelled')
        self.assertLess(pending[0].elapsed_s, 2.0)
        self.assertFalse(self.cancel().success)

        deadline = time.monotonic() + 2
        while len(self.results) < 4 and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual(sorted(m.error for m in self.results),
                         ['', 'busy', 'cancelled', 'timeout'])
        deadline = time.monotonic() + 2
        while self.device.counts() != (3, 3) and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertEqual(self.device.counts(), (3, 3))  # (triggers, releases); busy sends nothing


if __name__ == '__main__':
    rospy.init_node('barcode_reader_integration')
    rostest.rosrun('barcode_reader_serial', 'barcode_reader_node', Integration)
