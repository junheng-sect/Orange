#!/usr/bin/env python3
"""End-to-end through real pyserial against a pseudo-terminal device."""
import os
import sys
import time
import unittest

from barcode_reader_serial import reader as rd
from barcode_reader_serial.reader import BarcodeReader, config_from_params

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # nose does not add test/
from pty_device import PtyReaderDevice  # noqa: E402


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class PtyDevice(unittest.TestCase):
    def setUp(self):
        self.device = PtyReaderDevice(b'NEXT9017324\r')
        self.reader = BarcodeReader(config_from_params(
            {'port': self.device.path, 'read_timeout_ms': 500}))
        self.reader.start()

    def tearDown(self):
        self.reader.close()
        self.device.close()

    def test_read_then_timeout(self):
        result = self.reader.read()
        self.assertTrue(result.success, result)
        self.assertEqual(result.code, 'NEXT9017324')
        self.assertTrue(wait_for(lambda: self.device.counts() == (1, 1)), self.device.counts())

        self.device.reply = None
        self.assertEqual(self.reader.read(0.2).error, rd.TIMEOUT)
        self.assertTrue(wait_for(lambda: self.device.counts() == (2, 2)), self.device.counts())

        self.device.reply = b'SECOND\r\n'
        self.assertEqual(self.reader.read().code, 'SECOND')


if __name__ == '__main__':
    unittest.main()
