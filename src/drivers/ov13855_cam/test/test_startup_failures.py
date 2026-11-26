#!/usr/bin/env python3
"""Verify startup failures are explicit and return a nonzero status without camera capture."""
import subprocess
import unittest
import rospy
import rostest

class StartupFailures(unittest.TestCase):
    def check_failure(self, arguments, diagnostic):
        result = subprocess.run(
            ['rosrun', 'ov13855_cam', 'ov13855_cam_node'] + arguments,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn(diagnostic, result.stdout)

    def test_calibration_size_mismatch(self):
        self.check_failure(['_image_width:=640'], 'calibration resolution differs')

    def test_missing_device(self):
        self.check_failure(['_video_device:=/dev/ov13855_nonexistent'], 'ov13855_cam:')

if __name__ == '__main__':
    rospy.init_node('ov13855_startup_failures')
    rostest.rosrun('ov13855_cam', 'startup_failures', StartupFailures)
