#!/usr/bin/env python3
"""Check test12 dispatch, YWL display, and FAST-LIO2 fallback."""
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import rospy

import board_tasks
from monitor import Monitor, OUTDOOR_OBSERVER_MAP
from task_dispatch import catalog, task_for


class YwlMonitor(unittest.TestCase):
    def test_catalog_dispatches_both_manual_modes(self):
        data = catalog(Path(__file__).resolve().parent)
        key = data["maps"][OUTDOOR_OBSERVER_MAP]["key"]
        dll = task_for(data, "test12", key)
        ywl = task_for(data, "test12_ywl", key)
        self.assertEqual(board_tasks.script_command(dll), [str(board_tasks.SCRIPTS / "nav_test12.sh")])
        self.assertEqual(board_tasks.script_command(ywl), [str(board_tasks.SCRIPTS / "nav_test12.sh"), "--ywl"])
        self.assertTrue(ywl["manual"])
        self.assertFalse(ywl["automatic"])

    def test_switch_preserves_mount_and_breaks_the_trail(self):
        now = time.monotonic()
        stamp = 104_000_000_000
        monitor = Monitor(Path("/tmp/maps"))
        monitor.map_path = monitor.maps / OUTDOOR_OBSERVER_MAP / "map_world.pcd"
        monitor.map_key = "outdoor"
        monitor.solution = (np.array([0, 0, 0, 1]), np.zeros(3))
        monitor.transform = (now, *monitor.solution)
        monitor.relocation = (now, True, "LOCALIZED")
        monitor.source = "fastlio"
        monitor.pose = [104., 7., 8., 9., 0., 0., 0., 1.]
        monitor.poses["fastlio"] = (now, 104.)
        monitor.ywl_result_stamp = stamp - 200_000_000
        monitor.ywl_seed_stamp = stamp - 100_000_000
        mount = np.array([0, 0, np.sqrt(.5), np.sqrt(.5)])
        monitor.imu_to_body = (stamp - 1_000_000_000, mount)
        monitor.ywl_prior_hashes = ("pcd", "meta")
        monitor.ywl_status = (now, "TRACKING", "fixed prior", True)
        monitor.cache_ywl(stamp, "pose", ([1., 2., 3.], np.array([0, 0, 0, 1.])))
        monitor.cache_ywl(stamp, "status", ("pcd", "meta"))
        with patch.object(rospy.Time, "now", return_value=rospy.Time(104)):
            ywl = monitor.snapshot({})
            self.assertEqual(ywl["poseSource"], "ywl")
            self.assertEqual(ywl["pose"][1:4], [1., 2., 3.])
            np.testing.assert_allclose(ywl["pose"][4:], mount)
            monitor.ywl_seed_stamp = stamp - 2_000_000_000
            old = monitor.snapshot({})
            self.assertEqual(old["poseSource"], "fastlio")
            self.assertNotEqual(ywl["epoch"], old["epoch"])
            monitor.ywl_seed_stamp = stamp - 100_000_000
            monitor.cache_ywl(stamp, "status", ("wrong", "map"))
            self.assertIn("地图校验不一致", monitor.snapshot({})["ywlReason"])
            monitor.cache_ywl(stamp, "status", ("pcd", "meta"))
            self.assertEqual(monitor.snapshot({})["poseSource"], "ywl")
            monitor.on_ywl_status(SimpleNamespace(
                header=SimpleNamespace(stamp=rospy.Time(104)),
                status=[SimpleNamespace(name="ywl_localization", message="FAILED", level=2,
                                        values=[SimpleNamespace(key="reason", value="no valid match")])]))
            fallback = monitor.snapshot({})
            self.assertEqual(fallback["poseSource"], "fastlio")
            self.assertIn("no valid match", fallback["ywlReason"])
            self.assertEqual(fallback["trail"], [])


if __name__ == "__main__":
    unittest.main()
