"""Startup regressions on loopback, with a private master and no flight hardware."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import yaml

NAV = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAV))
import preflight_probe as probe


class UnitTests(unittest.TestCase):
    def test_rtc_discards_old_and_foreign_frames(self):
        spec = importlib.util.spec_from_file_location('fc_set_time', NAV.parent / 'fc_set_time.py')
        rtc = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, pymavlink=MagicMock()):
            spec.loader.exec_module(rtc)
        def msg(sysid, utc):
            m = MagicMock(time_unix_usec=utc * 1e6)
            m.get_srcSystem.return_value = sysid
            return m
        link = MagicMock()
        link.recv_match.side_effect = [msg(1, 80), None, msg(2, 100), msg(1, 90), msg(1, 100.5)]
        with patch.object(rtc.time, 'time', return_value=100):
            self.assertEqual(rtc.wait_aligned_offset(link, 1, 1), .5)
        self.assertEqual(link.recv_match.call_count, 5)
        link.recv_match.side_effect = None
        link.recv_match.return_value = None
        self.assertIsNone(rtc.wait_aligned_offset(link, 1, 1, timeout=.01))

    def test_receipt_fallback(self):
        from drone_mapping import prior_grid
        sha = 'a' * 64
        with tempfile.TemporaryDirectory() as tmp, patch.object(prior_grid, 'verify', return_value=sha) as full:
            folder = Path(tmp)
            pcd, prior = folder / 'map_world.pcd', folder / 'prior_grid.bin'
            pcd.write_bytes(b'pcd')
            prior.write_bytes(b'grid')
            receipt = sha + ':' + prior_grid.file_stamp(folder)
            self.assertEqual(probe.verified_prior(folder, sha, receipt), sha)
            full.assert_not_called()
            for invalid in (None, 'bad', receipt.replace(sha, 'b'*64)):
                self.assertEqual(probe.verified_prior(folder, sha, invalid), sha)
            before = prior.stat()
            os.utime(prior, ns=(before.st_atime_ns, before.st_mtime_ns + 1000000))
            self.assertEqual(probe.verified_prior(folder, sha, receipt), sha)
            receipt = sha + ':' + prior_grid.file_stamp(folder)
            pcd.write_bytes(b'new pcd')
            self.assertEqual(probe.verified_prior(folder, sha, receipt), sha)
            self.assertEqual(full.call_count, 5)
            with self.assertRaises(ValueError):
                probe.verified_prior(folder, 'c'*64, receipt)
            probe.verified_prior(folder, None, receipt)
            self.assertEqual(full.call_count, 7)

    def test_nav_config_and_limits(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'config.yaml'
            doc = yaml.safe_load((NAV / 'indoor_fly_nav.yaml').read_text())
            cfg = doc['config']
            cfg.update(map_id=tmp, deployment_verified=False)
            with self.assertRaisesRegex(ValueError, 'map_id'):
                probe.write_config(doc, '/different', True, output, 'test', None)
            with self.assertRaisesRegex(ValueError, 'deployment_verified'):
                probe.write_config(doc, tmp, False, output, 'test', None)
            probe.write_config(doc, tmp, True, output, 'test_indoor_takeoff_land', '.65')
            self.assertEqual(yaml.safe_load(output.read_text())['config']['takeoff_height'], .65)
            probe.write_config(doc, tmp, True, output, 'test_indoor6', '')
            self.assertEqual(cfg['takeoff_height'], .50)
            cfg['planner_limits'] = {'max_vel': .2}
            self.assertEqual(probe.ego_limits(cfg, 'test')[0], .2)
            cfg['planner_limits']['max_vel'] = cfg['max_vel'] + 1
            with self.assertRaises(ValueError):
                probe.ego_limits(cfg, 'test')


class RosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rosgraph
        import rospy
        from mavros_msgs.msg import State
        from mavros_msgs.srv import ParamGet, ParamGetResponse
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        root = Path(cls.tmp.name)
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        env = patch.dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_IP='127.0.0.1',
                         ROS_LOG_DIR=str(root / 'logs'), ROS_HOME=str(root / 'ros'))
        env.start()
        cls.addClassCleanup(env.stop)
        server = subprocess.Popen(['roscore', '-p', str(port)], start_new_session=True,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        def cleanup():
            os.killpg(server.pid, signal.SIGINT)
            server.wait(timeout=15)
        cls.addClassCleanup(cleanup)
        for _ in range(100):
            try:
                rosgraph.Master('/startup_fixture').getPid()
                break
            except OSError:
                time.sleep(.05)
        rospy.init_node('startup_fixture', anonymous=True, disable_signals=True)
        cls.addClassCleanup(rospy.signal_shutdown, 'done')
        rospy.set_param('/mavros/fcu_protocol', 'v2.0')
        cls.value = 15
        cls.delay = 0.
        def get(request):
            assert request.param_id == 'EKF2_EV_CTRL'
            time.sleep(cls.delay)
            reply = ParamGetResponse(success=True)
            reply.value.integer = cls.value
            return reply
        cls.service = rospy.Service('/mavros/param/get', ParamGet, get)
        cls.pubs = []
        for name, kind, msg in [
                ('/mavros/state', State, State(connected=True)),
                ('/fastlio/odom_world', Odometry, Odometry()),
                ('/drone_reloc/pose', PoseStamped, PoseStamped()),
                ('/drone_reloc/status', String, String(json.dumps(dict(state='LOCALIZED', valid=True)))),
                ('/drone_nav/input_status', String, String('{"state":"READY"}')),
                ('/drone_nav/gateway_status', String, String('{"real_output":false}'))]:
            pub = rospy.Publisher(name, kind, queue_size=1, latch=True)
            cls.pubs.append(pub)
            pub.publish(msg)

    def call_probe(self, *args):
        return subprocess.run([sys.executable, '-B', str(NAV/'preflight_probe.py'), *map(str, args)],
                              capture_output=True, text=True, timeout=12)

    def test_ev_service_and_deadline(self):
        for val, code in ((15, 0), (3, 2)):
            type(self).value = val
            result = self.call_probe('check-ev-ctrl', '--timeout', '3')
            self.assertEqual((result.returncode, result.stdout.strip()), (code, str(val)), result.stderr)
        type(self).delay = 4
        try:
            result = self.call_probe('check-ev-ctrl', '--timeout', '1')
            self.assertEqual((result.returncode, result.stdout.strip()), (1, 'unavailable'), result.stderr)
        finally:
            type(self).delay = 0

    def test_fusion_exit_and_camera_conflict(self):
        import rospy
        from std_msgs.msg import String
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            pf = folder / 'fusion.pid'
            pf.write_text('99999999 1')
            self.assertEqual(self.call_probe('wait-odom', pf).returncode, 1)
            camera = rospy.Publisher('/ov13855/image_raw', String, queue_size=1)
            try:
                res = self.call_probe('preflight', '--map', tmp, '--live-grid-only', '--task', 'test',
                                      '--ev-mode', 'odom', '--config', 'unused', '--output', 'unused',
                                      '--reloc-config', 'unused', '--forbid-topics-regex', '/ov13855/image_raw')
                self.assertNotEqual(res.returncode, 0)
                self.assertIn('相机或读码服务已有任务', res.stderr)
            finally:
                camera.unregister()

    def test_supervisor_orders_gate_and_cleans_children(self):
        from drone_mapping import mapio
        from drone_relocalization.matching import MapModel
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            ground = np.indices((40, 40)).reshape(2, -1).T * .05
            points = np.column_stack((ground, np.zeros(len(ground)))).astype('f4')
            mapio.write_pcd(str(folder/'map_world.pcd'), points, np.zeros(len(points), 'f4'))
            (folder/'meta.json').write_text('{"floor":{"trusted":true,"height_m":0}}')
            (folder/'reloc.yaml').write_text('{}')
            MapModel(folder)
            doc = yaml.safe_load((NAV/'indoor_fly_nav.yaml').read_text())
            doc['config']['map_id'] = tmp
            (folder/'config.yaml').write_text(yaml.safe_dump(doc))
            (folder/'ros_net.sh').touch()
            (folder/'fusion.sh').touch()
            for value, expected in ((3, 1), (15, 0)):
                type(self).value = value
                result = subprocess.run(['bash', str(NAV/'tests/startup_supervisor_fixture.sh'), str(NAV), tmp],
                                        capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
                self.assertIn('重定位完成', result.stdout, result.stderr)
                self.assertEqual('步骤 4/4' in result.stdout, value == 15)
                self.assertEqual((folder/'gateway.started').exists(), value == 15)
                self.assertFalse(any((folder/'identities').glob('*.alive')))
                for identity in (folder/'identities').glob('*.pid'):
                    pid, start = identity.read_text().split()
                    stat = Path('/proc', pid, 'stat')
                    if stat.exists():
                        self.assertNotEqual(stat.read_text().rsplit(')', 1)[1].split()[19], start)


if __name__ == '__main__':
    unittest.main()
