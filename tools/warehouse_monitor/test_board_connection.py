from pathlib import Path
from threading import Event
import json
import os
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import board_connection
import desktop
from camera_view import Camera, REMOTE
from task_dispatch import Dispatch


class BoardConnectionTests(unittest.TestCase):
    def test_camera_bootstrap_on_split_and_merged_workspaces(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for directory in ('devel', 'scripts', 'tools/warehouse_monitor'):
                (root / directory).mkdir(parents=True)
            (root / 'setup-noetic.bash').write_text('export CAMERA_TEST_ENV=noetic\n')
            (root / 'devel/setup.bash').write_text('export CAMERA_TEST_ENV+=-workspace\n')
            (root / 'tools/warehouse_monitor/camera_sender.py').write_text(
                'import os\nprint(os.environ["CAMERA_TEST_ENV"])\n')
            command = REMOTE.replace('/opt/ros/noetic/setup.bash', './setup-noetic.bash')
            helper = root / 'scripts/ros_net.sh'
            for script, expected, code in ((None, 'noetic-workspace', 0),
                                           ('export CAMERA_TEST_ENV+=-network', 'noetic-workspace-network', 0),
                                           ('return 1', '', 1)):
                with self.subTest(script=script):
                    if script is not None:
                        helper.write_text(script + '\n')
                    result = subprocess.run(['bash', '-c', command], cwd=root,
                                            capture_output=True, text=True)
                    self.assertEqual(result.returncode, code, result.stderr)
                    self.assertEqual(result.stdout.strip(), expected)

    def test_settings_precedence(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {}, clear=True):
            env = Path(tmp) / '.env'
            self.assertEqual(board_connection.settings(env),
                             {'ORANGE_SSH_HOST': 'orange', 'ORANGE_REMOTE_ROOT': '~/orange'})
            env.write_text('ORANGE_SSH_HOST=lab\nORANGE_REMOTE_ROOT="Lab checkout"\n')
            os.environ['ORANGE_SSH_HOST'] = 'field'
            self.assertEqual(board_connection.settings(env),
                             {'ORANGE_SSH_HOST': 'field', 'ORANGE_REMOTE_ROOT': 'Lab checkout'})

    def test_explicit_host_overrides_environment(self):
        with patch.dict(os.environ, ORANGE_SSH_HOST='configured-board'), \
                patch('sys.argv', ['warehouse-monitor', '--host', 'selected-board', '--sync', 'sample']), \
                patch.object(desktop, 'sync_map') as sync:
            desktop.main()
            self.assertEqual(sync.call_args.args[1:], ('selected-board', 'sample'))

    def test_checkout_and_map_paths_on_remote_host(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp) / 'board home'
            workspace = home / "Pilot's checkout"
            tools = workspace / 'tools/warehouse_monitor'
            tools.mkdir(parents=True)
            fake_ssh = Path(tmp) / 'ssh'
            fake_ssh.write_text('#!/bin/bash\n[[ "${@: -2:1}" == orange ]] || exit 2\nexec /bin/sh -c "${!#}"\n')
            fake_ssh.chmod(0o755)
            ssh = [str(fake_ssh)]
            (tools / 'board_tasks.py').write_text(
                'import sys,json\nprint(json.dumps([sys.argv[1], json.loads(sys.argv[2])]))\n')
            serve = tools / 'serve.sh'
            serve.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > called\n')
            serve.chmod(0o755)
            pcd = home / 'test_data/mapping/example/map_world.pcd'
            pcd.parent.mkdir(parents=True)
            pcd.write_bytes(b'synthetic map')
            cache = Path(tmp) / 'cache'
            with patch.dict(os.environ, HOME=str(home)), patch.object(desktop, 'SSH', ssh), \
                    patch.object(desktop, 'RVIZ_MAPS', Path(tmp) / 'empty'), \
                    patch.object(desktop, 'display_map', return_value=({'counts': {}}, b'')):
                desktop.sync_map(cache, 'orange', 'example')
                self.assertEqual((cache / 'maps/example/map_world.pcd').read_bytes(), pcd.read_bytes())
                for root in ('~/' + workspace.name, workspace.name, str(workspace)):
                    with self.subTest(root=root):
                        dispatch = Dispatch({}, cache, 'orange', ssh, Event(), root)
                        body = {'map': "Shelf A's"}
                        self.assertEqual(dispatch.remote('status', body), ['status', body])
                        with desktop.tunnel('orange', 8766, 8767, True, root) as process:
                            self.assertEqual(process.wait(timeout=4), 0)
                        self.assertIn('--board-status', (workspace / 'called').read_text().splitlines())
                        # Replace just the ROS payload; exercise Camera's real SSH argv and remote cd.
                        with patch('camera_view.REMOTE', 'pwd -P'):
                            command = Camera(ssh, 'orange', Event(), root).command
                            result = subprocess.check_output(command, text=True)
                        self.assertEqual(result.strip(), str(workspace.resolve()))
