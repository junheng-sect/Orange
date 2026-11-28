"""Contract between the indoor preset, desktop queue, board receipt and CLI."""
import json
from pathlib import Path
import subprocess
import tempfile
from threading import Event
import unittest
from unittest.mock import patch

import yaml
import board_tasks
from task_dispatch import Dispatch, TaskConflict, catalog, task_for, vertical_profile


class VerticalDispatchTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.cache = Path(self.temp.name)
        self.data = catalog(board_tasks.ROOT)
        version = self.data['maps'][vertical_profile.MAP]
        self.key = version['key']
        self.task = task_for(self.data, vertical_profile.TASK, self.key)
        display = self.cache / 'display'
        display.mkdir()
        (display / (self.key + '.json')).write_text(json.dumps(version))
        (display / (self.key + '.bin')).touch()
        self.queue = Dispatch(self.data, self.cache, 'mock', [], Event())
        self.body = dict(id='a'*32, taskId=vertical_profile.TASK, mapKey=self.key, confirmAuto=True,
                         vertical=dict(height=.73, hover=4.5, delay=2.25))

    def test_round_trip_and_replay(self):
        with patch.object(board_tasks, 'STATE', self.cache / 'board'), \
                patch.object(board_tasks, 'catalog', return_value=self.data), \
                patch.object(board_tasks, 'session_exists', return_value=False), \
                patch.object(board_tasks, 'tmux') as tmux, patch.object(self.queue, 'remote', board_tasks.rpc):
            tmux.return_value.returncode = 0
            self.queue.submit(self.body)
            self.queue.tick()
            self.queue.submit(self.body)
            payload = dict(self.body, remaining=60, boot=board_tasks.BOOT)
            self.assertNotIn('error', board_tasks.rpc('submit', payload))
            tmux.assert_called_once()
            changed = dict(self.body, vertical=dict(height=.51, hover=4.5, delay=2.25))
            with self.assertRaises(TaskConflict):
                self.queue.submit(changed)
            self.assertIn('error', board_tasks.rpc('submit', dict(payload, vertical=changed['vertical'])))
            record = board_tasks.get_record(self.body['id'])
        command = board_tasks.script_command(self.task, vertical=record['vertical'])
        self.assertEqual(command[1:], ['--height', '0.73', '--hover', '4.5', '--delay', '2.25'])
        dry = subprocess.check_output(command + ['--dry-run'], text=True)
        for value in ('0.73 m', '4.5 s', '2.25 s'):
            self.assertIn(value, dry)
        resolved = self.cache / 'navigation.resolved.yaml'
        cfg = board_tasks.SCRIPTS / 'nav_test/indoor_fly_nav.yaml'
        subprocess.run(['bash', '-c', 'source "$1"; shift; nt_write_nav_config "$@"', 'test',
                        str(cfg.with_name('common.sh')), str(cfg), str(Path.home()/'test_data/mapping'/vertical_profile.MAP),
                        '0', str(resolved), self.task['id'], command[2]], check=True, capture_output=True)
        settings = yaml.safe_load(resolved.read_text())['config']
        self.assertEqual(settings['takeoff_height'], record['vertical']['height'])
        self.assertIs(settings['vertical_test_guard'], True)

    def test_invalid_options_never_queue(self):
        values = self.body['vertical']
        for bad in ({}, None, dict(values, height=.29), dict(values, hover=31), dict(values, delay=.5),
                    dict(values, height=True), dict(values, height=float('nan')), dict(values, other=1)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.queue.submit(dict(self.body, vertical=bad))
        for change in (dict(taskId='test_indoor6'), dict(mapKey='outdoor'), dict(confirmAuto=False)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.queue.submit(dict(self.body, **change))
        self.assertIsNone(self.queue.request)
