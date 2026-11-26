from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import board_tasks


class IndoorDispatchTests(unittest.TestCase):
    def test_map_frame_comes_from_selected_script_workspace(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp) / 'Orange checkout'
            scripts = workspace / 'scripts'
            frame = workspace / 'src/localization/car_bridge/config/map_frame_indoor_fly.yaml'
            frame.parent.mkdir(parents=True)
            frame.touch()
            with patch.object(board_tasks, 'SCRIPTS', scripts):
                for task_id in ('test_indoor4', 'test_indoor5'):
                    args = board_tasks.script_command({'id': task_id, 'mapId': 'indoor'})
                    self.assertEqual(args[0], str(scripts / f'nav_{task_id}.sh'))
                    self.assertEqual(args[args.index('--map-frame') + 1], str(frame))
                    self.assertEqual(args[args.index('--nav-config') + 1],
                                     str(scripts / 'nav_test/indoor_fly_nav.yaml'))
