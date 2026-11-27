"""Desktop cache, board launch identity and versioned record compatibility."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import board_tasks
from scan_records import run_ids, run_detail
from task_dispatch import catalog, task_for


class MissionWebTest(unittest.TestCase):
    root = Path(__file__).resolve().parent

    def test_cached_digest_and_board_command(self):
        base = catalog(self.root)
        preset = next(t for t in base['tasks'] if t.get('mission') == 'indoor_shelf.yaml')
        with TemporaryDirectory() as work:
            cache = Path(work)
            display = cache / 'display'
            display.mkdir()
            key = preset['mapId'] + '-101-102'
            (display / (key + '.bin')).touch()
            manifest = display / (key + '.json')
            manifest.write_text(json.dumps(dict(sha256='bad')))
            self.assertFalse(any(t.get('mission') for t in catalog(self.root, cache)['tasks']))
            manifest.write_text(json.dumps(dict(sha256=preset['mapSha256'])))
            data = catalog(self.root, cache)
            selected = task_for(data, preset['id'], key)
            self.assertEqual(board_tasks.script_command(selected),
                             ['rosrun','drone_inspection','run_mission','--mission','missions/indoor_shelf.yaml'])
            for stale_id, stale_map in [(preset['id'] + '0', key), (preset['id'], key + '0')]:
                with self.assertRaises(ValueError):
                    task_for(data, stale_id, stale_map)
            with patch.object(board_tasks, 'fingerprint', return_value='changed'):
                with self.assertRaisesRegex(ValueError, 'YAML'):
                    board_tasks.check_map(data, selected)

    def test_mission_progress_and_archive(self):
        run = '20261127_110000_mission_indoor_shelf-abcd'
        state = dict(version=1, run_id=run, phase='ROUTE', end='RUNNING', observation='左下',
                     action='读取 QR', discovered=1, success=0, failed=0, tagId=5)
        progress = board_tasks.Progress()
        progress.collect(['INSPECTION_EVENT ' + json.dumps(state)])
        self.assertEqual(progress.scan['tagId'], 5)
        self.assertEqual(progress.stage(dict(mission='demo.yaml'), {}, True), ('running', '读取 QR'))
        with TemporaryDirectory() as work:
            folder = Path(work) / run
            folder.mkdir()
            state.update(end='ABORTED', phase='ABORTED', reason='地图重置')
            (folder / 'mission_state.json').write_text(json.dumps(state))
            result = dict(version=1, run_id=run, batches=[], tag_counts={'discovered':1},
                          scans=[dict(observation=1, id=5, status='success', qr='BOX-05')])
            (folder / 'scan_results.json').write_text(json.dumps(result))
            self.assertIn(run, run_ids(folder.parent))
            detail = run_detail(folder.parent, run)
            self.assertEqual((detail['state'],detail['reason'],detail['success']), ('ABORTED','地图重置',1))
            result['run_id'] = 'other'
            (folder / 'scan_results.json').write_text(json.dumps(result))
            with self.assertRaises(ValueError):
                run_detail(folder.parent, run)


if __name__ == '__main__':
    unittest.main()
