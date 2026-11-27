import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from archive_runtime_logs import archive, writer


class RuntimeLogsTest(unittest.TestCase):
    def test_copy_conflict_retains_logs_and_allows_retry(self):
        root = Path(tempfile.mkdtemp(prefix='nav_test_logs.', dir='/dev/shm'))
        self.addCleanup(shutil.rmtree, root, True)
        with tempfile.TemporaryDirectory() as dest:
            task = Path(dest) / 'task'
            fusion = Path(dest) / 'fusion'
            run = root / 'task/ros_log/run'
            run.mkdir(parents=True)
            (run / 'node.log').write_text('finished')
            (run.parent / 'latest').symlink_to(run)
            csv = root / 'fusion/watch.csv'
            csv.parent.mkdir()
            with csv.open('w') as stream:
                stream.write('last sample')
                self.assertEqual(writer(root), str(os.getpid()))
            fusion.mkdir()
            (fusion / csv.name).write_text('previous run')
            with self.assertRaises(FileExistsError):
                archive(root, task, fusion)
            self.assertEqual(csv.read_text(), 'last sample')
            self.assertEqual((fusion / csv.name).read_text(), 'previous run')
            (fusion / csv.name).rename(fusion / 'previous.csv')
            archive(root, task, fusion)
            self.assertFalse(root.exists())
            self.assertEqual((fusion / csv.name).read_text(), 'last sample')
            self.assertEqual((task / 'ros_log/latest/node.log').read_text(), 'finished')


if __name__ == '__main__':
    unittest.main()
