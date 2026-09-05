#!/usr/bin/env python3
"""Standalone tests for the vz_step bag sequence allocator.

Run with:  python3 test/test_sequence_allocation.py
"""

import importlib.util
import multiprocessing
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_PATH = os.path.join(
    TEST_DIR, os.pardir, 'scripts', 'sequence_allocator.py'
)

_spec = importlib.util.spec_from_file_location('sequence_allocator', MODULE_PATH)
sequence_allocator = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sequence_allocator)

SequenceAllocator = sequence_allocator.SequenceAllocator

CHILD_CODE = '''
import importlib.util, sys
spec = importlib.util.spec_from_file_location("sequence_allocator", sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
allocator = module.SequenceAllocator(sys.argv[2], "vz_step")
print(allocator.allocate(), flush=True)
'''


class SequenceAllocationTests(unittest.TestCase):
    def setUp(self):
        self.bag_dir = tempfile.mkdtemp(prefix='vz_step_seq_test_')
        self.addCleanup(shutil.rmtree, self.bag_dir, True)

    def alloc(self):
        return SequenceAllocator(self.bag_dir, 'vz_step').allocate()

    def test_empty_directory_starts_at_one(self):
        path = self.alloc()
        self.assertEqual(
            os.path.basename(path), 'vz_step_000001.bag'
        )
        self.assertTrue(
            os.path.exists(os.path.join(self.bag_dir, '.vz_step_sequence'))
        )
        self.assertTrue(
            os.path.exists(
                os.path.join(self.bag_dir, '.vz_step_sequence.lock')
            )
        )

    def test_state_file_advances_sequence(self):
        self.alloc()
        path = self.alloc()
        self.assertEqual(os.path.basename(path), 'vz_step_000002.bag')

    def test_existing_bags_are_never_reused(self):
        # A bag that the state file does not know about (e.g. copied in) must
        # still push the sequence forward.
        open(os.path.join(self.bag_dir, 'vz_step_000005.bag'), 'w').close()
        path = self.alloc()
        self.assertEqual(os.path.basename(path), 'vz_step_000006.bag')

    def test_active_bags_are_never_reused(self):
        open(
            os.path.join(self.bag_dir, 'vz_step_000003.bag.active'), 'w'
        ).close()
        path = self.alloc()
        self.assertEqual(os.path.basename(path), 'vz_step_000004.bag')

    def test_state_file_beats_existing_files(self):
        with open(os.path.join(self.bag_dir, '.vz_step_sequence'), 'w') as f:
            f.write('9\n')
        path = self.alloc()
        self.assertEqual(os.path.basename(path), 'vz_step_000010.bag')

    def test_invalid_state_file_raises(self):
        with open(os.path.join(self.bag_dir, '.vz_step_sequence'), 'w') as f:
            f.write('not-a-number\n')
        with self.assertRaises(ValueError):
            self.alloc()

    def test_concurrent_allocators_get_distinct_sequences(self):
        workers = 4
        procs = [
            subprocess.Popen(
                [sys.executable, '-c', CHILD_CODE, MODULE_PATH, self.bag_dir],
                stdout=subprocess.PIPE,
                text=True,
            )
            for _ in range(workers)
        ]
        names = sorted(
            os.path.basename(p.communicate(timeout=30)[0].strip())
            for p in procs
        )
        for p in procs:
            self.assertEqual(p.returncode, 0)
        self.assertEqual(
            names,
            [
                'vz_step_000001.bag',
                'vz_step_000002.bag',
                'vz_step_000003.bag',
                'vz_step_000004.bag',
            ],
        )

    def test_concurrent_processes_through_multiprocessing(self):
        def worker(queue):
            queue.put(os.path.basename(self.alloc()))

        queue = multiprocessing.Queue()
        procs = [
            multiprocessing.Process(target=worker, args=(queue,))
            for _ in range(3)
        ]
        for p in procs:
            p.start()
        names = sorted(queue.get() for _ in procs)
        for p in procs:
            p.join(timeout=30)
            self.assertEqual(p.exitcode, 0)
        self.assertEqual(
            names,
            [
                'vz_step_000001.bag',
                'vz_step_000002.bag',
                'vz_step_000003.bag',
            ],
        )


if __name__ == '__main__':
    unittest.main(verbosity=2)
