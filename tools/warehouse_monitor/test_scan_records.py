"""Regression cases for matching retry frames and older indoor scan records."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

from scan_records import frame_number, run_detail, run_ids


class ScanRecordsTest(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.TemporaryDirectory()
        self.addCleanup(self.work.cleanup)
        self.root = Path(self.work.name)
        self.run = "20261126_214843_test_indoor6-3b6d527c"
        self.folder = self.root / self.run
        self.folder.mkdir()
        self.session = self.folder / "barcode" / "session"
        self.session.mkdir(parents=True)

    def test_retry_uses_actual_frame_time_and_requires_unique_match(self):
        scan = dict(frame_ns=1795747628083532948, qr="BOX-01", session_dir="/board/barcode/session")
        csv_path = self.session / "attempts.csv"
        with csv_path.open("w") as f:
            writer = csv.writer(f)
            writer.writerow(["stamp", "result", "codes", "frames", "result_frame", "dir"])
            writer.writerow(["1795747628.016746", "ok", "QRCode:BOX-01", 2, 1, "attempt"])
        attempt = self.session / "attempt"
        attempt.mkdir()
        meta = attempt / "meta.yaml"
        self.assertIsNone(frame_number(self.folder, scan))
        meta.write_text("retries: [{index: 1, stamp: 1795747628.083533}]")
        self.assertEqual(frame_number(self.folder, scan), 2)
        self.assertIsNone(frame_number(self.folder, dict(scan, frame_ns=1795747628016746000)))
        with csv_path.open("a") as f:
            csv.writer(f).writerow(["1795747628.083533", "ok", "QRCode:BOX-01", 1, 0, "another"])
        self.assertIsNone(frame_number(self.folder, scan))

    def test_legacy_counts_and_running_task_updates(self):
        name = "20261120_164738_test_indoor4"
        folder = self.root / name
        folder.mkdir()
        goal = folder / "goal_test_indoor4_fixture.json"
        data = dict(end="RUNNING", batches=[
            dict(observation=1, name="左下", candidates=[
                dict(id=5, status="planned"), dict(id=2, status="skipped", reason="height"),
                dict(id=3, status="planned")], order=[5, 3]),
            dict(observation=2, name="左上", candidates=[
                dict(id=5, status="already_attempted"), dict(id=2, status="planned"),
                dict(id=4, status="skipped", reason="angle")], order=[2])],
            scans=[dict(id=5, observation=1, status="success", qr="BOX-05"),
                   dict(id=2, observation=2, status="running")])
        goal.write_text(json.dumps(data))
        first = run_detail(self.root, name)
        self.assertEqual((first["discovered"], first["success"], first["failed"]), (4, 1, 0))
        self.assertEqual([r["status"] for r in first["rows"]], ["success", "unattempted", "running", "skipped"])
        data["end"] = "ABORTED"
        goal.write_text(json.dumps(data))
        self.assertEqual(run_detail(self.root, name)["rows"][2]["status"], "aborted")
        data["scans"][1].update(status="failed", reason="decode_fail")
        goal.write_text(json.dumps(data))
        final = run_detail(self.root, name)
        self.assertEqual((final["state"], final["failed"]), ("ABORTED", 1))
        self.assertEqual(final["rows"][2]["reason"], "decode_fail")
        self.assertEqual(run_ids(self.root), [self.run, name])

    def test_flight_outcome_and_runs_without_goal(self):
        self.assertEqual(run_detail(self.root, self.run)["state"], "WAITING")
        (self.folder / "nav_test.log").write_text("[12:00:00] 全部记录：test")
        self.assertEqual(run_detail(self.root, self.run)["state"], "NO_RECORD")
        goal = self.folder / "goal_test_indoor6_fixture.json"
        goal.write_text('{"end":"SUCCEEDED"}')
        flight = dict(end="ABORTED", goal_record=goal.name, reason="landing check")
        (self.folder / "flight_test_indoor6_fixture.json").write_text(json.dumps(flight))
        self.assertEqual(run_detail(self.root, self.run)["state"], "ABORTED")


if __name__ == "__main__":
    unittest.main()
