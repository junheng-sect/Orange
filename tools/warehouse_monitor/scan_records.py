"""Read indoor scan records on the board. Also runs over SSH without a board install."""
import csv
from decimal import Decimal, InvalidOperation
from functools import lru_cache
import json
from pathlib import Path
import re
import select
import sys

RUN_ID = re.compile(r"[0-9]{8}_[0-9]{6}_test_indoor[456](?:-[A-Za-z0-9_-]+)?")
TERMINAL = {"SUCCEEDED", "ABORTED", "INTERRUPTED"}


def load_file(path):
    stat = path.stat()
    return parse_file(path, stat.st_ino, stat.st_size, stat.st_mtime_ns)


@lru_cache(maxsize=96)
def parse_file(path, inode, size, mtime):
    with path.open(encoding="utf-8", newline="") as stream:
        if path.suffix == ".csv":
            return list(csv.DictReader(stream))
        if path.suffix == ".yaml":
            try:
                import yaml
            except ImportError as error:
                raise ValueError("无法读取相机帧元数据") from error
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError as error:
                raise ValueError("相机帧元数据尚不完整") from error
        if path.suffix == ".log":
            # A run can finish its preflight before it creates a goal file.
            return any("全部记录：" in line for line in stream)
        return json.load(stream)


def run_ids(root):
    if not root.exists():
        return []
    return sorted((p.name for p in root.iterdir() if RUN_ID.fullmatch(p.name)
                   and not p.is_symlink() and p.is_dir()), reverse=True)


def frame_number(folder, scan):
    """Both QR text and the producing frame's timestamp must match uniquely."""
    if not scan.get("frame_ns") or not scan.get("session_dir"):
        return None
    session = folder / "barcode" / Path(scan["session_dir"]).name
    attempts = session / "attempts.csv"
    if not attempts.is_file() or attempts.resolve().parent.parent != folder.resolve() / "barcode":
        return None
    matches = []
    for row in load_file(attempts):
        if row.get("result") != "ok" or row.get("codes") != "QRCode:" + scan["qr"]:
            continue
        try:
            index = int(row["result_frame"])
            if not 0 <= index < int(row["frames"]):
                continue
            stamp = row["stamp"]
            if index:
                attempt = row["dir"]
                if not attempt or Path(attempt).name != attempt:
                    continue
                meta = session / attempt / "meta.yaml"
                if meta.resolve().parent.parent != session:
                    continue
                retry = next(r for r in load_file(meta)["retries"] if r["index"] == index)
                stamp = retry["stamp"]
            # Camera diagnostics round seconds to six decimal places.
            delta_ns = Decimal(str(stamp)) * 10**9 - scan["frame_ns"]
            if abs(delta_ns) <= 1000:
                matches.append(index + 1)
        except (OSError, KeyError, StopIteration, TypeError, ValueError, InvalidOperation):
            continue
    return matches[0] if len(matches) == 1 else None


def scan_rows(folder, goal, end):
    batches = goal.get("batches", [])
    rows = {}
    scans = {(s["observation"], s.get("family", "tag36h11"), s["id"]): s
             for s in goal.get("scans", [])}

    def add(name, item, scan=None):
        key = (item.get("family", "tag36h11"), item["id"])
        state = item.get("status")
        if not scan and (state == "already_attempted" or
                         (key in rows and rows[key]["status"] not in ("skipped", "unattempted"))):
            return
        frame = None
        if scan:
            state = scan["status"]
            if state == "success":
                frame = frame_number(folder, scan)
            elif state == "running" and end in TERMINAL:
                state = "aborted"
        elif state != "skipped":
            state = "unattempted"
        source = scan or item
        # A skipped tag can be planned again at a subsequent observation point.
        rows.pop(key, None)
        rows[key] = dict(observation=name, tag=key[1], status=state, qr=source.get("qr"),
                         reason=source.get("reason"), frame=frame)

    for batch in batches:
        obs, name = batch["observation"], batch["name"]
        candidates = {c["id"]: c for c in batch.get("candidates", [])}
        for item in batch.get("observations", []):
            candidates.setdefault(item["id"], item)
        for tag in dict.fromkeys([*batch.get("order", []), *candidates]):
            item = candidates.get(tag, {"id": tag})
            scan = scans.pop((obs, item.get("family", "tag36h11"), tag), None)
            add(name, item, scan)
    for (obs, _, _), scan in scans.items():
        name = next((b["name"] for b in batches if b["observation"] == obs), str(obs))
        add(name, scan, scan)
    return list(rows.values())


def run_detail(root, run_id):
    if not RUN_ID.fullmatch(run_id):
        raise ValueError("无效实验")
    folder = root / run_id
    if folder.is_symlink() or not folder.is_dir():
        raise FileNotFoundError(run_id)
    task = run_id[16:].split("-")[0]
    goals = sorted(folder.glob("goal_" + task + "_*.json"))
    flights = sorted(folder.glob("flight_" + task + "_*.json"))
    flight = load_file(flights[-1]) if flights else {}
    if flight.get("goal_record"):
        goal_path = folder / Path(flight["goal_record"]).name
    else:
        goal_path = goals[-1] if goals else None
    goal = load_file(goal_path) if goal_path and goal_path.is_file() else {}
    end = flight.get("end") or goal.get("end") or "WAITING"
    log = folder / "nav_test.log"
    if end not in TERMINAL and log.is_file() and load_file(log):
        end = "INTERRUPTED" if goal or flight else "NO_RECORD"
    rows = scan_rows(folder, goal, end)
    counts = goal.get("tag_counts", {})
    return dict(id=run_id, state=end, reason=flight.get("reason") or goal.get("reason"),
                discovered=counts.get("discovered", len(rows)),
                success=counts.get("read_ok", sum(r["status"] == "success" for r in rows)),
                failed=sum(r["status"] == "failed" for r in rows), rows=rows)


def main():
    root = Path.home() / "test_data/nav_test"
    # Release the remote interpreter when all visible viewers stop polling.
    while select.select([sys.stdin], [], [], 20)[0]:
        line = sys.stdin.readline()
        if not line:
            break
        key = line.rstrip("\n")
        try:
            value = run_detail(root, key) if key else run_ids(root)
            reply = dict(data=value)
        except FileNotFoundError:
            reply = dict(error="实验记录不存在")
        except (OSError, ValueError, TypeError, KeyError, csv.Error):
            reply = dict(error="实验记录暂时无法读取")
        print(json.dumps(reply, ensure_ascii=True, separators=(",", ":")), flush=True)


if __name__ == "__main__":
    main()
