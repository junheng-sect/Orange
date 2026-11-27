"""On-demand, shared read-only SSH connection to the board's scan records."""
from contextlib import suppress
import json
from pathlib import Path
import select
import shlex
import subprocess
from threading import Lock
import time

from scan_records import RUN_ID


class ScanConnection:
    def __init__(self, ssh, host):
        reader = Path(__file__).with_name("scan_records.py").read_text()
        self.command = [*ssh, "-T", "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=2",
                        host, "exec python3 -uB -c " + shlex.quote(reader)]
        self.lock = Lock()
        self.process = None
        self.cache = {}
        self.retry_at = 0

    def close(self):
        if self.process:
            with suppress(BrokenPipeError):
                self.process.stdin.close()
            try:
                self.process.wait(timeout=.3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            self.process.stdout.close()
            self.process = None

    def read(self, key=""):
        if key and not RUN_ID.fullmatch(key):
            raise ValueError("无效实验")
        with self.lock:
            prior, checked = self.cache.get(key, ({}, 0))
            now = time.monotonic()
            if prior and now - checked < 1:
                return prior
            error = "板端连接中断"
            if now >= self.retry_at:
                try:
                    if self.process is None or self.process.poll() is not None:
                        self.close()
                        self.process = subprocess.Popen(self.command, stdin=subprocess.PIPE,
                                                        stdout=subprocess.PIPE)
                    proc = self.process
                    proc.stdin.write((key + "\n").encode())
                    proc.stdin.flush()
                    if not select.select([proc.stdout], [], [], 7)[0]:
                        raise TimeoutError()
                    line = proc.stdout.readline()
                    if not line:
                        raise ConnectionError()
                    reply = json.loads(line)
                    if "data" in reply:
                        result = dict(online=True, updatedAt=int(time.time()), data=reply["data"])
                        self.cache[key] = (result, time.monotonic())
                        return result
                    error = reply["error"]
                except (OSError, ValueError):
                    self.close()
                    self.retry_at = time.monotonic() + 2
            return dict(online=False, updatedAt=prior.get("updatedAt"), data=prior.get("data"), error=error)
