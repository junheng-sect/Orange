#!/usr/bin/env python3
"""Verify car_web/serve.sh resolves its own directory across platforms and symlinks."""
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import unittest
from urllib.error import HTTPError
from urllib.request import ProxyHandler, build_opener

SCRIPT = Path(__file__).resolve().with_name('serve.sh')
HTTP = build_opener(ProxyHandler({}))


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class ServeScriptTests(unittest.TestCase):
    def assert_serves_car_web_only(self, launcher, cwd):
        port = free_port()
        with subprocess.Popen(
            [str(launcher), str(port)],
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        ) as process:
            try:
                deadline = time.monotonic() + 5.0
                body = None
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        _, stderr = process.communicate(timeout=2)
                        self.fail(f'serve.sh exited early ({process.returncode}): {stderr}')
                    try:
                        with HTTP.open(f'http://127.0.0.1:{port}/index.html', timeout=0.5) as response:
                            body = response.read()
                            cache_control = response.headers.get('Cache-Control')
                            break
                    except OSError:
                        time.sleep(0.05)
                self.assertIsNotNone(body, 'timed out waiting for serve.sh to answer')
                self.assertEqual(body, (SCRIPT.parent / 'index.html').read_bytes())
                self.assertEqual(cache_control, 'no-cache')
                with self.assertRaises(HTTPError) as ctx:
                    HTTP.open(f'http://127.0.0.1:{port}/secret.env', timeout=0.5)
                self.assertEqual(ctx.exception.code, 404)
            finally:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)

    def test_serves_script_directory_from_other_cwd_and_symlink(self):
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            (work / 'secret.env').write_text('TOP_SECRET=1\n')
            symlink = work / 'car-web-serve'
            symlink.symlink_to(SCRIPT)
            for launcher in (SCRIPT, symlink):
                with self.subTest(launcher=launcher.name):
                    self.assert_serves_car_web_only(launcher, work)


if __name__ == '__main__':
    unittest.main()
