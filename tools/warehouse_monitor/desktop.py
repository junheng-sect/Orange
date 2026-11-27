#!/usr/bin/env python3
"""Serve local maps, Orange Pi telemetry and preset task preparation."""
import argparse
from contextlib import closing, contextmanager, nullcontext
from http.client import HTTPException
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import tempfile
from threading import Event, Lock, Thread
import time
from urllib.error import URLError
from urllib.parse import parse_qs, urlsplit
from urllib.request import ProxyHandler, build_opener

from board_connection import DEFAULT_ROOT, fingerprint, in_checkout, remote_path, settings
from camera_view import Camera, MODES
from map_cloud import display_map
from scan_connection import ScanConnection
from inspection import Conflict, Inspection
from task_dispatch import Dispatch, TaskConflict, catalog


ROOT = Path(__file__).resolve().parent
DEFAULT_CACHE = Path.home() / ".cache/warehouse-monitor"
RVIZ_MAPS = Path(os.environ.get("XDG_DATA_HOME") or Path.home() / ".local/share") / "ego-swarm-rviz/maps"
REMOTE_MONITOR = "./tools/warehouse_monitor/serve.sh"
REMOTE_MAPS = "test_data/mapping"
MAP_ID = re.compile(r"[A-Za-z0-9_-]{1,160}")
MAP_KEY = re.compile(r"([A-Za-z0-9_-]{1,160})-(\d{1,24})-(\d{1,15})")
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5"]
HTTP = build_opener(ProxyHandler({}))
STATIC = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/scans.js": ("scans.js", "text/javascript; charset=utf-8"),
    "/camera.js": ("camera.js", "text/javascript; charset=utf-8"),
    "/monitor.js": ("monitor.js", "text/javascript; charset=utf-8"),
    "/monitor.css": ("monitor.css", "text/css; charset=utf-8"),
}


def read_identity(host, map_id):
    path = remote_path(f"{REMOTE_MAPS}/{map_id}/map_world.pcd")
    command = (
        "python3 -c " + shlex.quote(
            "import os,sys; s=os.stat(sys.argv[1]); print(s.st_mtime_ns, s.st_size)"
        ) + " " + path + " && sha256sum -- " + path
    )
    lines = subprocess.check_output([*SSH, host, command], text=True, timeout=30).splitlines()
    if len(lines) != 2:
        raise ValueError("读取板端地图版本失败")
    mtime, size = map(int, lines[0].split())
    digest = lines[1].split()[0]
    if not re.fullmatch(r"[a-f0-9]{64}", digest):
        raise ValueError("板端 PCD 校验值无效")
    return f"{map_id}-{mtime}-{size}", digest


@contextmanager
def temporary_path(directory):
    fd, name = tempfile.mkstemp(dir=directory)
    os.close(fd)
    path = Path(name)
    try:
        yield path
    finally:
        path.unlink(missing_ok=True)


def sync_map(cache, host, map_id):
    if not MAP_ID.fullmatch(map_id):
        raise ValueError("地图 ID 应为地图目录名")
    key, expected = read_identity(host, map_id)
    folder = cache / "maps" / map_id
    folder.mkdir(parents=True, exist_ok=True)
    local = folder / "map_world.pcd"
    rviz = RVIZ_MAPS / map_id / local.name
    source = None
    for path in (local, rviz):
        if path.is_file() and fingerprint(path) == expected:
            source = path
            break
    if source is None:
        remote = remote_path(f"{REMOTE_MAPS}/{map_id}/{local.name}")
        with temporary_path(folder) as temp:
            with temp.open("wb") as file:
                subprocess.run([*SSH, host, "cat -- " + remote], stdout=file, check=True, timeout=150)
            if fingerprint(temp) != expected:
                raise ValueError("同步时板端地图发生变化，请重试")
            temp.replace(local)
        source = local
    metadata, points = display_map(source)
    header = json.dumps(metadata, separators=(",", ":")).encode()
    payload = len(header).to_bytes(4, "little") + header + points
    display = cache / "display"
    display.mkdir(parents=True, exist_ok=True)
    with temporary_path(display) as temp:
        temp.write_bytes(payload)
        temp.replace(display / f"{key}.bin")
    with temporary_path(display) as temp:
        temp.write_text(json.dumps({"sha256": expected}) + "\n")
        temp.replace(display / f"{key}.json")
    count = sum(metadata["counts"].values())
    print(f"{map_id} · {count:,} 显示点 · {source}", flush=True)


class Telemetry:
    def __init__(self, host, local_port, board_port, stopped, board_status=False, remote_root=DEFAULT_ROOT):
        self.remote_root = remote_root
        self.host, self.local_port, self.board_port = host, local_port, board_port
        self.stopped = stopped
        self.board_status = board_status
        self.lock = Lock()
        self.last_view = -float("inf")
        self.worker = Thread(target=self.run, name="board-telemetry")

    def watch(self):
        with self.lock:
            self.last_view = time.monotonic()

    def wanted(self):
        with self.lock:
            return time.monotonic() - self.last_view < 15

    def run(self):
        while not self.stopped.is_set():
            if not self.wanted():
                self.stopped.wait(.25)
                continue
            try:
                with tunnel(self.host, self.local_port, self.board_port, self.board_status, self.remote_root) as connection:
                    while self.wanted() and connection.poll() is None and not self.stopped.wait(.25):
                        pass
            except (OSError, subprocess.SubprocessError) as error:
                print(f"Orange Pi 遥测隧道：{error}", flush=True)
            self.stopped.wait(2)

    @contextmanager
    def running(self):
        self.worker.start()
        try:
            yield
        finally:
            self.stopped.set()
            self.worker.join()


def serve(port, cache, upstream, telemetry=None, board_status=False, dispatch=None, camera=None, scans=None):
    inspection = Inspection(cache, RVIZ_MAPS, MAP_KEY, fingerprint, temporary_path)
    tasks = dispatch.data if dispatch else catalog(ROOT, cache)

    class Handler(BaseHTTPRequestHandler):
        def reply(self, payload, kind, code=200):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(payload)

        def json_reply(self, value, code=200):
            self.reply(json.dumps(value, ensure_ascii=False, allow_nan=False).encode(),
                       "application/json; charset=utf-8", code)

        def allowed_host(self):
            return self.headers.get("Host", "").split(":")[0].lower() in {"localhost", "127.0.0.1"}

        def do_GET(self):
            if not self.allowed_host():
                self.send_error(403)
                return
            request = urlsplit(self.path)
            try:
                if request.path in STATIC:
                    name, kind = STATIC[request.path]
                    self.reply((ROOT / "web" / name).read_bytes(), kind)
                elif request.path == "/camera/stream":
                    client = parse_qs(request.query).get("client", [""])[0]
                    if not camera or self.headers.get("Sec-Fetch-Site") not in (None, "same-origin") or not re.fullmatch(r"[a-f0-9-]{36}", client):
                        self.send_error(403)
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=ovframe")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Cross-Origin-Resource-Policy", "same-origin")
                    self.end_headers()
                    self.connection.settimeout(2)
                    stamp = 0
                    while True:
                        item = camera.jpeg(client, stamp)
                        if item is None:
                            break
                        new, jpeg = item
                        if jpeg and new != stamp:
                            self.wfile.write(("--ovframe\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\nX-Stamp-Ns: %d\r\n\r\n" % (len(jpeg), new)).encode())
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                            stamp = new
                    return
                elif request.path == "/scan-runs" or request.path.startswith("/scan-runs/"):
                    if self.headers.get("Sec-Fetch-Site") not in (None, "none", "same-origin"):
                        self.send_error(403)
                        return
                    if scans is None:
                        self.json_reply(dict(online=False, error="离线模式", data=None, updatedAt=None))
                    elif request.path == "/scan-runs":
                        self.json_reply(scans.read())
                    else:
                        key = request.path[len("/scan-runs/"):]
                        if not key:
                            self.send_error(404)
                            return
                        self.json_reply(scans.read(key))
                elif request.path == "/map":
                    key = parse_qs(request.query).get("key", [""])[0]
                    if not MAP_KEY.fullmatch(key):
                        self.send_error(404)
                        return
                    payload = (cache / "display" / f"{key}.bin").read_bytes()
                    self.reply(payload, "application/octet-stream")
                elif request.path == "/maps":
                    self.json_reply(inspection.catalog())
                elif request.path == "/tasks":
                    self.json_reply(tasks)
                elif request.path == "/task":
                    self.json_reply(dispatch.snapshot() if dispatch else {"enabled": False})
                elif request.path == "/inspection":
                    key = parse_qs(request.query).get("key", [""])[0]
                    self.json_reply(inspection.read(key))
                elif request.path == "/drone-status":
                    status_url = upstream if board_status else "http://127.0.0.1:8091"
                    if status_url is None:
                        self.send_error(503)
                        return
                    if board_status and telemetry is not None:
                        if self.headers.get("Sec-Fetch-Site") not in (None, "none", "same-origin"):
                            self.send_error(403)
                            return
                        telemetry.watch()
                    with HTTP.open(status_url + "/status", timeout=.8) as response:
                        self.reply(response.read(), "application/json; charset=utf-8")
                elif request.path == "/telemetry":
                    if self.headers.get("Sec-Fetch-Site") not in (None, "none", "same-origin"):
                        self.send_error(403)
                        return
                    if upstream is None:
                        self.json_reply({"offline": True})
                        return
                    if telemetry is not None:
                        telemetry.watch()
                    with HTTP.open(upstream + "/telemetry?" + request.query, timeout=.8) as response:
                        payload = response.read()
                    self.reply(payload, "application/json")
                else:
                    self.send_error(404)
            except FileNotFoundError:
                self.send_error(404)
            except (TimeoutError, URLError, HTTPException):
                self.send_error(503)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except ValueError as error:
                self.json_reply({"error": str(error)}, 400)
            except OSError:
                self.send_error(500)

        def do_POST(self):
            host = self.headers.get("Host", "")
            origin = self.headers.get("Origin")
            if not self.allowed_host() or (origin and origin != "http://" + host):
                self.send_error(403)
                return
            if self.path == "/camera/watch":
                if not camera:
                    self.json_reply({"error": "离线模式"}, 503)
                    return
                if origin != "http://" + host:
                    self.send_error(403)
                    return
                try:
                    size = int(self.headers.get("Content-Length", 0))
                    if not 0 < size <= 512 or self.headers.get_content_type() != "application/json":
                        raise ValueError()
                    data = json.loads(self.rfile.read(size))
                    client, mode = data["client"], data["mode"]
                    if not re.fullmatch(r"[a-f0-9-]{36}", client) or mode not in MODES | {"close"}:
                        raise ValueError()
                except (KeyError, TypeError, ValueError):
                    self.send_error(400)
                    return
                self.json_reply(camera.watch(client, mode))
                return
            is_task = self.path in {"/task/submit", "/task/cancel", "/task/stop"}
            if is_task and origin != "http://" + host:
                self.send_error(403)
                return
            if is_task and dispatch is None:
                self.json_reply({"error": "离线模式仅供查看任务"}, 403)
                return
            if self.path not in {"/inspection", "/preview"} and not is_task:
                self.send_error(404)
                return
            try:
                size = int(self.headers.get("Content-Length", 0))
                if self.headers.get_content_type() != "application/json" or not 0 < size <= 65536:
                    self.send_error(400)
                    return
                body = json.loads(self.rfile.read(size))
                if is_task:
                    verb = self.path.rsplit("/", 1)[1]
                    if verb == "submit":
                        dispatch.submit(body)
                    else:
                        dispatch.control(body, verb)
                    self.json_reply(dispatch.snapshot(), 202)
                    return
                key, revision = body["key"], body["revision"]
                if type(revision) is not int or revision < 0:
                    raise ValueError("版本号无效")
                if self.path == "/inspection":
                    data = inspection.save(key, revision, body["points"])
                else:
                    data = inspection.plan(key, revision, body["cruise"])
                self.json_reply(data)
            except FileNotFoundError:
                self.json_reply({"error": "本机没有缓存该地图版本"}, 404)
            except (Conflict, TaskConflict) as error:
                self.json_reply({"error": str(error)}, 409)
            except (ValueError, KeyError, TypeError) as error:
                self.json_reply({"error": str(error)}, 400)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except OSError:
                self.send_error(500)

        def log_message(self, *_):
            pass

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


@contextmanager
def tunnel(host, local_port, board_port, board_status=False, remote_root=DEFAULT_ROOT):
    forward = f"127.0.0.1:{local_port}:127.0.0.1:{board_port}"
    command = f"exec env PYTHONDONTWRITEBYTECODE=1 {REMOTE_MONITOR} --port {board_port} --telemetry-only --stop-on-stdin-close"
    if board_status:
        command += " --board-status"
    command = in_checkout(remote_root, command)
    with subprocess.Popen([*SSH, "-T", "-o", "ExitOnForwardFailure=yes", "-o", "ServerAliveInterval=3",
                           "-o", "ServerAliveCountMax=2", "-L", forward, host, command],
                          stdin=subprocess.PIPE, start_new_session=True) as connection:
        try:
            yield connection
        finally:
            if connection.stdin:
                connection.stdin.close()
            try:
                connection.wait(timeout=4)
            except subprocess.TimeoutExpired:
                connection.terminate()
                try:
                    connection.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    connection.kill()
                    connection.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    config = settings(ROOT.parents[1] / ".env")
    remote_root = config["ORANGE_REMOTE_ROOT"]
    parser.add_argument("--host", default=config["ORANGE_SSH_HOST"])
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--tunnel-port", type=int, default=8766)
    parser.add_argument("--board-port", type=int, default=8767 if sys.platform == "darwin" else 8765)
    parser.add_argument("--sync", metavar="MAP_ID", nargs="+")
    parser.add_argument("--offline", action="store_true", help="只打开电脑本地地图和路线预览")
    parser.add_argument("--board-status", action="store_true", default=sys.platform == "darwin",
                        help="从板端遥测隧道读取飞控状态（macOS 默认）")
    args = parser.parse_args()
    if args.host.startswith("-"):
        parser.error("无效 SSH 主机")
    if any(not 1 <= p <= 65535 for p in (args.port, args.tunnel_port, args.board_port)):
        parser.error("端口范围为 1–65535")
    if args.sync:
        try:
            for map_id in args.sync:
                sync_map(args.cache, args.host, map_id)
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            parser.exit(1, f"地图同步失败 · {exc}\n")
        return
    if not args.offline and args.port == args.tunnel_port:
        parser.error("页面端口和转发端口不能相同")
    stopped = Event()

    def stop(*_):
        stopped.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    upstream = None if args.offline else f"http://127.0.0.1:{args.tunnel_port}"
    telemetry = None if args.offline else Telemetry(args.host, args.tunnel_port, args.board_port, stopped, args.board_status, remote_root)
    dispatch = None if args.offline else Dispatch(catalog(ROOT, args.cache), args.cache, args.host, SSH, stopped, remote_root)
    camera = None if args.offline else Camera(SSH, args.host, stopped, remote_root)
    scans = None if args.offline else ScanConnection(SSH, args.host)
    with (closing(scans) if scans else nullcontext()), serve(
        args.port, args.cache, upstream, telemetry, args.board_status, dispatch, camera, scans
    ) as http:
        with (telemetry.running() if telemetry else nullcontext()), (dispatch.running() if dispatch else nullcontext()):
            http.timeout = .35
            print(f"仓库无人机监看 http://127.0.0.1:{args.port}/", flush=True)
            while not stopped.is_set():
                http.handle_request()



if __name__ == "__main__":
    main()
