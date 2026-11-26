"""Queue one flight preparation request from the loopback web server."""
from contextlib import contextmanager
import json
import math
import re
import shlex
import subprocess
from threading import Event, Lock, Thread
import time

from board_connection import DEFAULT_ROOT, in_checkout

REQUEST_ID = re.compile(r"[0-9a-f]{32}")
TTL = 15 * 60
TERMINAL = {"done", "failed", "cancelled", "expired"}
REMOTE = "tools/warehouse_monitor/board_tasks.py"


class TaskConflict(ValueError):
    pass


def catalog(root):
    return json.loads((root / "tasks.json").read_text())


def task_for(data, task_id, map_key):
    if not isinstance(task_id, str) or not isinstance(map_key, str):
        raise ValueError("任务或地图版本无效")
    for task in data["tasks"]:
        if task["id"] == task_id and data["maps"][task["mapId"]]["key"] == map_key:
            return task
    raise ValueError("任务与地图版本不匹配")


def check_damper(body):
    if 'damperDeg' in body:
        value = body['damperDeg']
        if body.get('taskId') != 'test_indoor3_5' or type(value) is not int or not 0 <= value <= 100:
            raise ValueError('减振球硬度应为 0 至 100 的整数，可留空')


class Dispatch:
    def __init__(self, data, cache, host, ssh, stopped, remote_root=DEFAULT_ROOT):
        self.remote_root = remote_root
        self.data = data
        self.cache = cache
        self.host = host
        self.ssh = ssh
        self.stopped = stopped
        self.lock = Lock()
        self.wake = Event()
        self.watched_at = -float("inf")
        self.request = None
        self.board = None
        self.online = False
        self.worker = Thread(target=self.run, name="task-dispatch")

    def remote(self, verb, body):
        command = shlex.join(["python3", "-B", REMOTE, verb, json.dumps(body, separators=(",", ":"))])
        result = subprocess.run([*self.ssh, "-T", self.host, in_checkout(self.remote_root, command)], stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, timeout=8, check=False)
        if result.returncode == 255:
            raise OSError("Orange Pi 尚未连接")
        if result.returncode:
            raise ValueError("板端任务入口不可用，请检查板端部署")
        return json.loads(result.stdout)

    def snapshot(self, watch=True):
        with self.lock:
            if watch:
                self.watched_at = time.monotonic()
                if self.board is None:
                    self.wake.set()
            return dict(request={key: val for key, val in self.request.items()
                                 if key in {"id", "taskId", "mapKey", "state", "message", "expiresAt"}}
                                if self.request else None,
                        board=self.board, online=self.online, enabled=True)

    def submit(self, body):
        if not isinstance(body, dict) or set(body) - {"damperDeg"} != {"id", "taskId", "mapKey", "confirmAuto"}:
            raise ValueError("任务请求字段无效")
        if not isinstance(body["id"], str) or not REQUEST_ID.fullmatch(body["id"]):
            raise ValueError("请求 ID 无效")
        check_damper(body)
        task = task_for(self.data, body["taskId"], body["mapKey"])
        if type(body["confirmAuto"]) is not bool or (task["automatic"] and not body["confirmAuto"]):
            raise ValueError("请确认地面解锁后自动起飞")
        key = body["mapKey"]
        map_file = self.cache / "display" / f"{key}.bin"
        manifest = json.loads(map_file.with_suffix(".json").read_text())
        if not map_file.is_file() or manifest.get("sha256") != self.data["maps"][task["mapId"]]["sha256"]:
            raise ValueError("电脑缓存与任务绑定的 PCD 不匹配，请同步该地图")
        with self.lock:
            if self.request and self.request["id"] == body["id"]:
                if self.request.get("damperDeg") != body.get("damperDeg") or any(self.request.get(key) != val for key, val in body.items()):
                    raise TaskConflict("请求 ID 已用于另一任务")
                return
            if (self.request and self.request["state"] not in TERMINAL) or (
                self.board and self.board.get("active")
            ):
                raise TaskConflict("已有待下发任务或板端任务")
            self.request = dict(body, state="queued", message="等待 Orange Pi 上线",
                                expiresAt=int(time.time()) + TTL,
                                deadline=time.monotonic() + TTL, boot=None, sent=False, command=None)
            self.wake.set()

    def control(self, body, verb):
        if not isinstance(body, dict) or set(body) != {"id"} or not isinstance(body["id"], str):
            raise ValueError("请求 ID 无效")
        with self.lock:
            request = self.request
            if request and request["id"] == body["id"] and request["state"] == "queued" and not request["sent"] and verb == "cancel":
                request.update(state="cancelled", message="已取消待下发任务")
                return
            active = self.board and self.board.get("active")
            if not active or active["id"] != body["id"]:
                if verb == "cancel" and request and request["id"] == body["id"] and request["state"] == "queued":
                    request.update(command="cancel", message="等待板端确认取消")
                    self.wake.set()
                    return
                raise TaskConflict("板端任务状态已变化，请刷新")
            allowed = active["canCancel"] if verb == "cancel" else self.online and active["canStop"]
            if not allowed:
                raise TaskConflict("板端当前无法执行该操作")
            if request is None or request["id"] != body["id"]:
                request = self.request = dict(id=active["id"], taskId=active["taskId"],
                                               state="delivered", command=None)
            request.update(command=verb, message="等待板端确认")
            self.wake.set()

    def tick(self):
        with self.lock:
            request = self.request
            send = request and request["state"] == "queued"
            command = request and request.get("command")
            if send and time.monotonic() >= request["deadline"] and not request["sent"]:
                request.update(state="expired", message="任务已过期，请重新确认")
                send = False
            if not (send or command or (request and request["state"] == "delivered")
                    or (self.board and self.board.get("active")) or
                    time.monotonic() - self.watched_at < 15):
                return
            request_id = request["id"] if request else ""
        response = self.remote("status", {"id": request_id})
        with self.lock:
            self.board, self.online = response, True
            if not request or self.request is not request:
                return
            if request.get("boot") and request["boot"] != response["boot"]:
                if request["state"] not in TERMINAL:
                    request.update(state="failed", message="Orange Pi 已重启，请重新下发", command=None)
                return
            matching = response.get("request")
            if matching:
                state = matching["state"] if matching["state"] in TERMINAL else "delivered"
                request.update(state=state, message=matching["message"], boot=response["boot"])
            command = request.get("command")
            if command:
                verb = command
                payload = dict(id=request_id, boot=response["boot"])
            elif request["state"] == "queued":
                if time.monotonic() >= request["deadline"]:
                    request.update(state="expired", message="任务已过期，请重新确认")
                    return
                if response.get("active"):
                    request.update(state="failed", message="板端已有任务，请先收尾")
                    return
                verb = "submit"
                request.update(sent=True, boot=response["boot"])
                payload = {key: request[key] for key in ("id", "taskId", "mapKey", "confirmAuto", "boot")}
                if "damperDeg" in request:
                    payload["damperDeg"] = request["damperDeg"]
                payload["remaining"] = math.ceil(request["deadline"] - time.monotonic())
            else:
                return
        result = self.remote(verb, payload)
        with self.lock:
            self.board, self.online = result, True
            if self.request is not request:
                return
            if verb != "submit" and request.get("command") == verb:
                request["command"] = None
            matching = result.get("request")
            if result.get("error"):
                self.board["error"] = result["error"]
                request.update(message=result["error"])
                if verb == "submit":
                    request.update(state="failed")
            elif matching:
                state = matching["state"] if matching["state"] in TERMINAL else "delivered"
                request.update(state=state, message=matching["message"])

    def run(self):
        while not self.stopped.is_set():
            try:
                self.tick()
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                with self.lock:
                    self.online = False
                    if self.request and self.request["state"] == "queued" and not self.request["command"]:
                        request = self.request
                        if time.monotonic() >= request["deadline"]:
                            request.update(state="expired", message="下发已过期。板端状态将在重连后更新")
                        else:
                            request["message"] = str(error) or "等待 Orange Pi 上线"
            self.wake.wait(2.0 if self.online else 3.0)
            self.wake.clear()

    @contextmanager
    def running(self):
        self.worker.start()
        try:
            yield
        finally:
            self.stopped.set()
            self.wake.set()
            self.worker.join()
