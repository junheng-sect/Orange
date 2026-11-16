#!/usr/bin/env python3
"""Board RPC and tmux supervisor for the preset navigation scripts."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
from threading import Lock, Thread
import time

from task_dispatch import REQUEST_ID, TERMINAL, TTL, catalog, task_for

ROOT = Path(__file__).resolve().parent
HOME = Path.home()
SCRIPTS = HOME / "mid360s_ws/scripts"
MAPS = HOME / "test_data/mapping"
BOOT = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
STATE = HOME / ".cache/warehouse-monitor/board-tasks" / BOOT
SESSION = "warehouse-nav"
ENV = ("source /opt/ros/noetic/setup.bash && "
       "source /home/orangepi/mid360s_ws/scripts/ros_net.sh")
CONFLICT_NODES = {"/drone_reloc", "/car_bridge", "/waypoint_manager", "/offboard_gateway",
                  "/trajectory_executor", "/laserMapping", "/gravity_bridge"}


def tmux(*args, **kwargs):
    return subprocess.run(["tmux", *args], timeout=3, **kwargs)


def session_exists():
    return tmux("has-session", "-t", "=" + SESSION, capture_output=True).returncode == 0


@contextmanager
def locked():
    STATE.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (STATE / "lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield


def read(path):
    try:
        value = json.loads(path.read_text())
    except FileNotFoundError:
        return None
    return value if value.get("boot") == BOOT else None


def write(path, value):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
    temp.replace(path)


def record_path(request_id):
    return STATE / f"{request_id}.json"


def get_record(request_id):
    return read(record_path(request_id)) if REQUEST_ID.fullmatch(request_id) else None


def save(value):
    write(record_path(value["id"]), value)


def active_record():
    pointer = read(STATE / "current.json")
    value = get_record(pointer["id"]) if pointer else None
    if value and value["state"] not in TERMINAL:
        if session_exists():
            return value
        value.update(state="failed", message="板端任务会话已退出，请检查运行日志")
        save(value)
    return None


def snapshot(request_id, error=None):
    active = active_record()
    response = dict(boot=BOOT, active=active, request=get_record(request_id))
    if active:
        is_fresh = time.monotonic() - active.get("healthAt", -100) < 2
        active["canCancel"] = active["state"] == "waiting_fcu"
        active["canStop"] = (is_fresh and active.get("safeGround", False)
                             and active["state"] not in {"waiting_fcu", "stopping"}
                             and not active.get("stopRequested"))
        for field in ("deadline", "healthAt", "safeGround", "confirmAuto", "stopRequested", "boot"):
            active.pop(field, None)
    if error:
        response["error"] = str(error)
    return response


def submit(body):
    expected = {"id", "taskId", "mapKey", "confirmAuto", "remaining", "boot"}
    if set(body) != expected or type(body["remaining"]) is not int or not 0 < body["remaining"] <= TTL:
        raise ValueError("下发请求无效或已过期")
    request_id = body["id"]
    task = task_for(catalog(ROOT), body["taskId"], body["mapKey"])
    if type(body["confirmAuto"]) is not bool or (task["automatic"] and not body["confirmAuto"]):
        raise ValueError("请确认自动起飞任务")
    previous = get_record(request_id)
    if previous:
        if "taskId" in previous and any(previous[key] != body[key] for key in ("taskId", "mapKey", "confirmAuto")):
            raise ValueError("请求 ID 已用于另一任务")
        return
    if active_record():
        raise ValueError("板端已有网页任务，请先收尾")
    if session_exists():
        raise ValueError("板端上一任务会话正在退出，稍后重新下发")
    value = dict(id=request_id, taskId=body["taskId"], mapKey=body["mapKey"],
                 confirmAuto=body["confirmAuto"], boot=BOOT, state="waiting_fcu",
                 message="板端已收到任务，等待飞控连接", deadline=time.monotonic() + body["remaining"],
                 safeGround=False)
    save(value)
    write(STATE / "current.json", dict(id=request_id, boot=BOOT))
    command = ENV + " && exec " + shlex.join(["python3", "-B", str(Path(__file__).resolve()), "run", request_id])
    try:
        result = tmux("new-session", "-d", "-s", SESSION, command, capture_output=True, text=True)
        if result.returncode:
            raise ValueError(result.stderr.strip() or "无法创建板端任务会话")
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        value.update(state="failed", message=str(error))
        save(value)


def control(verb, request_id):
    value = get_record(request_id)
    if verb == "cancel":
        if value is None:
            # A tombstone covers an ambiguous submit response followed by cancel.
            save(dict(id=request_id, boot=BOOT, state="cancelled", message="任务已取消"))
            return
        if value["state"] in TERMINAL:
            return
        if value["state"] != "waiting_fcu":
            raise ValueError("导航已开始准备，着地上锁后可收尾")
        value.update(state="cancelled", message="已取消待启动任务")
    else:
        if value and (value["state"] == "stopping" or value["state"] in TERMINAL):
            return
        if not value or time.monotonic() - value.get("healthAt", -100) >= 2 or not value.get("safeGround"):
            raise ValueError("等待飞控确认着地并上锁")
        if value["state"] == "waiting_fcu":
            raise ValueError("该任务尚未启动，可以取消")
        value["stopRequested"] = True
    save(value)


def rpc(verb, body):
    request_id = body.get("id", "")
    if not isinstance(request_id, str) or (not REQUEST_ID.fullmatch(request_id)
                                            and (request_id or verb != "status")):
        raise ValueError("请求 ID 无效")
    with locked():
        try:
            if verb != "status" and body.get("boot") != BOOT:
                raise ValueError("Orange Pi 已重启，请重新下发")
            if verb == "submit":
                submit(body)
            elif verb in {"cancel", "stop"}:
                control(verb, request_id)
            elif verb != "status":
                raise ValueError("未知操作")
        except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
            return snapshot(request_id, error)
        return snapshot(request_id)


def fingerprint(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def script_command(task):
    command = [str(SCRIPTS / ("nav_" + task["id"] + ".sh"))]
    if task["id"].startswith("test_indoor"):
        command += ["--map", str(MAPS / task["mapId"]),
                    "--nav-config", str(SCRIPTS / "nav_test/indoor_fly_nav.yaml"),
                    "--map-frame", str(SCRIPTS.parent / "src/car_bridge/config/map_frame_indoor_fly.yaml"),
                    "--ev-mode", "odom"]
    return command


def check_map(data, task):
    expected = data["maps"][task["mapId"]]
    path = MAPS / task["mapId"] / "map_world.pcd"
    info = path.stat()
    key = f"{task['mapId']}-{info.st_mtime_ns}-{info.st_size}"
    if key != expected["key"] or fingerprint(path) != expected["sha256"]:
        raise ValueError("板端 PCD 版本或 SHA-256 已变化，请更新任务目录与电脑缓存")


class FlightStatus:
    def __init__(self, rospy):
        from mavros_msgs.msg import EstimatorStatus, ExtendedState, State
        from std_msgs.msg import String

        self.lock = Lock()
        self.latest = {}
        self.prepared_at = None
        self.subscribers = [
            rospy.Subscriber("/mavros/state", State, self.set, callback_args="fcu", queue_size=3),
            rospy.Subscriber("/mavros/extended_state", ExtendedState, self.set, callback_args="ext", queue_size=3),
            rospy.Subscriber("/mavros/estimator_status", EstimatorStatus, self.set,
                             callback_args="estimator", queue_size=3),
        ]
        for topic, name in (("/drone_nav/gateway_status", "gateway"), ("/drone_nav/input_status", "input"),
                            ("/drone_reloc/status", "reloc"), ("/drone_reloc/result", "result")):
            self.subscribers.append(rospy.Subscriber(topic, String, self.status, callback_args=name, queue_size=3))

    def set(self, message, name):
        with self.lock:
            now = time.monotonic()
            self.latest[name] = (now, message)
            if name == "gateway":
                if message.get("state") == "PREPARED" and message.get("real_output") is True:
                    self.prepared_at = self.prepared_at or now
                else:
                    self.prepared_at = None

    def status(self, message, name):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                self.set(value, name)
        except (ValueError, AttributeError):
            pass

    def snapshot(self, task):
        with self.lock:
            now = time.monotonic()

            def fresh(name, max_age):
                at, value = self.latest.get(name, (0, None))
                return value if now - at < max_age else None

            fcu, ext = fresh("fcu", 2), fresh("ext", 1.5)
            est = fresh("estimator", 2)
            estimate = bool(est and est.attitude_status_flag and est.velocity_horiz_status_flag
                            and est.velocity_vert_status_flag
                            and (est.pos_horiz_rel_status_flag or est.pos_horiz_abs_status_flag)
                            and (est.pos_vert_abs_status_flag or est.pos_vert_agl_status_flag))
            nav, gateway = fresh("input", .6) or {}, fresh("gateway", .6) or {}
            loc = fresh("reloc", 2.5) or {}
            result = self.latest.get("result", (0, {}))[1]
            connected = bool(fcu and fcu.connected)
            mode = fcu.mode if connected else ""
            armed = fcu.armed if connected else None
            landed = ext.landed_state if ext else None
            ground = connected and not armed and landed == 1 and mode != "OFFBOARD"
            ready = (connected and nav.get("state") == "READY" and gateway.get("real_output") is True
                     and loc.get("state") == "LOCALIZED" and loc.get("valid") is True
                     and result.get("confirmed") is True and result.get("map") == str(MAPS / task["mapId"]))
            stream = ready and self.prepared_at is not None and now - self.prepared_at >= 1.0
            return dict(connected=connected, mode=mode, armed=armed, landed=landed, ground=ground,
                        ready=ready, stream=stream, estimate=estimate)


class Progress:
    def __init__(self):
        self.lock = Lock()
        self.note = ""
        self.scan = None
        self.phase = ""
        self.runner_started = False
        self.takeoff_ready = False
        self.finished = None
        self.stop_refused = False
        self.clean_exit = False

    def collect(self, stream):
        for line in stream:
            print(line.rstrip(), flush=True)
            line = line.strip()
            if not line or line.startswith("----"):
                continue
            with self.lock:
                if " INDOOR4 " in line:
                    try:
                        scan = json.loads(line.split(" INDOOR4 ", 1)[1])
                        if scan["phase"] != "识别标签" and self.scan and "route" not in scan:
                            scan["route"] = self.scan.get("route")
                        self.scan = scan
                        continue
                    except (ValueError, KeyError, TypeError):
                        pass
                self.note = line[:430]
                if "[ OK ] 导航就绪" in line:
                    self.runner_started = True
                if "待命：定点模式在地面解锁" in line:
                    self.takeoff_ready = line.endswith("解锁后自动起飞")
                if " 阶段：" in line:
                    self.phase = line.rsplit(" 阶段：", 1)[1].strip()
                if "收尾被拒" in line:
                    self.stop_refused = True
                if "[ OK ] 网页任务已收尾" in line:
                    self.clean_exit = True
                if any(mark in line for mark in ("测试动作结束。", "本圈终止：", "到点，原地保持。",
                                                  "完成，终点已保持", "切出了 OFFBOARD，没有",
                                                  "已落地上锁。通道 6", "飞手接管。再测：",
                                                  "进入 OFFBOARD：", "本次不再自动起飞",
                                                  "终止：", "没有降落：", "没有发：", "没到点（")):
                    self.finished = None if "进入 OFFBOARD：" in line else line[:430]
                if "已解锁、网关在地面预发送" in line or line.endswith("阶段：COUNTDOWN"):
                    self.finished = None

    def stage(self, task, health, has_flown):
        with self.lock:
            phase, note, finished = self.phase, self.note, self.finished
            if task["automatic"]:
                if phase == "DONE" or finished:
                    return "action_done", finished or note
                if phase in {"COUNTDOWN", "OFFBOARD_ENTRY", "TAKEOFF", "ROUTE", "LAND_REQUEST", "LANDING"} or (
                    health["armed"] and health["mode"] == "OFFBOARD"
                ):
                    return "running", note
                preflight_passed = (phase == "WAIT_ARM" if task["id"] != "test6" else self.takeoff_ready)
                if has_flown and health["ground"]:
                    return "action_done", note
                if preflight_passed and health["ready"] and health["ground"]:
                    if health["mode"] == "POSCTL":
                        return "wait_arm", "预检通过。按通道 6 录包，地面遥控解锁后约 3 s 自动起飞"
                    return "preparing", "保持地面上锁，用遥控器切到定点模式（POSCTL）"
            elif finished or (has_flown and health["ground"]):
                return "action_done", finished or note
            elif health["armed"] and health["mode"] in {"OFFBOARD", "AUTO.LAND"}:
                return "running", note
            elif health["stream"] and self.runner_started and health["armed"] and health["landed"] == 2:
                if task["id"] in {"test_indoor3", "test_indoor4"}:
                    return "wait_offboard", "设定值已预发送，切 OFFBOARD 后从当前位置规划到 1 号点"
                return "wait_offboard", "设定值已预发送，可用遥控器切 OFFBOARD"
            elif (health["ready"] and health["estimate"] and self.runner_started
                  and (health["ground"] or health["armed"])):
                return "wait_pilot", "定位就绪，可手飞起飞。悬停后等设定值预发送"
            return "preparing", note or "等待定位与导航预检"


def verify_idle():
    listing = subprocess.check_output(["rosnode", "list"], timeout=4, text=True)
    if CONFLICT_NODES.intersection(listing.splitlines()):
        raise ValueError("已有定位或导航节点，请结束旧链路后重新下发")
    if subprocess.run(["flock", "-n", "/tmp/mid360s_fusion.lock", "true"], timeout=2).returncode:
        raise ValueError("定位链路正在运行，请先收尾")
    if tmux("has-session", "-t", "=nav", capture_output=True).returncode == 0:
        raise ValueError("已有终端 nav 会话，请先收尾")


def supervise(request_id):
    import rospy

    with locked():
        value = get_record(request_id)
    if not value or value["state"] != "waiting_fcu":
        return
    data = catalog(ROOT)
    task = task_for(data, value["taskId"], value["mapKey"])
    rospy.init_node("warehouse_task", anonymous=True, disable_signals=True)
    flight = FlightStatus(rospy)
    while True:
        with locked():
            value = get_record(request_id)
            if value["state"] in TERMINAL:
                return
            if time.monotonic() >= value["deadline"]:
                value.update(state="expired", message="等待板端飞控连接超时，请重新确认")
                save(value)
                return
        health = flight.snapshot(task)
        if health["connected"] and health["armed"]:
            raise ValueError("飞控已解锁，请着地上锁后重新下发")
        if health["connected"] and health["landed"] in {2, 3, 4}:
            raise ValueError("飞控未着地，请着地上锁后重新下发")
        if health["ground"]:
            check_map(data, task)
            verify_idle()
            if flight.snapshot(task)["ground"]:
                break
        time.sleep(.25)
    with locked():
        value = get_record(request_id)
        if value["state"] != "waiting_fcu":
            return
        if time.monotonic() >= value["deadline"]:
            value.update(state="expired", message="任务已过期，请重新确认")
            save(value)
            return
        if not flight.snapshot(task)["ground"]:
            raise ValueError("启动时飞控地面上锁状态已变化")
        value.update(state="preparing", message="开始定位与导航预检")
        save(value)
        env = dict(os.environ, WAREHOUSE_TASK_ID=request_id, PYTHONUNBUFFERED="1")
        env.pop("NAV_TEST_IN_TMUX", None)
        child = subprocess.Popen(script_command(task), cwd=SCRIPTS, env=env, stdin=subprocess.DEVNULL,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, errors="replace", start_new_session=True)
    progress = Progress()
    reader = Thread(target=progress.collect, args=(child.stdout,), daemon=True)
    reader.start()
    has_flown = False
    stopping = False
    while child.poll() is None:
        health = flight.snapshot(task)
        has_flown = has_flown or bool(health["armed"] and health["landed"] == 2)
        state, message = progress.stage(task, health, has_flown)
        if not health["connected"]:
            message = "飞控状态中断，等待新的飞控数据"
        with progress.lock:
            scan = progress.scan
            if progress.stop_refused:
                stopping = False
                progress.stop_refused = False
        with locked():
            value = get_record(request_id)
            if value.pop("stopRequested", False):
                if flight.snapshot(task)["ground"]:
                    stopping = True
                    if child.poll() is None:
                        try:
                            child.send_signal(signal.SIGTERM)
                        except ProcessLookupError:
                            pass
                else:
                    message = "收尾前飞控状态已变化，等待着地上锁"
            value.update(state="stopping" if stopping else state,
                         message="正在收尾与写盘" if stopping else message,
                         mode=health["mode"], armed=health["armed"], landed=health["landed"],
                         safeGround=health["ground"], healthAt=time.monotonic())
            if task["id"] == "test_indoor4" and scan is not None:
                value["scan"] = scan
            save(value)
        time.sleep(.5)
    reader.join(timeout=1)
    with locked():
        value = get_record(request_id)
        cleaned = progress.clean_exit and child.returncode in (0, 130)
        value.update(state="done" if cleaned else "failed", safeGround=False,
                     message="已收尾，记录已写盘" if cleaned else (
                         f"导航脚本退出（{child.returncode}）：" + (progress.note or value["message"])))
        save(value)


def main():
    verb, data = sys.argv[1:]
    if verb != "run":
        print(json.dumps(rpc(verb, json.loads(data)), ensure_ascii=False))
        return
    try:
        supervise(data)
    except Exception as error:
        with locked():
            value = get_record(data)
            if value and value["state"] not in TERMINAL:
                value.update(state="failed", message=str(error), safeGround=False)
                save(value)
        raise


if __name__ == "__main__":
    main()
