#!/usr/bin/env python3
"""Board RPC and tmux supervisor for the preset navigation scripts."""
from contextlib import contextmanager
import fcntl
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

from board_connection import fingerprint
from task_dispatch import REQUEST_ID, TERMINAL, TTL, catalog, check_damper, check_vertical, task_for, vertical_profile

ROOT = Path(__file__).resolve().parent
HOME = Path.home()
WORKSPACE = ROOT.parents[1]
SCRIPTS = WORKSPACE / "scripts"
MAPS = HOME / "test_data/mapping"
BOOT = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
STATE = HOME / ".cache/warehouse-monitor/board-tasks" / BOOT
SESSION = "warehouse-nav"
ENV = ("source /opt/ros/noetic/setup.bash && "
       f"source {shlex.quote(str(WORKSPACE / 'devel/setup.bash'))} && "
       f"source {shlex.quote(str(SCRIPTS / 'ros_net.sh'))}")
CONFLICT_NODES = {"/drone_reloc", "/car_bridge", "/waypoint_manager", "/offboard_gateway",
                  "/trajectory_executor", "/laserMapping", "/gravity_bridge",
                  "/ywl_localization", "/ywl_init_adapter", "/fastlio_dll"}


MAX_LOGS = 200


def log_time():
    return time.strftime("%H:%M:%S", time.localtime())


def infer_level(text):
    if any(token in text for token in ("失败", "故障", "异常", "错误", "终止", "被拒", "超时", "退出", "中断", "ERROR", "[ERR]")):
        return "error"
    if any(token in text for token in ("取消", "等待", "接管", "WARN", "[WARN]")):
        return "warn"
    if any(token in text for token in ("[ OK ]", "就绪", "成功", "已封包", "已收尾", "完成", "通过", "已确认")):
        return "ok"
    return "info"


def append_log(entries, text, level=None):
    clean = str(text or "").strip()
    if not clean:
        return
    clean = clean[:430]
    if entries and entries[-1].get("text") == clean:
        return
    entries.append(dict(time=log_time(), text=clean, level=level or infer_level(clean)))
    if len(entries) > MAX_LOGS:
        del entries[:-MAX_LOGS]


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
        append_log(value.setdefault("logs", []), value["message"], "error")
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
    check_damper(body)
    check_vertical(body)
    if set(body) - {"damperDeg", "vertical"} != expected or type(body["remaining"]) is not int or not 0 < body["remaining"] <= TTL:
        raise ValueError("下发请求无效或已过期")
    request_id = body["id"]
    task = task_for(catalog(ROOT, board=True), body["taskId"], body["mapKey"])
    if type(body["confirmAuto"]) is not bool or (task["automatic"] and not body["confirmAuto"]):
        raise ValueError("请确认自动起飞任务")
    previous = get_record(request_id)
    if previous:
        if "taskId" in previous and any(previous.get(key) != body.get(key) for key in ("taskId", "mapKey", "confirmAuto", "damperDeg", "vertical")):
            raise ValueError("请求 ID 已用于另一任务")
        return
    if active_record():
        raise ValueError("板端已有网页任务，请先收尾")
    if session_exists():
        raise ValueError("板端上一任务会话正在退出，稍后重新下发")
    value = dict(id=request_id, taskId=body["taskId"], mapKey=body["mapKey"],
                 confirmAuto=body["confirmAuto"], boot=BOOT, state="waiting_fcu",
                 message="板端已收到任务，等待飞控连接", deadline=time.monotonic() + body["remaining"],
                 safeGround=False, logs=[])
    append_log(value["logs"], value["message"], "info")
    for field in ("damperDeg", "vertical"):
        if field in body:
            value[field] = body[field]
    save(value)
    write(STATE / "current.json", dict(id=request_id, boot=BOOT))
    command = ENV + " && exec " + shlex.join(["python3", "-B", str(Path(__file__).resolve()), "run", request_id])
    try:
        result = tmux("new-session", "-d", "-s", SESSION, "bash", "-c", command, capture_output=True, text=True)
        if result.returncode:
            raise ValueError(result.stderr.strip() or "无法创建板端任务会话")
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        value.update(state="failed", message=str(error))
        append_log(value["logs"], value["message"], "error")
        save(value)


def control(verb, request_id):
    value = get_record(request_id)
    if verb == "cancel":
        if value is None:
            # A tombstone covers an ambiguous submit response followed by cancel.
            tomb = dict(id=request_id, boot=BOOT, state="cancelled", message="任务已取消", logs=[])
            append_log(tomb["logs"], tomb["message"], "warn")
            save(tomb)
            return
        if value["state"] in TERMINAL:
            return
        if value["state"] != "waiting_fcu":
            raise ValueError("导航已开始准备，着地上锁后可收尾")
        value.update(state="cancelled", message="已取消待启动任务")
        append_log(value.setdefault("logs", []), value["message"], "warn")
    else:
        if value and (value["state"] == "stopping" or value["state"] in TERMINAL):
            return
        if not value or time.monotonic() - value.get("healthAt", -100) >= 2 or not value.get("safeGround"):
            raise ValueError("等待飞控确认着地并上锁")
        if value["state"] == "waiting_fcu":
            raise ValueError("该任务尚未启动，可以取消")
        value["stopRequested"] = True
        append_log(value.setdefault("logs", []), "收到收尾请求，正在通知任务脚本退出", "warn")
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


def script_command(task, damper_deg=None, vertical=None):
    if task.get("mission"):
        return ["rosrun", "drone_inspection", "run_mission", "--mission", "missions/" + task["mission"]]
    if task["id"] == "test12_ywl":
        return [str(SCRIPTS / "nav_test12.sh"), "--ywl"]
    command = [str(SCRIPTS / ("nav_" + task["id"] + ".sh"))]
    if task["id"] == vertical_profile.TASK:
        vertical_profile.validate(vertical)
        for key in vertical_profile.PARAMETERS:
            command += ["--" + key, str(vertical[key])]
        return command
    if task["id"].startswith("test_indoor"):
        command += ["--map", str(MAPS / task["mapId"]),
                    "--nav-config", str(SCRIPTS / "nav_test/indoor_fly_nav.yaml"),
                    "--map-frame", str(SCRIPTS.parent / "src/localization/car_bridge/config/map_frame_indoor_fly.yaml"),
                    "--ev-mode", "odom"]
    if damper_deg is not None:
        check_damper(dict(taskId=task["id"], damperDeg=damper_deg))
        command += ["--damper-deg", str(damper_deg)]
    return command


def check_map(data, task):
    if task.get("mission"):
        import yaml
        preset = WORKSPACE / "src/navigation/drone_inspection/missions" / task["mission"]
        if fingerprint(preset) != task["missionSha256"]:
            raise ValueError("任务 YAML 版本已改变")
        folder = (preset.parent / Path(yaml.safe_load(preset.read_text())["map"]["dir"]).expanduser()).resolve()
        info = (folder / "map_world.pcd").stat()
        if (f"{folder.name}-{info.st_mtime_ns}-{info.st_size}" != task["mapKey"]
                or fingerprint(folder / "map_world.pcd") != task["mapSha256"]):
            raise ValueError("巡检任务地图版本已改变")
        return
    expected = data["maps"][task["mapId"]]
    path = MAPS / task["mapId"] / "map_world.pcd"
    info = path.stat()
    key = f"{task['mapId']}-{info.st_mtime_ns}-{info.st_size}"
    if key != expected["key"] or fingerprint(path) != expected["sha256"]:
        raise ValueError("板端 PCD 版本或 SHA-256 已变化，请更新任务目录与电脑缓存")


class FlightStatus:
    def __init__(self, rospy):
        from diagnostic_msgs.msg import DiagnosticArray
        from mavros_msgs.msg import EstimatorStatus, ExtendedState, State
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String

        self.lock = Lock()
        self.ros_time = rospy.Time.now
        self.latest = {}
        self.prepared_at = None
        self.subscribers = [
            rospy.Subscriber("/mavros/state", State, self.set, callback_args="fcu", queue_size=3),
            rospy.Subscriber("/mavros/extended_state", ExtendedState, self.set, callback_args="ext", queue_size=3),
            rospy.Subscriber("/mavros/estimator_status", EstimatorStatus, self.set,
                             callback_args="estimator", queue_size=3),
            rospy.Subscriber("/fastlio/odom_world", Odometry, self.set, callback_args="fastlio", queue_size=1),
            rospy.Subscriber("/dll_localization/status", DiagnosticArray, self.set, callback_args="dll", queue_size=1),
            rospy.Subscriber("/ywl/status", DiagnosticArray, self.set, callback_args="ywl", queue_size=1),
            rospy.Subscriber("/ywl/odom", Odometry, self.set, callback_args="ywl_pose", queue_size=1),
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

            def fresh(name, max_age, stamped=False):
                at, value = self.latest.get(name, (0, None))
                if value is None or now - at >= max_age:
                    return None
                if stamped:
                    stamp = value.header.stamp
                    if stamp.is_zero() or not -0.05 <= (self.ros_time() - stamp).to_sec() < max_age:
                        return None
                return value

            manual = task.get("manual", False)
            fcu, ext = fresh("fcu", 2, manual), fresh("ext", 1.5, manual)
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
            localized = (connected and loc.get("state") == "LOCALIZED" and loc.get("valid") is True
                         and result.get("confirmed") is True and result.get("map") == str(MAPS / task["mapId"]))
            if manual:
                odom = fresh("fastlio", .6, stamped=True)
                ready = (localized and odom is not None and odom.header.frame_id == "drone_odom"
                         and odom.child_frame_id == "drone_base")
                ywl = task.get("observer") == "ywl"
                name = "YWL" if ywl else "DLL"
                diagnostics = fresh("ywl" if ywl else "dll", 1.5, stamped=not ywl)
                diagnostic_name = "ywl_localization" if ywl else "fastlio_dll"
                diag = next((s for s in diagnostics.status if s.name == diagnostic_name), None) if diagnostics else None
                observer_tracking = bool(diag and diag.message == "TRACKING" and diag.level == 0)
                reason = next((item.value for item in diag.values if item.key == "reason"), "") if diag else ""
                observer_note = f"{name} {diag.message} {reason}".strip() if diag else f"等待 {name} 状态"
                if observer_tracking and ywl:
                    pose = fresh("ywl_pose", .5, stamped=True)
                    observer_tracking = bool(pose and pose.header.frame_id == "drone_map"
                                             and pose.child_frame_id == "ywl_imu"
                                             and abs((pose.header.stamp - diagnostics.header.stamp).to_sec()) < .15)
                    if not observer_tracking:
                        observer_note = "等待新鲜 YWL 地图位姿"
            else:
                ready = localized and nav.get("state") == "READY" and gateway.get("real_output") is True
                observer_tracking, observer_note = False, ""
            stream = ready and self.prepared_at is not None and now - self.prepared_at >= 1.0
            return dict(connected=connected, mode=mode, armed=armed, landed=landed, ground=ground,
                        ready=ready, stream=stream, estimate=estimate, observer_tracking=observer_tracking, observer_note=observer_note)


class Progress:
    def __init__(self):
        self.lock = Lock()
        self.note = ""
        self.scan = None
        self.tag_coverage = ""
        self.phase = ""
        self.runner_started = False
        self.mission_state = None
        self.takeoff_ready = False
        self.finished = None
        self.stop_refused = False
        self.clean_exit = False
        self.vertical_error = None
        self.manual_ready = False
        self.video_state = ""
        self.recording = ""
        self.logs = []

    def collect(self, stream):
        for line in stream:
            print(line.rstrip(), flush=True)
            line = line.strip()
            if not line or line.startswith("----"):
                continue
            with self.lock:
                if line.startswith("INSPECTION_EVENT "):
                    try:
                        event = json.loads(line.partition(" ")[2])
                        if event["version"] != 1:
                            continue
                        self.mission_state = event
                        self.phase = event["phase"]
                        self.scan = (dict(event, phase=event["action"]) if event.get("action")
                                     and "discovered" in event and event["phase"] == "ROUTE" else None)
                        summary = " · ".join(str(v) for v in (event.get("observation"),
                            event.get("action") or event["phase"], event.get("reason")) if v)
                        self.note = summary
                        append_log(self.logs, summary)
                        continue
                    except (ValueError, KeyError, TypeError):
                        pass
                if " INDOOR4 " in line or " INDOOR5 " in line or " INDOOR6 " in line:
                    try:
                        scan = json.loads(line.split(" INDOOR", 1)[1].split(" ", 1)[1])
                        if scan["phase"] != "识别标签" and self.scan and "route" not in scan:
                            scan["route"] = self.scan.get("route")
                        self.scan = scan
                        tag = f" · Tag {scan['tagId']}" if scan.get("tagId") is not None else ""
                        obs = scan.get("observation") or ""
                        summary = " · ".join(part for part in (obs, scan.get("phase") or "") if part) + tag
                        if summary:
                            append_log(self.logs, summary, "info")
                        continue
                    except (ValueError, KeyError, TypeError):
                        pass
                if "原地起降中止：" in line and not self.vertical_error:
                    self.vertical_error = line.split("原地起降中止：", 1)[1]
                append_log(self.logs, line)
                if "AprilTag 观察 " in line:
                    self.tag_coverage = line[line.index("AprilTag 观察 "):][:250]
                self.note = line[:430]
                if "VIDEO34 " in line:
                    self.video_state = line.split("VIDEO34 ", 1)[1]
                if "室外重定位已确认" in line or "室内重定位已确认" in line:
                    self.manual_ready = True
                if "Channel 6 录包已待命" in line:
                    self.recording = "待录包"
                if "Channel 6 录包中：" in line or "Channel 6 恢复录包" in line:
                    self.recording = "录包中"
                if "Channel 6 已离开录制位" in line:
                    self.recording = "正在封包"
                if "已封包：" in line:
                    self.recording = "已封包"
                if "RECORD 进程已退出" in line:
                    self.recording = "录包器已退出，请检查 record.log"
                if "[ OK ] 导航就绪" in line:
                    self.runner_started = True
                if "待命：定点模式在地面解锁" in line:
                    self.takeoff_ready = line.endswith("解锁后自动起飞")
                if " 阶段：" in line:
                    self.phase = line.rsplit(" 阶段：", 1)[1].strip()
                    if "test_indoor6 阶段：" in line and self.phase != "ROUTE":
                        self.scan = None
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

    def stage(self, task, health, has_flown, vertical=None):
        with self.lock:
            phase, note, finished = self.phase, self.note, self.finished
            if task.get("mission") and self.mission_state:
                event = self.mission_state
                if event["end"] != "RUNNING":
                    return "action_done", event.get("reason") or "巡检动作已结束，等待地面收尾"
                if event["phase"] == "WAIT_ARM" and health["ready"] and health["ground"]:
                    return "wait_arm", f"地面预检通过，遥控解锁后 {task['delay']:g} s 自动起飞"
                if event["phase"] not in ("BRINGUP", "PREFLIGHT", "WAIT_ARM"):
                    return "running", event.get("action") or event["phase"]
            if task["id"] == "test_indoor3_4":
                display = self.video_state or self.note or "等待 OV13855 和室内定位"
                if has_flown and health["ground"]:
                    return "action_done", display + " · 关通道 6，视频封存后收尾"
                if health["armed"]:
                    return "running", "手飞 · " + display
                camera_ready = self.video_state.startswith("ov13855 就绪 ")
                if self.manual_ready and health["ready"] and camera_ready:
                    return "wait_pilot", "定位就绪 · " + display
                return "preparing", display
            if task.get("manual"):
                if self.recording == "录包中":
                    record = "录包中。起飞前静置录够 20 秒"
                elif self.recording == "已封包":
                    record = "已封包"
                else:
                    record = self.recording or "等待录包器就绪"
                if has_flown and health["ground"]:
                    if self.recording == "已封包":
                        return "action_done", "已封包，可结束任务并收尾"
                    return "action_done", record + "。关通道 6 并确认封包后收尾"
                if health["armed"] and health["landed"] == 2:
                    note = "遥控手飞 · " + (self.recording or record)
                    if not health["observer_tracking"]:
                        note += " · " + health["observer_note"] + ("，显示原定位" if task.get("observer") == "ywl" else "，旁路对照不可用")
                    return "running", note
                needs_ywl = task.get("observer") == "ywl"
                recorder_ok = self.recording in {"待录包", "录包中", "正在封包", "已封包"}
                if (self.manual_ready and recorder_ok and health["ready"]
                        and (not needs_ywl or health["observer_tracking"])):
                    if self.recording == "待录包":
                        record = "定位就绪。拨通道 6 开包，确认录包后静置 20 秒"
                    return "wait_pilot", record
                if needs_ywl and self.manual_ready and not health["observer_tracking"]:
                    return "preparing", health["observer_note"]
                return "preparing", note or "等待录包器、FAST-LIO2 和室外重定位"
            if task["automatic"]:
                if task["id"] == vertical_profile.TASK:
                    if self.vertical_error:
                        return "action_done", self.vertical_error
                    if phase == "DONE":
                        return "action_done", "已确认着地上锁，等待收尾"
                    labels = {"COUNTDOWN": "人工解锁后倒计时", "OFFBOARD_ENTRY": "切入 OFFBOARD",
                              "TAKEOFF": f"原地起飞 {vertical['height']:g} m",
                              "HOVER": f"原地悬停 {vertical['hover']:g} s",
                              "LAND_REQUEST": "准备原地慢降", "LANDING": "原地慢降，等待着地上锁"}
                    if phase in labels:
                        return "running", labels[phase]
                if phase == "DONE" or finished:
                    return "action_done", finished or note
                if task["id"] == "test_indoor6":
                    stage_label = {"COUNTDOWN": "人工解锁后倒计时", "OFFBOARD_ENTRY": "切入 OFFBOARD",
                                   "TAKEOFF": "原地起飞至离地 0.50 m", "ROUTE": "四点巡检",
                                   "LAND_REQUEST": "返航停稳，准备慢降", "LANDING": "慢降，等待着地上锁"}
                    if phase in stage_label:
                        label = self.scan["phase"] if phase == "ROUTE" and self.scan else stage_label[phase]
                        return "running", label + (" · " + note if note else "")
                if phase in {"COUNTDOWN", "OFFBOARD_ENTRY", "TAKEOFF", "ROUTE", "LAND_REQUEST", "LANDING"} or (
                    health["armed"] and health["mode"] == "OFFBOARD"
                ):
                    return "running", note
                preflight_passed = (phase == "WAIT_ARM" if task["id"] != "test6" else self.takeoff_ready)
                if has_flown and health["ground"]:
                    return "action_done", note
                if preflight_passed and health["ready"] and health["ground"]:
                    if health["mode"] == "POSCTL":
                        delay = vertical["delay"] if vertical else 3
                        return "wait_arm", f"预检通过。按通道 6 录包，地面遥控解锁后约 {delay:g} s 自动起飞"
                    return "preparing", "保持地面上锁，用遥控器切到定点模式（POSCTL）"
            elif finished or (has_flown and health["ground"]):
                return "action_done", finished or note
            elif health["armed"] and health["mode"] in {"OFFBOARD", "AUTO.LAND"}:
                return "running", note
            elif health["stream"] and self.runner_started and health["armed"] and health["landed"] == 2:
                if task["id"] == "test_indoor3_3":
                    return "wait_offboard", "将 Tag 1 放入 OV13855 视野，切入 OFFBOARD 开始一轮扫码"
                if task["id"] in {"test_indoor3", "test_indoor3_1", "test_indoor3_2", "test_indoor3_5", "test_indoor4", "test_indoor5"}:
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
    for name in ("nav", "nav12"):
        if tmux("has-session", "-t", "=" + name, capture_output=True).returncode == 0:
            raise ValueError(f"已有终端 {name} 会话，请先收尾")


def supervise(request_id):
    import rospy

    with locked():
        value = get_record(request_id)
    if not value or value["state"] != "waiting_fcu":
        return
    data = catalog(ROOT, board=True)
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
                append_log(value.setdefault("logs", []), value["message"], "error")
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
            append_log(value.setdefault("logs", []), value["message"], "error")
            save(value)
            return
        if not flight.snapshot(task)["ground"]:
            raise ValueError("启动时飞控地面上锁状态已变化")
        value.update(state="preparing", message="开始定位与录包预检" if task.get("manual") else "开始定位与导航预检")
        append_log(value.setdefault("logs", []), value["message"], "info")
        pre_logs = list(value["logs"])
        save(value)
        env = dict(os.environ, WAREHOUSE_TASK_ID=request_id, PYTHONUNBUFFERED="1")
        if task.get("mission"):
            env["WAREHOUSE_MISSION_SHA256"] = task["missionSha256"]
        env.pop("NAV_TEST_IN_TMUX", None)
        child = subprocess.Popen(script_command(task, value.get("damperDeg"), value.get("vertical")), cwd=SCRIPTS, env=env, stdin=subprocess.DEVNULL,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, errors="replace", start_new_session=True)
    progress = Progress()
    reader = Thread(target=progress.collect, args=(child.stdout,), daemon=True)
    reader.start()
    has_flown = False
    stopping = False
    last_state_note = None
    while child.poll() is None:
        health = flight.snapshot(task)
        has_flown = has_flown or bool(health["armed"] and health["landed"] == 2)
        state, message = progress.stage(task, health, has_flown, value.get("vertical"))
        if not health["connected"]:
            message = "飞控状态中断，等待新的飞控数据"
        with progress.lock:
            scan = progress.scan
            mission_state = progress.mission_state
            if task["id"] in {"test_indoor3_1", "test_indoor3_2"} and progress.tag_coverage and progress.tag_coverage not in message:
                message += " · " + progress.tag_coverage
            if progress.stop_refused:
                stopping = False
                progress.stop_refused = False
            script_logs = list(progress.logs)
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
            next_state = "stopping" if stopping else state
            next_message = "正在收尾与写盘" if stopping else message
            if (next_state, next_message) != last_state_note:
                last_state_note = (next_state, next_message)
                with progress.lock:
                    if not progress.logs or progress.logs[-1]["text"] != next_message:
                        append_log(progress.logs, next_message)
                    script_logs = list(progress.logs)
            value.update(state=next_state,
                         message=next_message,
                         mode=health["mode"], armed=health["armed"], landed=health["landed"],
                         safeGround=health["ground"], healthAt=time.monotonic(),
                         logs=(pre_logs + script_logs)[-MAX_LOGS:])
            if task.get("mission") and mission_state:
                value["missionState"] = mission_state
            if (task.get("mission") or task["id"] in {"test_indoor4", "test_indoor5", "test_indoor6"}) and scan is not None:
                value["scan"] = scan
            elif task.get("mission") or task["id"] == "test_indoor6":
                value.pop("scan", None)
            save(value)
        time.sleep(.5)
    reader.join(timeout=1)
    with locked():
        value = get_record(request_id)
        cleaned = progress.clean_exit and child.returncode in (0, 130)
        final_message = "已收尾，记录已写盘" if cleaned else (
            f"任务脚本退出（{child.returncode}）：" + (progress.note or value["message"]))
        with progress.lock:
            combined = (pre_logs + list(progress.logs))[-MAX_LOGS:]
        result = progress.mission_state if task.get("mission") else None
        if result and result["end"] in {"ABORTED", "INTERRUPTED"}:
            final_message = (result.get("reason") or result["end"]) + (" · 已收尾" if cleaned else " · 收尾异常")
            cleaned = False
        if task["id"] == vertical_profile.TASK and (progress.vertical_error or progress.phase != "DONE"):
            final_message = progress.vertical_error or "原地起降未完成"
            cleaned = False
        append_log(combined, final_message, "ok" if cleaned else "error")
        value.update(state="done" if cleaned else "failed", safeGround=False,
                     message=final_message, logs=combined)
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
                append_log(value.setdefault("logs", []), str(error), "error")
                value.update(state="failed", message=str(error), safeGround=False)
                save(value)
        raise


if __name__ == "__main__":
    main()
