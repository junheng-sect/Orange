"""Shared test12 drift measurement, in map axes at one scan-end timestamp."""
from collections import OrderedDict, deque
import json
import math

NS = 10**9
BASE = "/drone_reloc/pose"
DLL = "/dll_localization/pose"
EDGE = "/dll_localization/correction"
HEALTH = "/dll_localization/status"
RESULT = "/drone_reloc/result"
RELOC = "/drone_reloc/status"
MODE = "/test12/observer"
NAV_MODE = "/nav_test/observer"
MODE_TOPICS = (MODE, NAV_MODE)
ODOM = "/fastlio/odom_world"
TOPICS = (BASE, DLL, EDGE, HEALTH, RESULT, RELOC, "/tf", *MODE_TOPICS, ODOM)


def pose_values(pos, rot):
    xyz = (pos.x, pos.y, pos.z)
    q = (rot.x, rot.y, rot.z, rot.w)
    norm = math.sqrt(sum(v*v for v in q))
    if not all(math.isfinite(v) for v in (*xyz, norm)) or abs(norm-1) > .02:
        raise ValueError("invalid pose")
    return xyz, tuple(v/norm for v in q)


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def difference(base, observed):
    dx, dy, dz = (v-u for u, v in zip(base[0], observed[0]))
    angle = yaw(observed[1])-yaw(base[1])
    return dict(dx_m=dx, dy_m=dy, dz_m=dz, horizontal_m=math.hypot(dx, dy),
                yaw_deg=math.degrees(math.atan2(math.sin(angle), math.cos(angle))))


class Comparison:
    def __init__(self):
        self.mode_topic = None
        self.mode = ""
        self.run = ""
        self.note = ""
        self.mode_at = 0
        self.reset()

    def reset(self):
        self.solution = None
        self.reloc = deque(maxlen=32)
        self.health = deque(maxlen=64)
        self.edges = deque(maxlen=64)
        self.frames = OrderedDict()
        self.latest_odom = self.odom_at = self.health_at = 0

    def feed(self, topic, msg, received_ns):
        if topic in MODE_TOPICS:
            value = json.loads(msg.data)
            if self.run and self.run != value["run"]:
                self.reset()
            self.mode_topic = topic
            self.run, self.mode = value["run"], value["mode"]
            self.note, self.mode_at = value.get("note", ""), received_ns
            return
        if topic == RESULT:
            v = json.loads(msg.data)
            solution = None
            if v.get("ok") is True and v.get("confirmed") is True:
                t = v["transform"]
                solution = (str(v["map"]), round(v["stamp"]*NS),
                            tuple(t["translation"]), tuple(t["q_xyzw"]))
            if solution != self.solution:
                self.reset()
                self.solution = solution
            return
        if topic == RELOC:
            v = json.loads(msg.data)
            valid = v.get("state") == "LOCALIZED" and v.get("valid") is True
            if not valid:
                self.frames.clear()
                self.health.clear()
            self.reloc.append((round(v["stamp"]*NS), valid))
            return
        if topic == "/tf":
            for tr in msg.transforms:
                if (tr.header.frame_id, tr.child_frame_id) == ("drone_map", "drone_odom"):
                    pos, q = pose_values(tr.transform.translation, tr.transform.rotation)
                    match = bool(self.solution and max(abs(a-b) for a,b in zip(pos,self.solution[2])) <= 1e-4
                                 and abs(sum(a*b for a,b in zip(q,self.solution[3]))) >= 1-1e-8)
                    self.edges.append((tr.header.stamp.to_nsec(), match))
            return
        stamp = msg.header.stamp.to_nsec()
        if stamp <= 0:
            return
        if topic == HEALTH:
            for s in msg.status:
                if s.name != "fastlio_dll":
                    continue
                self.health_at = received_ns
                fields = {v.key:v.value for v in s.values}
                self.health.append((stamp, s.message, s.level == 0 and fields.get("local_input") == "fresh",
                                    int(fields.get("accepted_stamp_ns") or 0),
                                    int(fields.get("legacy_seed_stamp_ns") or 0), fields.get("reason", "")))
            return
        if topic == ODOM:
            if (msg.header.frame_id, msg.child_frame_id) == ("drone_odom", "drone_base"):
                if stamp < self.latest_odom:
                    self.reset()
                self.latest_odom = stamp
                self.odom_at = received_ns
            return
        if msg.header.frame_id != "drone_map" or not self.solution or stamp < self.solution[1]-1000:
            return
        if topic == EDGE:
            if msg.child_frame_id != "drone_odom":
                return
            pose_values(msg.transform.translation, msg.transform.rotation)
            value = True
        elif topic in (BASE, DLL):
            value = pose_values(msg.pose.position, msg.pose.orientation)
        else:
            return
        self.frames.setdefault(stamp, {})[topic] = value
        if len(self.frames) > 64:
            self.frames.popitem(last=False)

    @staticmethod
    def at(history, stamp):
        return max((v for v in reversed(history) if v[0] <= stamp), key=lambda v:v[0], default=None)

    def measure(self, stamp):
        result = dict(stamp_ns=stamp, valid=False, state="不可用", reason="", age_s=None)
        rel, tf, dll = (self.at(h, stamp) for h in (self.reloc, self.edges, self.health))
        frame = self.frames.get(stamp, {})
        if dll:
            result["state"] = dll[1]
            if dll[3]:
                result["age_s"] = (stamp-dll[3])/NS
        result["base_valid"] = False
        if not self.solution or stamp < self.solution[1]-1000:
            reason = "无有效重定位初值"
        elif not rel or not rel[1] or stamp-rel[0] > 2.5*NS:
            reason = "原定位不可用"
        elif not tf or not tf[1] or stamp-tf[0] > 1.25*NS:
            reason = "原地图 TF 不可用"
        else:
            result["base_valid"] = BASE in frame
            reason = None
        if reason:
            result["reason"] = reason
            return result
        if self.mode_topic == NAV_MODE and self.note:
            reason = self.note
        elif not dll or stamp-dll[0] > 1.5*NS:
            reason = self.note or "DLL 状态缺失或过期"
        elif not dll[2] or dll[1] != "TRACKING":
            reason = dll[5] or dll[1]
        elif abs(dll[4]-self.solution[1]) > 1000:
            reason = "DLL 重定位会话不匹配"
        elif not dll[3] or not 0 <= stamp-dll[3] < 1.5*NS:
            reason = "DLL 校正过期"
        elif BASE not in frame or DLL not in frame or EDGE not in frame:
            reason = "缺少同帧位姿或校正"
        else:
            result.update(valid=True, **difference(frame[BASE], frame[DLL]))
            return result
        result["reason"] = reason
        return result

    def live(self, now_ns, aligned, wall_ns=None):
        stamp = self.latest_odom
        result = self.measure(stamp)
        if not aligned:
            return dict(valid=False, state=result["state"], reason="原定位未对齐或已失效")
        if wall_ns is not None and wall_ns-self.odom_at > .5*NS:
            return dict(valid=False, state=result["state"], reason="主位姿过期")
        if wall_ns is not None and wall_ns-self.health_at > 1.5*NS:
            return dict(valid=False, state="不可用", reason=self.note or "DLL 状态缺失或过期")
        if not -.05*NS <= now_ns-stamp <= .5*NS:
            return dict(valid=False, state=result["state"], reason="主位姿过期")
        # Do not carry an older good frame through an explicit loss of tracking.
        if self.health and (self.health[-1][1] != "TRACKING" or not self.health[-1][2]):
            h = self.health[-1]
            return dict(valid=False, state=h[1], reason=h[5] or h[1])
        if result["valid"] and (now_ns-stamp)/NS + result["age_s"] >= 1.5:
            return dict(valid=False, state=result["state"], reason="DLL 校正过期")
        if result["reason"] == "缺少同帧位姿或校正":
            for prev in sorted(self.frames, reverse=True):
                if 0 < stamp-prev < .15*NS and now_ns-prev <= .5*NS:
                    paired = self.measure(prev)
                    if paired["valid"] and (now_ns-prev)/NS + paired["age_s"] < 1.5:
                        return paired
        return result
