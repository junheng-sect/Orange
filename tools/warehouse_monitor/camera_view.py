"""OV13855 viewing sessions, H.264 decoder and computer-side overlays."""
import collections
import json
import os
import select
import struct
import subprocess
import threading
import time

HEADER = struct.Struct("!cQI")
REMOTE = ("source /opt/ros/noetic/setup.bash && source ~/orange/devel/setup.bash && "
          "source ~/mid360s_ws/scripts/ros_net.sh && exec python3 -B ~/orange/tools/warehouse_monitor/camera_sender.py")
MODES = {"auto", "raw", "task", "local"}


class Decoder:
    def __init__(self):
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        import numpy as np
        self.gst, self.np = Gst, np
        Gst.init(None)
        self.pipe = Gst.parse_launch(
            "appsrc name=input is-live=true format=time caps=video/x-h264,stream-format=byte-stream,alignment=au "
            "! h264parse ! avdec_h264 max-threads=1 ! videoconvert ! video/x-raw,format=BGR "
            "! appsink name=output sync=false async=false max-buffers=2 drop=true")
        self.input = self.pipe.get_by_name("input")
        self.output = self.pipe.get_by_name("output")
        self.pipe.set_state(Gst.State.PLAYING)
        self.previous = None
        self.synced = False

    def frame(self, stamp, packet):
        seq = struct.unpack_from("!I", packet)[0]
        au = packet[4:]
        if self.previous is None or seq != (self.previous + 1) % (1 << 32):
            self.synced = False
        self.previous = seq
        if not self.synced:
            # Hardware repeats the SPS/PPS at each IDR (Annex B).
            self.synced = any((chunk[0] & 31) == 5 for chunk in au.split(b"\x00\x00\x01") if chunk)
            if not self.synced:
                return
        buf = self.gst.Buffer.new_wrapped(au)
        buf.pts = buf.dts = stamp
        if self.input.emit("push-buffer", buf) != self.gst.FlowReturn.OK:
            raise RuntimeError("本机 H.264 解码失败")
        sample = self.output.emit("try-pull-sample", 15*self.gst.MSECOND)
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps().get_structure(0)
            w, h = caps.get_value("width"), caps.get_value("height")
            pixels = buf.extract_dup(0, buf.get_size())
            return buf.pts, self.np.frombuffer(pixels, dtype="uint8").reshape(h, -1)[:, :3*w].reshape(h, w, 3)

    def close(self):
        self.pipe.set_state(self.gst.State.NULL)


def draw_boxes(image, boxes):
    import cv2
    import numpy as np
    for label, points in boxes:
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        if len(pts) != 4 or not np.isfinite(pts).all():
            continue
        pts = np.rint(pts).astype("int32")
        cv2.polylines(image, [pts], True, (50, 235, 85), 2, cv2.LINE_AA)
        cv2.putText(image, str(label)[:80], tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, .6, (40, 205, 255), 2)


class LocalDetector:
    def __init__(self):
        import cv2
        self.cv = cv2
        self.family = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector = cv2.aruco.ArucoDetector(self.family) if hasattr(cv2.aruco, "ArucoDetector") else None
        self.qr = cv2.QRCodeDetector()

    def boxes(self, image):
        gray = self.cv.cvtColor(image, self.cv.COLOR_BGR2GRAY)
        tags, ids, _ = (self.detector.detectMarkers(gray) if self.detector else
                         self.cv.aruco.detectMarkers(gray, self.family))
        boxes = [(int(i), p) for i, p in zip(ids.flatten(), tags)] if ids is not None else []
        ok, texts, points, _ = self.qr.detectAndDecodeMulti(gray)
        if ok:
            boxes.extend((text or "QR", p) for text, p in zip(texts, points))
        return boxes


class Camera:
    def __init__(self, ssh, host, stopped):
        self.command = [*ssh, "-T", "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2", host, REMOTE]
        self.stopped = stopped
        self.cv = threading.Condition()
        self.viewers = {}
        self.jpegs = {}
        self.frames = collections.deque(maxlen=6)
        self.tags = {}
        self.barcodes = {}
        self.events = collections.deque(maxlen=5)
        self.error = ""
        self.task = False
        self.times = collections.deque(maxlen=32)
        self.worker = None
        self.source = None
        self.last_shown = 0

    def modes(self):
        now = time.monotonic()
        self.viewers = {k: v for k, v in self.viewers.items() if v[1] > now}
        return {self.mode(v[0]) for v in self.viewers.values()}

    def mode(self, name):
        return ("task" if self.task else "raw") if name == "auto" else name

    def watch(self, client, mode):
        with self.cv:
            if mode == "close":
                self.viewers.pop(client, None)
            else:
                self.viewers[client] = (mode, time.monotonic() + 5)
                if not self.worker or not self.worker.is_alive():
                    self.worker = threading.Thread(target=self.run, daemon=True)
                    self.worker.start()
            self.cv.notify_all()
            times = self.times
            fresh = bool(times and time.monotonic()-times[-1] < 1.5)
            fps = round((len(times)-1)/(times[-1]-times[0]), 1) if fresh and len(times)>1 and times[-1]-times[0] >= 1 else None
            return dict(mode=self.mode(mode), fps=fps, live=fresh, error=self.error,
                        source=self.source, events=list(self.events))

    def jpeg(self, client, previous):
        with self.cv:
            def ready():
                viewer = self.viewers.get(client)
                if not viewer or viewer[1] <= time.monotonic():
                    return True
                item = self.jpegs.get(self.mode(viewer[0]))
                return item and item[0] != previous
            self.cv.wait_for(ready, timeout=1)
            v = self.viewers.get(client)
            if not v or v[1] <= time.monotonic():
                return None
            return self.jpegs.get(self.mode(v[0]), (previous, None))

    def clear(self):
        self.barcodes.clear(); self.frames.clear(); self.tags.clear(); self.jpegs.clear(); self.times.clear()
        self.events.clear()
        self.task = False
        self.last_shown = 0
        self.source = None

    def event(self, stamp, message):
        if message:
            self.events.appendleft({"stamp": str(stamp), "text": message})

    def receive(self, proc):
        decoder = Decoder()
        buffer = bytearray()
        last = time.monotonic()
        heartbeat = 0.0
        try:
            while not self.stopped.is_set():
                with self.cv:
                    if not self.modes():
                        break
                if time.monotonic() - heartbeat > 1:
                    proc.stdin.write(b".")
                    proc.stdin.flush()
                    heartbeat = time.monotonic()
                if not select.select([proc.stdout], [], [], .15)[0]:
                    if time.monotonic()-last > 8:
                        raise RuntimeError("等待相机超时，正在重连")
                    continue
                chunk = os.read(proc.stdout.fileno(), 262144)
                if not chunk:
                    raise RuntimeError("板端连接已关闭，正在重连")
                buffer.extend(chunk)
                while len(buffer) >= HEADER.size:
                    kind, stamp, size = HEADER.unpack_from(buffer)
                    if size > 4*1024*1024:
                        raise ValueError("相机数据包无效")
                    end = HEADER.size + size
                    if len(buffer) < end:
                        break
                    data = bytes(buffer[HEADER.size:end])
                    del buffer[:end]
                    last = time.monotonic()
                    if kind == b"E":
                        raise RuntimeError("板端硬编码失败 · " + data.decode(errors="replace"))
                    if kind in (b"V", b"S"):
                        source = "任务相机" if kind == b"V" else "独立监看"
                        if self.source and self.source != source:
                            decoder.close(); decoder = Decoder()
                            with self.cv:
                                self.clear()
                        result = decoder.frame(stamp, data)
                        with self.cv:
                            self.source = source
                            if result:
                                self.error = ""
                                self.times.append(time.monotonic())
                                self.frames.append((*result, time.monotonic()))
                                self.cv.notify_all()
                    elif kind == b"T":
                        obs = json.loads(data)
                        with self.cv:
                            self.task = True
                            if stamp <= self.last_shown:
                                if obs["tags"]:
                                    self.event(stamp, "Tag " + ", ".join(str(t[0]) for t in obs["tags"]))
                            else:
                                self.tags[stamp] = obs
                                if len(self.tags) > 24:
                                    self.tags.pop(next(iter(self.tags)))
                    elif kind == b"Q":
                        with self.cv:
                            obs = json.loads(data)
                            labels = [c[0] for c in obs["codes"]]
                            self.event(stamp, "扫码 · " + (", ".join(labels) or obs["status"]))
                            if obs["selfpath"] and stamp > self.last_shown and labels:
                                self.barcodes[stamp] = obs
                                if len(self.barcodes) > 24:
                                    self.barcodes.pop(next(iter(self.barcodes)))
        finally:
            decoder.close()

    def render(self, ending):
        import cv2
        cv2.setNumThreads(2)
        detector = None
        while not ending.is_set():
            with self.cv:
                self.cv.wait(timeout=.025)
                modes = self.modes()
                if not modes or not self.frames:
                    continue
                # Briefly allow native task corners to arrive; never match by wall time.
                if "task" in modes and self.frames[0][0] not in self.tags and time.monotonic()-self.frames[0][2] < .065:
                    continue
                while len(self.frames) > 1 and time.monotonic() - self.frames[0][2] > .15:
                    self.frames.popleft()
                stamp, pixels, _ = self.frames.popleft()
                self.last_shown = stamp
                obs = self.tags.pop(stamp, None)
                qr = self.barcodes.pop(stamp, None)
                for missed in [key for key in self.tags if key < stamp]:
                    lost = self.tags.pop(missed)
                    if lost["tags"]:
                        self.event(missed, "Tag " + ", ".join(str(t[0]) for t in lost["tags"]))
            try:
                for mode in modes:
                    image = pixels
                    if mode == "local":
                        detector = detector or LocalDetector()
                        image = pixels.copy()
                        draw_boxes(image, detector.boxes(image))
                    elif mode == "task":
                        boxes = []
                        for observation, key in ((obs, "tags"), (qr, "codes")):
                            if observation and (observation["height"], observation["width"]) == pixels.shape[:2]:
                                boxes.extend(observation[key])
                        if boxes:
                            image = pixels.copy()
                            draw_boxes(image, boxes)
                    ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 78])
                    if ok:
                        with self.cv:
                            self.jpegs[mode] = (stamp, jpeg.tobytes())
                            self.cv.notify_all()
            except Exception as e:
                with self.cv:
                    self.error = "本机图像处理失败 · " + str(e)

    def run(self):
        import contextlib
        while not self.stopped.is_set():
            with self.cv:
                if not self.modes():
                    self.clear()
                    self.worker = None
                    return
            ending = threading.Event()
            renderer = threading.Thread(target=self.render, args=(ending,), daemon=True)
            proc = None
            try:
                import cv2
                proc = subprocess.Popen(self.command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
                renderer.start()
                self.receive(proc)
            except Exception as e:
                with self.cv:
                    self.error = str(e)
            finally:
                ending.set()
                if renderer.is_alive():
                    renderer.join()
                if proc:
                    with contextlib.suppress(BrokenPipeError):
                        proc.stdin.close()
                    with contextlib.suppress(subprocess.TimeoutExpired):
                        proc.wait(timeout=4)
                    if proc.poll() is None:
                        proc.kill(); proc.wait()
                    proc.stdout.close()
                with self.cv:
                    self.clear()
            self.stopped.wait(.5)
