"""Carry hardware H.264 access units and same-frame observations over a dedicated SSH channel."""
import collections
import json
import os
import select
import signal
import struct
import subprocess
import sys
import time

import rospy
from sensor_msgs.msg import CompressedImage
from ov13855_apriltag_vision.msg import TagCornerArray
from barcode_reader_camera.msg import BarcodeFrame

HEADER = struct.Struct("!cQI")


def main():
    output = os.dup(1)
    os.dup2(2, 1)  # ROS log output must never enter the binary protocol.
    os.set_blocking(output, False)
    rospy.init_node("ov13855_monitor_sender", anonymous=True, disable_rosout=True)
    packets = collections.deque(maxlen=12)

    def packet(kind, stamp, data):
        if not isinstance(data, bytes):
            data = json.dumps(data, ensure_ascii=False).encode()
        packets.append(HEADER.pack(kind, stamp, len(data)) + data)

    def video(msg):
        if msg.format.startswith("error/"):
            packet(b"E", 0, msg.format[6:].encode())
        else:
            packet(b"S" if msg.format.endswith("/standalone") else b"V", msg.header.stamp.to_nsec(), struct.pack("!I", msg.header.seq) + msg.data)

    def corners(msg):
        packet(b"T", msg.header.stamp.to_nsec(), {"width": msg.width, "height": msg.height,
               "tags": [[t.id, [[p.x, p.y] for p in t.corners]] for t in msg.tags]})

    def barcodes(msg):
        packet(b"Q", msg.header.stamp.to_nsec(), {"width": msg.width, "height": msg.height,
               "selfpath": msg.selfpath, "status": msg.status,
               "codes": [[c.code, list(zip(c.corners[::2], c.corners[1::2]))] for c in msg.codes]})

    video_sub = rospy.Subscriber("/ov13855/monitor/h264", CompressedImage, video, queue_size=3, buff_size=2**20)
    rospy.Subscriber("/ov13855/monitor/corners", TagCornerArray, corners, queue_size=3)
    rospy.Subscriber("/ov13855/monitor/barcodes", BarcodeFrame, barcodes, queue_size=2)
    child = None
    heartbeat = time.monotonic()
    retry = time.monotonic() + 1
    try:
        while not rospy.is_shutdown():
            if select.select([0], [], [], 0)[0]:
                if not os.read(0, 64):
                    break
                heartbeat = time.monotonic()
            if time.monotonic() - heartbeat > 7:
                break
            if child and child.poll() is not None:
                child = None
            if not child and not video_sub.get_num_connections() and time.monotonic() >= retry:
                child = subprocess.Popen(["rosrun", "ov13855_cam", "ov13855_cam_node",
                    "__name:=ov13855_console_%d" % os.getpid(), "_monitor_standalone:=true",
                    "_image_topic:=/ov13855/monitor/raw", "_camera_info_topic:=/ov13855/monitor/info"],
                    stdin=subprocess.DEVNULL, stdout=sys.stderr, start_new_session=True)
                retry = time.monotonic() + 2
            if not packets:
                time.sleep(.006)
                continue
            view = memoryview(packets.popleft())
            while view:
                readable, writable, _ = select.select([0], [output], [], 1)
                if readable:
                    if not os.read(0, 64):
                        return
                    heartbeat = time.monotonic()
                if not writable:
                    raise TimeoutError("video receiver stalled")
                try:
                    view = view[os.write(output, view):]
                except BlockingIOError:
                    continue
    finally:
        if child and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        rospy.signal_shutdown("viewer disconnected")
        os.close(output)


if __name__ == "__main__":
    main()
