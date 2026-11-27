"""OV13855 camera subscriptions, calibration and fresh frame checks."""
from collections import deque
from pathlib import Path
import shutil
import threading
import time

import numpy as np
import rospkg
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from ov13855_apriltag_vision.msg import TagCornerArray
from barcode_reader_camera.srv import DecodeBarcodes
from std_srvs.srv import Trigger
from drone_navigation.core import finite, rotation
from nav_forward import say
from nav_square import Stopped

CAMERA_IMAGE = '/ov13855_full/image_raw'
CAMERA_INFO = '/ov13855_full/camera_info'
DECODE = '/barcode_reader_camera/decode'
CANCEL_DECODE = '/barcode_reader_camera/cancel'
FRAME = 'ov13855_optical_frame'
SIZE = (4224, 3136)


def camera_frame(message):
    width, height = SIZE
    return ((message.width, message.height) == SIZE and message.encoding == 'mono8'
            and message.header.frame_id == FRAME and message.step >= width
            and len(message.data) == message.step * height
            and message.header.stamp > rospy.Time(0))


def valid_info(message):
    return ((message.width, message.height) == SIZE and message.header.frame_id == FRAME
            and message.K[0] > 0 and message.K[4] > 0 and np.isfinite((*message.K, *message.D)).all())


def wait_until(get, seconds, guard, after=None):
    end = time.monotonic() + seconds
    while not rospy.is_shutdown():
        guard()
        value = get()
        if value is not None and (after is None or
                (value.header.stamp >= after and -.02 <= (rospy.Time.now() - value.header.stamp).to_sec() <= .5)):
            return value
        remaining = end - time.monotonic()
        if remaining <= 0:
            return None
        time.sleep(min(.025, remaining))


class ShelfVision:
    detection_topic = '/ov13855_apriltag_vision/corners'
    detection_type = TagCornerArray
    target_topic = '/drone_inspection/target_tag_pose'

    def __init__(self):
        self.small_info = None
        self.pending = deque(maxlen=10)
        self.lock = threading.Lock()
        self.latest = None
        self.info = None
        self.frames = deque(maxlen=100)
        self.target = None
        self.since = rospy.Time(0)
        self.forwarded = deque(maxlen=150)
        self.pose_pub = rospy.Publisher(self.target_topic, PoseStamped, queue_size=3)
        self.detections = rospy.Subscriber(self.detection_topic, self.detection_type, self.on_detections, queue_size=4)
        self.camera = rospy.Subscriber(CAMERA_INFO, CameraInfo, self.on_info, queue_size=1)
        self.decode = rospy.ServiceProxy(DECODE, DecodeBarcodes)
        self.cancel = rospy.ServiceProxy(CANCEL_DECODE, Trigger)
        self.small_sub = rospy.Subscriber('/ov13855/camera_info', CameraInfo, self.on_small_info, queue_size=1)

    def on_small_info(self, info):
        valid = ((info.width, info.height) == (1280,960) and info.header.frame_id == FRAME
                 and info.K[0] > 0 and info.K[4] > 0 and np.isfinite((*info.K,*info.D)).all()
                 and info.distortion_model in ('plumb_bob','rational_polynomial')
                 and len(info.D) in (0,4,5,8,12,14))
        self.small_info = info if valid else None

    def ready_info(self):
        return self.info if self.small_info is not None else None

    def on_info(self, msg):
        if valid_info(msg):
            self.info = msg

    def on_detections(self, msg):
        if ((msg.width, msg.height) != (1280, 960) or msg.header.frame_id != FRAME or msg.family != 'tag36h11'
                or not -.02 <= (rospy.Time.now() - msg.header.stamp).to_sec() <= .5
                or msg.header.stamp == rospy.Time(0)):
            return
        with self.lock:
            self.latest = msg
            self.frames.append(msg)
            self.on_target_frame(msg)

    def on_target_frame(self, msg):
        # Called with self.lock held. Camera frames often arrive before the bracketing odometry.
        if self.target is not None and msg.header.stamp >= self.since:
            matches = [tag for tag in msg.tags if (msg.family,tag.id) == self.target]
            if len(matches) == 1:
                self.pending.append((self.since, msg.header, matches[0]))


    def window(self, start, end):
        with self.lock:
            return [m for m in self.frames if start <= m.header.stamp <= end]

    def select(self, target):
        with self.lock:
            self.target = target
            self.since = rospy.Time.now()
            self.forwarded.clear()
            self.pending.clear()

    def matched_pose(self, stamp):
        with self.lock:
            poses = list(self.forwarded)
        closest = min(poses, key=lambda p: abs((p.header.stamp - stamp).to_sec()), default=None)
        if closest and abs((closest.header.stamp - stamp).to_sec()) <= .010:
            return closest
        return None

    def take_photo(self, after, guard):
        frame = None

        def on_image(msg):
            nonlocal frame
            if camera_frame(msg):
                frame = msg

        subscription = rospy.Subscriber(CAMERA_IMAGE, Image, on_image, queue_size=1,
                                        buff_size=2 * SIZE[0] * SIZE[1])
        try:
            return wait_until(lambda: frame, 2., guard, after)
        finally:
            subscription.unregister()


def load_calibration():
    packages = rospkg.RosPack()
    tf_path = Path(packages.get_path('front_marker_control')) / 'config/ov13855_camera_tf.yaml'
    reader = Path(packages.get_path('barcode_reader_camera')) / 'config'
    layout_path = reader / 'layouts/qr_top_tag_bottom.yaml'
    tf = yaml.safe_load(tf_path.read_text())
    layout = yaml.safe_load(layout_path.read_text())
    if tf['base_frame'] != 'drone_base' or tf['camera_frame'] != FRAME or layout['label']['tag_position'] != 'custom':
        raise ValueError('shelf camera calibration or label geometry is incompatible')
    return (finite(tf['translation']), rotation(tf['rotation_quaternion']), layout,
            (tf_path, layout_path, reader / 'barcode_reader_camera.yaml'))


def probe(out):
    rospy.init_node('inspection_camera_probe', anonymous=True)
    vision = ShelfVision()
    rospy.wait_for_service(DECODE, timeout=10.)
    rospy.wait_for_service(CANCEL_DECODE, timeout=3.)
    now = rospy.Time.now()
    if (wait_until(lambda: vision.latest, 6., lambda: None, now) is None
            or wait_until(vision.ready_info, 2., lambda: None, now) is None
            or vision.take_photo(now, lambda: None) is None):
        raise Stopped('OV13855 双路图像或标定不可用')
    if shutil.disk_usage(out).free < 1024 ** 3:
        raise Stopped('剩余空间不足以保存相片与扫码记录')
    say('相机就绪：1280×960 AprilTag、4224×3136 QR')
