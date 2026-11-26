"""读码服务调用和多行汇总日志（ROS 侧），CH9 触发与到位触发两个节点共用。

runner.py 保持不依赖 ROS；这里是它的 ROS 适配，保证两种触发的调用方式和日志格式一致。
"""
import rospy
from barcode_reader_camera.srv import DecodeBarcodes
from barcode_reader_serial.srv import ReadBarcode

from .runner import format_block, step_fields, summary_is_warning


class ReaderCalls(object):
    """按 run_sequence 需要的 {name: call} 提供两路读码调用；服务不在线时抛异常，由 run_sequence 记录。"""

    def __init__(self, camera_service, serial_service, serial_timeout_s, service_wait_s):
        self.camera_service = camera_service
        self.serial_service = serial_service
        self.serial_timeout_s = serial_timeout_s
        self.service_wait_s = service_wait_s
        self.camera_proxy = rospy.ServiceProxy(camera_service, DecodeBarcodes)
        self.serial_proxy = rospy.ServiceProxy(serial_service, ReadBarcode)
        self.calls = {'camera': self.camera, 'serial': self.serial}

    def camera(self):
        rospy.wait_for_service(self.camera_service, self.service_wait_s)
        return self.camera_proxy()

    def serial(self):
        rospy.wait_for_service(self.serial_service, self.service_wait_s)
        return self.serial_proxy(self.serial_timeout_s)


def log_summary(title, results, sections=()):
    """一块 YAML 风格汇总：sections（如 tag 段）在前，各路读码结果在后。

    读到/读不到都打 INFO；只有服务不在线打 WARN（见 runner.summary_is_warning）。
    """
    text = format_block(title, list(sections) + [(r.name, step_fields(r)) for r in results])
    if summary_is_warning(results):
        rospy.logwarn(text)
    else:
        rospy.loginfo(text)
