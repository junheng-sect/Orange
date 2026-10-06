"""ROS adapter: front_marker_control 进入 POS_HOLD -> 一次到位读码。

只做一件事：到位跳变且通过飞行门控时，按 mode 调用相机读码和/或读码器各一次，
发布 ~result 并打一行汇总日志。到位判定、失败重试、兜底策略都在本节点之外。
"""
import threading
import time

from barcode_reader_camera.msg import BarcodeArray
from barcode_reader_serial.msg import BarcodeResult
from barcode_rc_trigger.readers import ReaderCalls, log_summary
from barcode_rc_trigger.runner import parse_mode, run_sequence
from mavros_msgs.msg import State
from ov13855_apriltag_vision.msg import HomogeneousTransform
import rospy
from std_msgs.msg import String

from barcode_arrival_trigger.msg import ArrivalReadResult
from .policy import ArrivalHoldTrigger, gate_reason, marker_id_within


class BarcodeArrivalTriggerNode(object):
    def __init__(self):
        self.control_mode_topic = str(rospy.get_param(
            '~control_mode_topic', '/front_marker_control/control_mode'))
        self.state_topic = str(rospy.get_param('~state_topic', '/mavros/state'))
        self.transform_topic = str(rospy.get_param(
            '~transform_topic', '/ov13855_apriltag_vision/debug/homogeneous_transform'))
        self.state_timeout_s = float(rospy.get_param('~state_timeout_s', 2.0))
        self.marker_timeout_s = float(rospy.get_param('~marker_timeout_s', 0.5))
        self.steps = parse_mode(rospy.get_param('~mode', 'both'))
        self.camera_service = rospy.get_param(
            '~camera_service', '/barcode_reader_camera/decode')
        self.serial_service = rospy.get_param(
            '~serial_service', '/barcode_reader_serial/read')
        self.readers = ReaderCalls(self.camera_service, self.serial_service,
                                   float(rospy.get_param('~serial_timeout_s', 0.0)),
                                   float(rospy.get_param('~service_wait_s', 0.5)))

        self.lock = threading.Lock()
        self.trigger = ArrivalHoldTrigger()
        self.state = State()
        self.last_state_time = None
        self.marker_id = -1
        self.last_transform_time = None
        self.started = time.monotonic()

        self.result_pub = rospy.Publisher('~result', ArrivalReadResult, queue_size=10)
        rospy.Subscriber(
            self.control_mode_topic, String, self.on_control_mode, queue_size=10)
        rospy.Subscriber(self.state_topic, State, self.on_state, queue_size=10)
        rospy.Subscriber(
            self.transform_topic, HomogeneousTransform, self.on_transform, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.on_timer)
        rospy.loginfo(
            'barcode_arrival_trigger: %s -> POS_HOLD | mode=%s camera=%s serial=%s | '
            'state=%s timeout=%.1fs | marker=%s (%.1fs)',
            self.control_mode_topic, '+'.join(self.steps), self.camera_service,
            self.serial_service, self.state_topic, self.state_timeout_s,
            self.transform_topic, self.marker_timeout_s)

    def on_state(self, message):
        with self.lock:
            self.state = message
            self.last_state_time = time.monotonic()

    def on_transform(self, message):
        with self.lock:
            self.marker_id = int(message.marker_id)
            self.last_transform_time = time.monotonic()

    def on_control_mode(self, message):
        mode = str(message.data).strip()
        now = time.monotonic()
        with self.lock:
            decision = self.trigger.update(mode)
            if decision.ignored_seq is not None:
                rospy.logwarn('arrival#%d ignored: previous read still running',
                              decision.ignored_seq)
                return
            if decision.fire_seq is None:
                return
            seq = decision.fire_seq
            stamp = rospy.Time.now()
            marker_id = marker_id_within(
                self.marker_id,
                None if self.last_transform_time is None
                else now - self.last_transform_time,
                self.marker_timeout_s)
            age = None if self.last_state_time is None else now - self.last_state_time
            reason = gate_reason(self.state.connected, self.state.armed,
                                 self.state.mode, age, self.state_timeout_s)
            if reason:
                # 到位那一刻不满足飞行门控：放弃本次到位，不读也不重试。
                self.trigger.finish()
                rospy.logwarn('arrival#%d 放弃：%s', seq, reason)
                return
            rospy.loginfo('arrival#%d marker=%d -> %s', seq, marker_id,
                          '+'.join(self.steps))
            threading.Thread(target=self._run, args=(seq, stamp, marker_id),
                             name='barcode-arrival-read-%d' % seq, daemon=True).start()

    def on_timer(self, _event):
        with self.lock:
            waiting = self.trigger.mode is None
        if waiting and time.monotonic() - self.started > 5.0:
            rospy.logwarn_throttle(
                10.0, 'control_mode 话题 %s 还没收到任何模式（front_marker_control 在跑吗？）',
                self.control_mode_topic)

    def _run(self, seq, stamp, marker_id):
        try:
            # clock：每一路记下自己发起调用的时刻（both 时读码器要等相机读完才开始）。
            results = run_sequence(self.steps, self.readers.calls, clock=rospy.Time.now)
        finally:
            # run_sequence 已逐步兜错；这里只保证忙标记一定释放。
            with self.lock:
                hold_lost = self.trigger.finish()
        try:
            log_summary('arrival#%d marker=%d' % (seq, marker_id), results)
            message = self.build_result(stamp, seq, marker_id, results, hold_lost)
            if not rospy.is_shutdown():
                self.result_pub.publish(message)
        except Exception as error:
            # 关停时话题已关闭属于正常竞态；其余情况记一行带序号的错误，便于现场定位。
            if not rospy.is_shutdown():
                rospy.logerr('arrival#%d 结果发布失败: %s', seq, error)

    def build_result(self, stamp, seq, marker_id, results, hold_lost):
        message = ArrivalReadResult()
        message.header.stamp = stamp
        message.seq = seq
        message.marker_id = marker_id
        message.hold_lost_during_read = hold_lost
        for result in results:
            if result.name == 'camera':
                message.camera_called = True
                message.camera = self.camera_result(result)
            else:
                message.serial_called = True
                message.serial = self.serial_result(result)
        return message

    @staticmethod
    def camera_result(result):
        """把 ~decode 的响应映射成 BarcodeArray；header 用所解图像的 header（与相机节点
        ~result 一致），服务不在线时只填 error。"""
        message = BarcodeArray()
        if result.response is None:
            message.error = 'unavailable (%s)' % result.error
            return message
        response = result.response
        message.header = response.header
        message.success = response.success
        message.partial = response.partial
        message.error = response.error
        message.elapsed_s = response.elapsed_s
        message.used_tag_roi = response.used_tag_roi
        message.report = response.report
        message.codes = response.codes
        return message

    @staticmethod
    def serial_result(result):
        """把 ~read 的响应映射成 BarcodeResult；header 为本地发起读码的时刻，服务不在线时只填 error。"""
        message = BarcodeResult()
        if result.started is not None:
            message.header.stamp = result.started
        if result.response is None:
            message.error = 'unavailable (%s)' % result.error
            return message
        response = result.response
        message.request_seq = response.request_seq
        message.success = response.success
        message.code = response.code
        message.error = response.error
        message.elapsed_s = response.elapsed_s
        message.raw_hex = response.raw_hex
        return message


def main():
    rospy.init_node('barcode_arrival_trigger')
    BarcodeArrivalTriggerNode()
    rospy.spin()
