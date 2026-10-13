"""ROS adapter: front_marker_control 进入 POS_HOLD -> 一次到位读码。

只做一件事：到位跳变且通过飞行门控时，按 mode 调用相机读码和/或读码器各一次，
发布 ~result、打一块汇总日志（含 tag 位置），并往 arrival_read.csv 追加一行（和相机读码节点
这次启动的图片、attempts.csv 在同一个文件夹）。到位判定、失败重试、兜底策略都在本节点之外。
"""
import os
import threading
import time

from barcode_reader_camera.msg import BarcodeArray
from barcode_reader_serial.msg import BarcodeResult
from barcode_rc_trigger.readers import ReaderCalls, log_summary
from barcode_rc_trigger.record import ARRIVAL_HEADER, CsvRecorder, csv_path, csv_row
from barcode_rc_trigger.runner import parse_mode, run_sequence
from barcode_rc_trigger.tag_pose import TagHistory, pick_tag, sample_from_matrix, tag_fields
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
        self.mode = str(rospy.get_param('~mode', 'camera'))
        self.steps = parse_mode(self.mode)
        self.camera_service = rospy.get_param(
            '~camera_service', '/barcode_reader_camera/decode')
        self.serial_service = rospy.get_param(
            '~serial_service', '/barcode_reader_serial/read')
        self.readers = ReaderCalls(self.camera_service, self.serial_service,
                                   float(rospy.get_param('~serial_timeout_s', 0.0)),
                                   float(rospy.get_param('~service_wait_s', 0.5)))
        self.csv_dir = os.path.expanduser(str(rospy.get_param('~csv_dir', '~/test_data/barcode')))

        self.lock = threading.Lock()
        self.trigger = ArrivalHoldTrigger()
        self.state = State()
        self.last_state_time = None
        self.marker_id = -1
        self.last_transform_time = None
        self.tags = TagHistory()
        self.started = time.monotonic()
        self.start_time = time.time()
        # Opened with the first read, in the camera node's session directory (record.csv_path).
        self.recorder = None
        rospy.on_shutdown(self._close_recorder)

        self.result_pub = rospy.Publisher('~result', ArrivalReadResult, queue_size=10)
        rospy.Subscriber(
            self.control_mode_topic, String, self.on_control_mode, queue_size=10)
        rospy.Subscriber(self.state_topic, State, self.on_state, queue_size=10)
        rospy.Subscriber(
            self.transform_topic, HomogeneousTransform, self.on_transform, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.on_timer)
        # Only the services this mode calls, so camera mode prints nothing about the serial reader.
        services = {'camera': self.camera_service, 'serial': self.serial_service}
        rospy.loginfo(
            'barcode_arrival_trigger: %s -> POS_HOLD | mode=%s %s | '
            'state=%s timeout=%.1fs | marker=%s (%.1fs)',
            self.control_mode_topic, '+'.join(self.steps),
            ' '.join('%s=%s' % (name, services[name]) for name in self.steps),
            self.state_topic, self.state_timeout_s, self.transform_topic, self.marker_timeout_s)
        rospy.loginfo('csv: %s', 'arrival_read.csv in the camera session dir (else %s/<start>_%s/)'
                      % (self.csv_dir, self.mode) if self.csv_dir else 'off')

    def _close_recorder(self):
        if self.recorder is not None:
            self.recorder.close()

    def on_state(self, message):
        with self.lock:
            self.state = message
            self.last_state_time = time.monotonic()

    def on_transform(self, message):
        now = time.monotonic()
        sample = sample_from_matrix(message.header.stamp.to_sec(), now, message.marker_id,
                                    message.matrix)
        with self.lock:
            self.marker_id = int(message.marker_id)
            self.last_transform_time = now
            self.tags.add(sample)

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
            arrival_tag = self.tags.latest(now, self.marker_timeout_s)
            threading.Thread(target=self._run, args=(seq, stamp, marker_id, arrival_tag),
                             name='barcode-arrival-read-%d' % seq, daemon=True).start()

    def on_timer(self, _event):
        with self.lock:
            waiting = self.trigger.mode is None
        if waiting and time.monotonic() - self.started > 5.0:
            rospy.logwarn_throttle(
                10.0, 'control_mode 话题 %s 还没收到任何模式（front_marker_control 在跑吗？）',
                self.control_mode_topic)

    def _run(self, seq, stamp, marker_id, arrival_tag):
        try:
            # clock：每一路记下自己发起调用的时刻（both 时读码器要等相机读完才开始）。
            results = run_sequence(self.steps, self.readers.calls, clock=rospy.Time.now)
        finally:
            # run_sequence 已逐步兜错；这里只保证忙标记一定释放。
            with self.lock:
                hold_lost = self.trigger.finish()
        try:
            image_stamp = 0.0
            session_dir = ''
            for result in results:
                if result.name == 'camera' and result.response is not None:
                    image_stamp = result.response.header.stamp.to_sec()
                    session_dir = result.response.session_dir
            with self.lock:
                tag, source = pick_tag(self.tags, image_stamp, arrival_tag)
            log_summary('arrival#%d marker=%d' % (seq, marker_id), results,
                        [('tag', tag_fields(tag, source))])
            message = self.build_result(stamp, seq, marker_id, results, hold_lost)
            if not rospy.is_shutdown():
                self.result_pub.publish(message)
            # Reads run one at a time (trigger busy), so only this thread opens the recorder.
            if self.recorder is None and self.csv_dir:
                self.recorder = CsvRecorder(csv_path(session_dir, self.csv_dir, self.start_time,
                                                     self.mode, 'arrival_read.csv'), ARRIVAL_HEADER)
                rospy.loginfo('csv: %s', self.recorder.path)
            if self.recorder is not None:
                self.recorder.write(csv_row(time.time(), seq, tag, source, results,
                                            {'hold_lost': int(hold_lost)}, ARRIVAL_HEADER))
        except Exception as error:
            # 关停时话题已关闭属于正常竞态；其余情况记一行带序号的错误，便于现场定位。
            if not rospy.is_shutdown():
                rospy.logerr('arrival#%d 结果发布/记录失败: %s', seq, error)

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
