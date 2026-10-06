#!/usr/bin/env python3
"""ROS integration with a fake control_mode stream and fake reader services; run with rostest node.test."""
import threading
import time
import unittest

from barcode_reader_camera.msg import Barcode
from barcode_reader_camera.srv import DecodeBarcodes, DecodeBarcodesResponse
from barcode_reader_serial.srv import ReadBarcode, ReadBarcodeResponse
from mavros_msgs.msg import State
from ov13855_apriltag_vision.msg import HomogeneousTransform
import rospy
import rostest
from std_msgs.msg import Header, String

from barcode_arrival_trigger.msg import ArrivalReadResult

OFFBOARD = 'OFFBOARD'
IMAGE_STAMP = rospy.Time(123, 0)  # 假相机服务回传的「所解图像」时间戳


class Integration(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.calls = []
        self.camera_delay = 0.0
        self.results = []
        self.publish_state = True
        self.stop = False
        self.state = State(connected=True, armed=True, mode=OFFBOARD)
        self.marker = None  # 假 debug 话题当前会下发的 marker id；None = 还没有消息
        self.camera_srv = rospy.Service('/fake_camera/decode', DecodeBarcodes, self.on_decode)
        self.serial_srv = rospy.Service('/fake_serial/read', ReadBarcode, self.on_read)
        # latch=True 复刻 front_marker_control 的模式话题：订阅者晚到也能拿到当前模式。
        self.mode_pub = rospy.Publisher('/test_arrival/control_mode', String,
                                        queue_size=10, latch=True)
        self.state_pub = rospy.Publisher('/test_arrival/state', State, queue_size=10)
        self.transform_pub = rospy.Publisher('/test_arrival/transform',
                                             HomogeneousTransform, queue_size=10)
        self.result_sub = rospy.Subscriber('/barcode_arrival_trigger/result',
                                           ArrivalReadResult, self.on_result, queue_size=100)
        self.assertTrue(self.wait_for(
            lambda: (self.mode_pub.get_num_connections() > 0
                     and self.state_pub.get_num_connections() > 0
                     and self.transform_pub.get_num_connections() > 0
                     and self.result_sub.get_num_connections() > 0), 20))
        self.state_thread = threading.Thread(target=self.publish_states, daemon=True)
        self.state_thread.start()

    def tearDown(self):
        self.stop = True
        self.state_thread.join(1)
        self.result_sub.unregister()
        self.mode_pub.unregister()
        self.camera_srv.shutdown()
        self.serial_srv.shutdown()

    def publish_states(self):
        while not self.stop and not rospy.is_shutdown():
            if self.publish_state:
                self.state_pub.publish(self.state)
            time.sleep(0.05)

    def on_decode(self, _request):
        with self.lock:
            self.calls.append('camera')
        time.sleep(self.camera_delay)
        return DecodeBarcodesResponse(success=True, partial=False, error='', elapsed_s=0.01,
                                      used_tag_roi=True, report='QR=1/1',
                                      codes=[Barcode(format='QRCode', code='ABC')],
                                      header=Header(stamp=IMAGE_STAMP, frame_id='ov13855'))

    def on_read(self, _request):
        with self.lock:
            self.calls.append('serial')
        return ReadBarcodeResponse(success=True, code='NEXT9017324', error='', elapsed_s=0.01,
                                   request_seq=7, raw_hex='31 32')

    def on_result(self, message):
        with self.lock:
            self.results.append(message)

    @staticmethod
    def wait_for(condition, timeout):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True

    def seqs(self):
        with self.lock:
            return [message.seq for message in self.results]

    def result(self, index):
        with self.lock:
            self.assertGreater(len(self.results), index)
            return self.results[index]

    def last(self):
        return self.result(len(self.seqs()) - 1)

    def switch(self, mode):
        self.mode_pub.publish(String(data=mode))
        rospy.sleep(0.05)

    def enter_hold(self, marker=True):
        """一次「别的模式 -> POS_HOLD」跳变。

        marker=True 时先刷新一帧 debug 消息，复刻真实场景里该话题按相机帧率连续下发；
        marker=False 用于验证消息过期后 marker_id 记 -1。
        """
        if marker and self.marker is not None:
            self.transform_pub.publish(HomogeneousTransform(marker_id=self.marker))
            rospy.sleep(0.05)
        self.switch('ALIGN')
        self.switch('POS_HOLD')

    def expect(self, seqs, timeout=3.0):
        self.wait_for(lambda: self.seqs() == seqs, timeout)
        rospy.sleep(0.3)  # 之后再没有多出来的结果
        self.assertEqual(self.seqs(), seqs)

    def test_arrival_reads(self):
        # 第一条（latch 到的）POS_HOLD 只作基线：启动瞬间已停在定点不补读。
        self.switch('POS_HOLD')
        self.expect([], timeout=1.0)

        # 别的模式 -> POS_HOLD：读一次；此时还没收到 transform，marker_id = -1。
        self.enter_hold()
        self.expect([1])
        first = self.result(0)
        self.assertNotEqual(first.header.stamp, rospy.Time())  # 到位时刻
        self.assertEqual(first.marker_id, -1)
        self.assertTrue(first.camera_called)
        self.assertTrue(first.serial_called)
        self.assertFalse(first.hold_lost_during_read)
        self.assertTrue(first.camera.success)
        self.assertTrue(first.camera.used_tag_roi)
        self.assertEqual(first.camera.report, 'QR=1/1')
        self.assertEqual([code.code for code in first.camera.codes], ['ABC'])
        # camera.header 原样带回所解图像的 header，不是本地请求时刻。
        self.assertEqual(first.camera.header.stamp, IMAGE_STAMP)
        self.assertEqual(first.camera.header.frame_id, 'ov13855')
        self.assertTrue(first.serial.success)
        self.assertEqual(first.serial.code, 'NEXT9017324')
        self.assertEqual(first.serial.request_seq, 7)
        self.assertEqual(self.calls, ['camera', 'serial'])

        # 停在 POS_HOLD 期间重复发布不重读。
        self.switch('POS_HOLD')
        self.switch('POS_HOLD')
        self.expect([1])

        # marker_id 取到位时刻 debug 话题上 0.5 s 内的最新值（这里模拟连续下发）。
        self.marker = 7
        self.enter_hold()
        self.expect([1, 2])
        self.assertEqual(self.result(1).marker_id, 7)

        # 到位那一刻不满足飞行门控：放弃本次到位，不读也不重试。
        # 放弃的到位照样占用序号（本段 #3~#5），只是不发布结果。
        for state in (State(connected=True, armed=True, mode='MANUAL'),
                      State(connected=True, armed=False, mode=OFFBOARD),
                      State(connected=False, armed=True, mode=OFFBOARD)):
            self.switch('ALIGN')
            self.state = state
            rospy.sleep(0.2)
            self.switch('POS_HOLD')  # arrival#3 / #4 / #5：放弃
            self.expect([1, 2])
        self.state = State(connected=True, armed=True, mode=OFFBOARD)
        rospy.sleep(0.2)

        # state 不新鲜（停发超过 state_timeout_s）同样放弃，且放弃后停在
        # POS_HOLD 不补读：恢复后必须退出再进入才读。
        self.publish_state = False
        rospy.sleep(0.8)
        self.enter_hold()  # arrival#6：放弃
        self.expect([1, 2])
        self.publish_state = True
        rospy.sleep(0.2)
        self.expect([1, 2])
        self.enter_hold()  # arrival#7
        self.expect([1, 2, 7])

        # 读码中退出 POS_HOLD：不取消读码，结果里标 hold_lost_during_read。
        self.camera_delay = 1.0
        self.enter_hold()  # arrival#8
        rospy.sleep(0.2)
        self.switch('RETREAT')
        self.expect([1, 2, 7, 8], timeout=4.0)
        self.assertTrue(self.last().hold_lost_during_read)
        self.assertTrue(self.last().camera.success)
        # serial.header 是读码器自己被调用的时刻：相机读了 1 s 之后才开始。
        self.assertGreaterEqual(
            (self.last().serial.header.stamp - self.last().header.stamp).to_sec(), 0.9)
        self.camera_delay = 0.0

        # 忙时忽略：上一次读码未结束时的跳变占号但不读（#10 不发布结果）。
        self.camera_delay = 1.5
        self.enter_hold()  # arrival#9
        rospy.sleep(0.2)
        self.enter_hold()  # arrival#10：被忽略
        self.expect([1, 2, 7, 8, 9], timeout=4.0)
        self.camera_delay = 0.0
        # 上一次读码结束后再进入照常读，序号已跳过被忽略的 #10。
        self.enter_hold()  # arrival#11
        self.expect([1, 2, 7, 8, 9, 11])
        self.assertNotIn(10, self.seqs())

        # 串口服务离线：相机照常读，serial_called 仍为 true，error 记 unavailable。
        self.serial_srv.shutdown()
        self.enter_hold()  # arrival#12
        self.expect([1, 2, 7, 8, 9, 11, 12], timeout=3.0)
        self.assertTrue(self.last().camera.success)
        self.assertTrue(self.last().serial_called)
        self.assertFalse(self.last().serial.success)
        self.assertTrue(self.last().serial.error.startswith('unavailable'))

        # marker 消息超过 marker_timeout_s（0.5 s）没有更新：marker_id 记 -1，
        # 不用上一次到位残留的 id 冒充本次到位。
        rospy.sleep(0.8)
        self.enter_hold(marker=False)  # arrival#13
        self.expect([1, 2, 7, 8, 9, 11, 12, 13], timeout=3.0)
        self.assertEqual(self.last().marker_id, -1)
        self.assertEqual(self.last().seq, 13)


if __name__ == '__main__':
    rospy.init_node('barcode_arrival_trigger_integration')
    rostest.rosrun('barcode_arrival_trigger', 'barcode_arrival_trigger_node', Integration)
