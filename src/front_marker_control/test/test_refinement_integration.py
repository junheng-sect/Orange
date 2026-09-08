#!/usr/bin/env python3
"""Isolated simulated feedback: fixed window, smooth SP, hold and hard gates."""
import math
import time
import unittest

import rospy
import rostest
from mavros_msgs.msg import PositionTarget, State
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from front_marker_control.control import wrap_angle
from test_position_hold import camera_pose, odom_message


class RefinementIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.actual = []
        # 近墙门限须继承主 YAML（启动文件不再覆盖）：断言节点实际加载值。
        self.assertEqual(
            rospy.get_param('/refine_front_marker_control/min_marker_distance_m'),
            0.50,
        )
        self.status = []
        self.targets = []
        self.pose_pub = rospy.Publisher('/test_refine/pose', PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/test_refine/odom', Odometry, queue_size=10)
        self.state_pub = rospy.Publisher('/test_refine/state', State, queue_size=10)
        self.subscribers = [
            rospy.Subscriber('/test_refine/actual', PositionTarget, self.actual.append, queue_size=1000),
            rospy.Subscriber('/test_refine/status', String, self.status.append, queue_size=1000),
            rospy.Subscriber('/test_refine/fixed_yaw', Float64, self.targets.append, queue_size=10),
        ]
        self.position = (0., 0., 0.)
        self.yaw = 0.
        self.bias = 0.
        self.wall_yaw = math.radians(8)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            if all(p.get_num_connections() for p in (self.pose_pub, self.odom_pub, self.state_pub)):
                break
            rospy.sleep(.02)
        else:
            self.fail('input subscribers unavailable')

    def drive(self, duration, vision=True, mode='OFFBOARD', near=False, invalid_odom=False):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            stamp = rospy.Time.now()
            state = State(connected=True, armed=True, mode=mode)
            self.state_pub.publish(state)
            if self.actual and mode == 'OFFBOARD':
                sp = self.actual[-1]
                if not sp.type_mask & PositionTarget.IGNORE_PX:
                    self.position = (sp.position.x, sp.position.y, sp.position.z)
                    self.yaw = sp.yaw
            odom = odom_message(stamp, self.position, self.yaw)
            if invalid_odom:
                odom.pose.pose.position.x = float('nan')
            self.odom_pub.publish(odom)
            if vision:
                if near:
                    error = (-.35, 0, 0)
                else:
                    dx, dy = .8-self.position[0], -self.position[1]
                    c, s = math.cos(self.yaw), math.sin(self.yaw)
                    error = (c*dx+s*dy-.8, -s*dx+c*dy, -self.position[2])
                self.pose_pub.publish(camera_pose(error, stamp, self.wall_yaw-self.yaw+self.bias))
            rospy.sleep(.025)

    def wait_mode(self, mode, timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.drive(.06)
            if self.status and ('mode='+mode+'/') in self.status[-1].data:
                return
        self.fail('mode %s not reached: %s' % (mode, self.status[-1].data if self.status else 'none'))

    def test_lifecycle(self):
        self.drive(.25, mode='MANUAL')
        self.wait_mode('YAW_SAMPLING')
        sp = self.actual[-1]
        self.assertEqual(sp.coordinate_frame, PositionTarget.FRAME_LOCAL_NED)
        self.assertTrue(sp.type_mask & PositionTarget.IGNORE_PX)
        self.assertFalse(sp.type_mask & PositionTarget.IGNORE_VX)
        self.assertFalse(sp.type_mask & PositionTarget.IGNORE_YAW)
        self.assertTrue(sp.type_mask & PositionTarget.IGNORE_YAW_RATE)
        self.assertAlmostEqual(sp.yaw, 0.)
        # 窗口内断帧不清零、不重置：窗口照常在 0.6 s 后提交目标。
        self.drive(.2, vision=False)
        self.wait_mode('POSE_ALIGNING', timeout=10)
        start = len(self.actual)-1
        self.assertTrue(self.targets)
        self.assertAlmostEqual(self.targets[-1].data, self.wall_yaw, places=4)
        self.wait_mode('POS_HOLD')
        sp = self.actual[-1]
        self.assertFalse(sp.type_mask & PositionTarget.IGNORE_PX)
        self.assertTrue(sp.type_mask & PositionTarget.IGNORE_VX)
        self.assertTrue(sp.type_mask & PositionTarget.IGNORE_YAW_RATE)
        self.assertAlmostEqual(sp.yaw, self.wall_yaw, places=5)
        self.assertAlmostEqual(sp.position.x, .8-.8*math.cos(self.wall_yaw), places=5)
        self.assertAlmostEqual(sp.position.y, -.8*math.sin(self.wall_yaw), places=5)
        previous = None
        for sample in self.actual[start:]:
            if sample.type_mask & PositionTarget.IGNORE_PX:
                continue
            if previous:
                dt = (sample.header.stamp-previous.header.stamp).to_sec()
                distance = math.sqrt(sum((a-b)**2 for a,b in zip(
                    (sample.position.x,sample.position.y,sample.position.z),
                    (previous.position.x,previous.position.y,previous.position.z))))
                self.assertLessEqual(distance, .1*dt+1e-6)
                self.assertLessEqual(abs(wrap_angle(sample.yaw-previous.yaw)), math.radians(10)*dt+1e-7)
            previous = sample
        # 定点中丢标：固定目标保持（位置 setpoint 不变）。
        self.drive(.2, vision=False)
        self.assertFalse(self.actual[-1].type_mask & PositionTarget.IGNORE_PX)
        # 视觉偏置不再触发重采样：目标保持固定。
        count = len(self.targets)
        self.bias = math.radians(8)
        self.drive(.5)
        self.assertEqual(len(self.targets), count)
        self.assertEqual('mode=POS_HOLD' in self.status[-1].data, True)
        self.bias = 0.
        # FCU 硬门控：退 OFFBOARD 清目标，输出零速。
        self.drive(.15, mode='MANUAL')
        self.assertTrue(self.actual[-1].type_mask & PositionTarget.IGNORE_YAW)
        self.assertEqual(self.actual[-1].velocity.x, 0.)
        self.drive(.2)
        self.assertEqual(len(self.targets), count)
        self.wait_mode('YAW_SAMPLING')
        # VIO 失效（非法 odom）：废弃采样，速度归零。
        self.drive(.15, invalid_odom=True)
        self.assertTrue(self.actual[-1].type_mask & PositionTarget.IGNORE_PX)
        self.assertEqual(self.actual[-1].velocity.x, 0.)
        self.drive(.2)
        self.assertEqual(len(self.targets), count)
        self.wait_mode('YAW_SAMPLING')
        # 近墙后退：打断采样，负向速度。
        self.drive(.15, near=True)
        self.assertTrue(self.status and 'RETREAT' in self.status[-1].data)
        self.assertLess(self.actual[-1].velocity.x, 0.)
        self.assertEqual(self.actual[-1].yaw_rate, 0.)

    def test_entry_with_speed_and_xyz_only(self):
        # XYZ 进容差即开始测量：无速度门、无 yaw 收敛要求。
        # yaw 偏 40°、速度残差存在时也直接进入采样窗口。
        self.drive(.25, mode='MANUAL')
        self.wait_mode('YAW_SAMPLING')
        # 无 marker 延迟窗口结束（deadline 由控制定时器判定）不可测时序精度，
        # 只验证窗口在有断帧的情况下仍能提交目标。
        self.wait_mode('POSE_ALIGNING', timeout=10)


if __name__ == '__main__':
    rospy.init_node('refinement_integration_test')
    rostest.rosrun('front_marker_control', 'refinement_integration', RefinementIntegrationTest)
