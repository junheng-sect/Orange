#!/usr/bin/env python3
"""ROS integration using localhost only; run with rostest tcp_node.test."""
import math
import threading
import time
import unittest

import rosnode
import rospy
import rostest
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from tether_winch_control.msg import WinchStatus
from test_transport import Server, wait_for


class Integration(unittest.TestCase):
    def test_tcp_node(self):
        server = Server(18080)
        stop = threading.Event()
        self.status = None
        self.speed = 0.0
        self.available = False
        self.frame = 'drone_odom'
        publisher = rospy.Publisher('/fastlio/odom_world', Odometry, queue_size=1)
        subscriber = rospy.Subscriber('/tether_winch_control/status', WinchStatus,
                                      lambda message: setattr(self, 'status', message))

        def publish():
            radius = 10.0
            while not stop.wait(0.02):
                radius += self.speed * 0.02
                if not self.available:
                    continue
                message = Odometry()
                message.header.stamp = rospy.Time.now()
                message.header.frame_id = self.frame
                message.child_frame_id = 'drone_base'
                message.pose.pose.position.x = radius
                message.pose.pose.orientation.w = 1.0
                publisher.publish(message)

        thread = threading.Thread(target=publish, daemon=True)
        thread.start()
        try:
            rospy.wait_for_service('/tether_winch_control/set_enabled', timeout=10)
            enable = rospy.ServiceProxy('/tether_winch_control/set_enabled', SetBool)
            wait_for(lambda: self.status is not None and bool(server.messages), 10)
            self.assertFalse(self.status.enabled)
            self.assertEqual(set(server.messages), {b'DUTY=100#'})
            self.available = True
            wait_for(lambda: self.status.input_valid and not self.status.communication_error, 10)
            self.assertEqual(rospy.get_param('/tether_winch_control/controller/low_duty'), 10)
            self.assertEqual(rospy.get_param('/tether_winch_control/controller/high_duty'), 220)
            self.assertEqual(rospy.get_param('/tether_winch_control/odom_topic'), '/fastlio/odom_world')
            self.assertFalse(self.status.hardware_feedback_valid)
            self.assertTrue(math.isnan(self.status.applied_duty))
            wait_for(lambda: b'DUTY=100#' in server.messages)
            if rospy.get_param('/tether_winch_control/auto_enable'):
                wait_for(lambda: self.status.enabled)  # No enable service call on startup.
            else:
                self.assertFalse(self.status.enabled)
                self.assertTrue(enable(True).success)
            self.speed = -0.3
            wait_for(lambda: self.status.mode == 'HIGH')
            wait_for(lambda: b'DUTY=220#' in server.messages)
            self.assertEqual(self.status.target_duty, 220)

            self.available = False
            wait_for(lambda: self.status.mode == 'FAULT')
            wait_for(lambda: server.messages[-1] == b'DUTY=100#')
            self.available = True
            self.speed = 0.0
            wait_for(lambda: self.status.input_valid)
            time.sleep(0.25)  # Several watchdog cycles must not automatically re-arm.
            self.assertEqual(self.status.mode, 'FAULT')
            self.assertTrue(enable(True).success)
            self.speed = -0.3
            wait_for(lambda: self.status.mode == 'HIGH')
            self.assertTrue(enable(False).success)
            wait_for(lambda: self.status.mode == 'DISABLED')
            wait_for(lambda: server.messages[-1] == b'DUTY=100#')
            time.sleep(0.25)
            self.assertEqual(self.status.mode, 'DISABLED')

            self.assertTrue(enable(True).success)
            wait_for(lambda: self.status.mode == 'HIGH')
            server.close()
            wait_for(lambda: self.status.mode == 'FAULT' and bool(self.status.communication_error))
            self.assertFalse(enable(True).success)
            server = Server(18080)
            wait_for(lambda: len(server.messages) >= 2)
            self.assertEqual(set(server.messages), {b'DUTY=100#'})
            wait_for(lambda: not self.status.communication_error and self.status.input_valid)
            self.assertEqual(self.status.mode, 'FAULT')
            self.assertTrue(enable(True).success)
            wait_for(lambda: self.status.mode == 'HIGH')

            self.frame = 'wrong'
            wait_for(lambda: self.status.mode == 'FAULT')
            wait_for(lambda: server.messages[-1] == b'DUTY=100#')
            self.frame = 'drone_odom'
            wait_for(lambda: self.status.input_valid)
            self.assertTrue(enable(True).success)
            wait_for(lambda: self.status.mode == 'HIGH')
            wait_for(lambda: server.messages[-1] == b'DUTY=220#')
            wait_for(lambda: self.status.last_sent_duty == 220)
            self.assertGreater(self.status.last_send_time.to_sec(), 0)
            rosnode.kill_nodes(['/tether_winch_control'])
            wait_for(lambda: server.messages[-1] == b'DUTY=100#')
        finally:
            stop.set()
            thread.join(1)
            subscriber.unregister()
            publisher.unregister()
            server.close()


if __name__ == '__main__':
    rospy.init_node('winch_integration_test')
    rostest.rosrun('tether_winch_control', 'tcp_node', Integration)
