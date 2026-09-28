#!/usr/bin/env python3
"""Exercise the new vision output through the existing body-frame controller input."""
import time
import unittest
import cv2
import numpy as np
import rospy
import rostest
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import Vector3Stamped,PoseStamped
from mavros_msgs.msg import PositionTarget

class InputTest(unittest.TestCase):
    def test_new_camera_frame_to_body_errors(self):
        errors=[]; poses=[]; actual=[]
        subs=[rospy.Subscriber('/front_marker_control/position_error',Vector3Stamped,errors.append),
              rospy.Subscriber('/front_marker/pose',PoseStamped,poses.append),
              rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,actual.append)]
        image_pub=rospy.Publisher('/ov13855/image_raw',Image,queue_size=1)
        info_pub=rospy.Publisher('/ov13855/camera_info',CameraInfo,queue_size=1,latch=True)
        end=time.monotonic()+8
        while time.monotonic()<end and not(image_pub.get_num_connections() and info_pub.get_num_connections()):time.sleep(.02)
        self.assertTrue(image_pub.get_num_connections())
        info=CameraInfo();info.header.frame_id='ov13855_optical_frame';info.width=info.height=800
        info.K=[1000,0,400,0,1000,400,0,0,1];info.D=[0.0]*5;info.distortion_model='plumb_bob'
        info_pub.publish(info);time.sleep(.3)
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        for x_shift,y_shift in [(0,0),(80,50),(-80,-50)]:
            scene=np.full((800,800),255,np.uint8)
            scene[240+y_shift:560+y_shift,240+x_shift:560+x_shift]=cv2.aruco.drawMarker(dictionary,7,320)
            start=len(errors)
            for _ in range(8):
                image=Image();image.header.frame_id=info.header.frame_id;image.header.stamp=rospy.Time.now()
                image.width=image.height=800;image.step=800;image.encoding='mono8';image.data=scene.tobytes()
                image_pub.publish(image);time.sleep(.10)
            self.assertGreater(len(errors),start)
            error=errors[-1];self.assertEqual(error.header.frame_id,'base_link')
            pose=next(p for p in reversed(poses) if p.header.stamp==error.header.stamp)
            # Existing extrinsic: x_body=.18+z_cam, y_body=-x_cam, z_body=-y_cam.
            self.assertAlmostEqual(error.vector.x,.18+pose.pose.position.z-.8,places=6)
            self.assertAlmostEqual(error.vector.y,-pose.pose.position.x,places=6)
            self.assertAlmostEqual(error.vector.z,-pose.pose.position.y,places=6)
            if x_shift: self.assertLess(error.vector.y*x_shift,0)
            if y_shift: self.assertLess(error.vector.z*y_shift,0)
        self.assertEqual(actual,[], 'disabled control must not publish FCU setpoints')
        before=len(errors)
        for _ in range(4):
            image.header.stamp=rospy.Time.now();image.data=np.full((800,800),255,np.uint8).tobytes()
            image_pub.publish(image);time.sleep(.1)
        self.assertEqual(len(errors),before)
if __name__=='__main__':
    rospy.init_node('ov13855_control_input_test')
    rostest.rosrun('front_marker_control','ov13855_control_input',InputTest)
