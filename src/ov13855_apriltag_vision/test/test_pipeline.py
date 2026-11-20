#!/usr/bin/env python3
"""Synthetic AprilTag ROS acceptance: ID switch, invalid input, stamped Pose/TF."""
import time
import unittest
import cv2
import numpy as np
import rospy
import rostest
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage
from ov13855_apriltag_vision.msg import HomogeneousTransform, TagObservationArray, TagSurveyFrame, TagCornerArray

class PipelineTest(unittest.TestCase):
    def setUp(self):
        self.poses=[]; self.matrices=[]; self.transforms=[]; self.status=[]; self.processed=0
        self.image_pub=rospy.Publisher('/ov13855_test/image',Image,queue_size=1)
        self.info_pub=rospy.Publisher('/ov13855_test/info',CameraInfo,queue_size=1,latch=True)
        self.subs=[rospy.Subscriber('/ov13855_test/pose',PoseStamped,self.poses.append),
          rospy.Subscriber('/ov13855_test/matrix',HomogeneousTransform,self.matrices.append),
          rospy.Subscriber('/ov13855_test/status',String,lambda m:self.status.append(m.data)),
          rospy.Subscriber('/ov13855_test/time',Float64,self.tick),
          rospy.Subscriber('/tf',TFMessage,lambda m:self.transforms.extend(t for t in m.transforms if t.child_frame_id=='ov13855_test_marker'))]
        end=time.monotonic()+5
        while time.monotonic()<end and not(self.image_pub.get_num_connections() and self.info_pub.get_num_connections()):time.sleep(.02)
        self.assertTrue(self.image_pub.get_num_connections())
    def tick(self,_):self.processed+=1
    def info(self,invalid=False):
        m=CameraInfo();m.header.frame_id='ov13855_optical_frame';m.width=800;m.height=800
        m.distortion_model='plumb_bob';m.D=[0.0]*5
        m.K=[0 if invalid else 1000,0,400,0,1000,400,0,0,1]
        self.info_pub.publish(m);time.sleep(.1)
    def scene(self,id=7):
        image=np.full((800,800),255,np.uint8)
        if id is not None:image[240:560,240:560]=cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),id,320)
        return image
    def send(self,image,frame='ov13855_optical_frame'):
        before=self.processed
        m=Image();m.header.stamp=rospy.Time.now();m.header.frame_id=frame
        m.height,m.width=image.shape;m.encoding='mono8';m.step=m.width;m.data=image.tobytes()
        self.image_pub.publish(m);end=time.monotonic()+3
        while self.processed==before and time.monotonic()<end:time.sleep(.01)
        self.assertGreater(self.processed,before,'frame was not processed')
        time.sleep(.03)
        return m.header.stamp
    def counts(self):return len(self.poses),len(self.matrices),len(self.transforms)
    def assert_first_frame_published(self,id):
        before=self.counts()
        stamp=self.send(self.scene(id))
        self.assertTrue(all(a>b for a,b in zip(self.counts(),before)),str(self.status[-3:]))
        self.assertEqual(self.poses[-1].header.stamp,stamp)
        self.assertEqual(self.matrices[-1].header.stamp,stamp)
        self.assertEqual(self.transforms[-1].header.stamp,stamp)
        self.assertEqual(self.matrices[-1].marker_id,id)
        self.assertEqual(self.poses[-1].header.frame_id,'ov13855_optical_frame')
        self.assertTrue(any('pipeline=SOLVEPNP_ITERATIVE' in s for s in self.status))
        self.assertFalse(any('ref_state=' in s or 'REJECTED_REPROJECTION' in s for s in self.status))
        p=self.poses[-1].pose; t=self.transforms[-1].transform;matrix=self.matrices[-1].matrix
        self.assertAlmostEqual(p.position.z,.0375,delta=.0005)
        for a,b,c in zip([p.position.x,p.position.y,p.position.z], [t.translation.x,t.translation.y,t.translation.z], [matrix[3],matrix[7],matrix[11]]):
            self.assertAlmostEqual(a,b);self.assertAlmostEqual(a,c)
        for key in ['x','y','z','w']:self.assertAlmostEqual(getattr(p.orientation,key),getattr(t.rotation,key))
    def test_pipeline(self):
        before=self.counts();self.send(self.scene());self.assertEqual(self.counts(),before) # missing CameraInfo
        self.info();self.assert_first_frame_published(7);self.assert_first_frame_published(9) # ID change publishes immediately
        before=self.counts()
        self.send(self.scene(None));self.assertEqual(self.counts(),before)
        self.send(self.scene(9),'wrong_frame');self.assertEqual(self.counts(),before)
        self.send(np.full((600,800),255,np.uint8));self.assertEqual(self.counts(),before)
        self.info(invalid=True);self.send(self.scene(9));self.assertEqual(self.counts(),before)
        self.info();self.assert_first_frame_published(9)
        # Reacquisition publishes on its first valid frame, even after a long gap.
        time.sleep(2.1);self.assert_first_frame_published(9)
    def test_survey_id_with_and_without_pose(self):
        frames = []
        sub = rospy.Subscriber('/ov13855_test_vision/survey', TagSurveyFrame, frames.append)
        self.info(invalid=True)
        for valid in (False, True):
            if valid:
                self.info()
            stamp = self.send(self.scene(4))
            end = time.monotonic() + 2
            while not any(f.header.stamp == stamp for f in frames) and time.monotonic() < end:
                time.sleep(.02)
            frame, = [f for f in frames if f.header.stamp == stamp]
            self.assertEqual(list(frame.ids), [4])
            self.assertEqual([t.id for t in frame.tags], [4] if valid else [])
        sub.unregister()

    def test_tag_array_multiple_tags(self):
        arrays = []
        corners = []
        geometry = rospy.Subscriber('/ov13855/monitor/corners', TagCornerArray, corners.append)
        self.addCleanup(geometry.unregister)
        deadline = time.monotonic() + 3
        while not geometry.get_num_connections() and time.monotonic() < deadline:
            time.sleep(.02)
        sub = rospy.Subscriber('/ov13855_test_vision/detections', TagObservationArray, arrays.append)
        end = time.monotonic() + 5
        while not sub.get_num_connections() and time.monotonic() < end:
            time.sleep(.02)
        self.assertTrue(sub.get_num_connections())
        self.info()
        image = self.scene(None)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        image[300:500, 100:300] = cv2.aruco.drawMarker(dictionary, 7, 200)
        image[320:480, 520:680] = cv2.aruco.drawMarker(dictionary, 9, 160)
        stamp = self.send(image)
        frame, = [a for a in arrays if a.header.stamp == stamp]
        self.assertEqual(frame.family, 'tag36h11')
        self.assertEqual(frame.header.frame_id, 'ov13855_optical_frame')
        self.assertEqual({t.id for t in frame.tags}, {7, 9})
        monitor, = [m for m in corners if m.header.stamp == stamp]
        self.assertEqual((monitor.width, monitor.height), (800, 800))
        self.assertEqual({t.id for t in monitor.tags}, {7, 9})
        self.assertTrue(all(len(t.corners) == 4 for t in monitor.tags))
        self.assertEqual(self.poses[-1].header.stamp, stamp)
        self.assertTrue(all(t.pose.position.z > 0 and t.reprojection_rms_px < 2 for t in frame.tags))
        self.assertEqual(self.matrices[-1].marker_id, 7)
        stamp = self.send(self.scene(None))
        frame, = [a for a in arrays if a.header.stamp == stamp]
        self.assertFalse(frame.tags)
        sub.unregister()

    def test_preview(self):
        previews=[]
        sub=rospy.Subscriber('/ov13855_test/preview',Image,previews.append)
        end=time.monotonic()+5
        while time.monotonic()<end and not sub.get_num_connections():time.sleep(.02)
        self.assertTrue(sub.get_num_connections())
        self.info();stamp=self.send(self.scene(7))
        end=time.monotonic()+2
        while time.monotonic()<end and not any(m.header.stamp==stamp for m in previews):time.sleep(.01)
        preview=[m for m in previews if m.header.stamp==stamp]
        self.assertTrue(preview,'preview was not published')
        self.assertEqual((preview[-1].width,preview[-1].height,preview[-1].encoding),(400,400,'bgr8'))
        sub.unregister()
if __name__=='__main__':
    rospy.init_node('ov13855_pipeline_test')
    rostest.rosrun('ov13855_apriltag_vision','ov13855_pipeline',PipelineTest)
