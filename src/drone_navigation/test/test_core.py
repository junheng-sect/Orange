#!/usr/bin/env python3
import math,unittest
import numpy as np
from drone_navigation.core import *

class CoreTests(unittest.TestCase):
    def test_reference_lever_and_angular_velocity(self):
        p,v=reference_state([1,2,3],[.3,.4,0],rz(math.pi/2),[0,0,2],[.1,0,0])
        np.testing.assert_allclose(p,[1,2.1,3]);np.testing.assert_allclose(v,[.1,.4,0])
    def test_alignment_nonzero_translation_and_90deg(self):
        a=Alignment();r=rz(math.pi/2);t=np.array([2,3,4])
        for i in range(25):
            p=np.array([i*.01,.2,.3]);a.add(i*.1,p,.1,r@p+t,.1+math.pi/2)
        np.testing.assert_allclose(a.t,t,atol=1e-10);np.testing.assert_allclose(a.r,r,atol=1e-10)
        self.assertTrue(a.check(p,.1,r@p+t,.1+math.pi/2))
        self.assertFalse(a.check(p,.1,r@p+t+[.2,0,0],.1+math.pi/2))
    def test_alignment_rejects_clock_rollback(self):
        a=Alignment();a.add(1,[0,0,0],0,[1,0,0],0)
        with self.assertRaises(ValueError):a.add(.5,[0,0,0],0,[1,0,0],0)
    def test_alignment_unstable(self):
        a=Alignment()
        with self.assertRaises(ValueError):
            for i in range(30):a.add(i*.1,[0,0,0],0,[i%2*.2,0,0],0)
    def test_cubic_and_derivatives(self):
        s=Spline(3,range(8),[[0,0,0],[1,2,0],[2,4,0],[3,6,0]],1,1)
        for t in np.linspace(0,1,21):
            p,v,a=s.state(t);np.testing.assert_allclose(p,[1+t,2+2*t,0]);np.testing.assert_allclose(v,[1,2,0]);np.testing.assert_allclose(a,[0,0,0])
    def test_spline_rejects_nan_and_bad_knots(self):
        for knots,points in [([0]*8,[[0,0,0]]*4),(list(range(8)),[[float('nan'),0,0]]*4)]:
            with self.assertRaises(ValueError):Spline(3,knots,points,0,1)
    def test_braking_stopping_distance(self):
        p=np.zeros(3);v=np.array([.5,0,0])
        for _ in range(50):p,v=brake(p,v)
        self.assertAlmostEqual(p[0],.25,places=8);self.assertLess(np.linalg.norm(v),1e-10)
    def test_deployment_requires_measured_geometry(self):
        with self.assertRaises(ValueError):deployment({'deployment_verified':False})
    def test_heading_transform(self):
        self.assertAlmostEqual(heading(rz(math.pi/2),0),math.pi/2)
    def test_reference_point_covariance(self):
        c=np.zeros((6,6));c[5,5]=1.
        shifted=np.array(shift_pose_covariance(c,[0,.1,0])).reshape(6,6)
        self.assertAlmostEqual(shifted[0,0],.01)
        self.assertAlmostEqual(shifted[0,5],-.1)
        np.testing.assert_allclose(shifted,shifted.T)
        self.assertGreaterEqual(np.linalg.eigvalsh(shifted).min(),-1e-12)
    def test_mavros_feedback_is_not_a_command_conflict(self):
        pubs=[('/mavros/setpoint_raw/target_local',['/mavros']),('/mavros/setpoint_trajectory/desired',['/mavros'])]
        self.assertEqual(competing_publishers(pubs,'/offboard_gateway'),[])
    def test_velocity_competitor_and_self_publisher(self):
        pubs=[('/mavros/setpoint_raw/local',['/offboard_gateway']),('/mavros/setpoint_velocity/cmd_vel',['/another_controller'])]
        self.assertEqual(competing_publishers(pubs,'/offboard_gateway'),['/another_controller'])
    def test_quaternion_rejects_nonunit(self):
        with self.assertRaises(ValueError):rotation([0,0,0,0])
if __name__=='__main__':unittest.main()
