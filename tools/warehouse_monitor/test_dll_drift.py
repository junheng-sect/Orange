#!/usr/bin/env python3
import math
import json
from pathlib import Path
from collections import namedtuple
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from board_tasks import Progress
from dll_drift import BASE, DLL, EDGE, HEALTH, MODE, ODOM, RELOC, RESULT, NS, Comparison
from monitor import Monitor


class DriftTests(unittest.TestCase):
    def setUp(self):
        self.s = Comparison()
        self.t = 100*NS
        self.seed = dict(ok=True, confirmed=True, stamp=97., map='/maps/outdoor',
                         transform=dict(translation=[0.,0.,0.], q_xyzw=[0.,0.,0.,1.]))
        self.send(RESULT, String(data=json.dumps(self.seed)))
        self.send(RELOC,String(data=json.dumps(dict(valid=True,state='LOCALIZED',stamp=99.9))))
        self.tr=TransformStamped()
        self.tr.header.frame_id='drone_map'
        self.tr.child_frame_id='drone_odom'
        self.tr.header.stamp=rospy.Time(100)
        self.tr.transform.rotation.w=1
        self.send('/tf',TFMessage([self.tr]));self.send(EDGE,self.tr)
        od=Odometry()
        od.header.frame_id='drone_odom';od.child_frame_id='drone_base';od.header.stamp=rospy.Time(100)
        self.send(ODOM,od)
        for topic,deg,x,y,z in [(BASE,179.,1.,1.,1.),(DLL,-179.,1.3,.6,.8)]:
            msg=PoseStamped()
            msg.header.frame_id='drone_map';msg.header.stamp=rospy.Time(100)
            msg.pose.position.x=x;msg.pose.position.y=y;msg.pose.position.z=z
            msg.pose.orientation.w=math.cos(math.radians(deg)/2)
            msg.pose.orientation.z=math.sin(math.radians(deg)/2)
            self.send(topic,msg)
        self.status()

    def send(self,topic,msg):
        self.s.feed(topic,msg,self.t)

    def status(self,**overrides):
        fields=dict(local_input='fresh',accepted_stamp_ns=99*NS,legacy_seed_stamp_ns=97*NS,reason='test')
        fields.update(overrides)
        msg=DiagnosticArray()
        msg.header.stamp=rospy.Time(100)
        msg.status=[DiagnosticStatus(name='fastlio_dll',message='TRACKING',level=0,
                                    values=[KeyValue(k,str(v)) for k,v in fields.items()])]
        self.send(HEALTH,msg)

    def test_sign_wrap_and_health(self):
        value=self.s.measure(self.t)
        self.assertTrue(value['valid'])
        for k,expected in dict(dx_m=.3,dy_m=-.4,dz_m=-.2,horizontal_m=.5,yaw_deg=2.).items():
            self.assertAlmostEqual(value[k],expected)
        self.assertEqual(value,self.s.live(self.t,True))
        self.assertFalse(self.s.live(self.t+NS,True)['valid'])
        self.assertFalse(self.s.live(self.t,True,self.t+NS)['valid'])
        self.status(accepted_stamp_ns=self.t-1500000000)
        self.assertEqual(self.s.measure(self.t)['reason'],'DLL 校正过期')
        self.status(legacy_seed_stamp_ns=96*NS)
        self.assertFalse(self.s.measure(self.t)['valid'])
        self.status()
        self.s.frames[self.t].pop(EDGE)
        self.assertFalse(self.s.measure(self.t)['valid'])

    def test_session_and_main_source(self):
        mon=Monitor(Path('/tmp'))
        mon.drift=self.s
        self.send(RESULT,String(data=json.dumps(dict(self.seed,stamp=100.01))))
        self.assertFalse(self.s.frames)
        self.assertFalse(self.s.measure(self.t)['valid'])
        with patch.object(mon,'dll_mode',return_value=True), patch.object(rospy.Time,'now',return_value=rospy.Time(100)):
            snap=mon.snapshot({})
            self.assertIsNone(snap['pose'])
            self.assertFalse(snap['dllComparison']['valid'])
        self.send(MODE,String(data=json.dumps(dict(run='a', mode='dll'))))
        self.send(MODE,String(data=json.dumps(dict(run='b', mode='dll'))))
        self.assertIsNone(self.s.solution)

    def test_frozen_bag_clock(self):
        from export_dll_drift import bag_events
        Event = namedtuple("Event", ["topic", "message", "timestamp"])
        clock = rospy.Time(100, 50000000)
        primary = PoseStamped()
        primary.header.stamp = rospy.Time(100)
        stale = DiagnosticArray()
        stale.header.stamp = primary.header.stamp
        stale.status = [DiagnosticStatus(level=3)]
        invalid = String(data=json.dumps(dict(valid=False, stamp=clock.to_sec())))
        tied = [Event(RELOC, invalid, clock), Event(HEALTH, stale, clock), Event(BASE, primary, clock)]
        bag = SimpleNamespace(read_messages=lambda topics: iter(tied))
        self.assertEqual([topic for topic, _, _ in bag_events(bag)], [BASE, HEALTH, RELOC])

    def test_manual_readiness(self):
        p=Progress();p.manual_ready=True;p.recording='待录包'
        health=dict(ground=True,armed=False,landed=1,ready=True,observer_tracking=False,observer_note='等待 DLL 状态')
        self.assertEqual(p.stage(dict(manual=True),health,False)[0],'wait_pilot')
        self.assertEqual(p.stage(dict(manual=True,observer='ywl'),health,False)[0],'preparing')
        health['ready']=False
        self.assertEqual(p.stage(dict(manual=True),health,False)[0],'preparing')


if __name__ == '__main__':
    unittest.main()
