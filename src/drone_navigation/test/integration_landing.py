#!/usr/bin/env python3
"""Real Gateway/ROS services, simulated FCU and private arming service; preview output only."""
import json,os,sys,threading,time
from pathlib import Path
import numpy as np
import rospy,rosgraph
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,ExtendedState,EstimatorStatus,PositionTarget
from mavros_msgs.srv import CommandBool,CommandBoolResponse
from std_msgs.msg import String
from std_srvs.srv import Trigger
from drone_navigation.gateway import Gateway,LAND_PRESS

assert os.environ.get('ROS_MASTER_URI')=='http://127.0.0.1:11448'
rospy.init_node('landing_real_gateway_fixture')
rospy.set_param('~enable_output',False)
g=Gateway();g.real=True
assert g.output is None and g.disarm_service is None

class FCU:
    def __init__(self):
        self.mode,self.armed,self.landed='POSCTL',True,1
        self.z,self.follow,self.scenario=0.,False,''
        self.preview,self.contact_at,self.ground_at=None,None,None
        self.closed=False;self.events=[];self.commands=[];self.requests=[];self.preview_count=0
        self.pressed=False;self.disarm_ack_at=None;self.drop_ext=False;self.input_bad=False
        self.pubs={t:rospy.Publisher(t,c,queue_size=3) for t,c in (
            ('/drone_nav/odom',Odometry),('/mavros/local_position/odom',Odometry),
            ('/mavros/state',State),('/mavros/extended_state',ExtendedState),
            ('/mavros/estimator_status',EstimatorStatus),('/drone_nav/input_status',String))}
        rospy.Subscriber('/drone_nav/setpoint_preview',PositionTarget,self.on_preview)
        rospy.Subscriber('/drone_nav/gateway_status',String,self.on_status)
        self.service=rospy.Service('/landing_fixture/disarm',CommandBool,self.disarm)
        self.thread=threading.Thread(target=self.pump,daemon=True);self.thread.start()
    def on_preview(self,m):
        self.preview=m;self.preview_count+=1
        if self.follow and g.land_state in ('DESCENDING','PRESSING','TOUCHED'):
            self.commands.append(dict(z=m.position.z,vz=m.velocity.z,mask=m.type_mask,measured=self.z))
    def on_status(self,m):
        d=json.loads(m.data)
        if d.get('land')=='TOUCHED':assert self.landed==1,'TOUCHED before fresh PX4 ground confirmation'
        if not self.events or self.events[-1].get('land')!=d.get('land') or self.events[-1].get('land_disarm_requests')!=d.get('land_disarm_requests'):
            self.events.append(d)
    def disarm(self,req):
        assert req.value is False,'automatic arming must never occur'
        assert self.landed==1 and self.mode=='OFFBOARD','disarm must be ground gated'
        before=self.preview_count
        # A slow RPC must not interrupt the real Gateway's 50 Hz preview stream.
        time.sleep(.25)
        assert self.preview_count-before>=5,'disarm service blocked setpoint publication'
        accepted=self.scenario!='disarm_rejected'
        self.requests.append(dict(value=req.value,landed=self.landed,accepted=accepted,previews_during_rpc=self.preview_count-before))
        if accepted:self.disarm_ack_at=time.monotonic()
        return CommandBoolResponse(success=accepted,result=0 if accepted else 2)
    def pump(self):
        last=time.monotonic()
        while not self.closed and not rospy.is_shutdown():
            now=time.monotonic();dt=min(now-last,.05);last=now;previous=self.z
            if self.follow and self.preview is not None and g.land_state in ('DESCENDING','PRESSING','TOUCHED'):
                m=self.preview
                velocity_only=bool(m.type_mask & PositionTarget.IGNORE_PZ)
                if velocity_only and m.velocity.z<-.3:self.pressed=True
                # Model the reported 2 cm stall: slow commands cannot overcome it.
                floor=-.02 if self.pressed else .02
                if velocity_only:self.z=max(floor,self.z+m.velocity.z*dt)
                else:self.z=min(1.,max(floor,m.position.z+.02))
                if self.pressed and self.z<=-.02+1e-8:
                    self.contact_at=self.contact_at or now
                    if self.scenario!='never_landed' and now-self.contact_at>=.7:
                        self.landed=1;self.ground_at=self.ground_at or now
            if self.disarm_ack_at is not None and now-self.disarm_ack_at>.4:self.armed=False
            v=(self.z-previous)/max(dt,1e-5)
            stamp=rospy.Time.now()
            for topic,frame,child in (('/drone_nav/odom','drone_odom','drone_nav_base'),('/mavros/local_position/odom','px4_map','drone_base')):
                o=Odometry();o.header.stamp=stamp;o.header.frame_id=frame;o.child_frame_id=child
                o.pose.pose.position.z=self.z;o.pose.pose.orientation.w=1.;o.twist.twist.linear.z=v
                self.pubs[topic].publish(o)
            f=State(connected=True,armed=self.armed,mode=self.mode);f.header.stamp=stamp;self.pubs['/mavros/state'].publish(f)
            if not self.drop_ext:
                e=ExtendedState(landed_state=self.landed);e.header.stamp=stamp;self.pubs['/mavros/extended_state'].publish(e)
            est=EstimatorStatus();est.header.stamp=stamp
            for k in ('attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag','pos_horiz_rel_status_flag','pos_vert_abs_status_flag'):setattr(est,k,True)
            est.const_pos_mode_status_flag=self.landed==1 or (self.follow and self.z<.04)
            self.pubs['/mavros/estimator_status'].publish(est)
            self.pubs['/drone_nav/input_status'].publish(String('{"state":"FAULT"}' if self.input_bad else '{"state":"READY"}'))
            time.sleep(.02)
    def close(self):self.closed=True;self.thread.join(2)

def wait(predicate,timeout,label):
    until=time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>=until:raise AssertionError((label,g.fault,g.land_state))
        time.sleep(.02)

f=FCU();g.disarm_service=rospy.ServiceProxy('/landing_fixture/disarm',CommandBool)
results=[]
prepare_ground=rospy.ServiceProxy('/drone_nav/prepare_takeoff',Trigger)
prepare=rospy.ServiceProxy('/drone_nav/prepare',Trigger)
land=rospy.ServiceProxy('/drone_nav/land',Trigger)

def airborne(scenario):
    f.follow=False;f.mode='POSCTL';f.z=0.;f.armed=True;f.landed=1
    f.scenario=scenario;f.contact_at=f.ground_at=f.disarm_ack_at=None
    f.pressed=f.drop_ext=f.input_bad=False
    wait(lambda:g.alignment.r is not None and g.owners_ok and g.on_ground() and
         not g.was_offboard and g.px is not None and abs(g.px.pose.pose.position.z)<.01,8,'ground reference')
    r=prepare_ground();assert r.success,r.message
    f.landed=2
    for i in range(1,51):f.z=i*.02;time.sleep(.02)
    f.mode='OFFBOARD'
    wait(lambda:g.was_offboard and abs(g.px.pose.pose.position.z-1.)<.01,3,'mock airborne')
    r=prepare();assert r.success,r.message
    wait(lambda:g.ready,3,'landing READY')
    f.events=[];f.commands=[];f.requests=[]
    r=land();assert r.success,r.message
    f.follow=True

try:
    for scenario in ('two_cm_stall_then_ordinary_disarm','never_landed','disarm_rejected',
                     'press_takeover','press_input_fault','press_extended_state_stale'):
        airborne(scenario)
        wait(lambda:g.land_state=='PRESSING',18,'near-ground velocity press')
        assert f.pressed or f.z>=.019
        if scenario=='press_takeover':f.mode='POSCTL'
        if scenario=='press_input_fault':f.input_bad=True
        if scenario=='press_extended_state_stale':f.drop_ext=True
        if scenario=='two_cm_stall_then_ordinary_disarm':
            wait(lambda:not f.armed and g.land_state=='DONE',8,'ordinary disarm then actual state')
            assert f.requests and all(not r['value'] for r in f.requests)
            assert any(e.get('land')=='TOUCHED' for e in f.events)
        else:
            wait(lambda:g.land_state=='STOPPED',8,'bounded stop on fault')
            if scenario=='never_landed':assert 'landing not detected' in g.fault and not f.requests
            if scenario=='disarm_rejected':assert 'landed but not disarmed' in g.fault and len(f.requests)>=2
            if scenario=='press_takeover':assert 'OFFBOARD exited' in g.fault and not f.requests
            if scenario=='press_input_fault':assert 'inputs/clock stale' in g.fault and not f.requests
            if scenario=='press_extended_state_stale':assert 'landed status stale' in g.fault and not f.requests
        assert min(c['z'] for c in f.commands)>=-LAND_PRESS-1e-9
        assert any(c['mask'] & PositionTarget.IGNORE_PZ and c['vz']<-.3 for c in f.commands) or scenario.startswith('press_')
        results.append(dict(scenario=scenario,passed=True,reason=g.fault,events=f.events,
                            disarm_requests=f.requests,minimum_target=min(c['z'] for c in f.commands)))
        Path(sys.argv[1]).write_text(json.dumps(results,indent=2))
        print('PASS '+scenario,flush=True)
    pubs,_,services=rosgraph.Master(rospy.get_name()).getSystemState()
    assert not any(t in ('/mavros/setpoint_raw/local','/mavros/cmd/arming') for t,_ in pubs+services)
    print('PASS real Gateway landing: preview only, private mock disarm service',flush=True)
finally:
    f.close()
    class QuietOutput:
        def publish(self,*args,**kwargs):pass
    with g.lock:g.stat=g.preview=g.cancel=QuietOutput()
    rospy.signal_shutdown('landing integration finished')
