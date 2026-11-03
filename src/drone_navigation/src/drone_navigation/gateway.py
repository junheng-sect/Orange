import copy,math,threading,time
import numpy as np
import rospy,message_filters,rosgraph
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Empty
from std_srvs.srv import Trigger,TriggerResponse
from mavros_msgs.msg import PositionTarget,State,EstimatorStatus
from drone_navigation.msg import NavigationCommand
from .core import Alignment,brake,deployment,yaw,heading,rz,competing_publishers
from .ros_common import *

class Gateway:
    def __init__(self):
        self.c=config();self.real=bool(rospy.get_param('~enable_output',False))
        if self.real:
            deployment(self.c)
            radius=rospy.get_param('/drone_0_ego_planner_node/grid_map/nav_radius')
            height=rospy.get_param('/drone_0_ego_planner_node/grid_map/nav_half_height')
            if not math.isfinite(radius) or not math.isfinite(height) or radius<max(self.c['body_half_extent'][:2])+self.c['clearance'] or height<self.c['body_half_extent'][2]+self.c['clearance']:
                raise ValueError('planner inflation smaller than measured body+clearance')
        self.lock=threading.RLock();self.alignment=Alignment();self.odom=None;self.px=None;self.px_at=0.;self.state=None;self.state_at=0.;self.est=None;self.est_at=0.;self.cmd=None;self.cmd_at=0.;self.goal_stamp=None;self.expected_gen=None;self.input_ok=False;self.input_at=0.;self.fault='';self.prepared=False;self.running=False;self.was_offboard=False;self.prepared_at=0.;self.owners_ok=False;self.owners_at=0.;self.hold_p=None;self.hold_v=np.zeros(3);self.hold_yaw=0.;self.last_clock=None;self.clock_at=time.monotonic()
        self.preview=rospy.Publisher('/drone_nav/setpoint_preview',PositionTarget,queue_size=1)
        self.output=rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=1) if self.real else None
        self.stat=rospy.Publisher('/drone_nav/gateway_status',String,queue_size=1)
        self.cancel=rospy.Publisher('/drone_nav/cancel',Empty,queue_size=1)
        self.a=message_filters.Subscriber('/drone_nav/odom',Odometry);self.b=message_filters.Subscriber('/mavros/local_position/odom',Odometry)
        self.sync=message_filters.ApproximateTimeSynchronizer([self.a,self.b],30,.08);self.sync.registerCallback(self.on_pair)
        rospy.Subscriber('/mavros/state',State,self.on_state,queue_size=1)
        rospy.Subscriber('/mavros/estimator_status',EstimatorStatus,self.on_est,queue_size=1)
        from geometry_msgs.msg import PoseStamped
        rospy.Subscriber('/drone_nav/planner_goal',PoseStamped,self.on_goal,queue_size=1)
        rospy.Subscriber('/drone_nav/command',NavigationCommand,self.on_cmd,queue_size=1)
        rospy.Subscriber('/drone_nav/input_status',String,self.on_input,queue_size=1)
        rospy.Subscriber('/drone_nav/executor_status',String,self.on_executor,queue_size=1)
        rospy.Subscriber('/drone_nav/cancel',Empty,self.on_cancel,queue_size=1)
        rospy.Service('/drone_nav/prepare',Trigger,self.prepare)
        loop(self.ownership,2);loop(self.tick)
    def ownership(self):
        try:
            pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
            competitors=competing_publishers(pubs,rospy.get_name())
            with self.lock:self.owners_ok=not competitors;self.owners_at=time.monotonic()
        except Exception:
            with self.lock:self.owners_ok=False
    def on_state(self,m):
        with self.lock:
            off=m.connected and m.armed and m.mode=='OFFBOARD'
            if self.was_offboard and not off:self.fail('OFFBOARD exited; explicit prepare/start required');self.prepared=False
            self.was_offboard=off;self.state=m;self.state_at=time.monotonic()
    def on_est(self,m):
        with self.lock:self.est=m;self.est_at=time.monotonic()
    def on_input(self,m):
        with self.lock:self.input_ok=read_status(m).get('state')=='READY';self.input_at=time.monotonic()
    def on_executor(self,m):
        with self.lock:
            if read_status(m).get('state')=='FAULT' and self.running:self.fail('executor fault')
    def on_cancel(self,_):
        with self.lock:
            self.cmd=None
            if self.running:
                self.capture_brake()
                self.fault=self.fault or "task stopped; prepare before next route"
            self.running=False
    def on_goal(self,m):
        with self.lock:
            if self.prepared and not self.fault:
                self.goal_stamp=m.header.stamp;self.expected_gen=m.header.seq
    def on_cmd(self,m):
        with self.lock:
            if self.prepared and not self.fault and self.goal_stamp is not None and m.generation==self.expected_gen and m.header.stamp>=self.goal_stamp and m.header.frame_id=='drone_odom' and fresh(m,time.monotonic()):
                if not np.isfinite(xyz(m.position)).all() or not np.isfinite(xyz(m.velocity)).all() or not math.isfinite(m.yaw) or np.linalg.norm(xyz(m.velocity))>self.c['max_vel']*1.001:
                    self.fail('invalid navigation command');return
                self.cmd=m;self.cmd_at=time.monotonic();self.running=True
    def on_pair(self,o,p):
        with self.lock:
            try:
                if o.header.frame_id!='drone_odom' or p.header.frame_id!='px4_map' or p.child_frame_id!='drone_base':raise ValueError('alignment frames')
                if not fresh(o,time.monotonic()) or not fresh(p,time.monotonic()):raise ValueError('alignment stale')
                if self.px is not None and p.header.stamp<=self.px.header.stamp:raise ValueError('PX4 timestamp rollback')
                po,pp=xyz(o.pose.pose.position),xyz(p.pose.pose.position);ao,ap=yaw(quat(o.pose.pose.orientation)),yaw(quat(p.pose.pose.orientation))
                self.odom=o;self.px=p;self.px_at=time.monotonic()
                if self.alignment.r is None:self.alignment.add(o.header.stamp.to_sec(),po,ao,pp,ap)
                elif not self.alignment.check(po,ao,pp,ap):self.fail('LIO/PX4 frame residual');self.alignment=Alignment();self.prepared=False
            except Exception as e:
                if self.prepared:self.fail(str(e));self.prepared=False
                self.alignment=Alignment()
    def capture_brake(self):
        if self.px is not None:
            self.hold_p=xyz(self.px.pose.pose.position);self.hold_v=rotation(quat(self.px.pose.pose.orientation))@xyz(self.px.twist.twist.linear);self.hold_yaw=yaw(quat(self.px.pose.pose.orientation))
    def fail(self,reason):
        if not self.fault:
            self.fault=reason;self.capture_brake();self.running=False;self.cmd=None;self.cancel.publish(Empty())
    def prepare(self,_):
        with self.lock:
            if self.alignment.r is None or not fresh(self.px,self.px_at) or not self.input_ok or not self.owners_ok or time.monotonic()-self.owners_at>1.5:
                return TriggerResponse(False,'alignment, inputs or exclusive ownership not ready')
            self.fault='';self.prepared=True;self.running=False;self.cmd=None;self.goal_stamp=None;self.expected_gen=None;self.prepared_at=time.monotonic();self.capture_brake()
            self.hold_v=np.zeros(3)
            return TriggerResponse(True,'prepared: hold stream; manually select OFFBOARD, then start route')
    def estimator_ok(self):
        e=self.est
        return fresh(e,self.est_at,2.) and e.attitude_status_flag and e.velocity_horiz_status_flag and e.velocity_vert_status_flag and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag) and (e.pos_vert_abs_status_flag or e.pos_vert_agl_status_flag) and not e.const_pos_mode_status_flag
    def tick(self):
        with self.lock:
            wall=time.monotonic();now=rospy.Time.now().to_sec()
            if self.last_clock is None or now>self.last_clock:self.clock_at=wall
            if self.last_clock is not None and now<self.last_clock:self.fail('ROS clock rollback');self.prepared=False
            self.last_clock=now
            pxok=fresh(self.px,self.px_at) and (not self.real or self.estimator_ok())
            if self.prepared:
                if not self.owners_ok or wall-self.owners_at>1.5:self.fail('another setpoint publisher or ownership check lost');self.prepared=False
                if not self.input_ok or wall-self.input_at>.3 or wall-self.clock_at>.3:self.fail('navigation inputs/clock stale')
                if self.running and not fresh(self.cmd,self.cmd_at,.3):self.fail('trajectory command stale')
                if not pxok:self.fail('PX4 local estimate unavailable')
            ready=self.prepared and not self.fault and pxok and (not self.real or (self.was_offboard and wall-self.state_at<2. and wall-self.prepared_at>=1.))
            status(self.stat,'READY' if ready else ('FAULT' if self.fault else 'PREPARED' if self.prepared else 'ALIGNING'),reason=self.fault,real_output=self.real)
            if not self.prepared or not pxok or self.hold_p is None:return
            if self.real and (self.state is None or wall-self.state_at>2. or not self.state.connected):return
            if ready and self.running and self.cmd is not None:
                r,t=self.alignment.r,self.alignment.t;p=r@xyz(self.cmd.position)+t;v=r@xyz(self.cmd.velocity);a=heading(r,self.cmd.yaw)
            else:
                self.hold_p,self.hold_v=brake(self.hold_p,self.hold_v,self.c['max_acc']);p,v,a=self.hold_p,self.hold_v,self.hold_yaw
            m=PositionTarget();m.header.stamp=rospy.Time.now();m.header.frame_id='px4_map';m.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
            m.type_mask=PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW_RATE
            put(m.position,p);put(m.velocity,v);m.yaw=a
            self.preview.publish(m)
            if self.output:self.output.publish(m)
