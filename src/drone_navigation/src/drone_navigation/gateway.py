import copy,math,threading,time
import numpy as np
import rospy,message_filters,rosgraph
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Empty
from std_srvs.srv import Trigger,TriggerResponse
from mavros_msgs.msg import PositionTarget,State,EstimatorStatus,ExtendedState
from drone_navigation.msg import NavigationCommand
from .core import Alignment,brake,deployment,yaw,heading,rz,competing_publishers,climb_profile,descend_profile
from .ros_common import *

STOPPED='task stopped; prepare before next route'
GROUND_DROP=.3 # landed, before the climb: stream the hold this far below the vehicle so PX4 does not start its takeoff
LAND_PRESS=.02 # slow landing: stream the hold this far below the ground reference so the vehicle settles instead of hovering

class Gateway:
    def __init__(self):
        self.c=config();self.real=bool(rospy.get_param('~enable_output',False))
        if self.real:
            deployment(self.c)
            radius=rospy.get_param('/drone_0_ego_planner_node/grid_map/nav_radius')
            height=rospy.get_param('/drone_0_ego_planner_node/grid_map/nav_half_height')
            if not math.isfinite(radius) or not math.isfinite(height) or radius<max(self.c['body_half_extent'][:2])+self.c['clearance'] or height<self.c['body_half_extent'][2]+self.c['clearance']:
                raise ValueError('planner inflation smaller than measured body+clearance')
        self.lock=threading.RLock();self.alignment=Alignment();self.odom=None;self.px=None;self.px_at=0.;self.state=None;self.state_at=0.;self.est=None;self.est_at=0.;self.cmd=None;self.cmd_at=0.;self.goal_stamp=None;self.expected_gen=None;self.input_ok=False;self.input_at=0.;self.fault='';self.prepared=False;self.running=False;self.was_offboard=False;self.prepared_at=0.;self.owners_ok=False;self.owners_at=0.;self.hold_p=None;self.hold_v=np.zeros(3);self.hold_yaw=0.;self.last_clock=None;self.clock_at=time.monotonic();self.terminal=None;self.arrived_at=-math.inf
        # PX4 samples are only taken in pairs with LIO odometry, so their age follows the LIO limit too.
        self.odom_age=odom_timeout(self.c)
        # Ground takeoff: prepare_takeoff while armed on the ground, select OFFBOARD, then takeoff climbs takeoff_height
        # above the landed hold point at max_vel_z / max_acc_z and holds there.
        self.ext=None;self.ext_at=0.;self.takeoff_mode=False;self.climb=None;self.climb_end=None;self.takeoff_state='';self.ready=False
        self.takeoff_h=float(self.c.get('takeoff_height',1.));self.takeoff_v=float(self.c.get('max_vel_z',.3));self.takeoff_a=float(self.c.get('max_acc_z',.3))
        if not .3<=self.takeoff_h<=2.5 or not 0.<self.takeoff_v<=1. or not 0.<self.takeoff_a<=1.:raise ValueError('takeoff_height must be in [0.3, 2.5] m, max_vel_z/max_acc_z in (0, 1]')
        # Slow landing: land_start() lets the hold stream descend at land_vel down to the ground reference recorded
        # at prepare_takeoff, so the pilot is not forced into PX4's AUTO.LAND descent speed.
        self.ground_z=None;self.land=None;self.land_state=''
        self.land_v=float(rospy.get_param('~land_vel',.1))
        if not 0.<self.land_v<=.5:raise ValueError('land_vel must be in (0, 0.5] m/s')
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
        rospy.Subscriber('/drone_nav/arrived',Empty,self.on_arrived,queue_size=1)
        rospy.Subscriber('/mavros/extended_state',ExtendedState,self.on_ext,queue_size=1)
        rospy.Service('/drone_nav/prepare',Trigger,self.prepare)
        rospy.Service('/drone_nav/prepare_takeoff',Trigger,self.prepare_takeoff)
        rospy.Service('/drone_nav/takeoff',Trigger,self.takeoff)
        rospy.Service('/drone_nav/land',Trigger,self.land_start)
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
            # Freeze the latest measured hold point on the OFFBOARD edge, including
            # movement since the preceding 50 Hz tick. Never retain the prepare pose.
            if off and not self.was_offboard and self.prepared:self.capture_hold()
            self.was_offboard=off;self.state=m;self.state_at=time.monotonic()
    def on_est(self,m):
        with self.lock:self.est=m;self.est_at=time.monotonic()
    def on_ext(self,m):
        with self.lock:self.ext=m;self.ext_at=time.monotonic()
    def on_ground(self):
        return self.ext is not None and time.monotonic()-self.ext_at<1. and self.ext.landed_state==ExtendedState.LANDED_STATE_ON_GROUND
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
                self.fault=self.fault or STOPPED
            self.running=False
            self.hold_terminal()
    def on_arrived(self,_):
        with self.lock:self.arrived_at=time.monotonic();self.hold_terminal()
    def hold_terminal(self):
        # The mission reports a normal arrival with arrived + cancel; either may land first. Then hold the
        # final trajectory point streamed just before, not the braked current pose (that freezes wherever
        # the settle check fired). Any other stop (faults, action cancel, health loss) keeps braking in place.
        now=time.monotonic()
        if self.terminal is None or self.running or self.fault!=STOPPED or now-self.arrived_at>.3 or now-self.terminal[2]>.3:return
        self.hold_p,self.hold_v,self.hold_yaw=self.terminal[0].copy(),np.zeros(3),self.terminal[1];self.terminal=None
    def on_goal(self,m):
        with self.lock:
            if self.prepared and not self.fault:
                self.goal_stamp=m.header.stamp;self.expected_gen=m.header.seq
    def on_cmd(self,m):
        with self.lock:
            if self.prepared and not self.fault and self.land is None and self.goal_stamp is not None and m.generation==self.expected_gen and m.header.stamp>=self.goal_stamp and m.header.frame_id=='drone_odom' and fresh(m,time.monotonic()):
                if not np.isfinite(xyz(m.position)).all() or not np.isfinite(xyz(m.velocity)).all() or not np.isfinite(xyz(m.acceleration)).all() or not math.isfinite(m.yaw) or np.linalg.norm(xyz(m.velocity))>self.c['max_vel']*1.001 or np.linalg.norm(xyz(m.acceleration))>self.c['max_acc']*1.001:
                    self.fail('invalid navigation command');return
                self.cmd=m;self.cmd_at=time.monotonic();self.running=True
    def on_pair(self,o,p):
        with self.lock:
            try:
                if o.header.frame_id!='drone_odom' or p.header.frame_id!='px4_map' or p.child_frame_id!='drone_base':raise ValueError('alignment frames')
                if not fresh(o,time.monotonic(),self.odom_age) or not fresh(p,time.monotonic(),self.odom_age):raise ValueError('alignment stale')
                if self.px is not None and p.header.stamp<=self.px.header.stamp:raise ValueError('PX4 timestamp rollback')
                po,pp=xyz(o.pose.pose.position),xyz(p.pose.pose.position);ao,ap=yaw(quat(o.pose.pose.orientation)),yaw(quat(p.pose.pose.orientation))
                self.odom=o;self.px=p;self.px_at=time.monotonic()
                if self.alignment.r is None:self.alignment.add(o.header.stamp.to_sec(),po,ao,pp,ap)
                elif not self.alignment.check(po,ao,pp,ap):self.fail('LIO/PX4 frame residual');self.alignment=Alignment();self.prepared=False
            except Exception as e:
                if self.prepared:self.fail(str(e));self.prepared=False
                self.alignment=Alignment()
    def capture_hold(self):
        if self.px is not None:
            self.hold_p=xyz(self.px.pose.pose.position);self.hold_v=np.zeros(3);self.hold_yaw=yaw(quat(self.px.pose.pose.orientation))
    def capture_brake(self):
        if self.px is not None:
            self.hold_p=xyz(self.px.pose.pose.position);self.hold_v=rotation(quat(self.px.pose.pose.orientation))@xyz(self.px.twist.twist.linear);self.hold_yaw=yaw(quat(self.px.pose.pose.orientation))
    def fail(self,reason):
        if not self.fault:
            self.fault=reason;self.capture_brake();self.running=False;self.cmd=None;self.cancel.publish(Empty())
            self.climb=None;self.land=None
            if self.takeoff_state:self.takeoff_state='STOPPED'
            if self.land_state:self.land_state='STOPPED'
    def prepare(self,_):
        with self.lock:
            if self.alignment.r is None or not fresh(self.px,self.px_at,self.odom_age) or not self.input_ok or not self.owners_ok or time.monotonic()-self.owners_at>1.5:
                return TriggerResponse(False,'alignment, inputs or exclusive ownership not ready')
            self.fault='';self.prepared=True;self.running=False;self.cmd=None;self.goal_stamp=None;self.expected_gen=None;self.prepared_at=time.monotonic();self.terminal=None;self.capture_hold()
            self.takeoff_mode=False;self.climb=None;self.climb_end=None;self.takeoff_state='';self.land=None;self.land_state=''
            return TriggerResponse(True,'prepared: hold stream; manually select OFFBOARD, then start route')
    def prepare_takeoff(self,_):
        with self.lock:
            if not self.on_ground():return TriggerResponse(False,'PX4 does not report landed')
            if self.real and (self.state is None or time.monotonic()-self.state_at>2. or not self.state.armed or self.was_offboard):
                return TriggerResponse(False,'arm on the ground outside OFFBOARD first')
            r=self.prepare(None)
            if r.success:
                self.takeoff_mode=True;self.takeoff_state='GROUND';self.ground_z=float(self.hold_p[2])
                r.message='prepared for takeoff: hold streamed %.1f m below the vehicle; select OFFBOARD, then call takeoff'%GROUND_DROP
            return r
    def takeoff(self,_):
        with self.lock:
            if not self.takeoff_mode or not self.ready or self.climb is not None or self.climb_end is not None:
                return TriggerResponse(False,'not prepared for takeoff, not READY in OFFBOARD, or already climbed')
            if not self.on_ground():return TriggerResponse(False,'PX4 does not report landed')
            self.climb=(time.monotonic(),self.hold_p.copy());self.takeoff_state='CLIMBING'
            return TriggerResponse(True,'climbing %.2f m at %.2f m/s'%(self.takeoff_h,self.takeoff_v))
    def land_start(self,_):
        with self.lock:
            if not self.ready:return TriggerResponse(False,'not READY in OFFBOARD')
            if self.land is not None:return TriggerResponse(False,'already descending')
            if self.ground_z is None:return TriggerResponse(False,'no ground reference: prepare a takeoff on the ground first')
            z1=float(self.ground_z)-LAND_PRESS
            if self.hold_p[2]-z1<.05:return TriggerResponse(False,'already at the ground reference')
            self.land=(time.monotonic(),self.hold_p.copy(),z1);self.land_state='DESCENDING'
            return TriggerResponse(True,'descending %.2f m at %.2f m/s'%(self.hold_p[2]-z1,self.land_v))
    def estimator_ok(self):
        e=self.est
        # const_pos also means "vehicle at rest" (EV still fused): accepted only while landed and preparing a takeoff.
        return fresh(e,self.est_at,2.) and e.attitude_status_flag and e.velocity_horiz_status_flag and e.velocity_vert_status_flag and (e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag) and (e.pos_vert_abs_status_flag or e.pos_vert_agl_status_flag) and (not e.const_pos_mode_status_flag or (self.takeoff_mode and self.on_ground()) or (self.land_state in ('DESCENDING','TOUCHED') and self.near_ground()))
    def near_ground(self):
        # const_pos arrives a moment before PX4's landed state at touchdown; inside the last 10 cm of a
        # descent it is expected, not an estimator fault.
        return self.px is not None and self.ground_z is not None and xyz(self.px.pose.pose.position)[2]-self.ground_z<.10
    def tick(self):
        with self.lock:
            wall=time.monotonic();now=rospy.Time.now().to_sec()
            if self.last_clock is None or now>self.last_clock:self.clock_at=wall
            if self.last_clock is not None and now<self.last_clock:self.fail('ROS clock rollback');self.prepared=False
            self.last_clock=now
            pxok=fresh(self.px,self.px_at,self.odom_age) and (not self.real or self.estimator_ok())
            if self.prepared:
                if not self.owners_ok or wall-self.owners_at>1.5:self.fail('another setpoint publisher or ownership check lost');self.prepared=False
                if not self.input_ok or wall-self.input_at>self.odom_age or wall-self.clock_at>.3:self.fail('navigation inputs/clock stale')
                if self.running and not fresh(self.cmd,self.cmd_at,.3):self.fail('trajectory command stale')
                if not pxok:self.fail('PX4 local estimate unavailable')
                # Takeoff ends when PX4 leaves the ground; still landed 3 s after the climb means it never lifted.
                if self.takeoff_mode and self.climb_end is not None and not self.fault:
                    if self.ext is not None and wall-self.ext_at<1. and not self.on_ground():self.takeoff_mode=False;self.takeoff_state='DONE'
                    elif wall-self.climb_end>3.:self.fail('takeoff not detected by PX4')
            # Pre-stream at the current measured pose while the pilot is in control.
            if self.prepared and pxok and not self.was_offboard:self.capture_hold()
            ready=self.ready=self.prepared and not self.fault and pxok and (not self.real or (self.was_offboard and wall-self.state_at<2. and wall-self.prepared_at>=1.))
            if self.climb is not None and not ready:self.climb=None;self.takeoff_state='STOPPED' # never resume a climb clock that kept running
            if self.land is not None and not ready:self.land=None;self.land_state='STOPPED' # never resume a descent clock that kept running
            status(self.stat,'READY' if ready else ('FAULT' if self.fault else 'PREPARED' if self.prepared else 'ALIGNING'),reason=self.fault,real_output=self.real,takeoff=self.takeoff_state,land=self.land_state)
            if not self.prepared or not pxok or self.hold_p is None:return
            if self.real and (self.state is None or wall-self.state_at>2. or not self.state.connected):return
            acc=None
            if ready and self.land is not None:
                # The descent owns the stream while it runs: velocity-limited, ending at the ground reference and
                # holding there, so PX4's own land detector and COM_DISARM_LAND finish the touchdown.
                t0,p0,z1=self.land;s,ds=descend_profile(wall-t0,self.land_v,self.takeoff_a)
                p=p0.copy();p[2]=max(z1,p0[2]-s);v=np.array([0.,0.,-ds]);a=self.hold_yaw
                self.hold_p,self.hold_v=p.copy(),v.copy()
                if p[2]<=z1+1e-9:self.land=None;self.land_state='TOUCHED';self.hold_v=np.zeros(3);v=np.zeros(3)
            elif ready and self.running and self.cmd is not None:
                r,t=self.alignment.r,self.alignment.t;p=r@xyz(self.cmd.position)+t;v=r@xyz(self.cmd.velocity);acc=r@xyz(self.cmd.acceleration);a=heading(r,self.cmd.yaw)
                self.terminal=(p,a,wall) if self.cmd.terminal else None
            elif ready and self.climb is not None:
                # The hold follows the climb, so any interruption brakes from the last climb setpoint.
                t0,p0=self.climb;s,ds,dds,T=climb_profile(wall-t0,self.takeoff_h,self.takeoff_v,self.takeoff_a)
                p=p0+[0.,0.,s];v=np.array([0.,0.,ds]);acc=np.array([0.,0.,dds]);a=self.hold_yaw
                self.hold_p,self.hold_v=p.copy(),v.copy()
                if wall-t0>=T:self.climb=None;self.climb_end=wall;self.takeoff_state='HOLD';self.hold_v=np.zeros(3)
            else:
                self.hold_p,self.hold_v=brake(self.hold_p,self.hold_v,self.c['max_acc']);p,v,a=self.hold_p,self.hold_v,self.hold_yaw
                if self.takeoff_mode and self.on_ground() and (self.climb_end is None or self.fault):p=p-[0.,0.,GROUND_DROP]
            m=PositionTarget();m.header.stamp=rospy.Time.now();m.header.frame_id='px4_map';m.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
            # Trajectory setpoints carry EGO's acceleration as PX4 feedforward (less overshoot when braking);
            # the hold/brake stream stays position + velocity only.
            m.type_mask=PositionTarget.IGNORE_YAW_RATE if acc is not None else PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW_RATE
            put(m.position,p);put(m.velocity,v);m.yaw=a
            if acc is not None:put(m.acceleration_or_force,acc)
            self.preview.publish(m)
            if self.output:self.output.publish(m)
