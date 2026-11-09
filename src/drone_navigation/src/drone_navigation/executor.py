import threading,time,math,copy
import numpy as np
import rospy
from std_msgs.msg import Empty,String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from traj_utils.msg import Bspline
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import NavigationCommand
from .core import Spline,wrap,yaw
from .ros_common import *

class Executor:
    def __init__(self):
        self.lock=threading.RLock();self.spline=None;self.spline_goal=None;self.pending_spline=None;self.goal=None;self.odom=None;self.odom_at=0.;self.active=False;self.gen=0;self.yaw=0.;self.last_id=-1;self.plan_at=0.;self.plan='';self.fault='';self.goal_at=0.;self.c=config();self.checked_id=None;self.checked_at=0.;self.check_reason='waiting for map validation';self.last_reference=None
        self.early_spline=None;self.early_at=0.
        self.odom_age=odom_timeout(self.c)
        # Turn on the spot at yaw_rate_deg (deg/s, default 10), one step per 50 Hz tick.
        rate=float(self.c.get('yaw_rate_deg',10.))
        if not 0.<rate<=90.:raise ValueError('yaw_rate_deg must be in (0, 90]')
        self.yaw_step=math.radians(rate)*.02
        self.pub=rospy.Publisher('/drone_nav/command',NavigationCommand,queue_size=1)
        self.stat=rospy.Publisher('/drone_nav/executor_status',String,queue_size=1)
        self.cancel=rospy.Publisher('/drone_nav/cancel',Empty,queue_size=1)
        self.check=rospy.ServiceProxy('/drone_nav/check_trajectory',CheckTrajectory)
        rospy.Subscriber('/drone_nav/planner_goal',PoseStamped,self.on_goal,queue_size=1)
        rospy.Subscriber('/drone_nav/bspline',Bspline,self.on_spline,queue_size=1)
        rospy.Subscriber('/drone_nav/cancel',Empty,self.on_cancel,queue_size=1)
        rospy.Subscriber('/drone_nav/odom',Odometry,self.on_odom,queue_size=1)
        rospy.Subscriber('/drone_nav/planner_status',String,self.on_plan,queue_size=1)
        loop(self.validate,10);loop(self.tick)
    def on_plan(self,m):
        with self.lock:self.plan=m.data;self.plan_at=time.monotonic()
    def on_odom(self,m):
        with self.lock:self.odom=m;self.odom_at=time.monotonic()
    def on_cancel(self,_):
        with self.lock:self.active=False;self.spline=None;self.spline_goal=None;self.pending_spline=None;self.goal=None;self.early_spline=None
    def on_goal(self,m):
        with self.lock:
            # Retain the approved reference (normally its endpoint on a waypoint
            # handoff) until the next goal's candidate passes the map check.
            self.active=True;self.pending_spline=None;self.goal=m;self.goal_at=time.monotonic();self.gen=m.header.seq;self.fault=''
            if self.spline is None:
                self.checked_id=None;self.checked_at=0.;self.last_reference=None
                if self.odom is not None:self.yaw=yaw(quat(self.odom.pose.pose.orientation))
            # A stationary planner can publish before this subscriber receives
            # the goal on its separate connection. Adopt only a recent candidate
            # whose start timestamp belongs to this goal; map validation remains
            # mandatory before it can become the command reference.
            candidate=self.early_spline;self.early_spline=None
            if candidate is not None and time.monotonic()-self.early_at<=.3:
                self.on_spline(candidate)
    def on_spline(self,m):
        with self.lock:
            if not self.active or self.goal is None:
                try:
                    Spline(m.order,m.knots,[[p.x,p.y,p.z] for p in m.pos_pts],m.start_time.to_sec(),m.traj_id)
                    if abs(rospy.Time.now().to_sec()-m.start_time.to_sec())<=.3 and m.traj_id>self.last_id:
                        self.early_spline=copy.deepcopy(m);self.early_at=time.monotonic()
                except Exception:pass
                return
            if m.start_time<self.goal.header.stamp or m.traj_id<=self.last_id:return
            try:
                s=Spline(m.order,m.knots,[[p.x,p.y,p.z] for p in m.pos_pts],m.start_time.to_sec(),m.traj_id)
                if abs(rospy.Time.now().to_sec()-s.start)>.3:raise ValueError('stale/future spline')
                self.pending_spline=s;self.last_id=m.traj_id
            except Exception as e:self.fault=str(e)
    def fail(self,reason):
        self.fault=reason;self.active=False;self.spline=None;self.spline_goal=None;self.pending_spline=None;self.cancel.publish(Empty());status(self.stat,'FAULT',reason=reason)
    def validate(self):
        with self.lock:
            if not self.active or not fresh(self.odom,self.odom_at,self.odom_age):return
            current,pending,goal=self.spline,self.pending_spline,self.goal
            origin=xyz(self.odom.pose.pose.position)
        # Refresh the current trajectory's clearance lease before checking a new one.
        # A pending candidate never invalidates an otherwise usable command stream.
        for s in (current,pending):
            if s is None:continue
            t=min(s.duration,max(0.,rospy.Time.now().to_sec()-s.start))
            n=max(2,int(math.ceil((s.duration-t)*max(self.c['max_vel'],.1)/.04)))
            points=[origin]+[s.state(x)[0] for x in np.linspace(t,s.duration,min(n,1500))]
            try:
                response=self.check(path_of(points))
                with self.lock:
                    if not self.active:return
                    if s is current and self.spline is s:
                        if response.valid:self.checked_id=s.id;self.checked_at=time.monotonic()
                        else:self.fail('trajectory no longer clear: '+response.reason);return
                    elif self.pending_spline is s and self.goal is goal:
                        self.check_reason=response.reason
                        if response.valid:
                            self.spline=s;self.spline_goal=goal;self.pending_spline=None
                            self.checked_id=s.id;self.checked_at=time.monotonic()
            except Exception as e:
                with self.lock:self.check_reason='map check unavailable: '+str(e)

    def tick(self):
        with self.lock:
            if not self.active:status(self.stat,'FAULT' if self.fault else 'IDLE',reason=self.fault);return
            if not fresh(self.odom,self.odom_at,self.odom_age):
                self.fail('executor odometry stale');return
            if self.plan in ('FAULT','5'):
                self.fail('planner reported emergency');return
            if self.spline_goal is not self.goal and time.monotonic()-self.goal_at>3.:
                self.fail('no valid trajectory for current goal within 3s: '+self.check_reason);return
            if self.spline is None:
                if time.monotonic()-self.goal_at>3.:self.fail('no valid trajectory within 3s')
                else:status(self.stat,'PLANNING')
                return
            if time.monotonic()-self.plan_at>.3:
                self.fail("planner heartbeat lost during execution");return
            s=self.spline;now=rospy.Time.now().to_sec();t=now-s.start
            # EGO completes up to 10 ms before the mathematical endpoint. Allow
            # one 50 Hz tick there, but a running trajectory cannot survive a
            # return to WAIT_TARGET (also catches a missed one-shot FAULT).
            if self.plan in ('1','WAIT_TARGET') and t<s.duration-.02:
                self.fail('planner returned to WAIT_TARGET during execution');return
            if t<-.05:self.fail('trajectory clock rollback');return
            p,v,a=s.state(t)
            if np.linalg.norm(v)>self.c['max_vel']*1.001 or np.linalg.norm(a)>self.c['max_acc']*1.001:self.fail('trajectory limits');return
            if self.checked_id!=s.id or time.monotonic()-self.checked_at>.3:
                if time.monotonic()-self.goal_at>3. and self.checked_id is None:
                    self.fail(self.check_reason)
                else:status(self.stat,'VALIDATING',reason=self.check_reason)
                return
            if self.last_reference is not None:
                old_p,old_t=self.last_reference
                if np.linalg.norm(p-old_p)>.04+self.c['max_vel']*max(0.,now-old_t)*1.2:
                    self.fail('discontinuous trajectory reference');return
            self.last_reference=(p.copy(),now)
            at_end=t>=s.duration;terminal=at_end and self.spline_goal is self.goal
            if at_end:v,a=np.zeros(3),np.zeros(3)
            if terminal:
                if np.linalg.norm(xyz(self.odom.pose.pose.position)-p)<=.15 and np.linalg.norm(xyz(self.odom.twist.twist.linear))<=.1:
                    delta=wrap(yaw(quat(self.goal.pose.orientation))-self.yaw)
                    self.yaw=wrap(self.yaw+max(-self.yaw_step,min(self.yaw_step,delta)))
            cmd=NavigationCommand();cmd.header.stamp=rospy.Time.now();cmd.header.frame_id='drone_odom';put(cmd.position,p);put(cmd.velocity,v);put(cmd.acceleration,a);cmd.yaw=self.yaw;cmd.generation=self.gen;cmd.terminal=terminal
            self.pub.publish(cmd);status(self.stat,'HOLD' if terminal else 'RUNNING',trajectory_id=s.id)
