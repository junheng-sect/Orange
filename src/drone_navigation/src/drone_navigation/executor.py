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
        self.lock=threading.RLock();self.spline=None;self.goal=None;self.odom=None;self.odom_at=0.;self.active=False;self.gen=0;self.yaw=0.;self.last_id=-1;self.plan_at=0.;self.plan='';self.fault='';self.goal_at=0.;self.c=config();self.checked_id=None;self.checked_at=0.;self.check_reason='waiting for map validation';self.last_reference=None
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
        with self.lock:self.active=False;self.spline=None;self.goal=None
    def on_goal(self,m):
        with self.lock:
            self.active=True;self.spline=None;self.goal=m;self.goal_at=time.monotonic();self.gen=m.header.seq;self.fault='';self.checked_id=None;self.checked_at=0.;self.last_reference=None
            if self.odom is not None:self.yaw=yaw(quat(self.odom.pose.pose.orientation))
    def on_spline(self,m):
        with self.lock:
            if not self.active or self.goal is None or m.start_time<self.goal.header.stamp or m.traj_id<=self.last_id:return
            try:
                s=Spline(m.order,m.knots,[[p.x,p.y,p.z] for p in m.pos_pts],m.start_time.to_sec(),m.traj_id)
                if abs(rospy.Time.now().to_sec()-s.start)>.3:raise ValueError('stale/future spline')
                self.spline=s;self.last_id=m.traj_id
            except Exception as e:self.fault=str(e)
    def fail(self,reason):
        self.fault=reason;self.active=False;self.spline=None;self.cancel.publish(Empty());status(self.stat,'FAULT',reason=reason)
    def validate(self):
        with self.lock:
            if not self.active or self.spline is None or not fresh(self.odom,self.odom_at):return
            s=self.spline;t=rospy.Time.now().to_sec()-s.start
            points=[xyz(self.odom.pose.pose.position)]
        n=max(2,int(math.ceil(max(0.,s.duration-t)*max(self.c['max_vel'],.1)/.04)))
        points += [s.state(x)[0] for x in np.linspace(max(t,0.),s.duration,min(n,1500))]
        try:
            response=self.check(path_of(points))
            with self.lock:
                if not self.active or self.spline is not s:return
                self.check_reason=response.reason
                if response.valid:self.checked_id=s.id;self.checked_at=time.monotonic()
                elif self.checked_id==s.id:self.fail('trajectory no longer clear: '+response.reason)
        except Exception as e:
            with self.lock:self.check_reason='map check unavailable: '+str(e)

    def tick(self):
        with self.lock:
            if not self.active:status(self.stat,'FAULT' if self.fault else 'IDLE',reason=self.fault);return
            if not fresh(self.odom,self.odom_at):
                self.fail('executor odometry stale');return
            if self.plan in ('FAULT','5'):
                self.fail('planner reported emergency');return
            if self.spline is None:
                if time.monotonic()-self.goal_at>3.:self.fail('no valid trajectory within 3s')
                else:status(self.stat,'PLANNING')
                return
            if time.monotonic()-self.plan_at>.3:
                self.fail("planner heartbeat lost during execution");return
            s=self.spline;now=rospy.Time.now().to_sec();t=now-s.start
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
            terminal=t>=s.duration
            if terminal:
                v=np.zeros(3)
                if np.linalg.norm(xyz(self.odom.pose.pose.position)-p)<=.15 and np.linalg.norm(xyz(self.odom.twist.twist.linear))<=.1:
                    delta=wrap(yaw(quat(self.goal.pose.orientation))-self.yaw)
                    self.yaw=wrap(self.yaw+max(-math.radians(10)*.02,min(math.radians(10)*.02,delta)))
            cmd=NavigationCommand();cmd.header.stamp=rospy.Time.now();cmd.header.frame_id='drone_odom';put(cmd.position,p);put(cmd.velocity,v);cmd.yaw=self.yaw;cmd.generation=self.gen;cmd.terminal=terminal
            self.pub.publish(cmd);status(self.stat,'HOLD' if terminal else 'RUNNING',trajectory_id=s.id)
