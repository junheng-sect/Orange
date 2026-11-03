import copy,math,threading,time,json
import numpy as np
import rospy,actionlib,tf2_ros
from std_msgs.msg import String,Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import ExecuteRouteAction,ExecuteRouteResult,ExecuteRouteFeedback,ExecuteRouteGoal
from .core import finite,yaw,heading,wrap
from .ros_common import *

class Mission:
    def __init__(self):
        self.c=config();self.odom=None;self.odom_at=0.;self.ready=False;self.gw_at=0.;self.reloc={};self.reloc_at=0.;self.map_result={};self.cancelled=False;self.dispatch_seq=0
        self.buf=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.buf)
        self.goal_pub=rospy.Publisher('/drone_nav/planner_goal',PoseStamped,queue_size=1)
        self.cancel_pub=rospy.Publisher('/drone_nav/cancel',Empty,queue_size=1)
        self.check=rospy.ServiceProxy('/drone_nav/check_trajectory',CheckTrajectory)
        rospy.Subscriber('/drone_nav/odom',Odometry,self.on_odom,queue_size=1)
        rospy.Subscriber('/drone_nav/gateway_status',String,self.on_gateway,queue_size=1)
        rospy.Subscriber('/drone_nav/cancel',Empty,self.on_cancel,queue_size=1)
        rospy.Subscriber('/drone_reloc/result',String,self.on_result,queue_size=1)
        rospy.Subscriber('/drone_reloc/status',String,self.on_reloc,queue_size=1)
        self.server=actionlib.SimpleActionServer('/drone_nav/execute_route',ExecuteRouteAction,execute_cb=self.execute,auto_start=False);self.server.start()
        self.client=actionlib.SimpleActionClient('/drone_nav/execute_route',ExecuteRouteAction)
        rospy.Subscriber('/drone_nav/goal',PoseStamped,self.on_single,queue_size=1)
    def on_odom(self,m):self.odom=m;self.odom_at=time.monotonic()
    def on_gateway(self,m):self.ready=read_status(m).get('state')=='READY';self.gw_at=time.monotonic()
    def on_cancel(self,_):self.cancelled=True
    def on_result(self,m):self.map_result=read_status(m)
    def on_reloc(self,m):self.reloc=read_status(m);self.reloc_at=time.monotonic()
    def on_single(self,m):
        # A 2D RViz click must have height assigned explicitly by the operator/tool.
        if m.pose.position.z==0. and not rospy.get_param('~allow_zero_height',False):rospy.logwarn('RViz goal requires explicit z');return
        goal=ExecuteRouteGoal(map_id=self.c['map_id'],waypoints=[m]);self.client.send_goal(goal)
    def map_ok(self):
        return self.map_result.get('confirmed') is True and self.map_result.get('map')==self.c['map_id'] and time.monotonic()-self.reloc_at<1.5 and self.reloc.get('valid') is True and self.reloc.get('state')=='LOCALIZED'
    def abort(self,why,preempt=False):
        self.cancel_pub.publish(Empty());r=ExecuteRouteResult(success=False,reason=why)
        (self.server.set_preempted if preempt else self.server.set_aborted)(r)
    def execute(self,g):
        self.cancelled=False
        if not self.c['map_id'] or g.map_id!=self.c['map_id'] or not g.waypoints or len(g.waypoints)>100:self.abort('map_id/route invalid');return
        if not self.map_ok() or not self.ready or time.monotonic()-self.gw_at>.3:self.abort('localization/gateway not ready');return
        try:
            transform=self.buf.lookup_transform('drone_map','drone_odom',rospy.Time(0),rospy.Duration(.2))
            if not fresh(transform,time.monotonic(),.5):raise ValueError('stale map transform')
            r=rotation(quat(transform.transform.rotation));t=xyz(transform.transform.translation)
            goals=[]
            for w in g.waypoints:
                if w.header.frame_id!='drone_map':raise ValueError('route frame must be drone_map')
                p=r.T@(finite(xyz(w.pose.position))-t);a=heading(r.T,yaw(quat(w.pose.orientation)))
                if np.any(p<finite(self.c['bounds_min'])) or np.any(p>finite(self.c['bounds_max'])):raise ValueError('route outside configured bounds')
                q=PoseStamped();q.header.frame_id='drone_odom';put(q.pose.position,p);q.pose.orientation.z=math.sin(a/2);q.pose.orientation.w=math.cos(a/2)
                if not self.check(path_of([p])).valid:raise ValueError('goal footprint unknown/occupied')
                goals.append(q)
        except Exception as e:self.abort(str(e));return
        for i,goal in enumerate(goals):
            self.cancelled=False;self.dispatch_seq+=1;goal.header.seq=self.dispatch_seq;goal.header.stamp=rospy.Time.now();self.goal_pub.publish(goal);start=time.monotonic();settled=None
            while not rospy.is_shutdown():
                if self.server.is_preempt_requested():self.abort('cancelled by client',True);return
                if self.cancelled or not self.ready or time.monotonic()-self.gw_at>.3 or not fresh(self.odom,self.odom_at) or not self.map_ok():self.abort('cancelled or navigation health lost');return
                try:
                    tr=self.buf.lookup_transform('drone_map','drone_odom',rospy.Time(0),rospy.Duration(.02))
                    if not fresh(tr,time.monotonic(),.5) or np.linalg.norm(xyz(tr.transform.translation)-t)>.01 or np.linalg.norm(rotation(quat(tr.transform.rotation))-r)>.01:raise ValueError('map transform reset')
                except Exception as e:self.abort(str(e));return
                o=self.odom;ok=np.linalg.norm(xyz(o.pose.pose.position)-xyz(goal.pose.position))<=.15 and np.linalg.norm(xyz(o.twist.twist.linear))<=.1 and abs(wrap(yaw(quat(o.pose.pose.orientation))-yaw(quat(goal.pose.orientation))))<=math.radians(5)
                settled=(settled or time.monotonic()) if ok else None
                self.server.publish_feedback(ExecuteRouteFeedback(waypoint_index=i,state='SETTLING' if ok else 'NAVIGATING'))
                if settled and time.monotonic()-settled>=1.:break
                if time.monotonic()-start>120:self.abort('waypoint timeout');return
                time.sleep(.02)
        # Cancel EGO/executor; gateway transitions into local braking/hold without resuming an old trajectory.
        self.cancel_pub.publish(Empty());self.server.set_succeeded(ExecuteRouteResult(success=True,reason='all waypoints settled'))
