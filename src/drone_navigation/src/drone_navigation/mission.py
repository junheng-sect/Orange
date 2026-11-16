import copy,math,threading,time,json
import numpy as np
import rospy,actionlib,tf2_ros
from std_msgs.msg import String,Empty
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import ExecuteRouteAction,ExecuteRouteResult,ExecuteRouteFeedback,ExecuteRouteGoal
from drone_navigation.msg import ExecutePathAction, ExecutePathResult, ExecutePathFeedback, NavigationCommand
from .core import finite,yaw,heading,wrap,check_goal_bounds
from .ros_common import *

class Mission:
    def __init__(self):
        self.c=config();self.odom=None;self.odom_at=0.;self.ready=False;self.gw_at=0.;self.reloc={};self.reloc_at=0.;self.map_result={};self.cancelled=False
        self.last_completed_goal=None
        self.task_lock = threading.Lock()
        self.command = None
        self.executor_state = {}
        self.path_pub = rospy.Publisher('/drone_nav/planner_path', Path, queue_size=1)
        rospy.Subscriber('/drone_nav/command', NavigationCommand, self.on_command, queue_size=1)
        rospy.Subscriber('/drone_nav/executor_status', String, self.on_executor, queue_size=1)
        self.buf=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.buf)
        self.goal_pub=rospy.Publisher('/drone_nav/planner_goal',PoseStamped,queue_size=1)
        self.cancel_pub=rospy.Publisher('/drone_nav/cancel',Empty,queue_size=1)
        self.arrived_pub=rospy.Publisher('/drone_nav/arrived',Empty,queue_size=1)
        self.check=rospy.ServiceProxy('/drone_nav/check_trajectory',CheckTrajectory)
        rospy.Subscriber('/drone_nav/odom',Odometry,self.on_odom,queue_size=1)
        rospy.Subscriber('/drone_nav/gateway_status',String,self.on_gateway,queue_size=1)
        rospy.Subscriber('/drone_nav/cancel',Empty,self.on_cancel,queue_size=1)
        rospy.Subscriber('/drone_reloc/result',String,self.on_result,queue_size=1)
        rospy.Subscriber('/drone_reloc/status',String,self.on_reloc,queue_size=1)
        self.server=actionlib.SimpleActionServer('/drone_nav/execute_route',ExecuteRouteAction,execute_cb=self.execute,auto_start=False);self.server.start()
        self.path_server = actionlib.SimpleActionServer(
            '/drone_nav/execute_path', ExecutePathAction, execute_cb=self.execute_path, auto_start=False)
        self.path_server.start()
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
    def on_command(self, message):
        self.command = message

    def on_executor(self, message):
        self.executor_state = read_status(message)

    def execute(self, goal):
        if not self.task_lock.acquire(blocking=False):
            self.server.set_aborted(ExecuteRouteResult(success=False, reason='another navigation task is active'))
            return
        try:
            self.execute_route(goal)
        finally:
            self.task_lock.release()

    def path_transform(self):
        transform = self.buf.lookup_transform('drone_map', 'drone_odom', rospy.Time(0), rospy.Duration(.02))
        if not fresh(transform, time.monotonic(), .5):
            raise ValueError('stale map transform')
        return rotation(quat(transform.transform.rotation)), xyz(transform.transform.translation)

    def path_health(self):
        if (self.cancelled or not self.ready or time.monotonic() - self.gw_at > .3 or
                not self.map_ok() or not fresh(self.odom, self.odom_at, odom_timeout(self.c))):
            raise ValueError('cancelled or navigation health lost')

    def execute_path(self, goal):
        server = self.path_server
        if not self.task_lock.acquire(blocking=False):
            server.set_aborted(ExecutePathResult(success=False, reason='another navigation task is active'))
            return
        started = False
        self.cancelled = False
        try:
            if not goal.map_id or goal.map_id != self.c['map_id'] or not 2 <= len(goal.points) <= 50:
                raise ValueError('invalid path map or point count')
            self.path_health()
            r, t = self.path_transform()
            o = self.odom
            p0 = xyz(o.pose.pose.position)
            points = [r.T @ (finite(xyz(p)) - t) for p in goal.points]
            check_goal_bounds(points, self.c)
            if any(np.linalg.norm(b - a) < .2 for a, b in zip([p0] + points, points)):
                raise ValueError('adjacent path points must be at least 0.2 m apart')
            if np.linalg.norm(xyz(o.twist.twist.linear)) > .1:
                raise ValueError('path must start from a settled pose')
            if server.is_preempt_requested() or self.cancelled:
                raise ValueError('path cancelled before start')
            path = path_of(points)
            self.path_pub.publish(path)
            started = True
            start = time.monotonic()
            settled = None
            index = 0
            while not rospy.is_shutdown():
                if server.is_preempt_requested():
                    raise ValueError('path cancelled by client')
                self.path_health()
                current_r, current_t = self.path_transform()
                if np.linalg.norm(current_r - r) > .01 or np.linalg.norm(current_t - t) > .01:
                    raise ValueError('map transform reset')
                o, cmd = self.odom, self.command
                p = xyz(o.pose.pose.position)
                if index < len(points)-1 and np.linalg.norm(p - points[index]) <= .5:
                    index += 1
                terminal = (cmd is not None and cmd.generation == path.header.seq + PATH_GENERATION and cmd.header.stamp >= path.header.stamp
                            and fresh(cmd, time.monotonic(), .3) and cmd.terminal
                            and np.linalg.norm(xyz(cmd.position) - points[-1]) < 1e-4)
                if terminal and index != len(points)-1:
                    raise ValueError('guide point %d was not reached within 0.5 m' % (index+1))
                ok = (terminal and np.linalg.norm(p - points[-1]) <= .15 and
                      np.linalg.norm(xyz(o.twist.twist.linear)) <= .1 and
                      abs(wrap(yaw(quat(o.pose.pose.orientation)) - cmd.yaw)) <= math.radians(5))
                now = time.monotonic()
                settled = (settled if settled is not None else now) if ok else None
                state = 'SETTLING' if ok else self.executor_state.get('state', 'PLANNING')
                server.publish_feedback(ExecutePathFeedback(point_index=index, state=state))
                if settled is not None and now - settled >= 1.:
                    self.last_completed_goal = points[-1].copy()
                    self.arrived_pub.publish(Empty())
                    self.cancel_pub.publish(Empty())
                    server.set_succeeded(ExecutePathResult(success=True, reason='path endpoint settled'))
                    return
                if now - start >= 90.:
                    raise ValueError('path timeout (90 s)')
                time.sleep(.02)
            raise ValueError('ROS shutdown')
        except Exception as error:
            if started:
                self.cancel_pub.publish(Empty())
            result = ExecutePathResult(success=False, reason=str(error))
            (server.set_preempted if server.is_preempt_requested() else server.set_aborted)(result)
        finally:
            self.task_lock.release()

    def execute_route(self,g):
        self.cancelled=False
        if not self.c['map_id'] or g.map_id!=self.c['map_id'] or not g.waypoints or len(g.waypoints)>100:self.abort('map_id/route invalid');return
        if not self.map_ok() or not self.ready or time.monotonic()-self.gw_at>.3:self.abort('localization/gateway not ready');return
        try:
            transform=self.buf.lookup_transform('drone_map','drone_odom',rospy.Time(0),rospy.Duration(.2))
            if not fresh(transform,time.monotonic(),.5):raise ValueError('stale map transform')
            r=rotation(quat(transform.transform.rotation));t=xyz(transform.transform.translation)
            goals=[]
            for index,w in enumerate(g.waypoints):
                if w.header.frame_id!='drone_map':raise ValueError('route frame must be drone_map')
                p=r.T@(finite(xyz(w.pose.position))-t);a=heading(r.T,yaw(quat(w.pose.orientation)))
                check_goal_bounds([p], self.c, ['waypoint #%d' % (index+1)])
                q=PoseStamped();q.header.frame_id='drone_odom';put(q.pose.position,p);q.pose.orientation.z=math.sin(a/2);q.pose.orientation.w=math.cos(a/2)
                if not self.check(path_of([p])).valid:raise ValueError('goal footprint unknown/occupied')
                goals.append(q)
        except Exception as e:self.abort(str(e));return
        for i,goal in enumerate(goals):
            # EGO rejects translations shorter than 0.2 m. Do not let the 15 cm
            # arrival tolerance claim success before any trajectory exists. The
            # only exception is a stationary turn at an already settled anchor.
            if not fresh(self.odom,self.odom_at,odom_timeout(self.c)):self.abort('odometry stale before waypoint');return
            p=xyz(goal.pose.position);distance=np.linalg.norm(xyz(self.odom.pose.pose.position)-p)
            guarded_arrival = self.c.get('allow_short_goals', False)
            short_goal = guarded_arrival and distance < .2
            if short_goal and np.linalg.norm(xyz(self.odom.twist.twist.linear)) > .05:
                self.abort('short goal requires a settled start at <=0.05 m/s');return
            if not guarded_arrival and distance<.2 and (self.last_completed_goal is None or
                    np.linalg.norm(p-self.last_completed_goal)>1e-6 or distance>.15 or
                    np.linalg.norm(xyz(self.odom.twist.twist.linear))>.1):
                self.abort('translation below 0.2m requires an already settled identical waypoint');return
            # rospy supplies Header.seq on this single publisher. Gateway and executor
            # use the same received sequence as NavigationCommand.generation; never
            # assign a local seq that rospy will overwrite during serialization.
            self.cancelled=False;goal.header.stamp=rospy.Time.now();self.goal_pub.publish(goal);start=time.monotonic();settled=None
            while not rospy.is_shutdown():
                if self.server.is_preempt_requested():self.abort('cancelled by client',True);return
                if self.cancelled or not self.ready or time.monotonic()-self.gw_at>.3 or not fresh(self.odom,self.odom_at,odom_timeout(self.c)) or not self.map_ok():self.abort('cancelled or navigation health lost');return
                try:
                    tr=self.buf.lookup_transform('drone_map','drone_odom',rospy.Time(0),rospy.Duration(.02))
                    if not fresh(tr,time.monotonic(),.5) or np.linalg.norm(xyz(tr.transform.translation)-t)>.01 or np.linalg.norm(rotation(quat(tr.transform.rotation))-r)>.01:raise ValueError('map transform reset')
                except Exception as e:self.abort(str(e));return
                o = self.odom
                ok = (np.linalg.norm(xyz(o.pose.pose.position) - p) <= (.04 if short_goal else .15)
                      and np.linalg.norm(xyz(o.twist.twist.linear)) <= (.05 if short_goal else .1)
                      and abs(wrap(yaw(quat(o.pose.pose.orientation)) - yaw(quat(goal.pose.orientation)))) <= math.radians(5))
                if guarded_arrival:
                    cmd = self.command
                    ok = (ok and cmd is not None and cmd.generation == goal.header.seq
                          and cmd.header.stamp >= goal.header.stamp and fresh(cmd, time.monotonic(), .3)
                          and cmd.terminal and np.linalg.norm(xyz(cmd.position) - p) <= (.002 if short_goal else .05)
                          and abs(wrap(cmd.yaw - yaw(quat(goal.pose.orientation)))) <= math.radians(5))
                settled=(settled or time.monotonic()) if ok else None
                self.server.publish_feedback(ExecuteRouteFeedback(waypoint_index=i,state='SETTLING' if ok else 'NAVIGATING'))
                if settled and time.monotonic()-settled>=1.:break
                if time.monotonic()-start>120:self.abort('waypoint timeout');return
                time.sleep(.02)
            self.last_completed_goal=xyz(goal.pose.position).copy()
        # Cancel EGO/executor without resuming an old trajectory. arrived tells the gateway this stop is a
        # normal arrival, so it holds the final trajectory point instead of braking where the vehicle is.
        self.arrived_pub.publish(Empty());self.cancel_pub.publish(Empty());self.server.set_succeeded(ExecuteRouteResult(success=True,reason='all waypoints settled'))
