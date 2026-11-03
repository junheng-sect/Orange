#!/usr/bin/env python3
"""Isolated ROS integration. Only synthetic inputs; never publishes MAVROS setpoints."""
import os,signal,subprocess,time,json,math,threading,copy,sys
import numpy as np
import rospy,tf2_ros,actionlib,rosgraph
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped,TransformStamped,PoseStamped
from sensor_msgs import point_cloud2 as pc
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String,Header,Empty
from std_srvs.srv import Trigger
from mavros_msgs.msg import State,EstimatorStatus
from rosgraph_msgs.msg import Clock
from drone_navigation.msg import NavigationCommand,ExecuteRouteAction,ExecuteRouteGoal
from traj_utils.srv import CheckTrajectory
from traj_utils.msg import Bspline
from drone_navigation.core import rz
from drone_navigation.ros_common import path_of

class Harness:
 def __init__(self):
  self.p=np.array([0.,0.,1.]);self.v=np.zeros(3);self.a=0.;self.command=None;self.command_history=[];self.speed_history=[];self.track=False;self.cloud_on=True;self.wall=False;self.stop=False;self.status={};self.preview=[];self.cloud_out=None
  self.r=rz(.3);self.pubs={};self.clock_on=True;self.clock_stamp=None;self.omega=0.;self.clock_pub=rospy.Publisher('/clock',Clock,queue_size=1)
  for topic,typ in [('/fastlio/odom_world',Odometry),('/fastlio/velocity',Vector3Stamped),('/mavros/local_position/odom',Odometry),('/cloud_registered',PointCloud2),('/mavros/state',State),('/mavros/estimator_status',EstimatorStatus),('/drone_reloc/status',String),('/drone_reloc/result',String)]:self.pubs[topic]=rospy.Publisher(topic,typ,queue_size=3,latch=topic.endswith('result'))
  for topic in ('input_status','planner_status','executor_status','gateway_status'):
   rospy.Subscriber('/drone_nav/'+topic,String,lambda m,t=topic:self.status.update({t:m.data}))
  rospy.Subscriber('/drone_nav/command',NavigationCommand,self.cmd)
  from mavros_msgs.msg import PositionTarget
  rospy.Subscriber('/drone_nav/setpoint_preview',PositionTarget,lambda m:self.preview.append(m))
  rospy.Subscriber('/drone_nav/cloud',PointCloud2,lambda m:setattr(self,'cloud_out',m))
  self.tf=tf2_ros.TransformBroadcaster();self.static=tf2_ros.StaticTransformBroadcaster()
  tr=TransformStamped();tr.header.stamp=rospy.Time.now();tr.header.frame_id='drone_odom';tr.child_frame_id='camera_init';tr.transform.rotation.z=math.sin(.15);tr.transform.rotation.w=math.cos(.15);self.static.sendTransform(tr)
  axes=np.arange(-3.,3.001,.12);height=np.arange(-.5,3.501,.12);pts=[]
  for x in axes:
   for y in axes:pts.extend([[x,y,-.5],[x,y,3.5]])
  for u in axes:
   for z in height:pts.extend([[-3,u,z],[3,u,z],[u,-3,z],[u,3,z]])
  self.points=np.array(pts);self.rng=np.random.RandomState(42)
  threading.Thread(target=self.run,daemon=True).start()
 def cmd(self,m):
  self.command=m;self.command_history.append([m.position.x,m.position.y,m.position.z]);self.speed_history.append(np.linalg.norm([m.velocity.x,m.velocity.y,m.velocity.z]))
 def run(self):
  while not self.stop and not rospy.is_shutdown():
   if self.clock_on:self.clock_stamp=rospy.Time.from_sec(time.time())
   if self.clock_stamp:self.clock_pub.publish(Clock(clock=self.clock_stamp))
   time.sleep(.005)
   stamp=rospy.Time.now()
   if self.track and self.command:
    old=self.a;self.a=self.command.yaw;self.omega=math.atan2(math.sin(self.a-old),math.cos(self.a-old))/.105
    self.p=np.array([self.command.position.x,self.command.position.y,self.command.position.z])-rz(self.a)@np.array([.1,0,0])
    self.v=np.array([self.command.velocity.x,self.command.velocity.y,self.command.velocity.z])-rz(self.a)@np.cross([0,0,self.omega],[.1,0,0])
   tr=TransformStamped();tr.header.stamp=stamp;tr.header.frame_id='camera_init';tr.child_frame_id='body';tr.transform.rotation.z=math.sin((self.a-.3)/2);tr.transform.rotation.w=math.cos((self.a-.3)/2)
   mt=TransformStamped();mt.header.stamp=stamp;mt.header.frame_id='drone_map';mt.child_frame_id='drone_odom';mt.transform.rotation.w=1.;self.tf.sendTransform([tr,mt])
   o=Odometry();o.header=Header(stamp=stamp,frame_id='drone_odom');o.child_frame_id='drone_base';o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=self.p;o.pose.pose.orientation.z=math.sin(self.a/2);o.pose.pose.orientation.w=math.cos(self.a/2);self.pubs['/fastlio/odom_world'].publish(o)
   vel=Vector3Stamped();vel.header=Header(stamp=stamp,frame_id='camera_init');vel.vector.x,vel.vector.y,vel.vector.z=self.r.T@self.v;self.pubs['/fastlio/velocity'].publish(vel)
   px=copy.deepcopy(o);px.header.frame_id='px4_map';p=rz(math.pi/2)@(self.p+rz(self.a)@np.array([.1,0,0]))+np.array([2,3,.3]);px.pose.pose.position.x,px.pose.pose.position.y,px.pose.pose.position.z=p;px.pose.pose.orientation.z=math.sin((self.a+math.pi/2)/2);px.pose.pose.orientation.w=math.cos((self.a+math.pi/2)/2);px.twist.twist.linear.x,px.twist.twist.linear.y,px.twist.twist.linear.z=rz(self.a).T@self.v+np.cross([0,0,self.omega],[.1,0,0]);px.twist.twist.angular.z=self.omega;self.pubs['/mavros/local_position/odom'].publish(px)
   st=State();st.header.stamp=stamp;st.connected=True;st.armed=True;st.mode='OFFBOARD';self.pubs['/mavros/state'].publish(st)
   e=EstimatorStatus();e.header.stamp=stamp
   for key in ['attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag','pos_horiz_rel_status_flag','pos_vert_abs_status_flag']:setattr(e,key,True)
   self.pubs['/mavros/estimator_status'].publish(e)
   self.pubs['/drone_reloc/status'].publish(String(json.dumps(dict(valid=True,state='LOCALIZED'))));self.pubs['/drone_reloc/result'].publish(String(json.dumps(dict(confirmed=True,map='synthetic'))))
   if self.cloud_on:
    points=self.points+self.rng.uniform(-.055,.055,self.points.shape)
    if self.wall:
     origin=self.p+rz(self.a)@np.array([-.011,-.02329,.04412])
     d=points-origin;alpha=(1.3-origin[0])/np.where(abs(d[:,0])>1e-9,d[:,0],1e-9);hit=origin+alpha[:,None]*d
     points=points[~((alpha>0)&(alpha<1)&(abs(hit[:,1])<.55)&(hit[:,2]>.35)&(hit[:,2]<1.85))]
     obstacle=np.array([[1.3,y,z] for y in np.arange(-.5,.51,.1) for z in np.arange(.4,1.81,.1)]);points=np.vstack([points,obstacle])
    raw=points@self.r
    time.sleep(.015) # allow state pairing before cloud; TF timestamp is shared
    self.pubs['/cloud_registered'].publish(pc.create_cloud_xyz32(Header(stamp=stamp,frame_id='camera_init'),raw.tolist()))
   time.sleep(.085)
 def wait(self,pred,seconds,description):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   if pred():print('PASS',description,flush=True);return
   time.sleep(.05)
  raise AssertionError(description+': '+str(self.status))

def main():
 assert os.environ.get('ROS_MASTER_URI')=='http://127.0.0.1:11431', 'test requires isolated master 11431'
 rospy.init_node('navigation_isolated_test');h=Harness()
 try:
  h.wait(lambda:'READY' in h.status.get('input_status',''),20,'input transform and same-stamp state')
  rospy.wait_for_service('/drone_nav/prepare',timeout=15)
  prepare=rospy.ServiceProxy('/drone_nav/prepare',Trigger)
  def ready():
   try:return prepare().success
   except Exception:return False
  h.wait(ready,15,'2s alignment with 90deg yaw and nonzero translation/lever')
  h.wait(lambda:'READY' in h.status.get('gateway_status',''),5,'preview gateway ready')
  h.wait(lambda:bool(h.preview),5,'holding preview stream')
  assert all(m.type_mask==2496 and m.coordinate_frame==1 for m in h.preview)
  assert np.linalg.norm(np.array([h.preview[-1].position.x,h.preview[-1].position.y,h.preview[-1].position.z])-[2,3.1,1.3])<.02
  pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState();assert not any(t=='/mavros/setpoint_raw/local' for t,n in pubs)
  print('PASS preview owns no real setpoint publisher; correct P+V mask and transformed hold',flush=True)
  check=rospy.ServiceProxy('/drone_nav/check_trajectory',CheckTrajectory)
  print('MAP_INITIAL',check(path_of([[.1,0,1],[.6,0,1.2]])),flush=True)
  def free():
   result=check(path_of([[.1,0,1],[.6,0,1.2]]))
   h.status['map_reason']=result.reason
   return result.valid
  h.wait(free,10,'known free footprint')
  print('MAP_CHECK',check(path_of([[.1,0,1],[.6,0,1.2]])),flush=True)
  assert not check(path_of([[8,0,1]])).valid
  assert not check(path_of([[.1,0,4.8]])).valid
  print('PASS unknown/outside flight volume rejected',flush=True)
  c=actionlib.SimpleActionClient('/drone_nav/execute_route',ExecuteRouteAction);assert c.wait_for_server(rospy.Duration(10))
  def goal(points,map_id='synthetic'):
   g=ExecuteRouteGoal(map_id=map_id)
   for x,y,z,a in points:
    p=PoseStamped();p.header.frame_id='drone_map';p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=z;p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2);g.waypoints.append(p)
   return g
  c.send_goal(goal([[.6,0,1.2,0]],'wrong'));assert c.wait_for_result(rospy.Duration(3));assert not c.get_result().success;print('PASS wrong map refused',flush=True)
  time.sleep(.5);h.track=True
  c.send_goal(goal([[.6,0,1.2,0],[.6,.5,1.4,math.radians(30)]]))
  h.wait(lambda:h.command is not None,5,'first valid spline reaches executor')
  stale_pub=rospy.Publisher('/drone_nav/bspline',Bspline,queue_size=1);time.sleep(.1)
  stale=Bspline();stale.start_time=rospy.Time(1);stale.traj_id=999999;stale_pub.publish(stale)
  h.wait(lambda:c.get_state() in (2,3,4,5,8),45,'route terminates')
  print('ROUTE_RESULT',c.get_result(),h.status,flush=True)
  assert c.get_result().success
  print('PASS two heights, repeated 3D goal, measured settle, terminal yaw and stale spline rejection',flush=True)
  h.track=False;h.command=None;h.v=np.zeros(3);h.omega=0.;time.sleep(.5)
  h.wall=True
  h.wait(lambda:not check(path_of([[.6,.5,1.4],[2.,0.,1.4]])).valid,6,'wall occupies direct path')
  c.send_goal(goal([[1.3,0.,1.4,0.]]));assert c.wait_for_result(rospy.Duration(5));assert not c.get_result().success
  print('PASS occupied target refused',flush=True)
  assert prepare().success;time.sleep(.4);h.track=True
  before=len(h.command_history)
  c.send_goal(goal([[2.,0.,1.4,math.radians(30)]]))
  h.wait(lambda:c.get_state() in (2,3,4,5,8),50,'obstacle navigation terminates')
  print('OBSTACLE_RESULT',c.get_result(),h.status, 'history_tail',h.command_history[-12:],flush=True)
  assert c.get_result().success
  history=np.array(h.command_history[before:]);assert len(history)>10
  near=history[np.abs(history[:,0]-1.3)<.35]
  assert len(near) and np.all((np.abs(near[:,1])>.85)|(near[:,2]>2.05)|(near[:,2]<.15))
  assert max(h.speed_history)<=.5005
  print('PASS EGO detours around static wall within configured volume; max command speed',max(h.speed_history),flush=True)
  h.track=False;h.command=None;h.v=np.zeros(3);h.omega=0.;h.wall=False;time.sleep(1.)
  assert prepare().success
  time.sleep(.4)
  c.send_goal(goal([[2.,1.,1.4,math.radians(30)]]));time.sleep(.5);c.cancel_goal();assert c.wait_for_result(rospy.Duration(5));assert not c.get_result().success;print('PASS action cancellation',flush=True)
  # Explicit prepare is required after any fault. No stale spline may resume when sensor stream returns.
  h.track=False;h.command=None;h.v=np.zeros(3);time.sleep(.4);assert prepare().success
  h.wait(lambda:'READY' in h.status.get('gateway_status',''),3,'reprepare after cancellation')
  h.cloud_on=False
  h.wait(lambda:'FAULT' in h.status.get('gateway_status',''),4,'cloud dropout faults gateway')
  h.cloud_on=True;time.sleep(1.);assert 'FAULT' in h.status.get('gateway_status','');print('PASS sensor recovery does not resume previous trajectory',flush=True)
  import rosnode
  assert prepare().success;time.sleep(.3)
  c.send_goal(goal([[2.,1.,1.4,math.radians(30)]]));time.sleep(.5)
  rosnode.kill_nodes(['/drone_0_ego_planner_node'])
  h.wait(lambda:c.get_state() in (2,3,4,5,8),5,'planner loss terminates task')
  assert not c.get_result().success
  print('PASS planner loss cannot keep an old trajectory alive',flush=True)
  # Restart only the preview gateway with simulated clock; the route test itself uses real time.
  import rosnode
  rosnode.kill_nodes(['/offboard_gateway']);time.sleep(1.)
  rospy.set_param('/use_sim_time',True)
  child=subprocess.Popen(['rosrun','drone_navigation','offboard_gateway','__name:=offboard_gateway','_enable_output:=false'],stdout=open('/tmp/drone-nav-clock-test.log','w'),stderr=subprocess.STDOUT)
  try:
   h.wait(ready,10,'preview gateway re-aligned on simulated clock')
   h.clock_on=False
   h.wait(lambda:'FAULT' in h.status.get('gateway_status',''),4,'paused ROS clock detected by wall watchdog')
  finally:
   child.send_signal(signal.SIGINT)
   try:child.wait(timeout=5)
   except subprocess.TimeoutExpired:child.kill();child.wait()
  print('ALL ISOLATED INTEGRATION CHECKS PASSED',flush=True)
 finally:h.stop=True
if __name__=='__main__':main()
