import copy,threading,time
from collections import deque
import numpy as np
import rospy,tf2_ros,message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc
from std_msgs.msg import String
from .core import reference_state,rotation,finite,shift_pose_covariance
from .ros_common import *

class Inputs:
    def __init__(self):
        self.c=config();self.age=odom_timeout(self.c);self.lock=threading.RLock();self.records=deque(maxlen=30);self.last=0.;self.bad='waiting';self.cloud_receipt=0.;self.pending={};self.state_pending={};self.px_queue=deque(maxlen=30)
        self.buf=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.buf)
        self.odom_pub=rospy.Publisher('/drone_nav/odom',Odometry,queue_size=5)
        self.sensor_pub=rospy.Publisher('/drone_nav/sensor_odom',Odometry,queue_size=5)
        self.cloud_pub=rospy.Publisher('/drone_nav/cloud',PointCloud2,queue_size=1)
        self.stat=rospy.Publisher('/drone_nav/input_status',String,queue_size=1)
        self.subs=[message_filters.Subscriber('/fastlio/odom_world',Odometry),message_filters.Subscriber('/fastlio/velocity',Vector3Stamped)]
        self.sync=message_filters.TimeSynchronizer(self.subs,30);self.sync.registerCallback(self.on_lio)
        rospy.Subscriber('/mavros/local_position/odom',Odometry,self.on_px,queue_size=30)
        rospy.Subscriber('/cloud_registered',PointCloud2,self.on_cloud,queue_size=1,buff_size=16777216)
        loop(self.tick,10)
    def on_lio(self,o,v):
        with self.lock:
            self.state_pending[o.header.stamp.to_sec()]=(o,v)
            while len(self.state_pending)>5:self.state_pending.pop(min(self.state_pending))
            self.drain()
    def on_px(self,m):
        with self.lock:
            self.px_queue.append(m);self.drain()
    def drain(self):
        for stamp in sorted(list(self.state_pending)):
            if stamp<=self.last:
                self.state_pending.pop(stamp);continue
            if not self.px_queue:return
            px=min(self.px_queue,key=lambda m:abs(m.header.stamp.to_sec()-stamp))
            if abs(px.header.stamp.to_sec()-stamp)<=.08:
                o,v=self.state_pending.pop(stamp);self.on_state(o,v,px)
    def fail(self,e):
        with self.lock:self.bad=str(e);self.records.clear()
    def on_state(self,o,v,px):
        try:
            if o.header.frame_id!='drone_odom' or o.child_frame_id!='drone_base' or v.header.frame_id!='camera_init' or px.header.frame_id!='px4_map' or px.child_frame_id!='drone_base':raise ValueError('state frames')
            if o.header.stamp!=v.header.stamp or not all(fresh(m,time.monotonic(),self.age) for m in (o,v,px)):raise ValueError('state timestamps')
            stamp=o.header.stamp.to_sec()
            with self.lock:
                if stamp<=self.last:raise ValueError('LIO timestamp reset/repeat')
                r=rotation(quat(o.pose.pose.orientation));a,t=tf_matrix(self.buf,'drone_odom','camera_init',o.header.stamp)
                p,vel=reference_state(xyz(o.pose.pose.position),a@xyz(v.vector),r,xyz(px.twist.twist.angular),self.c['lever_body'])
                if self.records:
                    prev=self.records[-1]
                    if np.linalg.norm(p-xyz(prev[1].pose.pose.position))>.08+2.*(stamp-self.last):raise ValueError('LIO jump %.3fm in %.3fs' % (np.linalg.norm(p-xyz(prev[1].pose.pose.position)),stamp-self.last))
                nav=copy.deepcopy(o);nav.child_frame_id='drone_nav_base';put(nav.pose.pose.position,p);put(nav.twist.twist.linear,r.T@vel);put(nav.twist.twist.angular,xyz(px.twist.twist.angular));nav.pose.covariance=shift_pose_covariance(o.pose.covariance,r@finite(self.c["lever_body"]))
                self.records.append((stamp,nav,xyz(o.pose.pose.position),r,a,t));self.last=stamp;self.bad='';self.odom_pub.publish(nav)
                pending=self.pending.pop(stamp,None)
                if pending is not None:self.on_cloud(pending)
        except Exception as e:self.fail(e)
    def on_cloud(self,m):
        try:
            if m.header.frame_id!='camera_init' or not fresh(m,time.monotonic(),self.age):raise ValueError('cloud frame/stamp')
            with self.lock:
                records=[x for x in self.records if abs(x[0]-m.header.stamp.to_sec())<1e-6]
                if not records:
                    self.pending[m.header.stamp.to_sec()]=m
                    while len(self.pending)>3:self.pending.pop(min(self.pending))
                    return
                _,nav,imu,r,a,t=records[-1]
                points=np.array(list(pc.read_points(m,field_names=('x','y','z'),skip_nans=True)),dtype=float).reshape(-1,3)
                points=points[np.isfinite(points).all(axis=1)]@a.T+t
                local=(points-xyz(nav.pose.pose.position))@r
                body=finite(self.c['body_half_extent'])
                points=points[~np.all(np.abs(local)<=body,axis=1)]
                if not len(points):raise ValueError('empty cloud after filtering')
                origin=imu+a@rotation(quat(self.buf.lookup_transform('camera_init','body',m.header.stamp,rospy.Duration(.04)).transform.rotation))@finite(self.c['sensor_offset_imu'])
                sensor=copy.deepcopy(nav);put(sensor.pose.pose.position,origin)
                self.sensor_pub.publish(sensor)
                from std_msgs.msg import Header
                h=Header(stamp=m.header.stamp,frame_id='drone_odom')
                self.cloud_pub.publish(pc.create_cloud_xyz32(h,points.tolist()));self.cloud_receipt=time.monotonic();self.bad=''
        except Exception as e:self.fail(e)
    def tick(self):
        with self.lock:
            ok=self.records and fresh(self.records[-1][1],time.monotonic(),self.age) and time.monotonic()-self.cloud_receipt<self.age and not self.bad
            status(self.stat,'READY' if ok else 'FAULT',reason=self.bad or ('stale inputs' if not ok else ''))
