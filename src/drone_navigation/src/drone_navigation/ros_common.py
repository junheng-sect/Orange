import copy,json,math,threading,time
import numpy as np
import rospy
from std_msgs.msg import String,Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from .core import rotation,yaw,finite


def config():return rospy.get_param('/drone_nav/config')
def xyz(v):return np.array([v.x,v.y,v.z],dtype=float)
def quat(q):return [q.x,q.y,q.z,q.w]
def put(v,a):v.x,v.y,v.z=map(float,a)
def fresh(m,receipt,age=.3):
    if m is None or time.monotonic()-receipt>age:return False
    stamp=m.header.stamp.to_sec();delta=rospy.Time.now().to_sec()-stamp
    return stamp>0 and -.05<=delta<=age

def status(pub,state,**kw):
    kw.update(state=state,stamp=rospy.Time.now().to_sec());pub.publish(String(json.dumps(kw,allow_nan=False)))

def read_status(m):
    try:return json.loads(m.data)
    except (ValueError,TypeError):return {}

def loop(fn,hz=50):
    def run():
        while not rospy.is_shutdown():
            start=time.monotonic()
            try:fn()
            except Exception as e:rospy.logerr_throttle(2.,'%s: %s',rospy.get_name(),e)
            time.sleep(max(.001,1./hz-(time.monotonic()-start)))
    threading.Thread(target=run,daemon=True).start()

def path_of(points):
    p=Path();p.header.stamp=rospy.Time.now();p.header.frame_id='drone_odom'
    for x in points:
        q=PoseStamped();q.header=p.header;put(q.pose.position,x);q.pose.orientation.w=1.;p.poses.append(q)
    return p

def tf_matrix(buf,target,source,stamp):
    tr=buf.lookup_transform(target,source,stamp,rospy.Duration(.04)).transform
    return rotation(quat(tr.rotation)),xyz(tr.translation)
