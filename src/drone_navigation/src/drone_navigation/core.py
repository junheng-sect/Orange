"""ROS-independent navigation math and validity checks. All vectors are SI/ENU/FLU."""
import math
from collections import deque
import numpy as np


def finite(v, size=3):
    a = np.asarray(v, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError('invalid finite vector')
    return a


def check_goal_bounds(points, config, labels=None):
    """Check navigation reference points, with an optional inward boundary margin."""
    lo, hi = finite(config['bounds_min']), finite(config['bounds_max'])
    margin = float(config.get('goal_boundary_margin', 0.0))
    if not math.isfinite(margin) or margin < 0 or np.any(hi - lo <= 2 * margin):
        raise ValueError('invalid goal_boundary_margin/bounds')
    for i, point in enumerate(points):
        p = finite(point)
        gaps = np.r_[p - lo, hi - p]
        j = int(np.argmin(gaps))
        if gaps[j] < margin - 1e-9:
            label = labels[i] if labels is not None else 'waypoint #%d' % (i + 1)
            face = ('x_min', 'y_min', 'z_min', 'x_max', 'y_max', 'z_max')[j]
            raise ValueError('%s %s: %s clearance %.3f m < required %.3f m (drone_odom)' %
                             (label, p.tolist(), face, gaps[j], margin))
    return True


def rotation(q):
    x, y, z, w = finite(q, 4)
    if abs(x*x+y*y+z*z+w*w-1.) > .002:
        raise ValueError('non-unit quaternion')
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def yaw(q):
    r = rotation(q)
    return math.atan2(r[1,0], r[0,0])


def rz(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def heading(r, a):
    v=r@np.array([math.cos(a), math.sin(a), 0.])
    if np.linalg.norm(v[:2])<.9: raise ValueError('non-level heading transform')
    return math.atan2(v[1],v[0])


def reference_state(p, v, r, omega, lever):
    """Move lidar-IMU position to PX4 reference; lever/omega in body FLU."""
    return finite(p)+r@finite(lever), finite(v)+r@np.cross(finite(omega),finite(lever))


class Alignment:
    def __init__(self):
        self.samples=deque(maxlen=300);self.r=None;self.t=None
    def add(self, stamp, p, a, px, ax):
        if self.samples and stamp <= self.samples[-1][0]: raise ValueError('alignment stamp rollback')
        self.samples.append((stamp,finite(p),float(a),finite(px),float(ax)))
        if len(self.samples)<15 or stamp-self.samples[0][0]<2.:return False
        ds=np.array([wrap(s[4]-s[2]) for s in self.samples]);d=math.atan2(np.sin(ds).mean(),np.cos(ds).mean())
        r=rz(d);ts=np.array([s[3]-r@s[1] for s in self.samples]);t=np.median(ts,axis=0)
        if max(abs(wrap(x-d)) for x in ds)>math.radians(2) or np.max(np.linalg.norm(ts-t,axis=1))>.05:
            raise ValueError('unstable alignment')
        self.r,self.t=r,t;return True
    def check(self,p,a,px,ax):
        return self.r is not None and np.linalg.norm(finite(px)-(self.r@finite(p)+self.t))<=.15 and abs(wrap(ax-heading(self.r,a)))<=math.radians(3)


class Spline:
    def __init__(self, degree, knots, points, start, trajectory_id):
        self.degree=int(degree);self.knots=np.asarray(knots,dtype=float);self.points=np.asarray(points,dtype=float)
        self.start=float(start);self.id=int(trajectory_id)
        n=len(self.points);k=self.degree
        if k!=3 or self.points.shape!=(n,3) or n<k+1 or len(self.knots)!=n+k+1 or not np.isfinite(self.points).all() or not np.isfinite(self.knots).all() or not math.isfinite(self.start):
            raise ValueError('invalid spline structure')
        if np.any(np.diff(self.knots)<0):raise ValueError('unordered knots')
        self.begin=self.knots[k];self.duration=self.knots[n]-self.begin
        if not .02<self.duration<300:raise ValueError('invalid spline duration')
        if np.any(self.knots[k+1:n+1]-self.knots[1:n-k+1]<=0):raise ValueError('degenerate spline knots')
    @staticmethod
    def evaluate(points, knots, k, u):
        n=len(points);u=max(knots[k],min(float(u),knots[n]))
        j=min(n-1,max(k,int(np.searchsorted(knots,u,side='right')-1)))
        d=[p.copy() for p in points[j-k:j+1]]
        for r in range(1,k+1):
            for i in range(k,r-1,-1):
                left=j-k+i;den=knots[left+k-r+1]-knots[left]
                alpha=(u-knots[left])/den if den>0 else 0.
                d[i]=(1-alpha)*d[i-1]+alpha*d[i]
        return d[k]
    def state(self,t):
        u=self.begin+max(0.,min(t,self.duration));p=self.evaluate(self.points,self.knots,self.degree,u)
        pts=self.points;knots=self.knots;k=self.degree;out=[p]
        for _ in range(2):
            den=knots[k+1:len(pts)+k]-knots[1:len(pts)]
            if np.any(den<=0):raise ValueError('degenerate derivative')
            pts=k*np.diff(pts,axis=0)/den[:,None];knots=knots[1:-1];k-=1
            out.append(self.evaluate(pts,knots,k,u))
        return tuple(out)


def brake(p,v,a=.5,dt=.02):
    p,v=finite(p),finite(v);speed=np.linalg.norm(v)
    nv=v*max(0.,1-a*dt/speed) if speed>1e-9 else v*0
    return p+(v+nv)*.5*dt,nv


def climb_profile(t,h,v,a):
    """Rest-to-rest trapezoid over height h with speed v and acceleration a: (height, speed, acceleration, total time) at t."""
    if not (h>0 and v>0 and a>0):raise ValueError('invalid climb profile')
    ta=min(v/a,math.sqrt(h/a));vp=a*ta;tc=(h-a*ta*ta)/vp;T=2*ta+tc;t=min(max(t,0.),T)
    if t<ta:return .5*a*t*t,a*t,a,T
    if t<ta+tc:return .5*a*ta*ta+vp*(t-ta),vp,0.,T
    r=T-t
    return (h-.5*a*r*r,a*r,-a if r>0 else 0.,T)


def descend_profile(t, v, a=.3):
    """Distance travelled and descent speed at t for a descent that holds speed v after ramping at a (no end height)."""
    if not (v > 0 and a > 0):
        raise ValueError('invalid descent profile')
    t, t1 = max(0., t), v / a
    return (.5*a*t*t, a*t) if t < t1 else (v*v/(2*a)+v*(t-t1), v)


def deployment(c):
    if c.get('deployment_verified') is not True:raise ValueError('deployment geometry not verified')
    for key in ('lever_body','sensor_offset_imu','body_half_extent','bounds_min','bounds_max'):
        finite(c[key])
    if np.any(finite(c['body_half_extent'])<=0) or np.any(finite(c['bounds_max'])<=finite(c['bounds_min'])):raise ValueError('invalid geometry/bounds')
    for key in ('clearance','max_vel','max_acc'):
        if not math.isfinite(float(c[key])) or c[key]<=0:raise ValueError('invalid positive '+key)
    check_goal_bounds([], c)
    if not c.get('map_id'):raise ValueError('map_id is required')
    return True

# Command inputs, distinct from MAVROS target_* and trajectory/desired feedback.
SETPOINT_INPUTS = frozenset('/mavros/'+suffix for suffix in (
    'setpoint_accel/accel', 'setpoint_attitude/thrust', 'setpoint_attitude/cmd_vel',
    'setpoint_attitude/attitude', 'setpoint_attitude/cmd_vel_unstamped',
    'setpoint_position/local', 'setpoint_position/global', 'setpoint_position/global_to_local',
    'setpoint_raw/local', 'setpoint_raw/global', 'setpoint_raw/attitude',
    'setpoint_trajectory/local', 'setpoint_velocity/cmd_vel', 'setpoint_velocity/cmd_vel_unstamped'))


def competing_publishers(publishers, own_name):
    return sorted({n for topic,nodes in publishers if topic in SETPOINT_INPUTS for n in nodes if n!=own_name})


def shift_pose_covariance(covariance, lever_world):
    """Propagate parent-frame position/orientation covariance to a rigid reference point."""
    p=np.asarray(covariance,dtype=float).reshape(6,6)
    if not np.isfinite(p).all():raise ValueError('invalid pose covariance')
    x,y,z=finite(lever_world)
    cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    j=np.eye(6);j[:3,3:]=-cross
    return (j@p@j.T).reshape(36).tolist()
