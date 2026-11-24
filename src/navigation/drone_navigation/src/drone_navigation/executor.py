import threading,time,math,copy
import numpy as np
import rospy
from std_msgs.msg import Empty,String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path
from traj_utils.msg import Bspline,PathStart,PathTrajectory
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import NavigationCommand
from .core import Spline,wrap,yaw,brake
from .ros_common import *

class Executor:
    def __init__(self):
        self.lock=threading.RLock();self.spline=None;self.spline_goal=None;self.pending_spline=None;self.goal=None;self.odom=None;self.odom_at=0.;self.active=False;self.gen=0;self.yaw=0.;self.last_id=-1;self.plan_at=0.;self.plan='';self.fault='';self.goal_at=0.;self.c=config();self.checked_id=None;self.checked_at=0.;self.check_reason='waiting for map validation';self.last_reference=None
        self.early_spline=None;self.early_at=0.
        self.last_goal_stamp = rospy.Time(0)
        self.path_mode = False
        self.path_candidate = None
        self.path_checked_at = -math.inf
        self.approved_next = None
        self.align_since = None
        self.heading_since = None
        self.heading_at = None
        self.acknowledged_id = None
        self.start_pub = rospy.Publisher('/drone_nav/path_start', PathStart, queue_size=1)
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
        rospy.Subscriber('/drone_nav/planner_path',Path,self.on_path,queue_size=1)
        rospy.Subscriber('/drone_nav/path_trajectory',PathTrajectory,self.on_path_trajectory,queue_size=1)
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
        with self.lock:
            if self.path_mode:
                self.last_goal_stamp = max(self.last_goal_stamp, rospy.Time.now())
            self.active=False;self.spline=None;self.spline_goal=None;self.pending_spline=None;self.goal=None;self.early_spline=None
            self.path_candidate = self.approved_next = None
    def on_path(self, path):
        goal = path_endpoint(path)
        if goal is not None:
            self.on_goal(goal)

    def on_goal(self,m):
        with self.lock:
            path_mode = m.header.seq >= PATH_GENERATION
            if (path_mode or self.path_mode) and m.header.stamp <= self.last_goal_stamp:
                return
            self.last_goal_stamp = m.header.stamp
            self.path_mode = path_mode
            self.path_candidate = self.approved_next = None
            self.align_since = self.heading_since = self.heading_at = None
            self.acknowledged_id = None
            if path_mode:
                self.spline = self.spline_goal = None
                self.path_checked_at = -math.inf
                self.path_last_id = -1
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
            if self.path_mode and self.active:
                return
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
    def on_path_trajectory(self, message):
        with self.lock:
            if not self.active or not self.path_mode or self.goal is None:
                return
            ident, b = message.identity, message.spline
            if (ident.path_stamp != self.goal.header.stamp or ident.path_seq + PATH_GENERATION != self.gen
                    or ident.trajectory_id != b.traj_id or b.traj_id <= self.path_last_id):
                return
            if message.pending_start and (self.spline is not None or self.path_candidate is not None):
                return
            if not message.pending_start and self.spline is None and b.traj_id != self.acknowledged_id:
                return
            try:
                spline = Spline(b.order, b.knots, [xyz(p) for p in b.pos_pts],
                                b.start_time.to_sec(), b.traj_id)
                if message.pending_start:
                    for t in np.arange(0., spline.duration, .02):
                        velocity = spline.state(t)[1]
                        if np.linalg.norm(velocity[:2]) >= .03:
                            self.path_candidate = (spline, math.atan2(velocity[1], velocity[0]), ident)
                            return
                    self.fail('path has no horizontal departure tangent')
                elif -.60 <= rospy.Time.now().to_sec() - spline.start <= .3:
                    self.pending_spline = spline
                    self.path_last_id = b.traj_id
                    self.last_id = max(self.last_id, b.traj_id)
            except ValueError as error:
                self.fail(str(error))

    def check_alignment_path(self, candidate, goal, origin):
        spline = candidate[0]
        count = max(2, math.ceil(spline.duration * self.c['max_vel'] / .04) + 1)
        if count > 1500:
            with self.lock:
                if self.active and self.goal is goal:
                    self.fail('path candidate exceeds the validation window')
            return
        response = self.check(path_of([origin] + [spline.position(t) for t in np.linspace(0., spline.duration, count)]))
        with self.lock:
            if self.active and self.goal is goal and self.path_candidate is candidate and self.spline is None:
                if response.valid:
                    self.path_checked_at = time.monotonic()
                else:
                    self.check_reason = response.reason
                    self.path_checked_at = -math.inf
                    self.align_since = None

    def send_command(self, p, v, a, terminal=False, stamp=None):
        cmd = NavigationCommand()
        cmd.header.stamp = rospy.Time.now() if stamp is None else rospy.Time.from_sec(stamp)
        cmd.header.frame_id = 'drone_odom'
        put(cmd.position, p)
        put(cmd.velocity, v)
        put(cmd.acceleration, a)
        cmd.yaw = self.yaw
        cmd.generation = self.gen
        cmd.terminal = terminal
        self.pub.publish(cmd)

    def align_path(self):
        elapsed = time.monotonic() - self.goal_at
        if elapsed >= 12. or (elapsed >= 3. and self.path_candidate is None):
            self.fail('path start not ready: ' + self.check_reason)
            return
        if self.path_candidate is None or time.monotonic() - self.path_checked_at > .3:
            status(self.stat, 'VALIDATING', reason=self.check_reason)
            return
        spline, target_yaw, ident = self.path_candidate
        p = spline.state(0.)[0]
        measured_p = xyz(self.odom.pose.pose.position)
        if np.linalg.norm(p - measured_p) > .1:
            self.fail('drift during path heading alignment')
            return
        delta = wrap(target_yaw - self.yaw)
        stamp = rospy.Time.now().to_sec()
        dt = .02 if self.heading_at is None else max(0., min(.04, stamp - self.heading_at))
        step = self.yaw_step * dt / .02
        self.yaw = wrap(self.yaw + max(-step, min(step, delta)))
        self.heading_at = stamp
        self.send_command(p, np.zeros(3), np.zeros(3), stamp=stamp)
        status(self.stat, 'ALIGNING_HEADING', trajectory_id=spline.id, generation=self.gen)
        measured_yaw = yaw(quat(self.odom.pose.pose.orientation))
        now = time.monotonic()
        if (abs(wrap(target_yaw-measured_yaw)) <= math.radians(5) and
                abs(wrap(target_yaw-self.yaw)) < 1e-6 and np.linalg.norm(xyz(self.odom.twist.twist.linear)) <= .1):
            self.align_since = now if self.align_since is None else self.align_since
            if now - self.align_since >= 1.:
                self.acknowledged_id = spline.id
                self.start_pub.publish(ident)
        else:
            self.align_since = None

    def path_heading(self, velocity, stamp):
        # Body-frame odometry twist must be rotated before comparing its course.
        q = quat(self.odom.pose.pose.orientation)
        actual = rotation(q) @ xyz(self.odom.twist.twist.linear)
        angle = None
        wall = time.monotonic()
        if np.linalg.norm(actual[:2]) >= .15:
            angle = abs(wrap(math.atan2(actual[1], actual[0]) - yaw(q)))
        if angle is not None and angle > math.radians(15):
            self.heading_since = wall if self.heading_since is None else self.heading_since
            if wall - self.heading_since >= .5:
                self.fail('measured course differs from heading by more than 15 deg')
                return False, angle
        else:
            self.heading_since = None
        speed = np.linalg.norm(velocity[:2])
        if speed >= 1e-4:
            tangent = math.atan2(velocity[1], velocity[0])
            dt = .02 if self.heading_at is None else max(0., stamp - self.heading_at)
            step = self.yaw_step * dt / .02
            delta = wrap(tangent - self.yaw)
            if speed >= .03 and abs(delta) > step + math.radians(.02):
                self.fail('path heading jump %.2f deg in %.3f s at %.3f m/s' %
                          (math.degrees(delta), dt, speed))
                return False, angle
            # Below 3 cm/s the derivative can be nearly zero. Follow the
            # approaching tangent at the same bounded rate until it is usable.
            self.yaw = tangent if speed >= .03 else wrap(self.yaw + max(-step, min(step, delta)))
        self.heading_at = stamp
        return True, angle

    def fail(self,reason):
        self.approved_next = None
        self.fault=reason;self.active=False;self.spline=None;self.spline_goal=None;self.pending_spline=None;self.cancel.publish(Empty());status(self.stat,'FAULT',reason=reason)
    def validate(self):
        with self.lock:
            if not self.active or not fresh(self.odom,self.odom_at,self.odom_age):return
            current,pending,goal=self.spline,self.pending_spline,self.goal
            origin=xyz(self.odom.pose.pose.position)
            candidate = self.path_candidate if self.path_mode and current is None and pending is None else None
            path_mode = self.path_mode
            measured_velocity = rotation(quat(self.odom.pose.pose.orientation)) @ xyz(self.odom.twist.twist.linear)
        if candidate is not None:
            self.check_alignment_path(candidate, goal, origin)
            return
        # Refresh the current trajectory's clearance lease before checking a new one.
        # A pending candidate never invalidates an otherwise usable command stream.
        for s in (current,pending):
            if s is None:continue
            t=min(s.duration,max(0.,rospy.Time.now().to_sec()-s.start))
            n=max(2,int(math.ceil((s.duration-t)*max(self.c['max_vel'],.1)/.04)))
            points = [origin]
            if path_mode:
                # Check the actual braking corridor in the same map snapshot
                # as the spline. The return along that corridor is only a query.
                p_stop, v_stop = origin, measured_velocity
                while np.linalg.norm(v_stop) > 1e-5 and len(points) < 150:
                    p_stop, v_stop = brake(p_stop, v_stop, self.c['max_acc'])
                    points.append(p_stop)
                points.append(origin)
            points.extend(s.position(x) for x in np.linspace(t, s.duration, min(n, 1500)))
            if path_mode:
                end_p, end_v, _ = s.state(s.duration)
                points.append(end_p + end_v * np.linalg.norm(end_v) / (2 * self.c['max_acc']))
            try:
                response = self.check(path_of(points))
                with self.lock:
                    if not self.active or self.goal is not goal:return
                    if s is current and self.spline is s:
                        if response.valid:self.checked_id=s.id;self.checked_at=time.monotonic()
                        else:self.fail('trajectory no longer clear: '+response.reason);return
                    elif self.pending_spline is s and self.goal is goal:
                        self.check_reason=response.reason
                        if path_mode and not response.valid:
                            self.fail('path continuation not clear: ' + response.reason)
                            return
                        if response.valid:
                            if path_mode and current is not None:
                                old_p, old_v, old_a = current.state(s.start-current.start)
                                new_p, new_v, new_a = s.state(0.)
                                if (s.start >= current.start+current.duration or np.linalg.norm(old_p-new_p) > .02 or
                                        np.linalg.norm(old_v-new_v) > .02 or np.linalg.norm(old_a-new_a) > .05):
                                    self.fail('path replan does not match the scheduled p/v/a')
                                    return
                                if rospy.Time.now().to_sec() >= s.start:
                                    if self.approved_next is not None and self.approved_next[0] is s:
                                        continue
                                    self.fail('path replan map check missed scheduled start')
                                    return
                                self.approved_next = (s, time.monotonic())
                                continue
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
            if self.path_mode and self.spline is None:
                if time.monotonic() - self.plan_at > .3:
                    self.fail('planner heartbeat lost during path start')
                else:
                    self.align_path()
                return
            if self.spline_goal is not self.goal and time.monotonic()-self.goal_at>3.:
                self.fail('no valid trajectory for current goal within 3s: '+self.check_reason);return
            if self.spline is None:
                if time.monotonic()-self.goal_at>3.:self.fail('no valid trajectory within 3s')
                else:status(self.stat,'PLANNING')
                return
            if time.monotonic()-self.plan_at>.3:
                self.fail("planner heartbeat lost during execution");return
            now = rospy.Time.now().to_sec()
            if self.path_mode and self.pending_spline is not None and now >= self.pending_spline.start:
                if (self.approved_next is None or self.approved_next[0] is not self.pending_spline or
                        time.monotonic() - self.approved_next[1] > .3):
                    self.fail('no validated trajectory at scheduled path handoff')
                    return
                self.spline, self.checked_at = self.approved_next
                self.checked_id = self.spline.id
                self.pending_spline = self.approved_next = None
            s = self.spline
            t = now - s.start
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
            angle = None
            if self.path_mode:
                if abs(v[2]) > self.c['max_vel_z']*1.001 or abs(a[2]) > self.c['max_acc_z']*1.001:
                    self.fail('path vertical limits')
                    return
                if at_end and (np.linalg.norm(p-xyz(self.goal.pose.position)) > 1e-4 or np.linalg.norm(v) > .03):
                    self.fail('local horizon expired before a validated continuation')
                    return
                ok, angle = self.path_heading(v, now)
                if not ok:
                    return
            if at_end:v,a=np.zeros(3),np.zeros(3)
            if terminal and not self.path_mode:
                if np.linalg.norm(xyz(self.odom.pose.pose.position)-p)<=.15 and np.linalg.norm(xyz(self.odom.twist.twist.linear))<=.1:
                    delta=wrap(yaw(quat(self.goal.pose.orientation))-self.yaw)
                    self.yaw=wrap(self.yaw+max(-self.yaw_step,min(self.yaw_step,delta)))
            self.send_command(p, v, a, terminal, stamp=now)
            status(self.stat, 'HOLD' if terminal else 'RUNNING', trajectory_id=s.id, generation=self.gen,
                   course_error_deg=math.degrees(angle) if angle is not None else None)
