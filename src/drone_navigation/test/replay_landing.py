#!/usr/bin/env python3
"""Recorded positions and available PX4 flags drive the real Gateway with mocked ROS.

Recorded inputs do not respond to the new setpoints. This checks the state machine,
not whether a real aircraft would land. Missing raw flags are explicitly reported.
"""
import copy,json,sys
from pathlib import Path
import rosbag,rospy
import numpy as np
sys.path.insert(0,'/home/orangepi/orange/src/drone_navigation/test')
from test_land import GatewayLandTests
from drone_navigation.ros_common import xyz
from mavros_msgs.msg import PositionTarget

batch=Path(sys.argv[1]);out=Path(sys.argv[2])
flight=json.loads(next(batch.glob('flight_*.json')).read_text())
start,end=flight['land_service_wall'],flight['end_wall']
topics=['/mavros/local_position/odom','/mavros/setpoint_raw/local','/mavros/state',
        '/mavros/extended_state','/mavros/estimator_status']
with rosbag.Bag(str(next(batch.glob('*.bag')))) as bag:
    events=[(t.to_sec(),topic,m) for topic,m,t in bag.read_messages(topics=topics) if start-2.<=t.to_sec()<=end]
sp=[m for stamp,topic,m in events if stamp>=start and topic=='/mavros/setpoint_raw/local']
assert sp
states=flight.get('landing_states',[])
if states and states[0]['state'].get('land_ground_z') is not None:ground=states[0]['state']['land_ground_z']
else:ground=min(m.position.z for m in sp)+.02
raw_flags={topic:any(t==topic for _,t,_ in events) for topic in topics[-3:]}
test=GatewayLandTests();test.setUp()
try:
    g=test.airborne(z=sp[0].position.z)
    g.ground_z=ground;g.hold_p=xyz(sp[0].position);g.hold_yaw=sp[0].yaw
    assert g.land_start(None).success
    epoch=test.now;rows=[]
    for stamp,topic,m in events:
        test.now=epoch+stamp-start
        message=copy.deepcopy(m)
        if hasattr(message,'header'):message.header.stamp=rospy.Time.now()
        if topic=='/mavros/extended_state':g.on_ext(message)
        elif topic=='/mavros/estimator_status':g.on_est(message)
        elif topic=='/mavros/state':g.on_state(message)
        elif topic=='/mavros/local_position/odom' and stamp>=start:
            if not raw_flags['/mavros/extended_state'] or not raw_flags['/mavros/estimator_status']:
                test.refresh(g,p=xyz(message.pose.pose.position),airborne=True)
            g.px=message;g.px_at=g.input_at=g.owners_at=test.now
            g.input_ok=g.owners_ok=True
            g.tick();command=test.sent(g)
            rows.append(dict(dt=stamp-start,measured_z=m.pose.pose.position.z,
                             target_z=command.position.z,vz=command.velocity.z,land=g.land_state,
                             z_velocity_only=bool(command.type_mask & PositionTarget.IGNORE_PZ),
                             disarm_requests=g.land_disarm_count,reason=g.fault))
            if g.land_state in ('STOPPED','DONE'):break
    assert rows
    assert not any(x['land']=='TOUCHED' for x in rows),'unexpected ground in selected bag window; review raw input'
    assert not any(x['disarm_requests'] for x in rows)
    assert min(x['target_z'] for x in rows if x['land'] in ('DESCENDING','PRESSING'))>=ground-.10-1e-9
    press=[x for x in rows if x['land']=='PRESSING']
    assert press,'recording did not exercise velocity press'
    assert all(x['z_velocity_only'] and x['vz']<-.3 for x in press)
    result=dict(bag=str(next(batch.glob('*.bag'))),raw_flags=raw_flags,
                limitation='Fixed recorded positions cannot respond to new commands; absent flags use fresh IN_AIR/healthy assumptions. No flight acceptance.',
                ground_reference=ground,target_limit=ground-.1,rows=rows)
    out.write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(rows=len(rows),raw_flags=raw_flags,velocity_press_samples=len(press),
                         false_touched=False,disarm_without_ground=False,last=rows[-1]),indent=2))
finally:test.doCleanups()
