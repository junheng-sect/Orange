"""CH6 fixture on the private navigation master; no connection to hardware flight controls."""
from pathlib import Path
import time

import cv2
import rospy
from mavros_msgs.msg import RCIn, State
from std_msgs.msg import String


class Switch:
    def __init__(self):
        self.pwm = 1000
        self.events = []
        self.rc = rospy.Publisher('/fixture/ch6', RCIn, queue_size=5)
        self.sub = rospy.Subscriber('/rc_record/bag_path', String, lambda m: self.events.append(m.data))
        self.timer = rospy.Timer(rospy.Duration(.04), self.publish)

    def publish(self, _):
        self.rc.publish(RCIn(channels=[1500]*5+[self.pwm]))

    def set(self, high):
        count = len(self.events)
        self.pwm = 1900 if high else 1000
        deadline = time.monotonic()+6
        while len(self.events) == count and time.monotonic()<deadline:
            time.sleep(.04)
        assert len(self.events)>count and self.events[-1].startswith('OPEN ' if high else 'CLOSED '), self.events


def verify_route(out, record, stopped):
    assert record['end'] == ('ABORTED' if stopped else 'SUCCEEDED'),record.get('reason')
    points = record['legs'][:1] if stopped else record['legs']
    assert [leg['direction'] for leg in points] == ['左下','左上','右上','右下'][:1 if stopped else 4]
    for leg in points:
        t0 = leg['t0_ns']
        assert t0 < leg['confirmed_ns'] and leg['score_end_ns'] >= t0+5_000_000_000
        stamps = []
        for row in leg['photos']:
            assert row['status']=='saved',row
            assert abs(row['stamp_ns']-row['planned_ns']) <= 100_000_000
            stamps.append(row['stamp_ns'])
            assert cv2.imread(str(out/row['file']),cv2.IMREAD_UNCHANGED).shape==(3136,4224)
        assert len(set(stamps))==3
    print('PASS route, capture offsets, photos and navigation result',record['end'],flush=True)
