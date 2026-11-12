#!/usr/bin/env python3
import math, threading, time
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import String
from core import Alarm

def run():
    rospy.init_node('uav_landing_range_alarm')
    alarm=Alarm(); lock=threading.Lock()
    output=rospy.Publisher('~state',String,queue_size=1)
    demo=bool(rospy.get_param('~demo',True))
    source=rospy.Publisher('range_input',Range,queue_size=1) if demo else None
    def receive(m):
        # Range +/-inf means special sensor semantics; this tutorial treats it as INVALID.
        valid=(math.isfinite(m.min_range) and math.isfinite(m.max_range)
               and 0 <= m.min_range < m.max_range and m.min_range <= m.range <= m.max_range)
        with lock: alarm.update(m.range if valid else float('nan'),time.monotonic())
    rospy.Subscriber('range_input',Range,receive,queue_size=1)
    start=time.monotonic()
    while not rospy.is_shutdown():
        now=time.monotonic(); t=now-start
        if demo and not 8. < t < 9.5:
            m=Range(); m.header.stamp=rospy.Time.now(); m.header.frame_id='range_sensor'
            m.radiation_type=Range.ULTRASOUND; m.field_of_view=.1
            m.min_range=.1; m.max_range=5.
            m.range=.8 if 3.<t<6. else (1.15+.08*math.sin(t*12) if t<8 else 2.)
            source.publish(m)
        with lock: state=alarm.check(now)
        output.publish(state); time.sleep(.05)
