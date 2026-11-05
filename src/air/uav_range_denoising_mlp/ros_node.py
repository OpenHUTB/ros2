#!/usr/bin/env python3
import json, math, threading, time
from collections import deque
from pathlib import Path
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import String
from core import predict,WINDOW

def run():
    rospy.init_node('uav_range_denoising_mlp')
    model_path=rospy.get_param('~model_path',str(Path(__file__).parent/'model.json'))
    model=json.loads(Path(model_path).read_text())
    output=rospy.Publisher('range_denoised',Range,queue_size=1)
    status=rospy.Publisher('~status',String,queue_size=1)
    window=deque(maxlen=WINDOW);lock=threading.Lock();last=[None]
    def receive(m):
        now=time.monotonic()
        with lock:
            if last[0] is not None and now-last[0]>.5:window.clear()
            last[0]=now
            if not (math.isfinite(m.range) and math.isfinite(m.min_range) and math.isfinite(m.max_range)
                    and 0<=m.min_range<m.max_range and m.min_range<=m.range<=m.max_range):
                window.clear();status.publish('INVALID');return
            window.append(m.range/4.)
            if len(window)<WINDOW:status.publish('WARMUP');return
            value=float(predict([list(window)],model)[0,0])
            if not m.min_range<=value<=m.max_range:status.publish('ESTIMATE_OUT_OF_RANGE');return
            # Construct a fresh message, retain source stamp and sensor metadata.
            result=Range();result.header=m.header;result.radiation_type=m.radiation_type
            result.field_of_view=m.field_of_view;result.min_range=m.min_range;result.max_range=m.max_range;result.range=value
            output.publish(result);status.publish('ESTIMATED_SYNTHETIC_MODEL')
    rospy.Subscriber('range_input',Range,receive,queue_size=1)
    while not rospy.is_shutdown():
        with lock:
            if last[0] is None or time.monotonic()-last[0]>.5:
                window.clear();status.publish('STALE')
        time.sleep(.1)
