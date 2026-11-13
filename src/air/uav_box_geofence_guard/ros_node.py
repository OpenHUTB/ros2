#!/usr/bin/env python3
import threading, time
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from core import guard

def run():
    rospy.init_node('uav_box_geofence_guard')
    pub=rospy.Publisher('velocity_safe',TwistStamped,queue_size=1)
    status=rospy.Publisher('~status',String,queue_size=1)
    lock=threading.Lock(); data={'pose':None,'cmd':None,'pt':0.,'ct':0.}
    def pose_cb(m):
        with lock: data.update(pose=(m.pose.position.x,m.pose.position.y,m.pose.position.z)
                              if m.header.frame_id=='world' else None,pt=time.monotonic())
    def cmd_cb(m):
        with lock: data.update(cmd=(m.twist.linear.x,m.twist.linear.y,m.twist.linear.z,m.twist.angular.z)
                              if m.header.frame_id=='world' else None,ct=time.monotonic())
    rospy.Subscriber('pose_world',PoseStamped,pose_cb,queue_size=1)
    rospy.Subscriber('velocity_raw',TwistStamped,cmd_cb,queue_size=1)
    def publish(values):
        m=TwistStamped(); m.header.frame_id='world'; m.header.stamp=rospy.Time.now()
        m.twist.linear.x,m.twist.linear.y,m.twist.linear.z,m.twist.angular.z=values
        pub.publish(m)
    rospy.on_shutdown(lambda: publish((0.,0.,0.,0.)))
    try:
        while not rospy.is_shutdown():
            now=time.monotonic()
            with lock: d=dict(data)
            if d['pose'] is None or d['cmd'] is None: values,state=(0.,0.,0.,0.),'WAITING'
            else: values,state=guard(d['pose'],d['cmd'],now-d['pt'],now-d['ct'])
            publish(values); status.publish(state); time.sleep(.05)
    finally: publish((0.,0.,0.,0.))
