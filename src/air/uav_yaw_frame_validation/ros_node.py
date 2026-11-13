#!/usr/bin/env python3
import math, time
import rospy, tf2_ros, tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped
from std_msgs.msg import String
from core import transform, inverse

def run():
    rospy.init_node('uav_yaw_frame_validation')
    broadcaster=tf2_ros.StaticTransformBroadcaster()
    tf=TransformStamped(); tf.header.frame_id='world'; tf.child_frame_id='sensor_frame'
    tf.header.stamp=rospy.Time.now(); tf.transform.translation.x=2.; tf.transform.translation.y=-1.
    tf.transform.rotation.z=math.sin(.7/2); tf.transform.rotation.w=math.cos(.7/2)
    broadcaster.sendTransform(tf)
    buffer=tf2_ros.Buffer(); listener=tf2_ros.TransformListener(buffer)
    local_pub=rospy.Publisher('~local_point',PointStamped,queue_size=1)
    world_pub=rospy.Publisher('~world_point',PointStamped,queue_size=1)
    report=rospy.Publisher('~error',String,queue_size=1)
    start=time.monotonic()
    while not rospy.is_shutdown():
        t=time.monotonic()-start
        p=PointStamped(); p.header.frame_id='sensor_frame'; p.header.stamp=rospy.Time(0)
        p.point.x=math.cos(t); p.point.y=math.sin(t)
        local_pub.publish(p)
        try:
            q=buffer.transform(p,'world',rospy.Duration(.2))
            expected=transform((p.point.x,p.point.y),(2.,-1.),.7)
            recovered=inverse((q.point.x,q.point.y),(2.,-1.),.7)
            error=math.hypot(q.point.x-expected[0],q.point.y-expected[1])
            back=math.hypot(recovered[0]-p.point.x,recovered[1]-p.point.y)
            world_pub.publish(q); report.publish('tf_error={:.3g} roundtrip={:.3g}'.format(error,back))
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException) as exc:
            report.publish('WAITING '+str(exc))
        time.sleep(.1)
