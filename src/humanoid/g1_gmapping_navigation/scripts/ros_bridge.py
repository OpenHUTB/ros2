#!/usr/bin/env python3
"""ROS Noetic bridge. Simulation uses an isolated interpreter over stdio.

The odometry pose integrates simulated velocity and gyro measurements.
On WSL, an explicit Windows Python executable can run the MuJoCo process.
"""
import json
import math
import subprocess
import time
import base64

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState, Image
from rosgraph_msgs.msg import Clock


def main():
    rospy.init_node('humanoid_sim_bridge')
    interpreter = rospy.get_param('~sim_python')
    entry = rospy.get_param('~sim_entry')
    assets = rospy.get_param('~assets')
    seed = str(rospy.get_param('~seed',7))
    real_time_factor = float(rospy.get_param('~real_time_factor',1.0))
    command = [0.,0.,0.]
    last_command = [0.]
    def callback(msg):
        command[:] = [msg.linear.x,msg.linear.y,msg.angular.z]
        last_command[0] = time.monotonic()
    rospy.Subscriber('/cmd_vel',Twist,callback,queue_size=1)
    scan_pub = rospy.Publisher('/scan',LaserScan,queue_size=2)
    odom_pub = rospy.Publisher('/odom',Odometry,queue_size=2)
    clock_pub = rospy.Publisher('/clock',Clock,queue_size=2)
    joint_pub = rospy.Publisher('/joint_states',JointState,queue_size=2)
    image_pub = rospy.Publisher('/camera/image_raw',Image,queue_size=1)
    broadcaster = tf.TransformBroadcaster()
    child = subprocess.Popen([interpreter,entry,'--mode','bridge','--assets',assets,'--seed',seed],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,universal_newlines=True,bufsize=1)
    try:
        while not rospy.is_shutdown():
            begin = time.monotonic()
            if time.monotonic()-last_command[0] > .8:
                command[:] = [0.,0.,0.]
            child.stdin.write(json.dumps({'command':command})+'\n')
            child.stdin.flush()
            line = child.stdout.readline()
            if not line:
                raise RuntimeError('Simulator process exited')
            state = json.loads(line)
            stamp = rospy.Time.from_sec(state['time'])
            clock_pub.publish(Clock(clock=stamp))
            x,y,yaw = state['odom']
            quaternion = tf.transformations.quaternion_from_euler(0,0,yaw)
            broadcaster.sendTransform((x,y,0),quaternion,stamp,'base_link','odom')
            broadcaster.sendTransform((0,0,.65),(0,0,0,1),stamp,'laser','base_link')
            odom = Odometry()
            odom.header.stamp,odom.header.frame_id,odom.child_frame_id = stamp,'odom','base_link'
            odom.pose.pose.position.x,odom.pose.pose.position.y = x,y
            odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w = quaternion
            # Rotate MuJoCo world-frame linear velocity into the base_link frame.
            vx,vy = state['velocity']
            odom.twist.twist.linear.x = math.cos(yaw)*vx+math.sin(yaw)*vy
            odom.twist.twist.linear.y = -math.sin(yaw)*vx+math.cos(yaw)*vy
            odom.twist.twist.angular.z = state['angular_velocity']
            odom_pub.publish(odom)
            scan = LaserScan()
            scan.header.stamp,scan.header.frame_id = stamp,'laser'
            scan.angle_min = -math.pi
            scan.angle_increment = 2*math.pi/len(state['scan'])
            scan.angle_max = scan.angle_min+(len(state['scan'])-1)*scan.angle_increment
            scan.range_min,scan.range_max,scan.scan_time = .05,8.,.2
            scan.ranges = state['scan']
            scan_pub.publish(scan)
            joints=JointState()
            joints.header.stamp=stamp
            joints.name=state['joint_names']; joints.position=state['joints']
            joint_pub.publish(joints)
            camera_quaternion = tf.transformations.quaternion_from_matrix(state['camera_rotation'])
            broadcaster.sendTransform(state['camera_position'], camera_quaternion,
                                      stamp, 'front_camera', 'base_link')
            image=Image()
            image.header.stamp=stamp; image.header.frame_id='front_camera'
            image.height,image.width=state['image_shape'][:2]
            image.encoding='rgb8'; image.step=image.width*3
            image.data=base64.b64decode(state['image'])
            image_pub.publish(image)
            if state['fallen']:
                rospy.logerr('Robot fell; stopping simulator')
                break
            time.sleep(max(0,.2/real_time_factor-(time.monotonic()-begin)))
    finally:
        try:
            child.stdin.write('{"stop":true}\n')
            child.stdin.flush()
            child.wait(timeout=5)
        except (OSError,subprocess.TimeoutExpired):
            child.terminate()


if __name__ == '__main__':
    main()
