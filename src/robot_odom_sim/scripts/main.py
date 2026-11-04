#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

class RobotSim:
    def __init__(self):
        rospy.init_node('robot_odom_sim')
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.vx = 0.0
        self.vth = 0.0
        
        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(20)

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def run(self):
        print("机器人底层仿真节点已启动，等待速度指令...")
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            
            delta_x = self.vx * math.cos(self.th) * dt
            delta_y = self.vx * math.sin(self.th) * dt
            delta_th = self.vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.th)

            self.tf_broadcaster.sendTransform(
                (self.x, self.y, 0.0),
                quaternion,
                rospy.Time.now(),
                "base_link",
                "map"
            )

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.x = quaternion[0]
            odom.pose.pose.orientation.y = quaternion[1]
            odom.pose.pose.orientation.z = quaternion[2]
            odom.pose.pose.orientation.w = quaternion[3]
            self.odom_pub.publish(odom)

            self.last_time = current_time
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = RobotSim()
        robot.run()
    except rospy.ROSInterruptException:
        pass
