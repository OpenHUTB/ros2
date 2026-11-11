#!/usr/bin/env python3
import rospy
import math
import time
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
        self.last_cmd_time = time.time() # 记录最后一次收到指令的时间
        self.rate = rospy.Rate(20)

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z
        self.last_cmd_time = time.time() # 每次收到键盘指令就刷新时间

    def run(self):
        print("机器人底层仿真节点已启动，等待速度指令...")
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            
            # 【修复】：速度超时保护，0.5秒没收到新指令，速度归零，防止机器人飞出去
            if time.time() - self.last_cmd_time > 0.5:
                self.vx = 0.0
                self.vth = 0.0

            # 运动学积分
            delta_x = self.vx * math.cos(self.th) * dt
            delta_y = self.vx * math.sin(self.th) * dt
            delta_th = self.vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # 【修复】：TF树改为 odom -> base_link
            self.tf_broadcaster.sendTransform(
                (self.x, self.y, 0.0),
                quaternion,
                rospy.Time.now(),
                "base_link",
                "odom"
            )

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            
            # 【修复】：补全 Odometry 的线速度和角速度
            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.angular.z = self.vth
            
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
