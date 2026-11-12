#!/usr/bin/env python3
"""2D LiDAR 模拟器
发布 /scan (sensor_msgs/LaserScan) 和 /ground_truth_pose (geometry_msgs/PoseStamped)
模拟一个机器人在带障碍物的 2D 环境中运动。
"""
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft

class LidarSimulator:
    def __init__(self):
        rospy.init_node('lidar_simulator')
        
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.pose_pub = rospy.Publisher('/ground_truth_pose', PoseStamped, queue_size=1)
        
        self.rate = rospy.Rate(10) # 10 Hz
        
        # 机器人初始状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 障碍物设置（线段集合）
        # 一个 10x10 的封闭空间，里面放几个障碍物
        self.obstacles = [
            ((2.0, 2.0), (2.0, 4.0)), ((2.0, 4.0), (4.0, 4.0)),
            ((4.0, 4.0), (4.0, 2.0)), ((4.0, 2.0), (2.0, 2.0)),
            ((-2.0, -2.0), (-2.0, 2.0)), ((-2.0, 2.0), (2.0, 2.0)),
            ((-2.0, 2.0), (2.0, 2.0)), ((2.0, 2.0), (2.0, -2.0)),
            ((2.0, -2.0), (-2.0, -2.0)), ((-2.0, -2.0), (-2.0, 2.0)),
            ((5.0, 5.0), (5.0, 8.0)), ((5.0, 8.0), (8.0, 8.0)),
            ((8.0, 8.0), (8.0, 5.0)), ((8.0, 5.0), (5.0, 5.0))
        ]
        
    def ray_cast(self, start_pt, angle):
        """发射一条射线，返回最近的障碍物距离"""
        max_range = 10.0
        ray_dir = (math.cos(angle), math.sin(angle))
        
        min_dist = max_range
        
        for (p1, p2) in self.obstacles:
            # 射线与线段求交
            x1, y1 = p1
            x2, y2 = p2
            x3, y3 = start_pt
            x4 = x3 + ray_dir[0]
            y4 = y3 + ray_dir[1]
            
            den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if den == 0:
                continue
                
            t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
            u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den
            
            if 0 < t < 1 and u > 0:
                dist = u
                if dist < min_dist:
                    min_dist = dist
                    
        return min_dist

    def run(self):
        rospy.loginfo("2D Lidar Simulator Ready")
        
        while not rospy.is_shutdown():
            # 机器人运动模型：画个圆圈
            self.x += 0.05 * math.cos(self.theta)
            self.y += 0.05 * math.sin(self.theta)
            self.theta += 0.02
            
            # 生成雷达数据
            num_rays = 360
            scan_msg = LaserScan()
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = "base_link"
            scan_msg.angle_min = -math.pi
            scan_msg.angle_max = math.pi
            scan_msg.angle_increment = 2 * math.pi / num_rays
            scan_msg.range_min = 0.1
            scan_msg.range_max = 10.0
            
            ranges = []
            for i in range(num_rays):
                angle = -math.pi + i * scan_msg.angle_increment
                dist = self.ray_cast((self.x, self.y), angle + self.theta)
                ranges.append(dist)
            scan_msg.ranges = ranges
            self.scan_pub.publish(scan_msg)
            
            # 发布真值位姿（方便对比）
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = self.x
            pose_msg.pose.position.y = self.y
            q = tft.quaternion_from_euler(0, 0, self.theta)
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]
            self.pose_pub.publish(pose_msg)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        LidarSimulator().run()
    except rospy.ROSInterruptException:
        pass
