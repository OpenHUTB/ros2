#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

class GridMapper:
    def __init__(self):
        rospy.init_node('grid_mapper')
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        
        self.resolution = 0.1
        self.width = 200
        self.height = 200
        self.origin_x = -10.0
        self.origin_y = -10.0
        
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.current_odom = None
        
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.loginfo("Grid Mapper Ready")

    def odom_callback(self, msg):
        self.current_odom = msg

    def scan_callback(self, msg):
        if self.current_odom is None:
            return
            
        x = self.current_odom.pose.pose.position.x
        y = self.current_odom.pose.pose.position.y
        
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        updated = False
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            global_angle = theta + angle
            
            obs_x = x + r * math.cos(global_angle)
            obs_y = y + r * math.sin(global_angle)
            
            grid_x = int((obs_x - self.origin_x) / self.resolution)
            grid_y = int((obs_y - self.origin_y) / self.resolution)
            
            if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                self.grid[grid_y, grid_x] = 100
                updated = True
                
        if updated:
            self.publish_map(msg.header.stamp)

    def publish_map(self, stamp):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = stamp
        map_msg.header.frame_id = "odom"
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(map_msg)

if __name__ == '__main__':
    try:
        GridMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
