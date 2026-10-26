#!/usr/bin/env python3
import rospy
import numpy as np
import torch
import torch.nn as nn
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarAutoencoder(nn.Module):
    def __init__(self):
        super(LidarAutoencoder, self).__init__()
        self.encoder = nn.Sequential(nn.Linear(36, 16), nn.ReLU(), nn.Linear(16, 8))
        self.decoder = nn.Sequential(nn.Linear(8, 16), nn.ReLU(), nn.Linear(16, 36))

    def forward(self, x):
        return self.decoder(self.encoder(x))

class ControlMLP(nn.Module):
    def __init__(self):
        super(ControlMLP, self).__init__()
        self.net = nn.Sequential(nn.Linear(36, 16), nn.ReLU(), nn.Linear(16, 2))
    def forward(self, x):
        return self.net(x)

class RepairNode:
    def __init__(self):
        rospy.init_node('lidar_repair_node')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.device = torch.device('cpu')
        self.autoencoder = LidarAutoencoder().to(self.device)
        self.control_net = ControlMLP().to(self.device)
        
        rospy.loginfo("Autoencoder 雷达修复节点已启动")

    def scan_callback(self, msg):
        raw_ranges = np.array(msg.ranges[:36], dtype=np.float32)
        raw_ranges = np.where(np.isfinite(raw_ranges), raw_ranges, 3.5)
        
        noise = np.random.normal(0, 0.5, 36).astype(np.float32)
        corrupted_ranges = raw_ranges + noise
        corrupted_tensor = torch.tensor(corrupted_ranges).unsqueeze(0).to(self.device)

        with torch.no_grad():
            repaired_tensor = self.autoencoder(corrupted_tensor)
            control_output = self.control_net(repaired_tensor)

        twist = Twist()
        
        front = min(min(msg.ranges[0:15] + msg.ranges[345:360]), 3.5)
        left = min(min(msg.ranges[15:60]), 3.5)
        right = min(min(msg.ranges[300:345]), 3.5)

        if front < 0.8:
            twist.linear.x = 0.05
            twist.angular.z = 1.0 if left > right else -1.0
        else:
            twist.linear.x = 0.3
            twist.angular.z = 0.0
        
        twist.angular.z += float(control_output[0][1].item()) * 0.01 

        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        node = RepairNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
