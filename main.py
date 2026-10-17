#!/usr/bin/env python3
import rospy
import torch
import torch.nn as nn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AvoidNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(360, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 2)
        )

    def forward(self, x):
        return self.layers(x)

model = AvoidNet()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def scan_callback(msg):
    global model, pub
    ranges = torch.tensor(msg.ranges, dtype=torch.float32).nan_to_num(10.0)
    v, w = model(ranges)
    cmd = Twist()
    cmd.linear.x = torch.clamp(v, 0.1, 0.3).item()
    cmd.angular.z = torch.clamp(w, -2.0, 2.0).item()
    pub.publish(cmd)

rospy.init_node('nn_avoid')
rospy.Subscriber('/scan', LaserScan, scan_callback)
rospy.spin()