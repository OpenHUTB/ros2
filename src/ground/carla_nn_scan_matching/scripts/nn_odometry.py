#!/usr/bin/env python3
import os
import sys
import rospy
import math
import numpy as np
import torch
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf

pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(pkg_root, 'src'))
from model import ScanMatcherCNN

class NNOdometry:
    def __init__(self):
        rospy.init_node('nn_odometry')
        model_path = rospy.get_param('~model_path', 'scan_match_model.pth')
        
        self.model = ScanMatcherCNN()
        if os.path.exists(model_path):
            self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
            rospy.loginfo("Loaded model weights: " + model_path)
        else:
            rospy.logwarn("Model weights not found, using random init")
        self.model.eval()
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_scan = None
        rospy.loginfo("Neural Network Odometry Node Ready")

    def scan_callback(self, msg):
        current_scan = np.array(msg.ranges, dtype=np.float32)
        current_scan = np.clip(current_scan, 0.0, 10.0)
        
        if self.prev_scan is not None:
            input_data = np.stack([self.prev_scan, current_scan], axis=0)
            tensor = torch.from_numpy(input_data).unsqueeze(0)
            
            with torch.no_grad():
                out = self.model(tensor)
                
            dx, dy, dtheta = out[0].tolist()
            
            self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
            self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
            self.theta += dtheta
            
            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            self.odom_pub.publish(odom)
            
            self.tf_broadcaster.sendTransform(
                (self.x, self.y, 0), q, msg.header.stamp, "base_link", "odom"
            )
            
            rospy.loginfo_throttle(2.0, f"Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
            
        self.prev_scan = current_scan

if __name__ == '__main__':
    try:
        NNOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
