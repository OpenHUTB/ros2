import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import torch
import torch.nn as nn
import numpy as np

class AvoidNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(8,32),
            nn.ReLU(),
            nn.Linear(32,2)
        )
    def forward(self,x):
        return self.net(x)

class NNAvoidNode(Node):
    def __init__(self):
        super().__init__("nn_avoid_node")
        self.pub = self.create_publisher(Twist,"/cmd_vel",10)
        self.sub = self.create_subscription(LaserScan,"/scan",self.scan_cb,10)
        self.model = AvoidNet()
        self.model.load_state_dict(torch.load("model.pth"))
        self.model.eval()

    def scan_cb(self,msg):
        scan = np.array(msg.ranges[::10])[:8]
        scan = np.clip(scan,0,10)
        tensor = torch.from_numpy(scan.astype(np.float32)).unsqueeze(0)
        res = self.model(tensor).detach().numpy()[0]
        twist = Twist()
        twist.linear.x = float(res[0])
        twist.angular.z = float(res[1])
        self.pub.publish(twist)

def main():
    rclpy.init()
    node = NNAvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()