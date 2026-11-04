import rospkg
import rospy
import numpy as np
import torch
import torch.nn as nn
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

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

class NNAvoidNode():
    def __init__(self):
        # 加载模型
        self.model = AvoidNet()
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('nn_avoidance')
        model_full_path = pkg_path + "/nn_avoidance/model.pth"
        self.model.load_state_dict(torch.load(model_full_path))
        self.model.eval()
        # 订阅激光雷达
        self.sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        # 发布小车速度
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def scan_cb(self, msg):
        # 取8个激光点，和你原来逻辑保持一致
        scan = np.array(msg.ranges[::10][:8])
        scan = np.clip(scan,0,10)
        tensor = torch.from_numpy(scan.astype(np.float32)).unsqueeze(0)
        with torch.no_grad():
            res = self.model(tensor).detach().numpy()[0]
        twist = Twist()
        twist.linear.x = float(res[0])
        twist.angular.z = float(res[1])
        self.pub.publish(twist)

def main():
    rospy.init_node("nn_avoid_node")
    node = NNAvoidNode()
    rospy.spin()

if __name__=="__main__":
    main()