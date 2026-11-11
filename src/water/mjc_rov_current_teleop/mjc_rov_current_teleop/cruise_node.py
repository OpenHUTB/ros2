"""
cruise_node.py —— ROS 巡航节点：加载训练好的 MLP，订阅 Odom，输出速度指令。
为什么单独成节点：与 sim_node 解耦，launch 可只起 sim，
也可 sim+cruise 一起起（MLP 自动驾驶），也可 sim+keyboard 一起起（人控）。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from . import common
from .cruise_net import CruiseMLP, EXPERT_WAYPOINTS

import torch


class CruiseNode(Node):
    def __init__(self):
        super().__init__("rov_cruise_node")
        self.odom_topic = self.declare_parameter("odom_topic", "/rov_teleop/odom").value
        self.cmd_topic = self.declare_parameter("cmd_topic", "/rov_teleop/cmd_vel").value
        self.weights_path = self.declare_parameter("weights_path", "").value
        wp = self.declare_parameter("waypoints",
                                    [2.0, 0.0, -1.0, 0.0, 2.0, -1.0,
                                     -2.0, 0.0, -1.0, 0.0, -2.0, -1.0]).value
        self.waypoints = np.array(wp, dtype=np.float32).reshape(-1, 3)

        # 加载权重（不存在就给可操作错误）
        wpath = common.find_weights_path(self.weights_path)
        if not wpath.exists():
            raise FileNotFoundError(
                f"找不到 MLP 权重 {wpath}。\n"
                "请先在包目录执行： python3 main.py train"
            )
        ckpt = torch.load(wpath, map_location="cpu")
        self.net = CruiseMLP()
        self.net.load_state_dict(ckpt["model"])
        self.net.eval()
        self.get_logger().info(f"MLP 巡航策略已加载: {wpath}")

        self._state = np.zeros(7)  # x,y,z,yaw,vx,vy,wz
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_timer(0.05, self._step)

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # quat -> yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny, cosy)
        self._state = np.array([
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z, yaw,
            msg.twist.twist.linear.x, msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        ])

    def _step(self):
        # 选最近航点
        d = [np.hypot(w[0] - self._state[0], w[1] - self._state[1])
             for w in self.waypoints]
        wp = self.waypoints[int(np.argmin(d))]
        feat = np.array([wp[0] - self._state[0], wp[1] - self._state[1],
                         (np.arctan2(wp[1] - self._state[1], wp[0] - self._state[0])
                          - self._state[3]),
                         self._state[4], self._state[5], self._state[6]],
                        dtype=np.float32)
        with torch.no_grad():
            cmd = self.net(torch.tensor(feat).unsqueeze(0)).squeeze(0).numpy()
        twist = Twist()
        twist.linear.x = float(cmd[0])
        twist.linear.y = float(cmd[1])
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = CruiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
