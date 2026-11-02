"""ROS 2 节点：MuJoCo 里跑无人机，发布深度图/通行度/里程计，订阅速度指令。

话题：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 发布 | `/uav/depth` | `sensor_msgs/Image` | 64×64 前视深度图（32FC1，米） |
| 发布 | `/uav/sector_clearance` | `std_msgs/Float32MultiArray` | 网络预测的左/中/右通行度 |
| 发布 | `/uav/odom` | `nav_msgs/Odometry` | 位置与速度（简化，姿态只填偏航） |
| 订阅 | `/uav/cmd_vel` | `geometry_msgs/Twist` | `linear.x` 前向速度，`angular.z` 偏航角速度 |
| 订阅 | `/uav/scene` | `std_msgs/String` | 切换场景：slalom / gate / clutter |

没有收到 `/uav/cmd_vel` 时节点保持悬停，因此也可以只启动它观察传感器数据。
"""

from __future__ import annotations

import numpy as np


def main() -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray, String

    from .env import DepthUavEnv
    from .net import DepthCNN
    from .policy import PolicyConfig, act

    class UavDepthNode(Node):
        def __init__(self):
            super().__init__("uav_mujoco_depth_avoidance")
            self.declare_parameter("scene", "slalom")
            self.declare_parameter("use_policy", True)
            self.declare_parameter("rate_hz", 20.0)
            scene = str(self.get_parameter("scene").value)
            self.use_policy = bool(self.get_parameter("use_policy").value)

            self.env = DepthUavEnv(scene)
            self.net = DepthCNN(seed=0)
            self.cfg = PolicyConfig()
            self.vx_cmd = 0.0
            self.yaw_cmd = 0.0

            self.pub_depth = self.create_publisher(Image, "/uav/depth", 10)
            self.pub_clearance = self.create_publisher(Float32MultiArray, "/uav/sector_clearance", 10)
            self.pub_odom = self.create_publisher(Odometry, "/uav/odom", 10)
            self.create_subscription(Twist, "/uav/cmd_vel", self.on_cmd, 10)
            self.create_subscription(String, "/uav/scene", self.on_scene, 10)

            rate = float(self.get_parameter("rate_hz").value)
            self.timer = self.create_timer(1.0 / rate, self.tick)
            self.get_logger().info(
                f"uav depth avoidance 已启动：scene={scene}，{rate:.0f} Hz，"
                f"use_policy={self.use_policy}（未收到 /uav/cmd_vel 时按策略/悬停飞行）")

        def on_cmd(self, msg: Twist):
            self.vx_cmd = float(msg.linear.x)
            self.yaw_cmd = float(msg.angular.z)

        def on_scene(self, msg: String):
            name = msg.data.strip()
            if name in DepthUavEnv.SCENES:
                self.env.reset(scene=name)
                self.get_logger().info(f"切换场景 {name}")

        def tick(self):
            depth = self.env.depth()
            state = self.env.state()
            vx_policy, yaw_policy, c = act(self.net, depth, self.cfg, yaw=state.yaw)

            if abs(self.vx_cmd) < 1e-6 and abs(self.yaw_cmd) < 1e-6 and self.use_policy:
                vx, yaw_rate = vx_policy, yaw_policy
            else:
                vx, yaw_rate = self.vx_cmd, self.yaw_cmd

            state = self.env.step(vx, yaw_rate)

            img = Image()
            img.header.stamp = self.get_clock().now().to_msg()
            img.header.frame_id = "uav_depth_cam"
            img.height, img.width = depth.shape
            img.encoding = "32FC1"
            img.is_bigendian = 0
            img.step = depth.shape[1] * 4
            img.data = depth.astype(np.float32).tobytes()
            self.pub_depth.publish(img)

            msg = Float32MultiArray()
            msg.data = [float(v) for v in c]
            self.pub_clearance.publish(msg)

            odom = Odometry()
            odom.header = img.header
            odom.child_frame_id = "uav_base"
            odom.pose.pose.position.x = float(state.pos[0])
            odom.pose.pose.position.y = float(state.pos[1])
            odom.pose.pose.position.z = float(state.pos[2])
            odom.pose.pose.orientation.z = float(np.sin(state.yaw / 2))
            odom.pose.pose.orientation.w = float(np.cos(state.yaw / 2))
            odom.twist.twist.linear.x = float(state.vel[0])
            odom.twist.twist.linear.y = float(state.vel[1])
            odom.twist.twist.linear.z = float(state.vel[2])
            self.pub_odom.publish(odom)

            if state.crashed:
                self.get_logger().warn("检测到碰撞，重置场景")
                self.env.reset()

    rclpy.init()
    node = UavDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
