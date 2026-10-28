#!/usr/bin/env python3
"""
无人机兴趣点(POI)环绕航拍节点.

给定地面 POI 坐标、环绕半径、飞行高度, 自动生成圆周航点序列,
偏航角始终对准 POI(模拟云台俯视拍摄)。可选螺旋升降。

话题:
  发布: /drone/setpoint_position  geometry_msgs/PointStamped
  发布: /drone/target_yaw         std_msgs/Float64 (rad)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64


class PoiOrbitNode(Node):
    def __init__(self):
        super().__init__("uav_poi_orbit")

        self.declare_parameter("poi_x", 0.0)
        self.declare_parameter("poi_y", 0.0)
        self.declare_parameter("orbit_radius", 5.0)
        self.declare_parameter("orbit_height", 8.0)
        self.declare_parameter("omega_dps", 15.0)   # 角速度 deg/s
        self.declare_parameter("spiral", False)      # 是否螺旋升降
        self.declare_parameter("spiral_height_delta", 2.0)  # 每圈高度变化
        self.declare_parameter("waypoint_interval_s", 0.2)

        self.poi = (
            float(self.get_parameter("poi_x").value),
            float(self.get_parameter("poi_y").value),
        )
        self.r = float(self.get_parameter("orbit_radius").value)
        self.h = float(self.get_parameter("orbit_height").value)
        self.omega = math.radians(float(self.get_parameter("omega_dps").value))
        self.spiral = bool(self.get_parameter("spiral").value)
        self.spiral_dh = float(self.get_parameter("spiral_height_delta").value)
        self.dt = float(self.get_parameter("waypoint_interval_s").value)

        self.angle = 0.0
        self.turns = 0.0

        self.target_pub = self.create_publisher(
            PointStamped, "/drone/setpoint_position", 10)
        self.yaw_pub = self.create_publisher(
            Float64, "/drone/target_yaw", 10)

        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info(
            f"POI 环绕启动: POI={self.poi}, r={self.r}m, h={self.h}m, "
            f"omega={self.get_parameter('omega_dps').value}deg/s")

    def _step(self):
        self.angle += self.omega * self.dt
        if self.angle >= 2 * math.pi:
            self.angle -= 2 * math.pi
            self.turns += 1.0
            self.get_logger().info(f"完成第 {self.turns:.0f} 圈")

        # 圆周位置
        x = self.poi[0] + self.r * math.cos(self.angle)
        y = self.poi[1] + self.r * math.sin(self.angle)

        z = self.h
        if self.spiral:
            # 每转一圈抬升 spiral_dh
            z = self.h + self.turns * self.spiral_dh

        # 偏航对准 POI
        yaw = math.atan2(self.poi[1] - y, self.poi[0] - x)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.target_pub.publish(msg)
        self.yaw_pub.publish(Float64(data=float(yaw)))


def main(args=None):
    rclpy.init(args=args)
    node = PoiOrbitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
