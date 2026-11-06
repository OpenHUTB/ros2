#!/usr/bin/env python3
"""
无人机兴趣点(POI)环绕航拍节点.

给定地面 POI 坐标、环绕半径、飞行高度, 自动生成连续圆周航点序列,
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

from .orbit_math import (
    advance_angle,
    compute_height,
    compute_orbit_position,
    compute_yaw_to_poi,
    count_full_turns,
    validate_params,
)


class PoiOrbitNode(Node):
    def __init__(self):
        super().__init__("uav_poi_orbit")

        self.declare_parameter("poi_x", 0.0)
        self.declare_parameter("poi_y", 0.0)
        self.declare_parameter("orbit_radius", 5.0)
        self.declare_parameter("orbit_height", 8.0)
        self.declare_parameter("omega_dps", 15.0)
        self.declare_parameter("spiral", False)
        self.declare_parameter("spiral_height_delta", 2.0)
        self.declare_parameter("waypoint_interval_s", 0.2)
        self.declare_parameter("min_height", 0.5)

        self.poi = (
            float(self.get_parameter("poi_x").value),
            float(self.get_parameter("poi_y").value),
        )
        self.r = float(self.get_parameter("orbit_radius").value)
        self.h = float(self.get_parameter("orbit_height").value)
        self.omega_dps = float(self.get_parameter("omega_dps").value)
        self.omega = math.radians(self.omega_dps)
        self.spiral = bool(self.get_parameter("spiral").value)
        self.spiral_dh = float(self.get_parameter("spiral_height_delta").value)
        self.dt = float(self.get_parameter("waypoint_interval_s").value)
        self.min_h = float(self.get_parameter("min_height").value)

        # 参数校验: 非法参数直接拒绝启动, 避免无人机按错误航线飞行
        errors = validate_params(self.r, self.h, self.omega, self.dt, self.min_h)
        if errors:
            for e in errors:
                self.get_logger().error(f"参数非法: {e}")
            raise RuntimeError("POI 环绕参数校验失败: " + "; ".join(errors))

        # 状态: angle 归一化, total_angle 连续累积用于螺旋高度
        self.angle = 0.0
        self.total_angle = 0.0
        self.last_turns = 0.0

        self.target_pub = self.create_publisher(
            PointStamped, "/drone/setpoint_position", 10)
        self.yaw_pub = self.create_publisher(
            Float64, "/drone/target_yaw", 10)

        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info(
            f"POI 环绕启动: POI=({self.poi[0]:.1f},{self.poi[1]:.1f}), "
            f"r={self.r}m, h={self.h}m, omega={self.omega_dps}deg/s, "
            f"spiral={self.spiral}")

    def _step(self):
        self.angle, self.total_angle = advance_angle(
            self.angle, self.total_angle, self.omega, self.dt)

        turns = count_full_turns(self.total_angle)
        if turns >= self.last_turns + 1.0:
            self.last_turns = math.floor(turns)
            self.get_logger().info(f"完成第 {int(turns)} 圈")

        x, y = compute_orbit_position(
            self.poi[0], self.poi[1], self.r, self.angle)
        z = compute_height(
            self.h, self.total_angle, self.spiral, self.spiral_dh, self.min_h)
        yaw = compute_yaw_to_poi(self.poi[0], self.poi[1], x, y)

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
