#!/usr/bin/env python3
"""
无人机电池电量监控与自动返航(RTH)迫降节点.

分级状态:
  NORMAL          飞行正常 (不发布 setpoint, 由其他节点控制)
  WARNING         电量低于预警阈值, 提示但继续任务
  RETURN_HOME     电量低于返航阈值, 自动飞回起飞点
  EMERGENCY_LAND  电量不足以返航, 在最近安全点垂直迫降

话题:
  订阅: /drone/battery         std_msgs/Float32  (电量 %, 0~100)
  订阅: /drone/position        geometry_msgs/PointStamped (当前位置, map)
  发布: /drone/setpoint_position geometry_msgs/PointStamped (仅在 failsafe 激活时)
  发布: /drone/failsafe_state   std_msgs/String
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String

from rth_math import decide_state, dist_3d, ST_NORMAL, ST_WARNING, ST_RETURN_HOME, ST_EMERGENCY_LAND


class BatteryRthNode(Node):
    def __init__(self):
        super().__init__("battery_rth_failsafe")

        self.declare_parameter("warn_percent", 25.0)
        self.declare_parameter("rth_percent", 15.0)
        self.declare_parameter("emergency_percent", 8.0)
        self.declare_parameter("cruise_speed_mps", 3.0)
        self.declare_parameter("power_per_meter", 0.15)
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_z", 0.0)
        self.declare_parameter("battery_timeout", 3.0)
        self.declare_parameter("safe_zones", "")

        self.warn = float(self.get_parameter("warn_percent").value)
        self.rth = float(self.get_parameter("rth_percent").value)
        self.emer = float(self.get_parameter("emergency_percent").value)
        self.speed = float(self.get_parameter("cruise_speed_mps").value)
        self.power_per_m = float(self.get_parameter("power_per_meter").value)
        self.batt_timeout = float(self.get_parameter("battery_timeout").value)
        self.home = (
            float(self.get_parameter("home_x").value),
            float(self.get_parameter("home_y").value),
            float(self.get_parameter("home_z").value),
        )

        self.safe_zones = []
        sz = self.get_parameter("safe_zones").value
        if sz:
            for item in sz.split(";"):
                try:
                    x, y, z = [float(v) for v in item.split(",")]
                    self.safe_zones.append((x, y, z))
                except ValueError:
                    pass

        self.state = ST_NORMAL
        self.battery = 100.0
        self.pos = (0.0, 0.0, 0.0)
        self.last_batt_time = self.get_clock().now()
        self.failsafe_active = False

        self.batt_sub = self.create_subscription(
            Float32, "/drone/battery", self._batt_cb, 10)
        self.pos_sub = self.create_subscription(
            PointStamped, "/drone/position", self._pos_cb, 10)
        self.target_pub = self.create_publisher(
            PointStamped, "/drone/setpoint_position", 10)
        self.state_pub = self.create_publisher(
            String, "/drone/failsafe_state", 10)

        self.timer = self.create_timer(0.2, self._loop)
        self.get_logger().info(
            f"电池RTH节点已启动: 预警<{self.warn}%, 返航<{self.rth}%, "
            f"迫降<{self.emer}%")

    def _batt_cb(self, msg: Float32):
        self.battery = float(msg.data)
        self.last_batt_time = self.get_clock().now()

    def _pos_cb(self, msg: PointStamped):
        self.pos = (msg.point.x, msg.point.y, msg.point.z)

    def _nearest_safe_zone(self):
        if not self.safe_zones:
            return (self.pos[0], self.pos[1], 0.0)
        return min(self.safe_zones, key=lambda z: dist_3d(self.pos, z))

    def _publish_target(self, x, y, z):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.target_pub.publish(msg)

    def _loop(self):
        now = self.get_clock().now()
        batt_age = (now - self.last_batt_time).nanoseconds / 1e9

        # 电池话题断流: 安全起见进入迫降
        if batt_age > self.batt_timeout:
            self.state = ST_EMERGENCY_LAND
            zone = self._nearest_safe_zone()
            self._publish_target(zone[0], zone[1], 0.5)
            self.get_logger().warn(
                f"电池话题超时({batt_age:.1f}s), 就近迫降")
            self.state_pub.publish(String(data=self.state))
            return

        dist_home = dist_3d(self.pos, self.home)
        new_state = decide_state(
            self.battery, self.warn, self.rth, self.emer,
            dist_home, self.power_per_m)

        # 状态跃迁日志
        if new_state != self.state:
            self.get_logger().info(
                f"故障状态: {self.state} -> {new_state} "
                f"(电量={self.battery:.1f}%, dist_home={dist_home:.1f}m)")
        self.state = new_state

        # 只在 failsafe 激活时发布 setpoint
        if self.state == ST_RETURN_HOME:
            self._publish_target(self.home[0], self.home[1],
                                  self.home[2] + 3.0)
        elif self.state == ST_EMERGENCY_LAND:
            zone = self._nearest_safe_zone()
            self._publish_target(zone[0], zone[1], 0.5)

        self.state_pub.publish(String(data=self.state))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryRthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
