#!/usr/bin/env python3
"""
无人机电池电量监控与自动返航(RTH)迫降节点.

分级状态:
  NORMAL          飞行正常
  WARNING         电量低于预警阈值, 提示但继续任务
  RETURN_HOME     电量低于返航阈值, 自动飞回起飞点
  EMERGENCY_LAND  电量不足以返航, 在最近安全点垂直迫降

话题:
  订阅: /drone/battery         std_msgs/Float32  (电量 %, 0~100)
  订阅: /drone/position        geometry_msgs/PointStamped (当前位置, map)
  发布: /drone/setpoint_position geometry_msgs/PointStamped
  发布: /drone/failsafe_state   std_msgs/String
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String


ST_NORMAL = "NORMAL"
ST_WARNING = "WARNING"
ST_RETURN_HOME = "RETURN_HOME"
ST_EMERGENCY_LAND = "EMERGENCY_LAND"


class BatteryRthNode(Node):
    def __init__(self):
        super().__init__("battery_rth_failsafe")

        self.declare_parameter("warn_percent", 25.0)
        self.declare_parameter("rth_percent", 15.0)
        self.declare_parameter("emergency_percent", 8.0)
        self.declare_parameter("cruise_speed_mps", 3.0)
        self.declare_parameter("power_per_meter", 0.15)  # 每米距离耗电百分比
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_z", 0.0)
        # 预设安全迫降点列表 [(x,y,z), ...]
        self.declare_parameter("safe_zones", "")

        self.warn = float(self.get_parameter("warn_percent").value)
        self.rth = float(self.get_parameter("rth_percent").value)
        self.emer = float(self.get_parameter("emergency_percent").value)
        self.speed = float(self.get_parameter("cruise_speed_mps").value)
        self.power_per_m = float(self.get_parameter("power_per_meter").value)
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

    def _pos_cb(self, msg: PointStamped):
        self.pos = (msg.point.x, msg.point.y, msg.point.z)

    def _dist_to(self, target):
        return math.sqrt(
            (self.pos[0] - target[0]) ** 2 +
            (self.pos[1] - target[1]) ** 2 +
            (self.pos[2] - target[2]) ** 2
        )

    def _nearest_safe_zone(self):
        if not self.safe_zones:
            # 无预设安全点: 在当前位置正上方降低高度迫降
            return (self.pos[0], self.pos[1], 0.0)
        return min(self.safe_zones, key=self._dist_to)

    def _publish_target(self, x, y, z):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.target_pub.publish(msg)

    def _loop(self):
        dist_home = self._dist_to(self.home)
        needed_home = dist_home * self.power_per_m

        if self.battery <= self.emer:
            self.state = ST_EMERGENCY_LAND
            zone = self._nearest_safe_zone()
            self._publish_target(zone[0], zone[1], 0.5)
            self.get_logger().warn(
                f"电量{self.battery:.1f}% <= {self.emer}%, 就近迫降 {zone}")
        elif self.battery <= self.rth:
            self.state = ST_RETURN_HOME
            self._publish_target(self.home[0], self.home[1], self.home[2] + 3.0)
            self.get_logger().warn(
                f"电量{self.battery:.1f}% <= {self.rth}%, 返航 home={self.home}")
        elif self.battery <= self.warn:
            self.state = ST_WARNING
            if needed_home > self.battery - self.emer:
                # 即使现在返航, 电量也不够了, 直接进入返航
                self.state = ST_RETURN_HOME
                self._publish_target(
                    self.home[0], self.home[1], self.home[2] + 3.0)
                self.get_logger().warn(
                    f"预警时估算返航需{needed_home:.1f}%, "
                    f"余量不足, 立即返航")
        else:
            self.state = ST_NORMAL

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
