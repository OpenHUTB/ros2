"""ROS2 节点：订阅里程计/IMU，发布 /cmd_vel 与状态。

设计上刻意做得很"薄"：所有决策都在 ``pilot.py`` 里，这里只负责消息转换。这样
真正会出错的地方（制导、控制、失效判定）都能在无 ROS 环境里跑单元测试。

订阅：
    /odom           nav_msgs/Odometry     仅用位姿与 twist
    /imu            sensor_msgs/Imu       可选，用于航向角速度融合
    /cmd_vel_nav    geometry_msgs/Twist   任务层指令（仅用于对比，不直接透传）

发布：
    /cmd_vel                  geometry_msgs/Twist
    /station_keeping/status   std_msgs/String（JSON）

参数见 ``declare_parameters``。
"""
from __future__ import annotations

import json
import math

try:  # 允许在没有 ROS 的机器上 import 本模块（例如跑单元测试）
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - 仅在无 ROS 环境触发
    rclpy = None
    Node = object  # type: ignore[assignment]
    Twist = Odometry = String = object  # type: ignore[assignment]

from .geometry import wrap_angle
from .monitor import HealthLevel
from .pilot import PilotConfig, StationKeepingPilot
from .vehicle import VehicleParams, VehicleState


class StationKeepingNode(Node):  # type: ignore[misc]
    """把定点保持驾驶员挂到 ROS 话题上。"""

    def __init__(self) -> None:
        super().__init__("station_keeping_node")
        self._declare()
        params = VehicleParams()
        cfg = PilotConfig()
        self.pilot = StationKeepingPilot(params=params, config=cfg)
        self.state = VehicleState()
        self.target = (self.get_parameter("target_x").value, self.get_parameter("target_y").value)
        self._last_msg_time = self.get_clock().now()

        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 10)
        self.status_pub = self.create_publisher(String, "/station_keeping/status", 10)
        hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f"定点保持已启动：target=({self.target[0]:.2f}, {self.target[1]:.2f})，{hz:.0f} Hz"
        )

    def _declare(self) -> None:
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("use_crab_compensation", True)

    def _on_odom(self, msg) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        q = pose.orientation
        # 由四元数取偏航角（只关心平面运动）
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.state = VehicleState(
            x=pose.position.x,
            y=pose.position.y,
            psi=wrap_angle(yaw),
            u=twist.linear.x,
            v=twist.linear.y,
            r=twist.angular.z,
        )
        self._last_msg_time = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        age = (now - self._last_msg_time).nanoseconds * 1e-9
        msg = Twist()
        if age > 1.0:
            # 里程计断流：按 fail-closed 处理，输出零速，不给底盘任何速度
            self.cmd_pub.publish(msg)
            self.pilot.reset(self.state.psi)
            self._publish_status(age, zero_reason="odom_timeout")
            return

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        command = self.pilot.step(self.state, self.target, dt)
        msg.linear.x = float(command.u_cmd if command.holding else 0.0)
        msg.angular.z = float(-command.heading_error)  # 体坐标系下用航向误差近似角速度
        self.cmd_pub.publish(msg)
        self._publish_status(age)

    def _publish_status(self, odom_age: float, zero_reason: str | None = None) -> None:
        snapshot = self.pilot.flow
        payload = {
            "target": list(self.target),
            "distance": round(float(math.hypot(self.state.x - self.target[0], self.state.y - self.target[1])), 3),
            "current_estimate": [round(c, 3) for c in snapshot.current_body],
            "health": getattr(self.pilot.health_level, "value", str(self.pilot.health_level)),
            "efficiency": round(float(self.pilot.health.efficiency), 3),
            "odom_age": None if zero_reason else round(odom_age, 3),
            "zero_reason": zero_reason,
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(out)

    def shutdown(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    if rclpy is None:
        raise SystemExit("未找到 rclpy：在线运行需要先 source ROS 2 环境；离线复现请用 python main.py")
    rclpy.init(args=args)
    node = StationKeepingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
