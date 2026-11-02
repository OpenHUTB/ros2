# -*- coding: utf-8 -*-
"""ROS 2（Humble）封装：把仿真闭环包装成标准节点。

发布：
    ~/scan      sensor_msgs/LaserScan   射线扇面（frame_id 由参数指定，默认 lidar）
    ~/odom      nav_msgs/Odometry       里程计（map -> base_link）
    ~/decision  std_msgs/String         决策状态（JSON：状态/距离/目标速度/实际指令）
订阅：
    ~/cmd_vel   geometry_msgs/Twist     手动接管；超过 teleop_timeout_s 未收到指令则回到自主避障

说明：
    - 节点内部仍复用 AvoidanceRunner，因此"ROS 下跑"与"命令行下跑"是同一套逻辑；
    - 未安装 rclpy 时本文件不参与运行，独立入口（main.py）不受影响。
"""

import json
import math

from . import config
from .car_env import CarEnv, MujocoUnavailable, describe_environment
from .decision import DecisionParams
from .runner import AvoidanceRunner


def main(argv=None) -> int:  # pragma: no cover - 需要 ROS 2 运行环境
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
    except Exception as exc:
        print("无法导入 ROS 2 Python 接口（{}）。请先 source ROS 2 环境：\n"
              "  source /opt/ros/humble/setup.bash\n"
              "  ros2 launch mujoco_car_avoidance main.launch.py".format(exc))
        return 1

    class MujocoCarAvoidanceNode(Node):
        """仿真 + 避障 + 话题发布。"""

        def __init__(self):
            super().__init__("mujoco_car_avoidance")

            # ---------------- 参数 ----------------
            self.declare_parameter("scene", "")
            self.declare_parameter("control_hz", config.CONTROL_HZ)
            self.declare_parameter("frame_id", "lidar")
            self.declare_parameter("odom_frame_id", "map")
            self.declare_parameter("base_frame_id", "base_link")
            self.declare_parameter("teleop_timeout_s", config.EXTERNAL_CMD_TIMEOUT_S)
            self.declare_parameter("settle_s", config.SETTLE_SECONDS)
            self.declare_parameter("heading_gain", config.HEADING_HOLD_GAIN)
            self.declare_parameter("cruise_speed", DecisionParams.cruise_speed)
            self.declare_parameter("slow_start_m", DecisionParams.slow_start_m)
            self.declare_parameter("stop_target_m", DecisionParams.stop_target_m)
            self.declare_parameter("emergency_m", DecisionParams.emergency_m)
            self.declare_parameter("stale_timeout_s", DecisionParams.stale_timeout_s)

            scene = self.get_parameter("scene").get_parameter_value().string_value
            control_hz = float(self.get_parameter("control_hz").value)
            self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
            self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
            self.base_frame_id = self.get_parameter("base_frame_id").get_parameter_value().string_value
            self.teleop_timeout = float(self.get_parameter("teleop_timeout_s").value)

            params = DecisionParams(
                cruise_speed=float(self.get_parameter("cruise_speed").value),
                slow_start_m=float(self.get_parameter("slow_start_m").value),
                stop_target_m=float(self.get_parameter("stop_target_m").value),
                emergency_m=float(self.get_parameter("emergency_m").value),
                stale_timeout_s=float(self.get_parameter("stale_timeout_s").value),
            )

            # ---------------- 仿真与闭环 ----------------
            self.get_logger().info("环境检查：{}".format(describe_environment()))
            self.env = CarEnv(config.resolve_scene_path(scene), control_hz=control_hz)
            self.runner = AvoidanceRunner(
                self.env, params=params, control_hz=control_hz,
                settle_seconds=float(self.get_parameter("settle_s").value),
                heading_hold_gain=float(self.get_parameter("heading_gain").value),
            )
            # 起步静置：零轮速推进，等轮胎接触建立后再开话题（与命令行入口一致）
            self.runner.settle()
            self.get_logger().info(
                "起步静置完成（{:.2f}s），航向保持增益 {:.2f}".format(
                    float(self.get_parameter("settle_s").value),
                    float(self.get_parameter("heading_gain").value)))

            # ---------------- ROS 接口 ----------------
            self.publisher_scan = self.create_publisher(LaserScan, "~/scan", 10)
            self.publisher_odom = self.create_publisher(Odometry, "~/odom", 10)
            self.publisher_decision = self.create_publisher(String, "~/decision", 10)
            self.subscription = self.create_subscription(
                Twist, "~/cmd_vel", self.on_cmd_vel, 10)

            self._manual_cmd = None
            self._manual_stamp = None
            self._last_log = 0.0
            self.timer = self.create_timer(1.0 / control_hz, self.on_timer)

            angles = config.LIDAR_ANGLES_DEG
            self.get_logger().info(
                "节点就绪：{} 束射线，扇面 {:.0f}°~{:.0f}°，减速起点 {:.1f} m，"
                "停车目标 {:.1f} m".format(
                    len(angles), angles[0], angles[-1],
                    params.slow_start_m, params.stop_target_m))

        # -------------------------------------------------------------- 回调
        def on_cmd_vel(self, msg):
            """外部速度指令：记录时间戳，超时后自动交还自主避障。"""
            self._manual_cmd = (float(msg.linear.x), float(msg.angular.z))
            self._manual_stamp = self.get_clock().now()

        def _manual_if_fresh(self):
            if self._manual_cmd is None or self._manual_stamp is None:
                return None
            age = (self.get_clock().now() - self._manual_stamp).nanoseconds * 1e-9
            if age > self.teleop_timeout:
                return None
            return self._manual_cmd

        def on_timer(self):
            info = self.runner.step_once(self._manual_if_fresh())
            stamp = self.get_clock().now().to_msg()

            self.publisher_scan.publish(self._make_scan(info, stamp))
            self.publisher_odom.publish(self._make_odom(info, stamp))
            self.publisher_decision.publish(self._make_decision(info))

            if info.t - self._last_log >= 1.0:
                self._last_log = info.t
                self.get_logger().info(
                    "[t={:5.1f}s] {:<10} d={} v={:+.2f} cmd={:+.2f} 车头间距={:.2f}m{}".format(
                        info.t, info.state.value,
                        "N/A" if info.d_sensor is None else "{:.2f}m".format(info.d_sensor),
                        info.v_actual, info.v_cmd, info.clearance,
                        "  碰撞" if info.collision else ""))

        # -------------------------------------------------------------- 组包
        def _make_scan(self, info, stamp):
            from sensor_msgs.msg import LaserScan

            msg = LaserScan()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            angles = config.LIDAR_ANGLES_DEG
            msg.angle_min = math.radians(angles[0])
            msg.angle_max = math.radians(angles[-1])
            msg.angle_increment = (math.radians(angles[-1] - angles[0]) / (len(angles) - 1)
                                   if len(angles) > 1 else 0.0)
            msg.range_min = 0.05
            msg.range_max = float(self.env.lidar.max_range)
            msg.scan_time = self.runner.dt
            msg.time_increment = 0.0
            msg.ranges = [float(v) for v in info.ranges]
            return msg

        def _make_odom(self, info, stamp):
            from nav_msgs.msg import Odometry

            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = self.odom_frame_id
            msg.child_frame_id = self.base_frame_id
            msg.pose.pose.position.x = info.x
            msg.pose.pose.position.y = info.y
            msg.pose.pose.orientation.z = math.sin(info.yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(info.yaw / 2.0)
            msg.twist.twist.linear.x = info.v_actual
            msg.twist.twist.angular.z = info.angular
            return msg

        def _make_decision(self, info):
            from std_msgs.msg import String

            msg = String()
            msg.data = json.dumps({
                "state": info.state.value,
                "reason": info.reason,
                "distance_m": info.d_sensor,
                "target_speed_mps": info.v_target,
                "command_mps": info.v_cmd,
                "clearance_m": info.clearance,
                "collision": info.collision,
            }, ensure_ascii=False)
            return msg

    rclpy.init(args=argv)
    node = None
    try:
        node = MujocoCarAvoidanceNode()
        rclpy.spin(node)
    except MujocoUnavailable as exc:
        print(exc)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
