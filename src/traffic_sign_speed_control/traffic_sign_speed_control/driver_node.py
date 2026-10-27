# -*- coding: utf-8 -*-
"""驾驶节点：巡航 P 控制 + 神经网络限速器（运动控制）。

- 横向：沿圆形赛道取前方预瞄点，航向误差 P 控制转角；
- 纵向：目标速度 = min(巡航速度, 生效限速)，并施加加速度限幅（平滑减速）；
- 限速语义：检测节点报告视野内限速牌时生效，驶离视野并保持
  limit_hold_time 秒后自动解除（抗单帧漏检抖动）。
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32

from . import scene as S


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class DriverNode(Node):
    def __init__(self):
        super().__init__("driver_node")
        # ---- 参数 ----
        self.declare_parameter("cruise_speed", S.CRUISE_SPEED)   # 巡航速度 m/s
        self.declare_parameter("k_heading", 2.2)                 # 航向 P 增益
        self.declare_parameter("accel", 1.0)                     # 加速度限幅 m/s^2
        self.declare_parameter("decel", 1.6)                     # 减速度限幅 m/s^2
        self.declare_parameter("lookahead_m", 0.45)              # 预瞄弧长 m
        self.declare_parameter("limit_hold_time", 1.2)           # 限速保持时间 s
        self.cruise = float(self.get_parameter("cruise_speed").value)
        self.k = float(self.get_parameter("k_heading").value)
        self.accel = float(self.get_parameter("accel").value)
        self.decel = float(self.get_parameter("decel").value)
        self.lookahead = float(self.get_parameter("lookahead_m").value)
        self.hold = float(self.get_parameter("limit_hold_time").value)

        # ---- 状态 ----
        self.x = self.y = self.theta = 0.0
        self.v_cmd = 0.0
        self.active_limit = None        # 生效限速（m/s），None 表示无限速
        self.last_positive_ns = None    # 最近一次检测到牌的时间
        self._logged_limit = None

        self.sub_odom = self.create_subscription(Odometry, "world/odom",
                                                 self.on_odom, 10)
        self.sub_val = self.create_subscription(Int32, "speed_limit",
                                                self.on_limit, 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.05, self.control)   # 20 Hz 控制
        self.get_logger().info(
            f"driver 就绪: cruise={self.cruise} m/s, k={self.k}, "
            f"accel={self.accel}, decel={self.decel}, hold={self.hold}s")

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.theta = 2.0 * math.atan2(q.z, q.w)

    def on_limit(self, msg):
        """限速更新：>0 生效；-1 时按 hold 时间保持后解除。"""
        now = self.get_clock().now().nanoseconds
        if msg.data > 0:
            v_lim = msg.data * S.KMH2MS
            self.last_positive_ns = now
            if self.active_limit != v_lim:
                self.get_logger().info(
                    f"感知到限速牌 {msg.data} km/h -> 目标速度 {v_lim:.2f} m/s")
                self.active_limit = v_lim
        elif self.active_limit is not None:
            if self.last_positive_ns is not None and \
                    (now - self.last_positive_ns) * 1e-9 >= self.hold:
                self.get_logger().info(
                    f"限速解除 -> 恢复巡航 {self.cruise:.2f} m/s")
                self.active_limit = None

    def control(self):
        # 目标速度：限速生效时取 min(巡航, 限速)，并做加速度限幅
        if self.active_limit is not None:
            v_target = min(self.cruise, self.active_limit)
        else:
            v_target = self.cruise
        if v_target > self.v_cmd:
            self.v_cmd = min(v_target, self.v_cmd + self.accel * 0.05)
        else:
            self.v_cmd = max(v_target, self.v_cmd - self.decel * 0.05)
        # 横向预瞄 P 控制（世界节点初始切向恒沿角度增大方向）
        cx = S.TRACK_CENTER[0] / S.PX_PER_M
        cy = S.TRACK_CENTER[1] / S.PX_PER_M
        r = S.TRACK_RADIUS / S.PX_PER_M
        dx, dy = self.x - cx, self.y - cy
        ang = math.atan2(dy, dx)
        ang_t = ang + self.lookahead / r
        tx = cx + r * math.cos(ang_t)
        ty = cy + r * math.sin(ang_t)
        err = wrap_angle(math.atan2(ty - self.y, tx - self.x) - self.theta)
        w = max(-2.5, min(2.5, self.k * err))
        cmd = Twist()
        cmd.linear.x = float(self.v_cmd)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
