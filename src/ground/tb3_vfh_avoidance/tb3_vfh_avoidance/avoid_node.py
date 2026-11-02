"""TurtleBot3 VFH 局部避障 ROS 2 节点。

订阅 ``sensor_msgs/LaserScan``，用 :mod:`vfh_core` 的向量场直方图算法
决策，发布 ``geometry_msgs/Twist`` 到 ``/cmd_vel``。

与朴素三扇区避障的区别：
- 全向 72 扇区直方图 + 配置空间膨胀（车身半径按角度投影），
  而不是前/左/右三个扇区的最短距离；
- 在"可通行谷"中选最接近目标方位的方向，能绕出 U 形陷阱等局部极小值；
- 内置安全停障、航向平滑与死锁恢复。

话题
----
- 订阅 ``scan``（参数 ``scan_topic``，默认 ``/scan``）
- 订阅 ``vfh_goal_bearing``（``std_msgs/Float32``，可选：外部给定期望航向，
  单位弧度，机器人坐标系，正前方为 0；不给则始终朝正前方）
- 发布 ``cmd_vel``（参数 ``cmd_vel_topic``，默认 ``/cmd_vel``）
- 发布 ``vfh_state``（``std_msgs/String``：cruise / avoid / blocked / turn /
  guard / recovery / goal）
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

try:
    from .vfh_core import VfhPlanner
except ImportError:                                    # 允许作为普通脚本运行
    import os
    import sys
    sys.path.insert(0, os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))
    from tb3_vfh_avoidance.vfh_core import VfhPlanner  # noqa: E402


class VfhAvoidNode(Node):
    """VFH 局部避障节点。"""

    def __init__(self):
        super().__init__('tb3_vfh_avoidance')

        # ---------------- 话题参数 ----------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('state_topic', 'vfh_state')
        self.declare_parameter('goal_topic', 'vfh_goal_bearing')
        # 大于 0 时节点运行该秒数后自动退出（用于无头环境验证，正式使用设 0）
        self.declare_parameter('exit_after', 0.0)

        # ---------------- 算法参数 ----------------
        self.declare_parameter('n_sectors', 72)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('block_distance', 0.50)
        self.declare_parameter('min_gap_deg', 25.0)
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('safety_distance', 0.40)
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('max_angular', 1.82)
        self.declare_parameter('turn_gain', 1.6)

        scan_topic = self.get_parameter('scan_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        state_topic = self.get_parameter('state_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        exit_after = float(self.get_parameter('exit_after').value)

        self.planner = VfhPlanner(
            n_sectors=int(self.get_parameter('n_sectors').value),
            max_range=float(self.get_parameter('max_range').value),
            block_distance=float(self.get_parameter('block_distance').value),
            min_gap_deg=float(self.get_parameter('min_gap_deg').value),
            robot_radius=float(self.get_parameter('robot_radius').value),
            safety_distance=float(self.get_parameter('safety_distance').value),
            max_linear=float(self.get_parameter('max_linear').value),
            max_angular=float(self.get_parameter('max_angular').value),
            turn_gain=float(self.get_parameter('turn_gain').value))

        self.goal_bearing = 0.0
        self._last_state = None
        self._done = False

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.on_scan, 10)
        self.goal_sub = self.create_subscription(
            Float32, goal_topic, self.on_goal, 10)

        if exit_after > 0.0:
            self.create_timer(exit_after, lambda: setattr(self, '_done', True))

        self.get_logger().info(
            'VFH 避障节点启动: scan=%s cmd=%s | %d 扇区, 阻塞净空 %.2f m, '
            '车身半径 %.2f m'
            % (scan_topic, cmd_topic, self.planner.n_sectors,
               self.planner.block_distance, self.planner.robot_radius))

    # ------------------------------------------------------------------
    def on_goal(self, msg):
        """外部期望航向（弧度，机器人坐标系）。"""
        self.goal_bearing = float(msg.data)

    def on_scan(self, scan):
        """每收到一帧激光做一次决策并下发速度。"""
        cmd = self.planner.compute(
            list(scan.ranges), scan.angle_min, scan.angle_increment,
            target_angle=self.goal_bearing, range_min=scan.range_min)

        twist = Twist()
        twist.linear.x = float(cmd.linear)
        twist.angular.z = float(cmd.angular)
        self.cmd_pub.publish(twist)

        if cmd.state != self._last_state:
            self._last_state = cmd.state
            self.get_logger().info(
                '状态 -> %s (航向 %.1f°, 净空 %.2f m, v=%.2f, w=%.2f)'
                % (cmd.state, math.degrees(cmd.heading), cmd.clearance,
                   cmd.linear, cmd.angular))
            state_msg = String()
            state_msg.data = cmd.state
            self.state_pub.publish(state_msg)


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = VfhAvoidNode()
    try:
        # spin_once 循环以便支持 exit_after 自动退出（无头验证用）
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())     # 退出前停车
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
