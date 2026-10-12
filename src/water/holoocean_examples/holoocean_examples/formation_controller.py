#!/usr/bin/env python3
"""领航-跟随编队控制器（HoloOcean + ROS 2）。

功能：
  - 订阅领航者与跟随者的里程计（<agent>/DynamicsSensorOdom）
  - 按"领航者位置 + 旋转后的编队偏移"计算每个跟随者的期望位置
  - 通过 HoloOcean 自带的深度/航向/速度自动驾驶接口下达指令
      /holoocean/heading  (holoocean_interfaces/DesiredCommand, header.frame_id = 载具名)
      /holoocean/depth    (同上)
      /holoocean/speed    (同上)
  - 可选：启动时把跟随者切到 depthHeadingAutopilot（mode=1）
  - 可选：把期望编队位置画到 HoloOcean 视口（/holoocean/debug/points）

控制结构（外环-内环）：
  外环（本节点）：位置误差 -> 期望航向 / 期望深度 / 期望速度
  内环（HoloOcean 内置）：headingAutopilot / depthAutopilot 跟踪上述目标

坐标系约定：HoloOcean 使用 NWU（x 向北、y 向西、z 向上），
航向以角度表示、范围 (-180, 180]。若实际场景出现方向相反，调 yaw_sign / depth_sign。
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from holoocean_interfaces.msg import DesiredCommand
from holoocean_interfaces.srv import SetControlMode


class FormationController(Node):

    def __init__(self):
        super().__init__('formation_controller')

        # ---- 参数 ----
        self.declare_parameter('leader', 'auv0')
        self.declare_parameter('followers', ['auv1'])
        # 每个跟随者相对领航者的偏移 [dx, dy, dz]（米，领航者机体系）
        self.declare_parameter('offsets', [-3.0, 0.0, 0.0])
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('cruise_speed', 800.0)     # RPM，对应 DesiredCommand.data
        self.declare_parameter('speed_gain', 120.0)       # 每米误差增加的 RPM
        self.declare_parameter('max_standoff', 12.0)      # 超过该距离只追不充
        self.declare_parameter('use_leader_feedforward', True)  # 用领航者速度做前馈
        self.declare_parameter('depth_min', 0.5)
        self.declare_parameter('depth_max', 20.0)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('depth_sign', 1.0)
        self.declare_parameter('set_autopilot_on_start', True)
        self.declare_parameter('draw_targets', True)

        p = self.get_parameter
        self.leader = p('leader').value
        self.followers = list(p('followers').value)
        raw = list(p('offsets').value)
        if len(raw) == 3 * len(self.followers):
            self.offsets = [raw[3 * i:3 * i + 3] for i in range(len(self.followers))]
        elif len(raw) == 3:
            self.offsets = [raw for _ in self.followers]
        else:
            raise ValueError('offsets 长度必须是 3 或 3×跟随者数量')
        self.cruise = float(p('cruise_speed').value)
        self.k_speed = float(p('speed_gain').value)
        self.max_standoff = float(p('max_standoff').value)
        self.dmin = float(p('depth_min').value)
        self.dmax = float(p('depth_max').value)
        self.yaw_sign = float(p('yaw_sign').value)
        self.depth_sign = float(p('depth_sign').value)

        # ---- 状态 ----
        self.odom = {}          # agent_name -> (x, y, z, yaw_rad)
        self.leader_speed = 0.0 # 领航者当前速度（用于速度前馈）
        self.err_log = []       # (t, 跟随者, |e|)

        # ---- 接口 ----
        agents = [self.leader] + self.followers
        for name in agents:
            self.create_subscription(
                Odometry, f'{name}/DynamicsSensorOdom',
                lambda msg, n=name: self._odom_cb(n, msg), 10)

        self.pub_heading = self.create_publisher(DesiredCommand, 'heading', 10)
        self.pub_depth = self.create_publisher(DesiredCommand, 'depth', 10)
        self.pub_speed = self.create_publisher(DesiredCommand, 'speed', 10)
        self.pub_marker = self.create_publisher(Marker, 'debug/points', 10)

        self.cli = self.create_client(SetControlMode, 'control_mode')

        if bool(p('set_autopilot_on_start').value):
            self.create_timer(1.0, self._arm_once)

        period = 1.0 / float(p('rate_hz').value)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            'formation_controller 就绪：leader=%s followers=%s offsets=%s'
            % (self.leader, self.followers, self.offsets))

    # ------------------------------------------------------------------ 回调
    def _odom_cb(self, name, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pos = msg.pose.pose.position
        self.odom[name] = (pos.x, pos.y, pos.z, yaw)
        if name == self.leader:
            v = msg.twist.twist.linear
            self.leader_speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

    def _arm_once(self):
        """把跟随者切到 depthHeadingAutopilot（mode=1）。"""
        if not self.cli.service_is_ready():
            return
        for name in self.followers:
            req = SetControlMode.Request()
            req.vehicle_name = name
            req.mode = 1
            self.cli.call_async(req)
        self.get_logger().info('已请求将跟随者切换为 depthHeadingAutopilot')

    # ------------------------------------------------------------------ 主循环
    def _tick(self):
        if self.leader not in self.odom:
            self.get_logger().warn('尚未收到领航者里程计，等待中…', once=False)
            return

        lx, ly, lz, lyaw = self.odom[self.leader]
        targets = []

        for name, (dx, dy, dz) in zip(self.followers, self.offsets):
            # 1) 期望位置：领航者位置 + 机体系偏移旋转到世界系
            c, s = math.cos(lyaw), math.sin(lyaw)
            tx = lx + (c * dx - s * dy)
            ty = ly + (s * dx + c * dy)
            tz = lz + dz
            targets.append((name, tx, ty, tz))

            if name not in self.odom:
                continue

            # 2) 位置误差
            px, py, pz, _ = self.odom[name]
            ex, ey, ez = tx - px, ty - py, tz - pz
            dist = math.hypot(ex, ey)
            self.err_log.append((self.get_clock().now().nanoseconds, name, dist))

            # 3) 外环 -> 期望航向（指向目标点）与期望深度
            heading_deg = self.yaw_sign * math.degrees(math.atan2(ey, ex))
            depth_m = max(self.dmin, min(self.dmax, self.depth_sign * tz))
            # 速度指令 = 前馈（领航者当前速度对应的 RPM）+ 位置误差反馈。
            # 只用反馈时，dist→0 会让速度指令趋近 cruise，若领航者速度高于 cruise，
            # 就会留下 (v_leader - v_cruise)/speed_gain 的稳态误差；加前馈后该项被抵消。
            ff = 0.0
            if bool(self.get_parameter('use_leader_feedforward').value):
                ff = self.k_speed * min(max(self.leader_speed, 0.0), self.max_standoff)
            speed = 0.0 if dist < 0.15 else ff + self.cruise + self.k_speed * min(dist, self.max_standoff)

            # 4) 下发（以 header.frame_id 指定目标载具）
            self._send(self.pub_heading, name, heading_deg)
            self._send(self.pub_depth, name, depth_m)
            self._send(self.pub_speed, name, speed)

        if bool(self.get_parameter('draw_targets').value):
            self._draw(targets)

    @staticmethod
    def _send(pub, vehicle, value):
        msg = DesiredCommand()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.header.frame_id = vehicle          # 关键：用 frame_id 指定载具
        msg.data = float(value)
        pub.publish(msg)

    def _draw(self, targets):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.a = 0.8
        marker.lifetime.sec = 1
        for _name, tx, ty, tz in targets:
            pt = marker.pose.position.__class__()
            pt.x, pt.y, pt.z = tx, ty, tz
            marker.points.append(pt)
        self.pub_marker.publish(marker)

    def report(self):
        """收尾时打印编队误差统计，便于填评测表。"""
        if not self.err_log:
            return
        per_agent = {}
        for _t, name, e in self.err_log:
            per_agent.setdefault(name, []).append(e)
        for name, vals in per_agent.items():
            rms = math.sqrt(sum(v * v for v in vals) / len(vals))
            self.get_logger().info(
                '编队误差[%s]: RMS=%.3f m  最大=%.3f m  样本=%d'
                % (name, rms, max(vals), len(vals)))


def main(args=None):
    rclpy.init(args=args)
    node = FormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
