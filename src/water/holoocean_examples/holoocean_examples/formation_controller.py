#!/usr/bin/env python3
"""领航-跟随编队控制器（HoloOcean + ROS 2）。

控制结构（外环-内环）：
  外环（本节点）：位置误差 -> 期望速度矢量 -> 期望航向 + 期望推力
  内环（HoloOcean 内置 Fossen 自动驾驶）：depthHeadingAutopilot 跟踪航向/深度

控制律（速度矢量控制，默认）：
  目标点速度   v_t = v_leader + ω_leader × r_offset   （领航者平动 + 偏移点旋转切向）
  期望速度矢量 v_d = v_t + Kp·e − Kd·(v_follower − v_t)
  期望航向     ψ_d = atan2(v_d)          期望推力(RPM) = |v_d| · speed_per_mps
  其中 e 为目标点与本机位置之差。比例项负责消除位置偏差，前馈项保证稳态时
  与领航者速度一致（不产生追踪滞后），阻尼项抑制振荡。

四组实测标定结论（详见 docs/water/rov_multi_agent_formation.md）：
  1. 航向指令无需取反：标定实验给出 指令 0/45/90/135/180° -> 实测偏航 0/16.5/32.5/48.5/64.3°，
     故 yaw_sign = +1。
  2. 速度换算：600 RPM ≈ 1.0 m/s（推进器上限 1525 RPM），故 speed_per_mps = 600。
  3. 深度指令为正值向下，场景 z 向下为负，故 depth_sign = -1。
  4. 鱼雷转向能力上限约 3.75 deg/s（Nomoto 增益 K=0.25、舵角上限 15°）。
     90° 阶跃转向物理上至少需要 24 s，跟随者无法跟踪一个持续旋转的目标点，
     故领航航迹必须做角速率限制（leader_turn_rate）。
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from holoocean_interfaces.msg import DesiredCommand
from holoocean_interfaces.srv import SetControlMode


def wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class FormationController(Node):

    def __init__(self):
        super().__init__('formation_controller')

        # ---- 编队 ----
        self.declare_parameter('leader', 'auv0')
        self.declare_parameter('followers', ['auv1'])
        self.declare_parameter('offsets', [-3.0, 0.0, 0.0])
        self.declare_parameter('rate_hz', 10.0)

        # ---- 速度控制 ----
        self.declare_parameter('speed_per_mps', 600.0)   # 实测：600 RPM ≈ 1.0 m/s
        self.declare_parameter('speed_gain', 120.0)      # 每米误差增加的 RPM
        self.declare_parameter('max_standoff', 12.0)     # 误差饱和距离（米）
        self.declare_parameter('align_floor', 0.2)       # 闭合项的对齐度下限
        self.declare_parameter('max_rpm', 1525.0)        # Fossen 鱼雷上限
        self.declare_parameter('k_pos', 0.25)            # 速度矢量控制：位置误差 -> m/s 增益
        self.declare_parameter('k_damp', 0.8)            # 相对速度阻尼增益
        self.declare_parameter('k_int', 0.0)             # 位置误差积分增益（消除转向静差）
        self.declare_parameter('int_clamp', 4.0)         # 积分项限幅（m·s）
        self.declare_parameter('use_velocity_vector', True)  # 用速度矢量控制（否则退回纯追踪）

        # ---- 深度/航向标定 ----
        self.declare_parameter('depth_min', 0.5)
        self.declare_parameter('depth_max', 20.0)
        self.declare_parameter('yaw_sign', 1.0)          # 实测无需取反
        self.declare_parameter('depth_sign', -1.0)       # 深度指令正值向下

        # ---- 领航者驱动 ----
        self.declare_parameter('leader_drive', False)
        self.declare_parameter('leader_depth', 10.0)
        self.declare_parameter('leader_rpm', 600.0)
        self.declare_parameter('leader_turn_start', 20.0)   # 开始转向的时刻（s）
        self.declare_parameter('leader_turn_rate', 1.0)     # 转向角速率（deg/s）
        self.declare_parameter('leader_turn_total', 90.0)   # 总转向角度（deg）
        self.declare_parameter('set_autopilot_on_start', True)
        self.declare_parameter('draw_targets', True)
        self.declare_parameter('debug_trace', False)
        self.declare_parameter('debug_path', '/tmp/ctl_dbg.csv')

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

        self.speed_per_mps = float(p('speed_per_mps').value)
        self.k_speed = float(p('speed_gain').value)
        self.max_standoff = float(p('max_standoff').value)
        self.align_floor = float(p('align_floor').value)
        self.dmin = float(p('depth_min').value)
        self.dmax = float(p('depth_max').value)
        self.max_rpm = float(p('max_rpm').value)
        self.yaw_sign = float(p('yaw_sign').value)
        self.depth_sign = float(p('depth_sign').value)

        self.odom = {}
        self.leader_speed = 0.0
        self.leader_vel = (0.0, 0.0)   # 领航者世界系速度矢量
        self.follower_vel = {}         # 各跟随者世界系速度矢量
        self.leader_yaw_rate = 0.0     # 领航者偏航角速率（rad/s），低通
        self._last_yaw = None
        self._last_yaw_t = None
        self.k_pos = float(p('k_pos').value)
        self.k_damp = float(p('k_damp').value)
        self.k_int = float(p('k_int').value)
        self.int_clamp = float(p('int_clamp').value)
        self.e_int = {}
        self.use_vv = bool(p('use_velocity_vector').value)
        self.dbg = bool(p('debug_trace').value)
        self.dbg_path = str(p('debug_path').value)
        self.dbg_fh = open(self.dbg_path, 'w') if self.dbg else None
        if self.dbg_fh:
            self.dbg_fh.write('t,agent,dist,desired_deg,cur_yaw_deg,rpm,leader_v\n')

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

        self.create_timer(1.0 / float(p('rate_hz').value), self._tick)
        self.get_logger().info(
            'formation_controller(v2) 就绪：leader=%s followers=%s offsets=%s'
            % (self.leader, self.followers, self.offsets))
        self._t0 = None

    def _odom_cb(self, name, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pos = msg.pose.pose.position
        self.odom[name] = (pos.x, pos.y, pos.z, yaw)
        v0 = msg.twist.twist.linear
        self.follower_vel[name] = (v0.x, v0.y)
        if name == self.leader:
            v = msg.twist.twist.linear
            self.leader_speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            self.leader_vel = (v.x, v.y)
            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self._last_yaw is not None and now > self._last_yaw_t:
                dt = now - self._last_yaw_t
                d = math.atan2(math.sin(yaw - self._last_yaw),
                               math.cos(yaw - self._last_yaw))
                rate = d / dt
                self.leader_yaw_rate = 0.8 * self.leader_yaw_rate + 0.2 * rate
            self._last_yaw = yaw
            self._last_yaw_t = now

    def _arm_once(self):
        if not self.cli.service_is_ready():
            return
        targets = list(self.followers)
        if bool(self.get_parameter('leader_drive').value) and self.leader not in targets:
            targets.append(self.leader)
        for name in targets:
            req = SetControlMode.Request()
            req.vehicle_name = name
            req.mode = 1
            self.cli.call_async(req)
        self.get_logger().info('已请求切换自动驾驶模式: %s' % targets)

    def _tick(self):
        if self.leader not in self.odom:
            self.get_logger().warn('尚未收到领航者里程计，等待中…')
            return

        if bool(self.get_parameter('leader_drive').value):
            now = self.get_clock().now()
            if self._t0 is None:
                self._t0 = now
            t = (now - self._t0).nanoseconds / 1e9
            t_start = float(self.get_parameter('leader_turn_start').value)
            rate = float(self.get_parameter('leader_turn_rate').value)
            total = float(self.get_parameter('leader_turn_total').value)
            # 角速率受限的缓弧转向：鱼雷模型最大偏航约 3.75 deg/s，
            # 直接给 90 度阶跃会让跟随者跟不上（实测卡住约 70 度偏差）。
            heading = 0.0 if t < t_start else min(total, (t - t_start) * rate)
            self._send(self.pub_heading, self.leader, heading)
            self._send(self.pub_depth, self.leader,
                       float(self.get_parameter('leader_depth').value))
            self._send(self.pub_speed, self.leader,
                       float(self.get_parameter('leader_rpm').value))

        lx, ly, lz, lyaw = self.odom[self.leader]
        targets = []

        for name, (dx, dy, dz) in zip(self.followers, self.offsets):
            c, s = math.cos(lyaw), math.sin(lyaw)
            tx = lx + (c * dx - s * dy)
            ty = ly + (s * dx + c * dy)
            tz = lz + dz
            targets.append((name, tx, ty, tz))

            if name not in self.odom:
                continue

            px, py, pz, pyaw = self.odom[name]
            ex, ey, ez = tx - px, ty - py, tz - pz
            dist = math.hypot(ex, ey)

            depth_m = max(self.dmin, min(self.dmax, self.depth_sign * tz))

            if self.use_vv:
                # 速度矢量控制：
                #   目标点速度 = 领航者平动 + 偏移点随领航者偏航旋转的切向速度
                vtx = self.leader_vel[0] - self.leader_yaw_rate * (ty - ly)
                vty = self.leader_vel[1] + self.leader_yaw_rate * (tx - lx)
                #   期望速度矢量 = 目标点速度（前馈） + 位置误差反馈
                vdx = vtx + self.k_pos * ex
                vdy = vty + self.k_pos * ey
                # 相对速度阻尼：跟随者速度与目标点速度之差 -> 抑制振荡
                fv = self.follower_vel.get(name, (0.0, 0.0))
                vdx -= self.k_damp * (fv[0] - vtx)
                vdy -= self.k_damp * (fv[1] - vty)
                # 积分项：消除"持续转向"下的比例控制静差
                if self.k_int > 0.0:
                    dt = 1.0 / float(self.get_parameter('rate_hz').value)
                    ix, iy = self.e_int.get(name, (0.0, 0.0))
                    ix += ex * dt
                    iy += ey * dt
                    nrm = math.hypot(ix, iy)
                    if nrm > self.int_clamp:
                        ix *= self.int_clamp / nrm
                        iy *= self.int_clamp / nrm
                    self.e_int[name] = (ix, iy)
                    vdx += self.k_int * ix
                    vdy += self.k_int * iy
                speed_mps = math.hypot(vdx, vdy)
                desired_deg = (math.degrees(math.atan2(vdy, vdx))
                               if speed_mps > 1e-3 else math.degrees(pyaw))
                heading_deg = self.yaw_sign * desired_deg
                rpm = speed_mps * self.speed_per_mps
                misalign = math.radians(wrap_deg(desired_deg - math.degrees(pyaw)))
            else:
                desired_deg = math.degrees(math.atan2(ey, ex))
                heading_deg = self.yaw_sign * desired_deg
                misalign = math.radians(wrap_deg(desired_deg - math.degrees(pyaw)))
                align = max(self.align_floor, math.cos(misalign))
                rpm_ff = self.speed_per_mps * self.leader_speed
                rpm_close = self.k_speed * min(dist, self.max_standoff) * align
                rpm = rpm_ff + rpm_close
                if dist < 0.15:
                    rpm = 0.0
            rpm = max(0.0, min(rpm, self.max_rpm))

            if self.dbg_fh:
                self.dbg_fh.write('%.2f,%s,%.3f,%.1f,%.1f,%.1f,%.2f\n' % (
                    self.get_clock().now().nanoseconds / 1e9, name, dist,
                    desired_deg, math.degrees(pyaw), rpm, self.leader_speed))
                self.dbg_fh.flush()

            self._send(self.pub_heading, name, heading_deg)
            self._send(self.pub_depth, name, depth_m)
            self._send(self.pub_speed, name, rpm)

        if bool(self.get_parameter('draw_targets').value):
            self._draw(targets)

    @staticmethod
    def _send(pub, vehicle, value):
        msg = DesiredCommand()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.header.frame_id = vehicle
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


def main(args=None):
    rclpy.init(args=args)
    node = FormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
