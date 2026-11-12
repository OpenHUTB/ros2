#!/usr/bin/env python3
"""PX4 Offboard 视觉追踪控制（纯速度控制形态，状态机实现）。

设计要点（对应 review 中的 P0/P1 修复）：
- 只使用 velocity 控制：OffboardControlMode.velocity=True，position=False；
  TrajectorySetpoint.position 全部为 NaN。避免 z 轴 position+velocity 混用。
- 状态机：WAIT_TARGET → SET_OFFBOARD → ARM → TAKEOFF → TRACK；
  每一步都等待 PX4 状态回读（nav_state / arming_state），失败自动重试并超时降级到 LAND。
- 高度保持同样用速度环（vz = K_ALT * (takeoff_alt - z)），与视觉速度指令同一控制形态。
- 目标面积期望值可配置（desired_area），默认 3750 与 image_pub 合成目标标定一致；
  面积误差带死区，避免像旧实现那样恒饱和前飞。
- 目标丢失超时 → 悬停（水平零速、保持高度）；节点退出时主动发 LAND。

注意：本节点配套的 /vision/target 来自 image_pub 的合成图像（开环演示），
不是 Gazebo 相机；若需真实闭环，请把 /camera/image_raw 换成 Gazebo 相机桥接。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# PX4 常量
PX4_CMD_DO_SET_MODE = 176
PX4_CMD_ARM_DISARM = 400
PX4_CMD_LAND = 21
PX4_MAIN_MODE_OFFBOARD = 6.0
PX4_ARMING_STATE_ARMED = 2
PX4_NAV_STATE_OFFBOARD = 14


class OffboardControl(Node):
    """把图像偏差转成 PX4 Offboard 速度指令的视觉追踪控制节点。"""

    # 状态
    WAIT_TARGET = 0
    SET_OFFBOARD = 1
    ARM = 2
    TAKEOFF = 3
    TRACK = 4
    LAND = 5

    def __init__(self):
        super().__init__('offboard_control')

        # ---- 参数 ----
        self.declare_parameter('takeoff_alt', -2.0)      # NED 向下为正：-2.0 = 离地 2 m
        self.declare_parameter('desired_area', 3750.0)   # 期望目标像素面积（与合成目标标定一致）
        self.declare_parameter('max_v', 1.0)             # 水平速度限幅 m/s
        self.declare_parameter('k_y', 0.12)              # 水平居中比例增益
        self.declare_parameter('k_d', 0.02)              # 距离（面积）比例增益
        self.declare_parameter('k_alt', 0.8)             # 高度保持比例增益
        self.declare_parameter('min_area', 300.0)        # 有效目标最小面积
        self.declare_parameter('target_timeout', 3.0)    # 目标丢失超时（s）
        self.declare_parameter('deadband_px', 15.0)      # 水平居中死区（像素）
        self.declare_parameter('deadband_area_ratio', 0.05)  # 面积死区（占期望面积比例）
        self.declare_parameter('state_timeout', 10.0)    # 切模式/解锁超时（s）

        p = self.get_parameter
        self.takeoff_alt = float(p('takeoff_alt').value)
        self.desired_area = float(p('desired_area').value)
        self.max_v = float(p('max_v').value)
        self.k_y = float(p('k_y').value)
        self.k_d = float(p('k_d').value)
        self.k_alt = float(p('k_alt').value)
        self.min_area = float(p('min_area').value)
        self.target_timeout = float(p('target_timeout').value)
        self.deadband_px = float(p('deadband_px').value)
        self.deadband_area = self.desired_area * float(p('deadband_area_ratio').value)
        self.state_timeout = float(p('state_timeout').value)

        # ---- 发布器 ----
        self.off_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # ---- 订阅器 ----
        qos_out = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            PointStamped, '/vision/target', self.target_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_pos_cb, qos_out)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_cb, qos_out)

        # ---- 状态 ----

        self.dx = self.dy = self.area = 0.0
        self.last_target_t = 0.0
        self.z = 0.0
        self.arming_state = 0
        self.nav_state = 0
        self.state = self.WAIT_TARGET
        self.state_since = self.now_s()
        self._last_set_mode_t = 0.0
        self._last_arm_t = 0.0
        self._hold_alt_since = None

        self.create_timer(0.1, self.control)  # 10 Hz
        self.get_logger().info(
            'offboard_control started: 等待 /vision/target（desired_area=%.0f px^2）'
            % self.desired_area)

    # ---------- 工具 ----------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def enter(self, state, reason=''):
        names = {
            self.WAIT_TARGET: 'WAIT_TARGET',
            self.SET_OFFBOARD: 'SET_OFFBOARD',
            self.ARM: 'ARM',
            self.TAKEOFF: 'TAKEOFF',
            self.TRACK: 'TRACK',
            self.LAND: 'LAND',
        }
        self.state = state
        self.state_since = self.now_s()
        msg = 'state -> %s' % names.get(state, str(state))
        if reason:
            msg += ' (%s)' % reason
        self.get_logger().info(msg)

    def send_cmd(self, command, p1=0.0, p2=0.0):
        vc = VehicleCommand()
        vc.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vc.command = int(command)
        vc.param1 = float(p1)
        vc.param2 = float(p2)
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        vc.from_external = True
        self.cmd_pub.publish(vc)

    def publish_mode_and_setpoint(self, vx, vy, vz):
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        om = OffboardControlMode()
        om.timestamp = now_us
        om.position = False
        om.velocity = True   # 只用速度通道
        self.off_pub.publish(om)

        ts = TrajectorySetpoint()
        ts.timestamp = now_us
        ts.position = [math.nan, math.nan, math.nan]
        ts.velocity = [float(vx), float(vy), float(vz)]
        ts.yaw = 0.0
        self.traj_pub.publish(ts)

    # ---------- 回调 ----------
    def target_cb(self, msg):
        self.dx = float(msg.point.x)
        self.dy = float(msg.point.y)
        self.area = float(msg.point.z)
        if self.area > self.min_area:
            self.last_target_t = self.now_s()

    def local_pos_cb(self, msg):
        self.z = float(msg.z)

    def status_cb(self, msg):
        self.arming_state = int(msg.arming_state)
        self.nav_state = int(msg.nav_state)

    # ---------- 控制主循环 ----------
    def control(self):
        now = self.now_s()
        state_elapsed = now - self.state_since

        if self.state == self.WAIT_TARGET:
            if self.last_target_t > 0.0 and (now - self.last_target_t) < self.target_timeout:
                self.enter(self.SET_OFFBOARD, '首帧有效目标已收到')
            return

        if self.state == self.LAND:
            if now - self._last_set_mode_t > 1.0:
                self.send_cmd(PX4_CMD_LAND)
                self._last_set_mode_t = now
            return

        # 以下状态都持续发送 OffboardControlMode + 速度 setpoint
        if self.state == self.SET_OFFBOARD:
            self.publish_mode_and_setpoint(0.0, 0.0, 0.0)
            if self.nav_state == PX4_NAV_STATE_OFFBOARD:
                self.enter(self.ARM, 'nav_state=14')
                return
            if state_elapsed > 1.0 and now - self._last_set_mode_t > 0.5:
                self.send_cmd(PX4_CMD_DO_SET_MODE, 1.0, PX4_MAIN_MODE_OFFBOARD)
                self._last_set_mode_t = now
            if state_elapsed > self.state_timeout:
                self.get_logger().error('切换 Offboard 超时，执行 LAND')
                self.enter(self.LAND)
            return

        if self.state == self.ARM:
            self.publish_mode_and_setpoint(0.0, 0.0, 0.0)
            if self.arming_state == PX4_ARMING_STATE_ARMED:
                self.enter(self.TAKEOFF, 'arming_state=2')
                return
            if now - self._last_arm_t > 0.5:
                self.send_cmd(PX4_CMD_ARM_DISARM, 1.0, 0.0)
                self._last_arm_t = now
                self.get_logger().info('ARM 指令已发送（等待 arming_state=2）')
            if state_elapsed > self.state_timeout:
                self.get_logger().error('ARM 超时，执行 LAND')
                self.enter(self.LAND)
            return

        if self.state == self.TAKEOFF:
            dz = self.takeoff_alt - self.z
            vz = max(-0.8, min(0.8, self.k_alt * dz))
            self.publish_mode_and_setpoint(0.0, 0.0, vz)
            if abs(dz) < 0.15:
                if self._hold_alt_since is None:
                    self._hold_alt_since = now
                elif now - self._hold_alt_since > 2.0:
                    self.enter(self.TRACK, '高度已保持 %.2f m' % self.z)
            else:
                self._hold_alt_since = None
            if state_elapsed > 25.0:
                self.get_logger().error('起飞超时，执行 LAND')
                self.enter(self.LAND)
            return

        if self.state == self.TRACK:
            target_fresh = (now - self.last_target_t) < self.target_timeout
            # 高度保持（速度环）
            vz = max(-0.5, min(0.5, self.k_alt * (self.takeoff_alt - self.z)))
            if target_fresh and self.area > self.min_area:
                # 水平居中：图像偏差 → 侧向速度
                if abs(self.dx) < self.deadband_px:
                    vy = 0.0
                else:
                    vy = max(-self.max_v, min(self.max_v, -self.k_y * self.dx))
                # 距离保持：目标面积误差 → 前向速度（带死区）
                area_err = self.desired_area - self.area
                if abs(area_err) < self.deadband_area:
                    vx = 0.0
                else:
                    vx = max(-self.max_v, min(self.max_v, self.k_d * area_err))
            else:
                # 目标丢失：悬停等待
                vx = vy = 0.0
                self.get_logger().warn(
                    '目标丢失超过 %.1f s，悬停等待' % self.target_timeout,
                    throttle_duration_sec=2.0)
            self.publish_mode_and_setpoint(vx, vy, vz)
            if self.nav_state != PX4_NAV_STATE_OFFBOARD:
                self.get_logger().warn('已退出 Offboard（nav_state=%d）' % self.nav_state,
                                       throttle_duration_sec=2.0)
            return


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 主动收尾：发 LAND，避免依赖 offboard 失联 failsafe
        try:
            for _ in range(5):
                node.send_cmd(PX4_CMD_LAND)
                rclpy.spin_once(node, timeout_sec=0.2)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

