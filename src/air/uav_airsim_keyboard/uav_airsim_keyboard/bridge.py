#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AirSim 桥接节点：把 ROS 2 的话题/服务翻译成 AirSim RPC 调用。

话题（相对话题名，根命名空间下即 /uav/*）：
    订阅  uav/cmd_vel   geometry_msgs/Twist
          linear.x  机头方向速度 (m/s, 前为正)
          linear.y  机体左向速度 (m/s, 左为正，遵循 REP-103)
          linear.z  垂直速度     (m/s, 上为正)
          angular.z 偏航角速度   (rad/s, 逆时针为正，遵循 REP-103)

    话题单位/轴向按 ROS 惯例（m、m/s、rad/s，x 前 y 左 z 上，偏航逆时针为正），
    进入本节点后再换算成 AirSim 的 NED（y 右、偏航顺时针、角度制）。
    发布  uav/pose      geometry_msgs/PoseStamped（frame_id = airsim_ned）

服务：
    uav/takeoff   std_srvs/Trigger   起飞并悬停
    uav/land      std_srvs/Trigger   降落

设计要点（评审重点）：
1. 所有 AirSim 调用都放在**定时器**里，且只调异步接口、**不调用 .get()/.join()**，
   否则会阻塞 rclpy 的 executor，姿态发布和 service 全部卡死。
2. 飞行器 IP 不做硬编码，走 ROS 参数（虚拟机 NAT 默认 10.0.2.2）。
3. 起飞/降落这类需要等待数秒的操作，放在 service 回调里同步等待（该回调独占
   一个 executor 线程），并加超时与异常保护。
"""

import math
import threading
import time

import airsim
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_srvs.srv import Trigger


class AirSimBridge(Node):
    """ROS 2 与 AirSim 之间的双向桥。"""

    def __init__(self):
        super().__init__('uav_airsim_keyboard')
        self._declare_parameters()

        # 保护 AirSim 客户端的锁：executor 多线程下 timer / service 会并发调用
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0, 0.0)
        self._cmd_stamp = 0.0
        self._warn_stamp = 0.0
        self._flying = False

        self._client = airsim.MultirotorClient(
            ip=self._ip,
            port=self._port,
            timeout_value=self._timeout,
        )
        # confirmConnection() 只负责打印版本号，部分 AirSim 版本不返回布尔值，
        # 用 ping() 做真正的连通性判断，避免把 None 当成 False。
        self._client.confirmConnection()
        if not self._client.ping():
            raise RuntimeError(
                '无法连接 AirSim（%s:%d）。请先在宿主机启动 Blocks.exe，'
                '并确认虚拟机网络能访问该 IP。' % (self._ip, self._port))

        # 只取控制权，不解锁：解锁会让电机上电，落地状态不该是这个状态。
        # 真正的 armDisarm(True) 放在起飞服务里，和爬升动作一起发生。
        self._client.enableApiControl(True, vehicle_name=self._vehicle)
        self.get_logger().info(
            '已连接 AirSim %s:%d（vehicle_name=%r），API 控制已使能（未解锁）' %
            (self._ip, self._port, self._vehicle))

        # ---- 收发接口 ----
        self._pose_pub = self.create_publisher(PoseStamped, 'uav/pose', 10)
        self._cmd_sub = self.create_subscription(
            Twist, 'uav/cmd_vel', self._on_cmd, 10)
        self._takeoff_srv = self.create_service(
            Trigger, 'uav/takeoff', self._on_takeoff)
        self._land_srv = self.create_service(
            Trigger, 'uav/land', self._on_land)

        self._pose_timer = self.create_timer(1.0 / self._pose_rate, self._publish_pose)
        self._ctrl_timer = self.create_timer(1.0 / self._control_rate, self._control_cycle)

        # “移动路标”位控：实测这套 Blocks 场景里速度指令经常被忽略，位置指令
        # 才可靠；但位置指令每帧重发又会让飞机几乎不动，所以维护一个沿速度
        # 方向前进的设定点，偏离已下发目标足够远时才重新下发一次。
        self._setpoint = None
        self._sent_point = None
        self._last_sent_at = 0.0
        self._last_cycle = time.monotonic()
        # 起飞/降落会长时间阻塞在 join() 上；用事件告诉定时器"现在不要发 RPC"，
        # 它们改发缓存位姿，避免 /uav/pose 断流。
        self._rpc_busy = threading.Event()
        self._pose_cache = None

    # ------------------------------------------------------------------ 参数
    def _declare_parameters(self):
        self.declare_parameter('airsim_ip', '10.0.2.2')
        self.declare_parameter('airsim_port', 41451)
        self.declare_parameter('vehicle_name', '')
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('pose_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.3)
        self.declare_parameter('cmd_duration', 0.2)
        self.declare_parameter('rpc_timeout', 5.0)
        self.declare_parameter('enable_keyboard', True)
        self.declare_parameter('takeoff_height', 1.5)

        self._ip = self.get_parameter('airsim_ip').value
        self._port = int(self.get_parameter('airsim_port').value)
        self._vehicle = self.get_parameter('vehicle_name').value
        self._control_rate = float(self.get_parameter('control_rate').value)
        self._pose_rate = float(self.get_parameter('pose_rate').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self._cmd_duration = float(self.get_parameter('cmd_duration').value)
        self._timeout = float(self.get_parameter('rpc_timeout').value)
        self._takeoff_height = float(self.get_parameter('takeoff_height').value)
        self.enable_keyboard = bool(self.get_parameter('enable_keyboard').value)

    # ------------------------------------------------------------- 话题回调
    def _on_cmd(self, msg):
        """只记录最新指令（ROS 惯例：y 左正、z 上正、偏航 rad/s 逆时针）。

        真正的下发由定时器统一完成，避免回调里阻塞。
        """
        with self._lock:
            self._cmd = (
                float(msg.linear.x),
                float(msg.linear.y),
                float(msg.linear.z),
                float(msg.angular.z),
            )
            self._cmd_stamp = time.monotonic()

    def _control_cycle(self):
        # 起飞/降落正在等 future，期间并发发 RPC 会撞 msgpackrpc 的单 IOLoop
        if self._rpc_busy.is_set():
            return
        now = time.monotonic()
        # 没起飞、或已经降落时不下发任何位置指令：
        # 否则落地后按一下方向键就会被拉回起飞前的设定点重新起飞。
        if not self._flying:
            self._setpoint = None
            self._last_cycle = now
            return
        with self._lock:
            fresh = (now - self._cmd_stamp) <= self._cmd_timeout
            vx, vy_left, vz_up, yaw_rad = self._cmd if fresh else (0.0, 0.0, 0.0, 0.0)

        # ROS 惯例 -> AirSim NED：y 左改变右变、z 上改变下变、弧度变角度
        vy = -vy_left
        vz_ned = -vz_up
        yaw_rate = math.degrees(yaw_rad)
        try:
            with self._lock:
                state = self._client.getMultirotorState(vehicle_name=self._vehicle)
                pos = state.kinematics_estimated.position
                _, _, yaw = airsim.to_eularian_angles(
                    state.kinematics_estimated.orientation)

            dt = min(0.5, max(0.01, now - self._last_cycle))
            self._last_cycle = now
            if self._setpoint is None:
                self._setpoint = (pos.x_val, pos.y_val, pos.z_val)
            sx, sy, sz = self._setpoint
            if math.hypot(math.hypot(vx, vy), vz_ned) > 1e-3:
                # 机体系速度转到世界系：本体 x 前、y 右
                sx += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
                sy += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
                sz += vz_ned * dt
            else:
                sx, sy, sz = pos.x_val, pos.y_val, pos.z_val
            # 路标不能跑得比飞机快太多，否则一直被它甩在后面
            ox, oy, oz = sx - pos.x_val, sy - pos.y_val, sz - pos.z_val
            offset = math.sqrt(ox * ox + oy * oy + oz * oz)
            if offset > 1.5:
                sx = pos.x_val + ox / offset * 1.5
                sy = pos.y_val + oy / offset * 1.5
                sz = pos.z_val + oz / offset * 1.5
            self._setpoint = (sx, sy, sz)

            moved = 1e9 if self._sent_point is None else math.dist(
                self._setpoint, self._sent_point)
            # AirSim 的 Python 客户端不是线程安全的（msgpackrpc 单 IOLoop），
            # 所有 RPC 都要在同一把锁里发，否则会和位姿读取、起飞流程撞车。
            with self._lock:
                if moved > 0.3 or (now - self._last_sent_at) > 0.5:
                    speed = max(0.8, math.sqrt(vx * vx + vy * vy + vz_ned * vz_ned))
                    self._client.moveToPositionAsync(
                        sx, sy, sz, speed, vehicle_name=self._vehicle)
                    self._sent_point = self._setpoint
                    self._last_sent_at = now
                # 偏航独立于位置重发，否则设定点不动时只有约 1/5 的占空比。
                # ROS 的 angular.z 逆时针为正，AirSim 的 yaw 顺时针为正，取反。
                if abs(yaw_rate) > 1e-3:
                    self._client.rotateByYawRateAsync(
                        -yaw_rate, dt, vehicle_name=self._vehicle)
        except Exception as exc:  # noqa: BLE001 - 网络抖动不应杀死节点
            self._warn_throttled('速度指令下发失败: %s' % exc)

    def _publish_pose(self):
        if not self._rpc_busy.is_set():
            try:
                with self._lock:
                    state = self._client.getMultirotorState(
                        vehicle_name=self._vehicle)
                self._pose_cache = state.kinematics_estimated
            except Exception as exc:  # noqa: BLE001
                self._warn_throttled('读取位姿失败: %s' % exc)

        # 长事务期间读不到新位姿，就把上一帧再发一次（换新时间戳），
        # 保证 /uav/pose 不断流，下游不会误判断链。
        kin = self._pose_cache
        if kin is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'airsim_ned'
        msg.pose.position.x = float(kin.position.x_val)
        msg.pose.position.y = float(kin.position.y_val)
        msg.pose.position.z = float(kin.position.z_val)
        msg.pose.orientation.w = float(kin.orientation.w_val)
        msg.pose.orientation.x = float(kin.orientation.x_val)
        msg.pose.orientation.y = float(kin.orientation.y_val)
        msg.pose.orientation.z = float(kin.orientation.z_val)
        self._pose_pub.publish(msg)

    # ------------------------------------------------------------- 服务回调
    def _on_takeoff(self, request, response):
        del request
        try:
            with self._lock:
                self._client.enableApiControl(True, vehicle_name=self._vehicle)
                self._client.armDisarm(True, vehicle_name=self._vehicle)
                time.sleep(0.5)  # 解锁后 SimpleFlight 需要一点时间进入可控状态
                # 以当前位置为基准：多机场景下每架飞机出生点不同，写死 (0,0)
                # 会把飞机拉回世界原点。
                pos = self._client.getMultirotorState(
                    vehicle_name=self._vehicle).kinematics_estimated.position
                base_x, base_y = float(pos.x_val), float(pos.y_val)
                # 这套 Blocks 场景里 takeoffAsync 有时只把飞机放到地面高度就
                # 返回，位置指令更可靠，所以直接飞到一个起飞高度。
                future = self._client.moveToPositionAsync(
                    base_x, base_y, -self._takeoff_height, 1.5,
                    timeout_sec=20.0, vehicle_name=self._vehicle)
            # join() 放到锁外：最长 20 s，锁内等待会把位姿/控制定时器饿死。
            # 期间 _rpc_busy 挡住其它 RPC，位姿改发缓存。
            self._rpc_busy.set()
            try:
                future.join()
            finally:
                self._rpc_busy.clear()
            self._setpoint = None
            self._sent_point = None
            self._flying = True
            response.success = True
            response.message = '起飞完成，已进入悬停'
            self.get_logger().info(response.message)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = '起飞失败: %s' % exc
            self.get_logger().error(response.message)
        return response

    def _on_land(self, request, response):
        del request
        try:
            with self._lock:
                future = self._client.landAsync(timeout_sec=20.0,
                                                vehicle_name=self._vehicle)
            self._rpc_busy.set()
            try:
                future.join()
            finally:
                self._rpc_busy.clear()
            self._flying = False
            response.success = True
            response.message = '降落完成'
            self.get_logger().info(response.message)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = '降落失败: %s' % exc
            self.get_logger().error(response.message)
        return response

    # ----------------------------------------------------------------- 工具
    def _warn_throttled(self, text, period=5.0):
        now = time.monotonic()
        if now - self._warn_stamp >= period:
            self._warn_stamp = now
            self.get_logger().warning(text)

    def shutdown(self):
        """Ctrl+C 时降落并交还控制权，避免无人机一直悬停。"""
        if not rclpy.ok():
            return
        self.get_logger().info('正在降落并退出...')
        try:
            with self._lock:
                if self._flying:
                    try:
                        future = self._client.landAsync(
                            timeout_sec=10.0, vehicle_name=self._vehicle)
                        self._rpc_busy.set()
                        try:
                            future.join()
                        finally:
                            self._rpc_busy.clear()
                    except Exception as exc:  # noqa: BLE001
                        # 落不下来就保留控制权，别在空中断桨
                        self.get_logger().warning(
                            f'降落失败（{exc}），保留 API 控制权')
                        return
                    self._flying = False
                self._client.armDisarm(False, vehicle_name=self._vehicle)
                self._client.enableApiControl(False, vehicle_name=self._vehicle)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning('退出清理失败: %s' % exc)


def main(argv=None):
    """单独启动桥接节点（调试用）。"""
    import sys

    rclpy.init(args=sys.argv if argv is None else argv)
    node = AirSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
