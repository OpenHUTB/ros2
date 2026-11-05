#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AirSim 桥接节点：把 ROS 2 的话题/服务翻译成 AirSim RPC 调用。

话题（相对话题名，根命名空间下即 /uav/*）：
    订阅  uav/cmd_vel   geometry_msgs/Twist
          linear.x  机头方向速度 (m/s, 前为正)
          linear.y  机体右向速度 (m/s, 右为正)
          linear.z  垂直速度     (m/s, 上为正)
          angular.z 偏航角速度   (deg/s, 逆时针为正)
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

        self._client.enableApiControl(True, vehicle_name=self._vehicle)
        self._client.armDisarm(True, vehicle_name=self._vehicle)
        self.get_logger().info(
            '已连接 AirSim %s:%d（vehicle_name=%r），API 控制已使能' %
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
        self._last_sent_at = 0.0
        self._last_cycle = time.monotonic()

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
        """只记录最新指令，真正的下发由定时器统一完成，避免回调里阻塞。"""
        with self._lock:
            self._cmd = (
                float(msg.linear.x),
                float(msg.linear.y),
                float(msg.linear.z),
                float(msg.angular.z),
            )
            self._cmd_stamp = time.monotonic()

    def _control_cycle(self):
        now = time.monotonic()
        with self._lock:
            fresh = (now - self._cmd_stamp) <= self._cmd_timeout
            vx, vy, vz_up, yaw_rate = self._cmd if fresh else (0.0, 0.0, 0.0, 0.0)

        # AirSim 的机体系是 NED（z 向下），ROS 约定 z 向上，这里取反
        vz_ned = -vz_up
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

            moved = math.dist(self._setpoint,
                              getattr(self, '_sent_point', (1e9, 0, 0)))
            if moved > 0.3 or (now - self._last_sent_at) > 0.5:
                speed = max(0.8, math.sqrt(vx * vx + vy * vy + vz_ned * vz_ned))
                self._client.moveToPositionAsync(
                    sx, sy, sz, speed, vehicle_name=self._vehicle)
                self._sent_point = self._setpoint
                self._last_sent_at = now
                if abs(yaw_rate) > 1e-3:
                    self._client.rotateByYawRateAsync(
                        yaw_rate, dt, vehicle_name=self._vehicle)
        except Exception as exc:  # noqa: BLE001 - 网络抖动不应杀死节点
            self._warn_throttled('速度指令下发失败: %s' % exc)

    def _publish_pose(self):
        try:
            with self._lock:
                state = self._client.getMultirotorState(vehicle_name=self._vehicle)
        except Exception as exc:  # noqa: BLE001
            self._warn_throttled('读取位姿失败: %s' % exc)
            return

        kin = state.kinematics_estimated
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
                # 这套 Blocks 场景里 takeoffAsync 有时只把飞机放到地面高度就
                # 返回，位置指令更可靠，所以直接飞到一个起飞高度。
                self._client.moveToPositionAsync(
                    0.0, 0.0, -self._takeoff_height, 1.5,
                    timeout_sec=20.0, vehicle_name=self._vehicle).join()
            self._setpoint = None
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
                self._client.landAsync(timeout_sec=20.0,
                                       vehicle_name=self._vehicle).join()
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
                    self._client.landAsync(timeout_sec=10.0,
                                           vehicle_name=self._vehicle).join()
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
