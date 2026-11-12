# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""AirSim 数据源节点。

把 AirSim 的位姿、RGB 图像、深度图翻译成标准 ROS2 消息，
同时把 ``/uav/cmd_vel``（机体系速度）转成 AirSim 的 API 调用。

设计要点：
* 所有 AirSim 的阻塞调用都放在后台线程 ``_worker`` 里，绝不在定时器里 ``.join()``，
  否则会把 ``MultiThreadedExecutor`` 卡死、``/map`` 与 ``/plan`` 一起停摆。
* 每个 AirSim 客户端调用都用 ``self._lock`` 保护，避免多线程并发调用 RPC。
"""

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

try:  # airsim 只有数据源节点需要，缺失时不影响建图/规划/控制节点
    import airsim
except ImportError:  # pragma: no cover - 取决于运行环境
    airsim = None


class AirSimSource(Node):
    """AirSim 与 ROS2 之间的双向桥：发布传感器，接收速度指令。"""

    def __init__(self):
        super().__init__('airsim_source', namespace='uav')

        self.declare_parameter('airsim_ip', '10.0.2.2')
        self.declare_parameter('airsim_port', 41451)
        self.declare_parameter('vehicle_name', '')
        self.declare_parameter('camera_name', '0')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('depth_pixels_as_float', True)
        self.declare_parameter('cmd_duration', 0.5)
        self.declare_parameter('connect_period', 2.0)
        self.declare_parameter('pose_frame_id', 'airsim_ned')
        self.declare_parameter('camera_frame_id', 'uav_camera')
        self.declare_parameter('takeoff_height', 1.5)
        self.declare_parameter('climb_speed', 1.5)
        self.declare_parameter('setpoint_lead', 1.5)
        self.declare_parameter('setpoint_jump', 0.6)

        self._ip = str(self.get_parameter('airsim_ip').value)
        self._port = int(self.get_parameter('airsim_port').value)
        self._vehicle = str(self.get_parameter('vehicle_name').value)
        self._camera = str(self.get_parameter('camera_name').value)
        self._rate = max(0.5, float(self.get_parameter('publish_rate').value))
        self._pixels_as_float = bool(self.get_parameter('depth_pixels_as_float').value)
        self._cmd_duration = float(self.get_parameter('cmd_duration').value)
        self._connect_period = float(self.get_parameter('connect_period').value)
        self._pose_frame = str(self.get_parameter('pose_frame_id').value)
        self._camera_frame = str(self.get_parameter('camera_frame_id').value)
        self._takeoff_height = float(self.get_parameter('takeoff_height').value)
        self._climb_speed = float(self.get_parameter('climb_speed').value)
        self._setpoint_lead = float(self.get_parameter('setpoint_lead').value)
        self._setpoint_jump = float(self.get_parameter('setpoint_jump').value)

        self._lock = threading.Lock()
        self._client = None
        self._connected = False
        self._armed = False
        self._running = True
        # 机体系速度指令 (vx, vy, vz, yaw_rate)，vz 向下为正（NED）
        self._cmd = (0.0, 0.0, 0.0, 0.0)
        self._retry_count = 0
        # 起飞/降落请求只置位，由工作线程执行。AirSim 客户端不是线程安全的，
        # 两个线程同时调用会报 "Existing exports of data" / "IOLoop is already running"。
        self._want_takeoff = False
        self._want_land = False
        # “移动路标”位控：设定点沿速度方向前进，只在偏离已下发目标时重发一次
        self._setpoint = None
        self._last_sent = None
        self._last_tick = time.monotonic()
        self._last_cmd_time = 0.0

        self._pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self._depth_pub = self.create_publisher(Image, 'depth', 5)
        self._image_pub = self.create_publisher(Image, 'image', 5)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_service(Trigger, 'takeoff', self._on_takeoff)
        self.create_service(Trigger, 'land', self._on_land)

        self._thread = threading.Thread(target=self._worker, name='airsim_io', daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'AirSim 数据源启动：目标 {self._ip}:{self._port}，'
            f'发布频率 {self._rate:.1f} Hz')

    # ------------------------------------------------------------------ 回调
    def _on_cmd_vel(self, msg):
        """缓存速度指令；真正的下发在后台线程里做，避免回调被 RPC 阻塞。"""
        with self._lock:
            self._cmd = (
                float(msg.linear.x),
                float(msg.linear.y),
                float(msg.linear.z),
                float(msg.angular.z),
            )

    def _on_takeoff(self, request, response):
        """起飞服务：只置位，真正的爬升由工作线程执行，避免阻塞 executor。"""
        if not self._connected:
            response.success = False
            response.message = 'AirSim 尚未连接，无法起飞'
            return response
        with self._lock:
            if self._armed:
                # 已经在空中：再走一次起飞流程会把飞机拉回起飞点，
                # 这里直接返回成功，避免上层重试时把航迹搞乱。
                response.success = True
                response.message = '已在飞行中，忽略重复起飞请求'
                return response
        self._want_takeoff = True
        response.success = True
        response.message = '起飞指令已受理，无人机正在爬升'
        return response

    def _on_land(self, request, response):
        """降落服务：只置位，降落过程由工作线程完成。"""
        if not self._connected:
            response.success = False
            response.message = 'AirSim 尚未连接，无法降落'
            return response
        with self._lock:
            self._armed = False
            self._cmd = (0.0, 0.0, 0.0, 0.0)
        self._want_land = True
        response.success = True
        response.message = '降落指令已受理'
        return response

    # -------------------------------------------------------------- AirSim 任务
    def _run_pending_tasks(self):
        """在工作线程里执行起飞/降落，保证同一时刻只有一个线程碰 AirSim 客户端。"""
        if self._want_takeoff:
            self._want_takeoff = False
            try:
                self._client.enableApiControl(True, vehicle_name=self._vehicle)
                self._client.armDisarm(True, vehicle_name=self._vehicle)
                time.sleep(0.5)  # 解锁后 SimpleFlight 需要一点时间进入可控状态
                # 先让 SimpleFlight 自己完成一次起飞：不先离地时 moveToPosition 会被
                # 地面约束吃掉，飞机一动不动；这一步失败也不影响后面的位置指令。
                try:
                    self._client.takeoffAsync(
                        timeout_sec=10.0, vehicle_name=self._vehicle).join()
                except Exception:  # noqa: BLE001 - 起飞接口各版本行为不一致
                    pass
                # 实测：这套 Blocks 场景里 moveByVelocityZ 常常被忽略，但
                # moveToPosition 稳定有效，所以起飞用位置指令一步到位。
                target = -self._takeoff_height
                z = float(self._client.getMultirotorState(
                    vehicle_name=self._vehicle)
                    .kinematics_estimated.position.z_val)
                if z > -1.0:
                    # takeoffAsync 之后偶尔停在 0.8 m 左右，位置指令会被地面约束
                    # 吃掉；先给 1.5 s 垂直速度把飞机"拔起来"，再切位置控制。
                    try:
                        self._client.moveByVelocityZAsync(
                            0.0, 0.0, -1.0, 1.5,
                            vehicle_name=self._vehicle).join()
                    except Exception:  # noqa: BLE001
                        pass
                # moveToPosition 有时会在爬升途中提前返回，重试几次把高度补足，
                # 否则飞机停在 0.8 m 左右，上层会一直认为"还没起飞"。
                for _ in range(3):
                    self._client.moveToPositionAsync(
                        0.0, 0.0, target, self._climb_speed,
                        timeout_sec=20.0, vehicle_name=self._vehicle).join()
                    z = float(self._client.getMultirotorState(
                        vehicle_name=self._vehicle)
                        .kinematics_estimated.position.z_val)
                    if abs(target - z) < 0.3:
                        break
                    time.sleep(0.5)
                self._armed = True
                # 起飞后重新锚定路标，避免沿用地面时的旧设定点
                self._setpoint = None
                self._last_sent = None
                self._last_tick = time.monotonic()
                if abs(target - z) >= 0.3:
                    self.get_logger().warn(f'起飞高度偏差较大，当前 {-z:.2f} m')
                self.get_logger().info(
                    f'已起飞并解锁（高度 {-z:.2f} m），可以接收 /uav/cmd_vel')
            except Exception as exc:  # noqa: BLE001 - 任何 RPC 异常都要保住节点
                self.get_logger().error(f'起飞失败：{exc}')

        if self._want_land:
            self._want_land = False
            try:
                self._client.landAsync(
                    timeout_sec=30.0, vehicle_name=self._vehicle).join()
                self._client.armDisarm(False, vehicle_name=self._vehicle)
                self._armed = False
                self.get_logger().info('已降落并上锁')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'降落失败：{exc}')

    # ------------------------------------------------------------- 后台主循环
    def _worker(self):
        """后台线程：按发布频率读传感器、下发速度，阻塞调用都在这里。"""
        period = 1.0 / self._rate
        while self._running and rclpy.ok():
            start = time.monotonic()
            if not self._connected:
                self._try_connect()
            if self._connected:
                try:
                    self._run_pending_tasks()
                    self._tick()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'AirSim 读写异常，准备重连：{exc}')
                    self._connected = False
            remain = period - (time.monotonic() - start)
            if remain > 0.0:
                time.sleep(remain)

    def _try_connect(self):
        """连接 AirSim，失败就等下一个周期重试（AirSim 比节点晚启动也能救回来）。"""
        if airsim is None:
            self.get_logger().error(
                '未安装 airsim 客户端：请在 Ubuntu 里执行 pip3 install airsim')
            time.sleep(self._connect_period)
            return
        try:
            client = airsim.MultirotorClient(ip=self._ip, port=self._port)
            client.confirmConnection()
        except Exception as exc:  # noqa: BLE001
            self._retry_count += 1
            if self._retry_count % 5 == 1:
                self.get_logger().warn(
                    f'连接 AirSim {self._ip}:{self._port} 失败（第 {self._retry_count} 次）：'
                    f'{exc}，{self._connect_period:.0f}s 后重试')
            time.sleep(self._connect_period)
            return
        with self._lock:
            self._client = client
        self._connected = True
        self._retry_count = 0
        self.get_logger().info(f'已连接 AirSim {self._ip}:{self._port}')

    def _tick(self):
        """一次采样：读位姿 + 两路图像，然后下发速度指令、发布消息。"""
        stamp = self.get_clock().now().to_msg()
        with self._lock:
            client = self._client
            cmd = self._cmd
            armed = self._armed
            state = client.getMultirotorState(vehicle_name=self._vehicle)
            responses = client.simGetImages(
                [
                    airsim.ImageRequest(self._camera, airsim.ImageType.Scene, False, False),
                    airsim.ImageRequest(
                        self._camera, airsim.ImageType.DepthPerspective,
                        self._pixels_as_float, False),
                ],
                vehicle_name=self._vehicle,
            )
            if armed:
                vx, vy, vz, yaw_rate = cmd
                # 这套 Blocks 场景里速度指令会被忽略，位置指令才可靠；但位置
                # 指令每帧重下发会让飞机几乎不动（控制器被反复重置）。实测
                # 有效的办法是“移动路标”：维护一个沿速度方向前进的设定点，
                # 只有设定点比上次下发的目标远出 0.6 m 时才重新下发一次。
                pos = state.kinematics_estimated.position
                quat = state.kinematics_estimated.orientation
                _, _, yaw = airsim.to_eularian_angles(quat)
                now = time.monotonic()
                dt = min(0.5, max(0.01, now - self._last_tick))
                self._last_tick = now
                if self._setpoint is None:
                    self._setpoint = (pos.x_val, pos.y_val, pos.z_val)
                    self._last_sent = self._setpoint
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                world_vx = vx * cos_yaw - vy * sin_yaw
                world_vy = vx * sin_yaw + vy * cos_yaw
                sx, sy, sz = self._setpoint
                # 速度为 0（悬停/到达）时不再推进路标，否则飞机越过终点还会继续飞
                if math.hypot(math.hypot(vx, vy), vz) < 0.05:
                    sx, sy, sz = pos.x_val, pos.y_val, pos.z_val
                else:
                    sx += world_vx * dt
                    sy += world_vy * dt
                    sz += vz * dt
                # 路标不要跑得比飞机快太多，否则飞机会一直追不上
                ox, oy, oz = sx - pos.x_val, sy - pos.y_val, sz - pos.z_val
                offset = math.sqrt(ox * ox + oy * oy + oz * oz)
                if offset > self._setpoint_lead:
                    sx = pos.x_val + ox / offset * self._setpoint_lead
                    sy = pos.y_val + oy / offset * self._setpoint_lead
                    sz = pos.z_val + oz / offset * self._setpoint_lead
                self._setpoint = (sx, sy, sz)
                sent = self._last_sent
                jump = math.dist(self._setpoint, sent)
                # 即使路标没动也要周期性地重发：SimpleFlight 单条位置指令不会
                # 一直生效，实测每 1 s 重发一次才能持续跟进。
                stale = (now - self._last_cmd_time) > self._cmd_duration
                if jump > self._setpoint_jump or stale:
                    speed = max(0.8, math.sqrt(vx * vx + vy * vy + vz * vz))
                    client.moveToPositionAsync(
                        sx, sy, sz, speed, vehicle_name=self._vehicle)
                    self._last_sent = self._setpoint
                    self._last_cmd_time = now
                    if abs(yaw_rate) > 0.05:
                        client.rotateByYawRateAsync(
                            math.degrees(yaw_rate), dt, vehicle_name=self._vehicle)
                self._dbg = getattr(self, '_dbg', 0) + 1
                if self._dbg % 10 == 0:
                    self.get_logger().info(
                        f'DBG pos=({pos.x_val:.2f},{pos.y_val:.2f},{pos.z_val:.2f}) '
                        f'v=({vx:.2f},{vy:.2f},{vz:.2f}) '
                        f'set=({sx:.2f},{sy:.2f},{sz:.2f})')

        self._publish_pose(state, stamp)
        if responses:
            self._publish_image(responses[0], stamp)
        if len(responses) > 1:
            self._publish_depth(responses[1], stamp)

    # ----------------------------------------------------------------- 发布端
    def _publish_pose(self, state, stamp):
        """发布位姿，坐标系为 AirSim 原始 NED（x 北 / y 东 / z 向下）。"""
        pose = state.kinematics_estimated.position
        quat = state.kinematics_estimated.orientation
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._pose_frame
        msg.pose.position.x = float(pose.x_val)
        msg.pose.position.y = float(pose.y_val)
        msg.pose.position.z = float(pose.z_val)
        msg.pose.orientation.w = float(quat.w_val)
        msg.pose.orientation.x = float(quat.x_val)
        msg.pose.orientation.y = float(quat.y_val)
        msg.pose.orientation.z = float(quat.z_val)
        self._pose_pub.publish(msg)

    def _publish_image(self, response, stamp):
        """发布前视 RGB 图（bgr8）。AirSim 原生是 BGRA，这里丢掉 alpha 通道。"""
        width = int(response.width)
        height = int(response.height)
        if width <= 0 or height <= 0:
            return
        raw = np.frombuffer(bytes(response.image_data_uint8), dtype=np.uint8)
        channels = raw.size // (width * height)
        if channels == 4:
            pixels = raw.reshape(height, width, 4)[:, :, :3]
        elif channels == 3:
            pixels = raw.reshape(height, width, 3)
        else:
            return
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._camera_frame
        msg.height = height
        msg.width = width
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = np.ascontiguousarray(pixels).tobytes()
        self._image_pub.publish(msg)

    def _publish_depth(self, response, stamp):
        """发布深度图（32FC1，单位米）。必须用 float 数据，uint8 会被压缩到 0-255。"""
        width = int(response.width)
        height = int(response.height)
        if width <= 0 or height <= 0:
            return
        depth = np.asarray(response.image_data_float, dtype=np.float32)
        if depth.size != width * height:
            return
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._camera_frame
        msg.height = height
        msg.width = width
        msg.encoding = '32FC1'
        msg.is_bigendian = 0
        msg.step = width * 4
        msg.data = depth.reshape(height, width).tobytes()
        self._depth_pub.publish(msg)

    # ----------------------------------------------------------------- 生命周期
    def shutdown(self):
        """停止后台线程并释放 AirSim 控制权。"""
        self._running = False
        thread = getattr(self, '_thread', None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        if self._connected and self._client is not None:
            with self._lock:
                try:
                    self._client.armDisarm(False, vehicle_name=self._vehicle)
                    self._client.enableApiControl(False, vehicle_name=self._vehicle)
                except Exception:  # noqa: BLE001 - 退出阶段不再抛出
                    pass
        self.get_logger().info('AirSim 数据源已停止')

def main(args=None):
    """ROS 2 入口：单独启动数据源节点。"""
    rclpy.init(args=args)
    node = None
    try:
        node = AirSimSource()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - 启动失败要打印出来而不是静默退出
        print(f'[uav_depth_track_nn] AirSim 数据源启动失败: {exc}')
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
