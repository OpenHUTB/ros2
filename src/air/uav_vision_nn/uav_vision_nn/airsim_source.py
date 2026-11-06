#!/usr/bin/env python3
"""AirSim 与 ROS2 的桥接节点。

上行：AirSim -> /uav/image (bgr8)、/uav/pose (NED)、/uav/depth (32FC1)
下行：/uav/cmd_vel -> AirSim 机体系速度指令
服务：/uav/takeoff、/uav/land

设计约束：
1. 所有 AirSim RPC 都放在独立工作线程里，ROS 回调只做赋值，绝不阻塞 executor；
2. 工作线程里的 AirSim 调用一律持锁，避免多线程争用同一个 gRPC 连接；
3. 速度指令只下发不 join（由定时循环持续覆盖），但起飞/降落这类一次性
   指令必须 join，否则 AirSim 不会真正执行，见 _do_takeoff/_do_land。
"""

import threading
import time

import airsim
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

MODE_IDLE = 'idle'
MODE_TAKEOFF = 'taking_off'
MODE_FLYING = 'flying'
MODE_LANDING = 'landing'


class AirSimSource(Node):
    """把 AirSim 的相机、位姿和控制接口包装成 ROS2 话题与服务。"""

    def __init__(self):
        super().__init__('airsim_source')

        self._ip = str(self.declare_parameter('airsim_ip', '10.0.2.2').value)
        self._port = int(self.declare_parameter('airsim_port', 41451).value)
        self._vehicle = str(self.declare_parameter('vehicle_name', '').value)
        self._frame_rate = float(self.declare_parameter('frame_rate', 10.0).value)
        self._pose_rate = float(self.declare_parameter('pose_rate', 20.0).value)
        self._depth_rate = float(self.declare_parameter('depth_rate', 5.0).value)
        self._control_rate = float(self.declare_parameter('control_rate', 10.0).value)
        self._enable_depth = bool(self.declare_parameter('enable_depth', True).value)
        self._cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.5).value)
        # 爬升目标高度（米，相对 AirSim 原点；NED 下 z 为负）与爬升速度
        self._takeoff_height = float(
            self.declare_parameter('takeoff_height', 1.5).value)
        self._climb_speed = float(
            self.declare_parameter('climb_speed', 1.5).value)

        self._lock = threading.RLock()
        self._cmd = Twist()
        self._cmd_stamp = 0.0
        self._mode = MODE_IDLE
        self._stop = threading.Event()
        # 起飞/降落只置位，由工作线程执行：AirSim 客户端不是线程安全的。
        self._takeoff_requested = False
        self._land_requested = False

        self._client = airsim.MultirotorClient(ip=self._ip, port=self._port)
        self._client.confirmConnection()
        self._client.enableApiControl(True, vehicle_name=self._vehicle)
        self._client.armDisarm(True, vehicle_name=self._vehicle)
        self.get_logger().info(
            f'已连接 AirSim {self._ip}:{self._port}，API 控制已使能（未起飞）')

        self._pub_image = self.create_publisher(Image, '~/image', 1)
        self._pub_pose = self.create_publisher(PoseStamped, '~/pose', 10)
        self._pub_depth = self.create_publisher(Image, '~/depth', 1)

        self.create_subscription(Twist, '~/cmd_vel', self._on_cmd_vel, 10)
        self.create_service(Trigger, '~/takeoff', self._on_takeoff)
        self.create_service(Trigger, '~/land', self._on_land)

        self._thread = threading.Thread(
            target=self._worker, name='airsim_worker', daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ 回调
    def _on_cmd_vel(self, msg):
        """只记录最新指令，真正的下发在工作线程里做。"""
        self._cmd = msg
        self._cmd_stamp = time.monotonic()

    def _on_takeoff(self, request, response):
        if self._mode != MODE_IDLE:
            response.success = False
            response.message = f'当前状态为 {self._mode}，忽略起飞请求'
            return response
        # 只置位，由工作线程执行 takeoffAsync().join()：在服务回调里同步等待
        # 会阻塞 executor，放到另一个线程又会和取图像的线程争用同一个 AirSim
        # 客户端（表现为 "Existing exports of data" / "IOLoop is already running"）。
        self._mode = MODE_TAKEOFF
        self._takeoff_requested = True
        response.success = True
        response.message = '已下达起飞指令'
        self.get_logger().info('收到起飞请求')
        return response

    def _handle_pending_actions(self):
        """在工作线程里执行一次性动作，保证同一时刻只有一个线程碰客户端。"""
        if self._takeoff_requested:
            self._takeoff_requested = False
            try:
                with self._lock:
                    self._client.enableApiControl(True, vehicle_name=self._vehicle)
                    self._client.armDisarm(True, vehicle_name=self._vehicle)
                # 不用 takeoffAsync：在 OpenHUTB 的 Blocks 场景里它有时只是把
                # 飞机放到地面高度就返回。改用速度指令持续爬升，目标高度可控。
                deadline = time.monotonic() + 15.0
                z = 0.0
                # 留 5 cm 容差：浮点下几乎不会精确等于目标值
                target = -(self._takeoff_height - 0.05)
                while time.monotonic() < deadline:
                    self._client.moveByVelocityZAsync(
                        0.0, 0.0, -self._climb_speed, 1.0,
                        vehicle_name=self._vehicle)
                    time.sleep(0.3)
                    state = self._client.getMultirotorState(
                        vehicle_name=self._vehicle)
                    z = float(state.kinematics_estimated.position.z_val)
                    if z <= target:
                        break
                if z > target:
                    self.get_logger().warn(
                        f'爬升未达标，当前高度 {-z:.2f} m')
            except Exception as exc:
                self.get_logger().warn(f'起飞执行失败: {exc}')
                self._mode = MODE_IDLE

        if self._land_requested:
            self._land_requested = False
            try:
                self._client.landAsync(
                    timeout_sec=20.0, vehicle_name=self._vehicle).join()
                self.get_logger().info('降落动作执行完成')
            except Exception as exc:
                self.get_logger().warn(f'降落执行失败: {exc}')

    def _on_land(self, request, response):
        if self._mode not in (MODE_FLYING, MODE_TAKEOFF):
            response.success = False
            response.message = f'当前状态为 {self._mode}，忽略降落请求'
            return response
        self._mode = MODE_LANDING
        self._land_requested = True
        response.success = True
        response.message = '已下达降落指令'
        self.get_logger().info('收到降落请求')
        return response

    # -------------------------------------------------------------- 工作线程
    def _worker(self):
        period = 1.0 / max(self._control_rate, 1.0)
        image_period = 1.0 / max(self._frame_rate, 1.0)
        pose_period = 1.0 / max(self._pose_rate, 1.0)
        depth_period = 1.0 / max(self._depth_rate, 1.0)

        next_image = next_pose = next_depth = 0.0
        while not self._stop.is_set():
            begin = time.monotonic()
            try:
                self._handle_pending_actions()
                self._update_mode()
                if begin >= next_pose:
                    self._publish_pose()
                    next_pose = begin + pose_period
                if begin >= next_image:
                    self._publish_image()
                    next_image = begin + image_period
                if self._enable_depth and begin >= next_depth:
                    self._publish_depth()
                    next_depth = begin + depth_period
                self._send_velocity(period)
            except Exception as exc:  # 单次 RPC 失败不应终止整个桥
                self.get_logger().warn(
                    f'AirSim 调用异常: {exc}', throttle_duration_sec=2.0)

            rest = period - (time.monotonic() - begin)
            self._stop.wait(rest if rest > 0.0 else 0.0)

    def _update_mode(self):
        """起飞/降落是异步指令，靠轮询高度确认状态迁移。"""
        if self._mode not in (MODE_TAKEOFF, MODE_LANDING):
            return
        with self._lock:
            state = self._client.getMultirotorState(vehicle_name=self._vehicle)
        z = float(state.kinematics_estimated.position.z_val)

        if self._mode == MODE_TAKEOFF and z < -1.0:
            self._mode = MODE_FLYING
            self.get_logger().info(f'起飞完成，当前高度 {-z:.2f} m')
        elif self._mode == MODE_LANDING and z > -0.3:
            with self._lock:
                self._client.armDisarm(False, vehicle_name=self._vehicle)
            self._mode = MODE_IDLE
            self.get_logger().info('已落地并解除解锁')

    def _send_velocity(self, period):
        if self._mode != MODE_FLYING:
            return

        if time.monotonic() - self._cmd_stamp > self._cmd_timeout:
            vx = vy = vz = 0.0  # 指令超时，发零速度保持悬停
        else:
            vx = float(self._cmd.linear.x)
            vy = float(self._cmd.linear.y)
            vz = float(self._cmd.linear.z)

        with self._lock:
            self._client.moveByVelocityBodyFrameAsync(
                vx, vy, vz, max(0.2, 2.0 * period), vehicle_name=self._vehicle)

    # ------------------------------------------------------------------ 发布
    def _publish_pose(self):
        with self._lock:
            state = self._client.getMultirotorState(vehicle_name=self._vehicle)
        position = state.kinematics_estimated.position
        orientation = state.kinematics_estimated.orientation

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world_ned'
        msg.pose.position.x = float(position.x_val)
        msg.pose.position.y = float(position.y_val)
        msg.pose.position.z = float(position.z_val)
        msg.pose.orientation.w = float(orientation.w_val)
        msg.pose.orientation.x = float(orientation.x_val)
        msg.pose.orientation.y = float(orientation.y_val)
        msg.pose.orientation.z = float(orientation.z_val)
        self._pub_pose.publish(msg)

    def _publish_image(self):
        with self._lock:
            responses = self._client.simGetImages(
                [airsim.ImageRequest('fpv', airsim.ImageType.Scene, False, False)],
                vehicle_name=self._vehicle)
        if not responses or not responses[0].image_data_uint8:
            return

        response = responses[0]
        rgb = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        rgb = rgb.reshape(response.height, response.width, 3)
        bgr = np.ascontiguousarray(rgb[:, :, ::-1])  # AirSim 给的是 RGB
        self._pub_image.publish(self._to_image_msg(bgr, 'bgr8', 'fpv'))

    def _publish_depth(self):
        with self._lock:
            responses = self._client.simGetImages(
                [airsim.ImageRequest('fpv', airsim.ImageType.DepthPerspective, True, False)],
                vehicle_name=self._vehicle)
        if not responses or not responses[0].image_data_float:
            return

        response = responses[0]
        depth = np.array(response.image_data_float, dtype=np.float32)
        depth = depth.reshape(response.height, response.width)
        self._pub_depth.publish(self._to_image_msg(depth, '32FC1', 'fpv'))

    def _to_image_msg(self, array, encoding, frame_id):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = int(array.shape[0])
        msg.width = int(array.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(msg.width * array.dtype.itemsize * (3 if array.ndim == 3 else 1))
        msg.data = array.tobytes()
        return msg

    # ------------------------------------------------------------------ 退出
    def shutdown(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._mode == MODE_FLYING:
            try:
                with self._lock:
                    self._client.landAsync(timeout_sec=15.0, vehicle_name=self._vehicle)
                self.get_logger().info('退出前已下发降落指令')
            except Exception as exc:
                self.get_logger().warn(f'退出时降落失败: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AirSimSource()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # 连不上 AirSim 时给出可读原因
        print(f'[uav_vision_nn] AirSim 桥启动失败: {exc}')
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
