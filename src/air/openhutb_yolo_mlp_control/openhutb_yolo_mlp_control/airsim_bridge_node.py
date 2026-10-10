import math
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from .vendor import airsim


class AirSimBridgeNode(Node):
    """OpenHUTB / AirSim 与 ROS 2 之间的桥接节点。

    ROS 接口：
        发布：
            /openhutb/camera/rgb
            /openhutb/odom
            /openhutb/metrics/airsim_capture_ms

        订阅：
            /openhutb/cmd_vel

    当前测试的 OpenHUTB AIR 构建能够可靠响应 moveToPositionAsync()，
    但不能可靠响应 moveByVelocityAsync()。

    因此桥接节点仍对外提供基于速度的 ROS 控制接口
    （geometry_msgs/Twist），将速度指令积分为位移，并周期性发送
    非阻塞的 moveToPositionAsync() 指令。

    注意：
        本文件依赖 vendor/msgpackrpc_compat.py 提供真正异步的
        call_async()，以保证 moveToPositionAsync() 立即返回。
    """

    def __init__(self):
        super().__init__('airsim_bridge')

        # ============================================================
        # 参数声明
        # ============================================================
        self.declare_parameter(
            'airsim_ip',
            '127.0.0.1',
        )
        self.declare_parameter(
            'airsim_port',
            41451,
        )
        self.declare_parameter(
            'camera_name',
            '0',
        )

        self.declare_parameter(
            'image_hz',
            0.5,
        )
        self.declare_parameter(
            'state_hz',
            20.0,
        )

        self.declare_parameter(
            'enable_api_control',
            False,
        )
        self.declare_parameter(
            'arm_on_start',
            False,
        )
        self.declare_parameter(
            'auto_takeoff',
            False,
        )

        # 用于将 Twist 速度指令积分为位移：
        #
        # dx = vx * command_duration
        # dy = vy * command_duration
        # dz = vz * command_duration
        self.declare_parameter(
            'command_duration',
            0.15,
        )

        # 如果超过该时长未收到 Twist，则丢弃尚未执行的运动量。
        self.declare_parameter(
            'command_timeout',
            0.60,
        )

        # OpenHUTB 对很小的位置变化响应不稳定，因此先累计指令，
        # 达到该最小位移后再发送。
        self.declare_parameter(
            'min_dispatch_distance',
            1.0,
        )

        # 单次位置指令的安全位移上限。
        self.declare_parameter(
            'max_dispatch_distance',
            1.5,
        )

        self.declare_parameter(
            'min_position_speed',
            0.50,
        )
        self.declare_parameter(
            'max_position_speed',
            2.0,
        )

        self.declare_parameter(
            'control_backend',
            'auto',
        )

        self.declare_parameter(
            'command_smoothing',
            0.25,
        )

        self.declare_parameter(
            'pose_max_speed',
            1.5,
        )

        self.declare_parameter(
            'camera_topic',
            '/openhutb/camera/rgb',
        )
        self.declare_parameter(
            'odom_topic',
            '/openhutb/odom',
        )
        self.declare_parameter(
            'cmd_vel_topic',
            '/openhutb/cmd_vel',
        )
        self.declare_parameter(
            'capture_metric_topic',
            '/openhutb/metrics/airsim_capture_ms',
        )

        # ============================================================
        # 读取参数
        # ============================================================
        ip = str(
            self.get_parameter(
                'airsim_ip'
            ).value
        )

        port = int(
            self.get_parameter(
                'airsim_port'
            ).value
        )

        self.camera_name = str(
            self.get_parameter(
                'camera_name'
            ).value
        )

        self.command_duration = float(
            self.get_parameter(
                'command_duration'
            ).value
        )

        self.command_timeout = float(
            self.get_parameter(
                'command_timeout'
            ).value
        )

        self.min_dispatch_distance = float(
            self.get_parameter(
                'min_dispatch_distance'
            ).value
        )

        self.max_dispatch_distance = float(
            self.get_parameter(
                'max_dispatch_distance'
            ).value
        )

        self.min_position_speed = float(
            self.get_parameter(
                'min_position_speed'
            ).value
        )

        self.max_position_speed = float(
            self.get_parameter(
                'max_position_speed'
            ).value
        )

        requested_backend = str(
            self.get_parameter(
                'control_backend'
            ).value
        ).strip().lower()

        self.command_smoothing = min(
            max(
                float(
                    self.get_parameter(
                        'command_smoothing'
                    ).value
                ),
                0.0,
            ),
            1.0,
        )

        self.pose_max_speed = max(
            float(
                self.get_parameter(
                    'pose_max_speed'
                ).value
            ),
            0.1,
        )

        if requested_backend not in (
            'auto',
            'flight',
            'pose',
        ):
            raise ValueError(
                'control_backend must be auto, flight, or pose'
            )

        self.enable_api_control = bool(
            self.get_parameter(
                'enable_api_control'
            ).value
        )

        # 启动过程必须保持非阻塞。部分 OpenHUTB 构建在无人机已经开始运动后，
        # 起飞 RPC Future 仍不会结束；如果在此调用 .join()，ROS 发布器和
        # 定时器将永远无法创建。
        self._takeoff_future = None
        self._takeoff_started_at = None
        self._takeoff_complete_logged = False

        # ============================================================
        # AirSim 客户端
        #
        # 感知与控制使用相互独立的 RPC 连接。
        # ============================================================
        self.sensor_client = (
            airsim.MultirotorClient(
                ip=ip,
                port=port,
            )
        )

        self.control_client = (
            airsim.MultirotorClient(
                ip=ip,
                port=port,
            )
        )

        self.get_logger().info(
            f'Connecting sensor RPC to '
            f'{ip}:{port}'
        )

        self.sensor_client.confirmConnection()

        self.get_logger().info(
            f'Connecting control RPC to '
            f'{ip}:{port}'
        )

        self.control_client.confirmConnection()

        self.get_logger().info(
            'AirSim sensor/control RPC '
            'connections established.'
        )

        flight_state = self.control_client.getMultirotorState()
        flight_ready = bool(flight_state.ready)

        self._pose_control = (
            requested_backend == 'pose'
            or (
                requested_backend == 'auto'
                and not flight_ready
            )
        )

        if self._pose_control:
            self.get_logger().warning(
                'AirSim flight controller is not ready; '
                'using incremental simSetVehiclePose control. '
                f'ready={flight_state.ready}, '
                f'can_arm={flight_state.can_arm}, '
                f'landed_state={flight_state.landed_state}'
            )
        else:
            self.get_logger().info(
                'Using native AirSim flight control backend.'
            )

        # ============================================================
        # API 控制
        # ============================================================
        if self.enable_api_control and not self._pose_control:
            self.get_logger().info(
                'Enabling AirSim API control.'
            )

            self.control_client.enableApiControl(
                True
            )

            if bool(
                self.get_parameter(
                    'arm_on_start'
                ).value
            ):
                self.get_logger().info(
                    'Arming multirotor.'
                )

                self.control_client.armDisarm(
                    True
                )

            if bool(
                self.get_parameter(
                    'auto_takeoff'
                ).value
            ):
                self.get_logger().info(
                    'Auto takeoff requested.'
                )

                # 不要在 __init__ 中等待该 Future。部分 OpenHUTB 构建的 RPC
                # 永远不会报告完成，会阻塞状态发布和全部 ROS 回调。
                self._takeoff_future = (
                    self.control_client.takeoffAsync(5.0)
                )
                self._takeoff_started_at = time.monotonic()

                self.get_logger().info(
                    'Auto takeoff command dispatched; '
                    'continuing ROS startup without waiting for RPC join.'
                )

        # ============================================================
        # ROS 话题
        # ============================================================
        camera_topic = str(
            self.get_parameter(
                'camera_topic'
            ).value
        )

        odom_topic = str(
            self.get_parameter(
                'odom_topic'
            ).value
        )

        cmd_vel_topic = str(
            self.get_parameter(
                'cmd_vel_topic'
            ).value
        )

        metric_topic = str(
            self.get_parameter(
                'capture_metric_topic'
            ).value
        )

        self.image_pub = (
            self.create_publisher(
                Image,
                camera_topic,
                qos_profile_sensor_data,
            )
        )

        self.odom_pub = (
            self.create_publisher(
                Odometry,
                odom_topic,
                qos_profile_sensor_data,
            )
        )

        self.capture_pub = (
            self.create_publisher(
                Float32,
                metric_topic,
                10,
            )
        )

        self.cmd_sub = (
            self.create_subscription(
                Twist,
                cmd_vel_topic,
                self.cmd_vel_callback,
                10,
            )
        )

        # ============================================================
        # 控制工作线程状态
        # ============================================================
        self._cmd_lock = threading.Lock()
        self._cmd_event = threading.Event()
        self._stop_worker = threading.Event()

        self._latest_cmd = None
        self._filtered_cmd = np.zeros(
            3,
            dtype=np.float64,
        )

        # 尚未发送的期望位移累计值。
        self._accumulated_delta = np.zeros(
            3,
            dtype=np.float64,
        )

        self._last_cmd_time = None
        self._first_cmd_logged = False

        # 保留最近一次 Future，使桥接代码也持有明确引用。
        self._last_move_future = None

        self._control_worker = (
            threading.Thread(
                target=self._control_worker_loop,
                name='openhutb-position-worker',
                daemon=True,
            )
        )

        self._control_worker.start()

        # ============================================================
        # 定时器
        # ============================================================
        image_hz = max(
            float(
                self.get_parameter(
                    'image_hz'
                ).value
            ),
            0.1,
        )

        state_hz = max(
            float(
                self.get_parameter(
                    'state_hz'
                ).value
            ),
            0.1,
        )

        self.image_timer = (
            self.create_timer(
                1.0 / image_hz,
                self.publish_image,
            )
        )

        self.state_timer = (
            self.create_timer(
                1.0 / state_hz,
                self.publish_state,
            )
        )

        self.watchdog_timer = (
            self.create_timer(
                0.1,
                self.command_watchdog,
            )
        )

        self.get_logger().info(
            'Bridge ready; '
            'backend=non-blocking '
            'thresholded moveToPositionAsync, '
            f'min_dispatch='
            f'{self.min_dispatch_distance:.2f} m'
        )

    # ================================================================
    # 将 AirSim 状态发布为 ROS 里程计
    # ================================================================
    def publish_state(self):
        try:
            state = (
                self.sensor_client
                .getMultirotorState()
            )

        except Exception as exc:
            self.get_logger().error(
                'getMultirotorState failed: '
                f'{exc}'
            )
            return

        kin = state.kinematics_estimated

        p = kin.position
        q = kin.orientation
        v = kin.linear_velocity
        w = kin.angular_velocity

        msg = Odometry()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            'airsim_ned'
        )

        msg.child_frame_id = (
            'drone'
        )

        msg.pose.pose.position.x = (
            float(p.x_val)
        )
        msg.pose.pose.position.y = (
            float(p.y_val)
        )
        msg.pose.pose.position.z = (
            float(p.z_val)
        )

        msg.pose.pose.orientation.x = (
            float(q.x_val)
        )
        msg.pose.pose.orientation.y = (
            float(q.y_val)
        )
        msg.pose.pose.orientation.z = (
            float(q.z_val)
        )
        msg.pose.pose.orientation.w = (
            float(q.w_val)
        )

        msg.twist.twist.linear.x = (
            float(v.x_val)
        )
        msg.twist.twist.linear.y = (
            float(v.y_val)
        )
        msg.twist.twist.linear.z = (
            float(v.z_val)
        )

        msg.twist.twist.angular.x = (
            float(w.x_val)
        )
        msg.twist.twist.angular.y = (
            float(w.y_val)
        )
        msg.twist.twist.angular.z = (
            float(w.z_val)
        )

        self.odom_pub.publish(
            msg
        )

    # ================================================================
    # 将 AirSim RGB 图像发布为 ROS 图像
    # ================================================================
    def publish_image(self):
        start = time.perf_counter()

        try:
            responses = (
                self.sensor_client
                .simGetImages([
                    airsim.ImageRequest(
                        self.camera_name,
                        airsim.ImageType.Scene,
                        False,
                        True,
                    )
                ])
            )

        except Exception as exc:
            self.get_logger().warning(
                'simGetImages failed: '
                f'{exc}'
            )
            return

        metric = Float32()

        metric.data = float(
            (
                time.perf_counter()
                - start
            )
            * 1000.0
        )

        self.capture_pub.publish(
            metric
        )

        if not responses:
            return

        response = responses[0]

        if (
            response.width <= 0
            or response.height <= 0
            or not response.image_data_uint8
        ):
            return

        compressed = np.frombuffer(
            response.image_data_uint8,
            dtype=np.uint8,
        )

        frame = cv2.imdecode(
            compressed,
            cv2.IMREAD_COLOR,
        )

        if frame is None:
            return

        msg = Image()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            'airsim_camera_0'
        )

        msg.height = int(
            frame.shape[0]
        )

        msg.width = int(
            frame.shape[1]
        )

        msg.encoding = (
            'bgr8'
        )

        msg.is_bigendian = (
            False
        )

        msg.step = int(
            frame.shape[1]
            * 3
        )

        msg.data = (
            frame.tobytes()
        )

        self.image_pub.publish(
            msg
        )

    # ================================================================
    # ROS Twist 输入
    # ================================================================
    def cmd_vel_callback(
        self,
        msg: Twist,
    ):
        if not self.enable_api_control:
            return

        # 异步起飞指令仍在执行时不发送水平位置指令。Future 完成后继续控制；
        # 对于始终不报告完成的 OpenHUTB 构建，由看门狗提供安全超时，
        # 同时保持状态发布和控制回调可用。
        if (
            self._takeoff_future is not None
            and not self._takeoff_future.done()
            and self._takeoff_started_at is not None
            and time.monotonic() - self._takeoff_started_at < 8.0
        ):
            return

        vx = float(
            msg.linear.x
        )
        vy = float(
            msg.linear.y
        )
        vz = float(
            msg.linear.z
        )

        self._last_cmd_time = (
            time.monotonic()
        )

        if not self._first_cmd_logged:
            self._first_cmd_logged = True

            self.get_logger().info(
                'First /openhutb/cmd_vel '
                'received: '
                f'({vx:.2f}, '
                f'{vy:.2f}, '
                f'{vz:.2f})'
            )

        with self._cmd_lock:
            raw_cmd = np.array(
                [vx, vy, vz],
                dtype=np.float64,
            )

            # 指数速度平滑可削弱学习控制器在航点附近产生的急剧反向指令。
            alpha = self.command_smoothing
            self._filtered_cmd = (
                (1.0 - alpha) * self._filtered_cmd
                + alpha * raw_cmd
            )

            if self._pose_control:
                horizontal_speed = float(
                    np.linalg.norm(
                        self._filtered_cmd[:2]
                    )
                )
                if horizontal_speed > self.pose_max_speed:
                    self._filtered_cmd[:2] *= (
                        self.pose_max_speed
                        / horizontal_speed
                    )

            filtered = self._filtered_cmd.copy()
            self._latest_cmd = tuple(
                float(value)
                for value in filtered
            )

            self._accumulated_delta += (
                filtered
                * self.command_duration
            )

        self._cmd_event.set()

    # ================================================================
    # 控制工作线程
    #
    # 注意：此处不能等待 moveToPositionAsync() 完成。
    # ================================================================
    def _control_worker_loop(self):
        while not self._stop_worker.is_set():

            self._cmd_event.wait(
                timeout=0.1
            )

            if self._stop_worker.is_set():
                break

            with self._cmd_lock:
                cmd = self._latest_cmd

                delta = (
                    self._accumulated_delta
                    .copy()
                )

                self._cmd_event.clear()

            if cmd is None:
                continue

            lead = float(
                np.linalg.norm(
                    delta
                )
            )

            # 姿态后备控制必须与旧的同步速度循环一致：每条新指令只执行一次
            # 短时积分。若累计到距离阈值后才执行，会产生明显跳变，并可能
            # 执行已经过期的指令。
            if self._pose_control:
                vx, vy, vz = cmd
                delta = np.array(
                    [vx, vy, vz],
                    dtype=np.float64,
                ) * self.command_duration
                lead = float(np.linalg.norm(delta))

                if lead <= 1e-9:
                    continue

                with self._cmd_lock:
                    self._accumulated_delta[:] = 0.0

            # 不发送过小的位置目标。
            if (
                not self._pose_control
                and
                lead
                < self.min_dispatch_distance
            ):
                continue

            try:
                # 发送前读取一次当前位置。
                state = (
                    self.control_client
                    .getMultirotorState()
                )

                p = (
                    state
                    .kinematics_estimated
                    .position
                )

                current = np.array(
                    [
                        float(p.x_val),
                        float(p.y_val),
                        float(p.z_val),
                    ],
                    dtype=np.float64,
                )

                # 限制单次发送的位移，保证安全。
                if (
                    lead
                    > self.max_dispatch_distance
                ):
                    delta *= (
                        self.max_dispatch_distance
                        / lead
                    )

                    lead = (
                        self.max_dispatch_distance
                    )

                target = (
                    current
                    + delta
                )

                vx, vy, vz = cmd

                speed = math.sqrt(
                    vx * vx
                    + vy * vy
                    + vz * vz
                )

                speed = max(
                    speed,
                    self.min_position_speed,
                )

                speed = min(
                    speed,
                    self.max_position_speed,
                )

                self.get_logger().info(
                    'Dispatching OpenHUTB '
                    'position move: '
                    f'lead={lead:.2f} m, '
                    f'target=('
                    f'{target[0]:.2f}, '
                    f'{target[1]:.2f}, '
                    f'{target[2]:.2f}), '
                    f'speed={speed:.2f}'
                )

                # ====================================================
                # 真正的非阻塞指令
                #
                # 此处不调用 .join()。ROS 保持运行时，新的 AirSim 异步运动
                # 指令可以替换或取消上一条指令。
                # ====================================================
                if self._pose_control:
                    pose = self.control_client.simGetVehiclePose()
                    pose.position.x_val = float(target[0])
                    pose.position.y_val = float(target[1])
                    pose.position.z_val = float(target[2])
                    self.control_client.simSetVehiclePose(
                        pose,
                        True,
                    )
                    self._last_move_future = None
                else:
                    self._last_move_future = (
                        self.control_client
                        .moveToPositionAsync(
                            float(target[0]),
                            float(target[1]),
                            float(target[2]),
                            float(speed),
                            timeout_sec=10.0,
                        )
                    )

                self.get_logger().info(
                    'OpenHUTB position '
                    'command dispatched.'
                )

                # 只减去本次已经发送的位移。工作线程准备 RPC 期间收到的
                # 新指令仍保留在累计值中。
                with self._cmd_lock:
                    if not self._pose_control:
                        self._accumulated_delta -= delta

            except Exception as exc:
                self.get_logger().error(
                    'OpenHUTB position '
                    'dispatch failed: '
                    f'{exc}'
                )

    # ================================================================
    # 指令看门狗
    # ================================================================
    def command_watchdog(self):
        if self._last_cmd_time is None:
            return

        if (
            time.monotonic()
            - self._last_cmd_time
            > self.command_timeout
        ):
            with self._cmd_lock:
                self._latest_cmd = None
                self._filtered_cmd[:] = 0.0

                self._accumulated_delta[
                    :
                ] = 0.0

            self._last_cmd_time = (
                None
            )

    # ================================================================
    # 清理资源
    # ================================================================
    def destroy_node(self):
        self._stop_worker.set()
        self._cmd_event.set()

        if getattr(
            self,
            '_control_worker',
            None,
        ) is not None:
            self._control_worker.join(
                timeout=1.0
            )

        if (
            not getattr(self, '_pose_control', False)
            and getattr(
            self,
            'enable_api_control',
            False,
            )
        ):
            try:
                self.control_client.enableApiControl(
                    False
                )
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(
        args=args
    )

    node = AirSimBridgeNode()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
