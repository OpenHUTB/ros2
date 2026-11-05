#!/usr/bin/env python3
"""数据采集节点：为行为克隆制造 (图像, 专家动作) 样本对。

两种工作模式：
- waypoint（默认）：节点自己是一个简单的比例控制器，沿固定航点飞行，
  把「当前图像 + 当前下发的速度」成对写入数据集，即专家示教。
- follow：节点不发布指令，只旁听 /uav/cmd_vel，把外部遥控的指令当作标签。

输出：
    <data_dir>/frames/<run>_000123.png   原始分辨率 BGR 图像
    <data_dir>/controls.csv              image,vx,vy,x,y,z,stamp
"""

import csv
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

CSV_HEADER = ['image', 'vx', 'vy', 'x', 'y', 'z', 'stamp']


class CollectNode(Node):
    """采集 (图像, 速度) 样本。"""

    def __init__(self):
        super().__init__('uav_vision_collect')

        self.declare_parameter('data_dir', '~/.ros/uav_vision_nn/data')
        self.declare_parameter('sample_rate', 2.0)
        self.declare_parameter('mode', 'waypoint')
        self.declare_parameter('waypoints', '0,0;5,0;5,5;0,5;0,0')
        self.declare_parameter('altitude', -3.0)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('gain', 0.9)
        self.declare_parameter('reach_radius', 0.5)
        self.declare_parameter('control_rate', 10.0)
        # 航点走完后自动降落并退出，避免采集进程一直挂着（评审 m00035 提出）
        self.declare_parameter('auto_land', True)
        self.declare_parameter('exit_delay', 3.0)
        self.declare_parameter('land_service', '/uav/land')
        # 一条命令跑完采集：自动起飞，不用另外开一个终端调服务
        self.declare_parameter('auto_takeoff', True)
        self.declare_parameter('takeoff_service', '/uav/takeoff')

        self._data_dir = os.path.expanduser(
            str(self.get_parameter('data_dir').value))
        sample_rate = max(float(self.get_parameter('sample_rate').value), 0.1)
        self._sample_interval = 1.0 / sample_rate
        self._mode = str(self.get_parameter('mode').value)
        self._waypoints = self._parse_waypoints(
            str(self.get_parameter('waypoints').value))
        self._altitude = float(self.get_parameter('altitude').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._gain = float(self.get_parameter('gain').value)
        self._reach_radius = float(self.get_parameter('reach_radius').value)
        control_rate = max(float(self.get_parameter('control_rate').value), 1.0)
        self._auto_land = bool(self.get_parameter('auto_land').value)
        self._exit_delay = max(float(self.get_parameter('exit_delay').value), 0.0)
        self._land_service = str(self.get_parameter('land_service').value)
        self._auto_takeoff = bool(self.get_parameter('auto_takeoff').value)
        self._takeoff_service = str(
            self.get_parameter('takeoff_service').value)

        if self._mode not in ('waypoint', 'follow'):
            self.get_logger().warn(f'未知 mode={self._mode}，回退为 waypoint')
            self._mode = 'waypoint'

        self._frames_dir = os.path.join(self._data_dir, 'frames')
        os.makedirs(self._frames_dir, exist_ok=True)
        self._csv_path = os.path.join(self._data_dir, 'controls.csv')
        new_file = (not os.path.isfile(self._csv_path)
                    or os.path.getsize(self._csv_path) == 0)
        self._csv_file = open(self._csv_path, 'a', newline='', encoding='utf-8')
        self._writer = csv.writer(self._csv_file)
        if new_file:
            self._writer.writerow(CSV_HEADER)

        self._run_tag = time.strftime('%Y%m%d_%H%M%S')
        self._frame_index = 0
        self._last_sample = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._pose = None
        self._airborne = False
        self._waypoint_index = 0
        self._finished = False
        self._exit_timer = None
        self._land_client = None
        if self._auto_land:
            self._land_client = self.create_client(Trigger, self._land_service)
        self._takeoff_client = None
        self._takeoff_requested = False
        self._start_time = time.monotonic()
        if self._auto_takeoff and self._mode == 'waypoint':
            self._takeoff_client = self.create_client(
                Trigger, self._takeoff_service)
            self.create_timer(1.0, self._maybe_takeoff)

        self.create_subscription(Image, '~/image', self._on_image, 1)
        self.create_subscription(PoseStamped, '~/pose', self._on_pose, 10)

        if self._mode == 'waypoint':
            self._pub_cmd = self.create_publisher(Twist, '~/cmd_vel', 10)
            self.create_timer(1.0 / control_rate, self._on_control)
        else:
            self.create_subscription(Twist, '~/cmd_vel', self._on_external_cmd, 10)
            self._pub_cmd = None

        self.get_logger().info(
            f'采集节点就绪 | 模式 {self._mode} | 保存到 {self._data_dir} '
            f'| 采样 {sample_rate:.1f} Hz | 航点 {len(self._waypoints)} 个')
        self.get_logger().info('提示：先调用 /uav/takeoff 起飞，爬升到 1 m 以上才开始记录')

    @staticmethod
    def _parse_waypoints(text):
        points = []
        for chunk in text.split(';'):
            chunk = chunk.strip()
            if not chunk:
                continue
            parts = [p for p in chunk.replace('，', ',').split(',') if p.strip()]
            if len(parts) < 2:
                continue
            points.append((float(parts[0]), float(parts[1])))
        return points

    # ------------------------------------------------------------------ 回调
    def _on_pose(self, msg):
        self._pose = msg.pose.position
        if self._pose.z < -1.0 and not self._airborne:
            self._airborne = True
            self.get_logger().info(f'检测到已起飞，当前高度 {-self._pose.z:.2f} m')

    def _on_external_cmd(self, msg):
        """follow 模式：把外部遥控指令当作专家动作。"""
        self._vx = float(msg.linear.x)
        self._vy = float(msg.linear.y)

    def _on_control(self):
        """waypoint 模式：比例控制器沿航点飞行，同时维持巡航高度。"""
        if not self._airborne or self._pose is None:
            return

        vz = max(-0.5, min(0.5, 0.5 * (self._altitude - self._pose.z)))

        if self._waypoint_index >= len(self._waypoints):
            if not self._finished:
                self._finished = True
                self._vx = self._vy = 0.0
                self.get_logger().info(
                    f'航点全部到达，本次共采集 {self._frame_index} 帧')
                self._begin_exit()
            self._publish(0.0, 0.0, vz)
            return

        target_x, target_y = self._waypoints[self._waypoint_index]
        error_x = target_x - self._pose.x
        error_y = target_y - self._pose.y
        distance = math.hypot(error_x, error_y)

        if distance < self._reach_radius:
            self.get_logger().info(
                f'到达航点 {self._waypoint_index}: ({target_x:.2f}, {target_y:.2f})')
            self._waypoint_index += 1
            return

        vx = max(-self._max_speed, min(self._max_speed, self._gain * error_x))
        vy = max(-self._max_speed, min(self._max_speed, self._gain * error_y))
        self._vx = vx
        self._vy = vy
        self._publish(vx, vy, vz)

    def _on_image(self, msg):
        """限定采样频率后把图像和当前指令成对入库。"""
        if self._finished or not self._airborne:
            return
        now = time.monotonic()
        if now - self._last_sample < self._sample_interval:
            return
        self._last_sample = now

        try:
            frame = self._decode(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        name = f'{self._run_tag}_{self._frame_index:06d}.png'
        path = os.path.join(self._frames_dir, name)
        if not cv2.imwrite(path, frame):
            self.get_logger().warn(f'写图像失败: {path}', throttle_duration_sec=5.0)
            return

        position = self._pose
        self._writer.writerow([
            os.path.join('frames', name),
            f'{self._vx:.4f}',
            f'{self._vy:.4f}',
            f'{position.x:.4f}' if position else '',
            f'{position.y:.4f}' if position else '',
            f'{position.z:.4f}' if position else '',
            f'{time.time():.3f}',
        ])
        self._frame_index += 1
        # 2 Hz 采样，逐行 flush 的开销可以忽略；这样即使进程被强杀，
        # 已经采集的样本也不会丢在缓冲区里。
        self._csv_file.flush()
        if self._frame_index % 50 == 0:
            self.get_logger().info(f'已采集 {self._frame_index} 帧')

    @staticmethod
    def _decode(msg):
        if msg.encoding == 'mono8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        if msg.encoding in ('bgr8', 'rgb8'):
            array = np.frombuffer(msg.data, dtype=np.uint8)
            array = array.reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                array = np.ascontiguousarray(array[:, :, ::-1])
            return array
        raise ValueError(f'不支持的图像编码: {msg.encoding}')

    def _publish(self, vx, vy, vz=0.0):
        if self._pub_cmd is None:
            return
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)
        self._pub_cmd.publish(cmd)

    def _maybe_takeoff(self):
        """启动后自动请求起飞，升到 1 m 以上开始记录。"""
        if self._airborne or self._takeoff_client is None:
            return
        elapsed = time.monotonic() - self._start_time
        if self._takeoff_requested:
            if elapsed > 15.0:
                self.get_logger().warn(
                    '已请求起飞但 15 s 内未检测到爬升，请检查仿真器或手动调用 '
                    f'{self._takeoff_service}')
                self._takeoff_requested = False
                self._start_time = time.monotonic()
            return
        if elapsed > 20.0:
            return
        if self._takeoff_client.service_is_ready():
            self._takeoff_client.call_async(Trigger.Request())
            self._takeoff_requested = True
            self.get_logger().info(f'自动起飞：已请求 {self._takeoff_service}')

    def _begin_exit(self):
        """航点走完：请求降落，等 exit_delay 秒后让采集节点自己退出。"""
        if self._land_client is not None:
            if self._land_client.service_is_ready():
                self._land_client.call_async(Trigger.Request())
                self.get_logger().info(f'已请求降落：{self._land_service}')
            else:
                self.get_logger().warn(
                    f'降落服务 {self._land_service} 不可用，飞机保持悬停')
        self._exit_timer = self.create_timer(self._exit_delay, self._on_exit_timeout)

    def _on_exit_timeout(self):
        """定时器回调：收尾并关闭 rclpy，spin() 返回后 main 负责清理。"""
        if self._exit_timer is not None:
            self.destroy_timer(self._exit_timer)
            self._exit_timer = None
        self._publish(0.0, 0.0, 0.0)
        self.get_logger().info(
            '采集完成，collect 节点退出（airsim_source 仍在运行，Ctrl-C 结束）')
        rclpy.shutdown()

    def shutdown(self):
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
        self.get_logger().info(
            f'采集结束，共 {self._frame_index} 帧，标签文件 {self._csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CollectNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'[uav_vision_nn] 采集节点启动失败: {exc}')
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
