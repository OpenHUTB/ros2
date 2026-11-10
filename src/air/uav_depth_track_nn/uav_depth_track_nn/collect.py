#!/usr/bin/env python3
"""数据采集节点：让"专家"沿给定圆轨迹飞行，同时记录 (深度图, 参考偏差, 速度标签)。

任务 2 要求「用传感器数据做感知 + 给定轨迹的控制」。这里的做法是行为克隆：

    专家 = 前视点纯比例控制器（知道自己的真实位姿，属于"作弊"的教师）
    学生 = 深度图 + 参考偏差 -> 神经网络 -> (vx, vy)

采集完成后 ``train`` 用这份数据训练学生，``main`` 里学生只看深度图和参考偏差。

输出目录（默认 ``~/.ros/uav_depth_track_nn/data``）：

    frames/<run>_000123.png    64x64 的 16 位深度图（单位毫米）
    controls.csv               image,vx,vy,dx,dy,sin_err,cos_err,x,y,z,stamp
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

from uav_depth_track_nn.model import (DEFAULT_IMAGE_SIZE, reference_point,
                                      track_features)

CSV_HEADER = ['image', 'vx', 'vy', 'dx', 'dy', 'sin_err', 'cos_err',
              'x', 'y', 'z', 'stamp']


def _parse_pair(text):
    parts = [p for p in str(text).replace('，', ',').split(',') if p.strip()]
    if len(parts) < 2:
        return (0.0, 0.0)
    return (float(parts[0]), float(parts[1]))


class CollectNode(Node):
    """沿圆轨迹采集 (深度图, 参考偏差) -> 速度 的样本。"""

    def __init__(self):
        super().__init__('uav_depth_collect')

        self.declare_parameter('data_dir', '~/.ros/uav_depth_track_nn/data')
        self.declare_parameter('sample_rate', 5.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('center', '0,0')
        self.declare_parameter('radius', 3.0)
        self.declare_parameter('height', 2.5)
        self.declare_parameter('lookahead_angle', 0.55)
        self.declare_parameter('gain', 1.4)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('laps', 2.0)
        self.declare_parameter('image_size', DEFAULT_IMAGE_SIZE)
        self.declare_parameter('auto_takeoff', True)
        self.declare_parameter('auto_land', True)
        self.declare_parameter('exit_delay', 3.0)

        self._data_dir = os.path.expanduser(
            str(self.get_parameter('data_dir').value))
        self._sample_dt = 1.0 / max(
            float(self.get_parameter('sample_rate').value), 0.2)
        control_rate = max(float(self.get_parameter('control_rate').value), 1.0)
        self._center = _parse_pair(self.get_parameter('center').value)
        self._radius = float(self.get_parameter('radius').value)
        self._height = abs(float(self.get_parameter('height').value))
        self._lookahead = float(self.get_parameter('lookahead_angle').value)
        self._gain = float(self.get_parameter('gain').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._laps = max(float(self.get_parameter('laps').value), 0.2)
        self._image_size = int(self.get_parameter('image_size').value)
        self._auto_takeoff = bool(self.get_parameter('auto_takeoff').value)
        self._auto_land = bool(self.get_parameter('auto_land').value)
        self._exit_delay = max(float(self.get_parameter('exit_delay').value), 0.0)

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
        self._depth = None
        self._pose = None
        self._yaw = 0.0
        self._airborne = False
        self._start_time = None
        self._last_theta = None
        self._travel = 0.0
        self._features = np.zeros(4, dtype=np.float32)
        self._vx = 0.0
        self._vy = 0.0
        self._finished = False
        self._exit_timer = None
        self._last_sample = 0.0

        self._takeoff_client = (
            self.create_client(Trigger, '/uav/takeoff')
            if self._auto_takeoff else None)
        self._land_client = (
            self.create_client(Trigger, '/uav/land')
            if self._auto_land else None)
        self._takeoff_requested = False

        self.create_subscription(Image, '/uav/depth', self._on_depth, 1)
        self.create_subscription(PoseStamped, '/uav/pose', self._on_pose, 10)
        self._pub_cmd = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.create_timer(1.0 / control_rate, self._on_control)
        self.create_timer(0.05, self._maybe_sample)
        if self._auto_takeoff:
            self.create_timer(1.0, self._maybe_takeoff)

        self.get_logger().info(
            f'深度跟踪采集就绪 | 保存到 {self._data_dir} | 采样 {1.0 / self._sample_dt:.1f} Hz '
            f'| 圆轨迹 中心{self._center} 半径{self._radius:.1f} m 高度{self._height:.1f} m '
            f'| {self._laps:.1f} 圈')

    # ------------------------------------------------------------------ 回调
    def _on_pose(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        self._pose = p
        # 四元数 -> 偏航（绕 z 轴，NED 下顺时针为正）
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)
        if p.z < -0.6 and not self._airborne:
            self._airborne = True
            self._start_time = time.monotonic()
            self.get_logger().info(f'检测到已起飞（高度 {-p.z:.2f} m），开始沿圆轨迹采集')

    def _on_depth(self, msg):
        if msg.encoding != '32FC1':
            self.get_logger().warn(
                f'深度图编码应为 32FC1，实际 {msg.encoding}',
                throttle_duration_sec=10.0)
            return
        self._depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width)

    def _phase(self):
        """按飞机在圆上的当前角度累积相位。

        原先用 ``omega * (now - start)`` 按时间推进，飞得慢一点前视点就跑到
        前面十几米，专家会直接切弦穿过圆，采出来的数据也不是真正的圆周运动。
        改成用飞机自身的极角增量推进相位后，前视点始终只领先一个 ``lookahead``
        角度，轨迹才是稳定的圆。
        """
        if self._pose is None:
            return 0.0
        dx = self._pose.x - self._center[0]
        dy = self._pose.y - self._center[1]
        theta = math.atan2(dy, dx)
        if self._last_theta is not None:
            # 单独累计走过的角度用于"飞满 N 圈"判定；参考点仍用瞬时极角，
            # 这样即使角度在 ±pi 处回绕也不会让前视点突然跳到对面。
            delta = math.atan2(math.sin(theta - self._last_theta),
                               math.cos(theta - self._last_theta))
            if math.hypot(dx, dy) > 0.6:
                self._travel += abs(delta)
        self._last_theta = theta
        return theta

    def _on_control(self):
        """专家控制律：追圆轨迹上的前视点，并维持巡航高度。"""
        if self._pose is None or not self._airborne:
            return

        phase = self._phase()
        # _phase() 返回的是瞬时极角（范围 (-pi, pi]），不能拿来判圈数；
        # 圈数要用绕圆心累计转过的角度 self._travel。
        if self._travel >= 2.0 * math.pi * self._laps and not self._finished:
            self._finished = True
            self.get_logger().info(
                f'飞满 {self._laps:.1f} 圈，共采集 {self._frame_index} 帧')
            self._begin_exit()

        target = reference_point(self._center, self._radius, self._height,
                                 phase + self._lookahead)
        self._features = track_features(
            (self._pose.x, self._pose.y), self._yaw, target[:2])

        if self._finished:
            self._vx = self._vy = 0.0
        else:
            # 特征里的 dx/dy 已经是"机体前向/右向偏移（除以 5 m）"，乘回去再乘增益
            vx = self._gain * float(self._features[0]) * 5.0
            vy = self._gain * float(self._features[1]) * 5.0
            speed = math.hypot(vx, vy)
            if speed > self._max_speed:
                vx *= self._max_speed / speed
                vy *= self._max_speed / speed
            self._vx, self._vy = vx, vy

        vz = max(-0.6, min(0.6, 1.2 * (-self._height - self._pose.z)))
        self._publish(self._vx, self._vy, vz)

    def _maybe_sample(self):
        """按采样周期把当前深度图和当前标签写盘。"""
        if self._finished or not self._airborne or self._depth is None:
            return
        now = time.monotonic()
        if now - self._last_sample < self._sample_dt:
            return
        self._last_sample = now

        depth = cv2.resize(self._depth, (self._image_size, self._image_size),
                           interpolation=cv2.INTER_AREA)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        # 存 16 位毫米图：比 float32 npy 省一半以上空间，又不像 8 位 PNG 那样丢精度
        millimeters = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
        name = f'{self._run_tag}_{self._frame_index:06d}.png'
        if not cv2.imwrite(os.path.join(self._frames_dir, name), millimeters):
            self.get_logger().warn('写深度图失败', throttle_duration_sec=5.0)
            return

        position = self._pose
        self._writer.writerow([
            os.path.join('frames', name),
            f'{self._vx:.4f}', f'{self._vy:.4f}',
            f'{float(self._features[0]):.5f}', f'{float(self._features[1]):.5f}',
            f'{float(self._features[2]):.5f}', f'{float(self._features[3]):.5f}',
            f'{position.x:.4f}', f'{position.y:.4f}', f'{position.z:.4f}',
            f'{time.time():.3f}',
        ])
        self._frame_index += 1
        self._csv_file.flush()  # 进程被强杀也不丢已采样本
        if self._frame_index % 50 == 0:
            self.get_logger().info(f'已采集 {self._frame_index} 帧')

    def _publish(self, vx, vy, vz=0.0):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)
        self._pub_cmd.publish(cmd)

    def _maybe_takeoff(self):
        # 即使遥测显示已经在空中，也要请求一次起飞：桥节点只有在完成自己的
        # 起飞流程后才会打开 cmd_vel 通道，否则飞机停在空中却收不到速度指令。
        if self._takeoff_client is None:
            return
        if self._takeoff_requested:
            return
        if self._takeoff_client.service_is_ready():
            self._takeoff_client.call_async(Trigger.Request())
            self._takeoff_requested = True
            self.get_logger().info('自动起飞：已请求 /uav/takeoff')

    def _begin_exit(self):
        if self._land_client is not None and self._land_client.service_is_ready():
            self._land_client.call_async(Trigger.Request())
            self.get_logger().info('已请求降落：/uav/land')
        self._exit_timer = self.create_timer(self._exit_delay, self._on_exit_timeout)

    def _on_exit_timeout(self):
        if self._exit_timer is not None:
            self.destroy_timer(self._exit_timer)
            self._exit_timer = None
        self._publish(0.0, 0.0, 0.0)
        self.get_logger().info(
            f'采集完成：{self._frame_index} 帧，标签文件 {self._csv_path}')
        rclpy.shutdown()

    def shutdown(self):
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CollectNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - 启动失败要打印出来而不是静默退出
        print(f'[uav_depth_track_nn] 采集节点启动失败: {exc}')
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
