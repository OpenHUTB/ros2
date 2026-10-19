"""
水下机器人端到端视觉控制 ROS 2 节点 (End-to-End Vision Control Node)
====================================================================
《机器人操作系统及应用》课程任务 4 核心 ROS 2 节点。
功能：
  1. 订阅前视相机画面 /rov/camera/image_raw (sensor_msgs/Image)
  2. 利用 PyTorch GPU/CPU 深度神经网络进行纯端到端前向推理
  3. 直接输出推力速度指令 /cmd_vel (geometry_msgs/Twist)
  4. 支持一键切换不同网络模型架构与不同强化学习策略算法
"""

import os
import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, Float32MultiArray

try:
    from rov_mujoco.end_to_end_networks import VisuomotorController
except ImportError:
    try:
        from underwater_rov_mujoco.end_to_end_networks import VisuomotorController
    except ImportError:
        from end_to_end_networks import VisuomotorController


class EndToEndVisionControlNode(Node):
    """端到端水下视觉伺服控制节点"""

    def __init__(self):
        super().__init__('end_to_end_vision_control_node')

        # 声明可配置 ROS 2 参数
        self.declare_parameter('architecture', 'spatial_softmax')  # nature_cnn / resnet / spatial_softmax
        self.declare_parameter('algorithm', 'sac')                 # sac / ppo / bc
        self.declare_parameter('nominal_speed', 0.45)
        self.declare_parameter('weights_path', '')

        self.arch = self.get_parameter('architecture').value
        self.algo = self.get_parameter('algorithm').value
        self.nominal_speed = float(self.get_parameter('nominal_speed').value)
        weights_path = self.get_parameter('weights_path').value

        if not weights_path:
            # 自动定位默认权重文件
            candidate_paths = [
                os.path.join(os.getcwd(), "models", f"task4_{self.arch}_{self.algo}.pt"),
                os.path.join(os.path.dirname(__file__), "..", "..", "models", f"task4_{self.arch}_{self.algo}.pt"),
                os.path.join(os.path.dirname(__file__), "models", f"task4_{self.arch}_{self.algo}.pt"),
            ]
            try:
                from ament_index_python.packages import get_package_share_directory
                for pkg in ['rov_mujoco', 'underwater_rov_mujoco']:
                    try:
                        candidate_paths.append(os.path.join(get_package_share_directory(pkg), "models", f"task4_{self.arch}_{self.algo}.pt"))
                    except Exception:
                        pass
            except ImportError:
                pass

            for cp in candidate_paths:
                if os.path.exists(cp):
                    weights_path = cp
                    break

        # 初始化端到端神经网络控制器
        self.controller = VisuomotorController(
            architecture=self.arch,
            algorithm=self.algo,
            weights_path=weights_path if os.path.exists(weights_path) else None,
            nominal_speed=self.nominal_speed
        )

        # 订阅水下相机图像
        self.sub_image = self.create_subscription(
            Image,
            '/rov/camera/image_raw',
            self._image_callback,
            1
        )

        # 控制指令发布
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_diagnostics = self.create_publisher(Float32MultiArray, '/rov/vision_control/debug', 10)

        # 统计与性能评价
        self.inference_count = 0
        self.total_inference_time_ms = 0.0
        self.last_log_time = time.time()
        self.latest_action = [0.0, 0.0, 0.0, 0.0]

        device_str = str(self.controller.device).upper()
        self.get_logger().info(
            f"端到端视觉控制节点已就绪 | 架构: {self.arch.upper()} | 算法: {self.algo.upper()} | 推理引擎: {device_str}"
        )

    def _decode_image_msg(self, msg: Image) -> np.ndarray:
        """纯 NumPy 零依赖高效解析 ROS 2 Image 消息为 (H, W, 3) 图像"""
        h, w = msg.height, msg.width
        raw_data = np.frombuffer(msg.data, dtype=np.uint8)

        if msg.encoding in ['rgb8', 'bgr8']:
            img = raw_data.reshape((h, w, 3))
            if msg.encoding == 'bgr8':
                img = img[:, :, ::-1]  # 转为 RGB
            return img
        elif msg.encoding == 'mono8':
            gray = raw_data.reshape((h, w))
            return np.repeat(gray[:, :, np.newaxis], 3, axis=2)
        else:
            # 通用回退
            try:
                return raw_data.reshape((h, w, -1))[:, :, :3]
            except Exception:
                return np.zeros((h, w, 3), dtype=np.uint8)

    def _image_callback(self, msg: Image):
        """图像接收与端到端控制高频闭环回调"""
        t0 = time.perf_counter()

        rgb_img = self._decode_image_msg(msg)
        cmd_dict = self.controller.predict_action(rgb_img)

        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.total_inference_time_ms += dt_ms
        self.inference_count += 1

        # 发布 /cmd_vel 控制指令
        twist = Twist()
        twist.linear.x = cmd_dict['vx']
        twist.linear.y = cmd_dict['vy']
        twist.linear.z = cmd_dict['vz']
        twist.angular.z = cmd_dict['wz']
        self.pub_cmd_vel.publish(twist)

        self.latest_action = [cmd_dict['vx'], cmd_dict['vy'], cmd_dict['vz'], cmd_dict['wz']]

        # 周期性输出运行监控 (1Hz)
        now = time.time()
        if now - self.last_log_time >= 1.0:
            avg_ms = self.total_inference_time_ms / max(1, self.inference_count)
            self.get_logger().info(
                f"[视觉控制] 推理耗时: {avg_ms:4.2f}ms | 推进: vx={cmd_dict['vx']:.2f}m/s, "
                f"横移: vy={cmd_dict['vy']:.2f}m/s, 转向: wz={cmd_dict['wz']:.2f}rad/s"
            )
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = EndToEndVisionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
