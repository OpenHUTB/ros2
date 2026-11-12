"""采集 Gazebo 中的相机（模拟声呐）图像，作为训练数据集的一类样本。

用法（每个类别采集一次，改变 ROV 位姿/光照多采几轮）：
    ros2 run water capture_dataset --out dataset/rock --interval 0.5 --max 120
    ros2 run water capture_dataset --out dataset/pipe --interval 0.5 --max 120
    ros2 run water capture_dataset --out dataset/sand --interval 0.5 --max 120

保存的图片按 ImageFolder 约定放在 --out 目录下，目录名即类别名，
可直接配合 `ros2 run water train_sonar --data dataset` 训练。
"""

import argparse
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DatasetCaptureNode(Node):

    def __init__(self, out_dir, topic, interval, max_images):
        super().__init__('capture_dataset')
        self.out_dir = out_dir
        self.interval = interval
        self.max_images = max_images
        self.bridge = CvBridge()
        self.count = 0
        self.last_stamp = -1.0
        os.makedirs(out_dir, exist_ok=True)
        self.create_subscription(Image, topic, self.image_callback, 10)
        self.get_logger().info(
            f'订阅 {topic}，按 {interval}s 间隔保存到 {os.path.abspath(out_dir)}，'
            f'上限 {max_images} 张（0 表示不限）')

    def image_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_stamp >= 0 and now - self.last_stamp < self.interval:
            return
        if self.max_images > 0 and self.count >= self.max_images:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        label = os.path.basename(os.path.normpath(self.out_dir))
        path = os.path.join(self.out_dir, f'{label}_{self.count:04d}.png')
        if not cv2.imwrite(path, img):
            self.get_logger().error(f'写入失败：{path}')
            return
        self.count += 1
        self.last_stamp = now
        self.get_logger().info(f'已保存 {path}（第 {self.count} 张）')
        if self.max_images > 0 and self.count >= self.max_images:
            self.get_logger().info('已达上限，可 Ctrl+C 退出')


def main(args=None):
    parser = argparse.ArgumentParser(description='采集声呐/相机图像作为训练样本')
    parser.add_argument('--out', required=True, help='保存目录，目录名作为类别名')
    parser.add_argument('--topic', default='/sonar/camera/image_raw')
    parser.add_argument('--interval', type=float, default=0.5, help='保存间隔（秒）')
    parser.add_argument('--max', type=int, default=100, help='最多保存张数，0 表示不限')
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = None
    try:
        node = DatasetCaptureNode(parsed.out, parsed.topic, parsed.interval, parsed.max)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
