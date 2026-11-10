"""订阅相机图像并按固定间隔保存，避免无节流写盘占满磁盘。"""

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageRecordNode(Node):
    """相机图像录制节点。"""

    def __init__(self):
        super().__init__("image_record_node")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("image_save_dir", "./img_save")
        self.declare_parameter("min_interval", 1.0)  # 最短保存间隔（秒）
        self.declare_parameter("max_images", 100)    # 最多保存张数，0 表示不限

        topic = self.get_parameter("image_topic").value
        self.save_dir = self.get_parameter("image_save_dir").value
        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.last_save = 0.0
        self.count = 0
        self.sub = self.create_subscription(Image, topic, self.callback, 10)
        self.get_logger().info(
            f"订阅 {topic}，保存到 {os.path.abspath(self.save_dir)}，"
            f"间隔 {self.get_parameter('min_interval').value}s，"
            f"上限 {self.get_parameter('max_images').value} 张")

    def callback(self, msg):
        max_images = int(self.get_parameter("max_images").value)
        if max_images > 0 and self.count >= max_images:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_save < float(self.get_parameter("min_interval").value):
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            path = os.path.join(self.save_dir, f"img_{self.count:04d}.png")
            cv2.imwrite(path, cv_img)
        except Exception as exc:  # 图像编码失败不应导致节点退出
            self.get_logger().error(f"图像转换或保存失败：{exc}")
            return
        self.last_save = now
        self.count += 1
        self.get_logger().info(f"已保存第 {self.count} 张：{path}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageRecordNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
