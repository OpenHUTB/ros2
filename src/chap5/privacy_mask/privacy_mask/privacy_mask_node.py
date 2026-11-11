"""Pedestrian privacy masking node.

Loads one static test image, detects pedestrians with YOLOv8n
(COCO class id 0), blurs every bounding box and publishes the raw image and
the masked image as CompressedImage topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

PERSON_CLASS_ID = 0  # COCO 数据集中 person 的类别 id


class PrivacyMaskNode(Node):
    """Publish the raw image and the blurred image."""

    def __init__(self):
        super().__init__('privacy_mask_node')

        pkg_dir = get_package_share_directory('privacy_mask')
        self.declare_parameter(
            'image_path', os.path.join(pkg_dir, 'resource', 'test.png'))
        self.declare_parameter(
            'model_path', os.path.join(pkg_dir, 'resource', 'yolov8n.pt'))
        # 0 表示按包围盒尺寸自动选择核大小；显式给值时必须是不小于 3 的奇数
        self.declare_parameter('blur_kernel', 0)
        self.declare_parameter('publish_period', 0.1)  # 单位秒

        self.img_path = self.get_parameter('image_path').value
        self.model_path = self.get_parameter('model_path').value
        self.blur_kernel = int(self.get_parameter('blur_kernel').value)
        if self.blur_kernel != 0 and (
                self.blur_kernel < 3 or self.blur_kernel % 2 == 0):
            raise RuntimeError(
                f'blur_kernel 需为 0（自动）或不小于 3 的奇数，'
                f'当前为 {self.blur_kernel}')
        self.person_count = 0

        self.pub_raw = self.create_publisher(
            CompressedImage, '/image_raw/compressed', 10)
        self.pub_masked = self.create_publisher(
            CompressedImage, '/image_masked/compressed', 10)

        self.img_origin = self.load_image(self.img_path)
        self.model = self.load_model(self.model_path)
        self.img_masked = self.mask_pedestrians(self.img_origin)

        # 静态图只编码一次，定时器里仅更新时间戳，避免每帧重复 JPEG 编码
        self.raw_bytes = self.encode(self.img_origin)
        self.masked_bytes = self.encode(self.img_masked)

        period = float(self.get_parameter('publish_period').value)
        if period <= 0.0:
            raise RuntimeError(f'publish_period 必须大于 0，当前为 {period}')
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info(
            f'图片：{self.img_path}，模型：{self.model_path}，'
            f'行人目标数：{self.person_count}，发布周期：{period}s')

    def load_image(self, path):
        """Read the test image and fail fast when it is missing."""
        img = cv2.imread(path)
        if img is None:
            raise RuntimeError(
                f'无法读取测试图片：{path}。请确认 resource/test.png 存在，'
                '并重新执行 colcon build 安装到 share 目录。')
        return img

    def load_model(self, path):
        """Load the packaged model, fall back to online download if absent."""
        if os.path.exists(path):
            return YOLO(path)
        self.get_logger().warning(
            f'未找到本地模型 {path}，回退为自动下载 yolov8n.pt（需要联网）')
        return YOLO('yolov8n.pt')

    def kernel_size(self, roi_width, roi_height):
        """Return the Gaussian kernel size for one bounding box."""
        if self.blur_kernel > 0:
            return self.blur_kernel
        # 取短边的 40%，保证人脸等细节无法辨认；上限 199 控制单帧耗时
        size = min(int(min(roi_width, roi_height) * 0.4), 199)
        size = max(size, 21)
        return size + 1 if size % 2 == 0 else size

    def mask_pedestrians(self, img):
        """Blur every pedestrian box and return the masked image."""
        masked = img.copy()
        height, width = masked.shape[:2]
        results = self.model(img, verbose=False)

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                if int(box.cls[0]) != PERSON_CLASS_ID:
                    continue
                # 坐标裁剪到图像范围内，避免越界坐标取到错误的 ROI
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(width, x2), min(height, y2)
                if x2 - x1 < 2 or y2 - y1 < 2:
                    continue
                kernel = self.kernel_size(x2 - x1, y2 - y1)
                roi = masked[y1:y2, x1:x2]
                masked[y1:y2, x1:x2] = cv2.GaussianBlur(
                    roi, (kernel, kernel), 0)
                self.person_count += 1
        return masked

    def encode(self, img):
        """Encode an OpenCV image into JPEG bytes."""
        ok, buffer = cv2.imencode('.jpg', img)
        if not ok:
            raise RuntimeError('JPEG 编码失败，请检查图像是否为空')
        return buffer.tobytes()

    def to_msg(self, data):
        """Build a compressed image message with the current timestamp."""
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = data
        return msg

    def timer_cb(self):
        self.pub_raw.publish(self.to_msg(self.raw_bytes))
        self.pub_masked.publish(self.to_msg(self.masked_bytes))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PrivacyMaskNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, cv2.error) as exc:
        rclpy.logging.get_logger('privacy_mask_node').error(str(exc))
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
