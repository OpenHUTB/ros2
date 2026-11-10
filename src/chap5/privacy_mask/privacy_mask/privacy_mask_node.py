"""行人隐私脱敏视觉节点。

读取一张静态测试图片，用 YOLOv8n 检测行人（COCO 类别 id = 0），
对行人包围盒区域做高斯模糊，并把原图与脱敏图分别以 CompressedImage 话题发布。
"""

import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

PERSON_CLASS_ID = 0  # COCO 数据集中 person 的类别 id

class PrivacyMaskNode(Node):
    """发布原始图像与行人模糊图像。"""

    def __init__(self):
        super().__init__("privacy_mask_node")

        pkg_dir = get_package_share_directory("privacy_mask")
        # 参数默认值直接指向安装后的 share 目录，launch 文件也可以覆盖
        self.declare_parameter(
            "image_path", os.path.join(pkg_dir, "resource", "test.png"))
        self.declare_parameter(
            "model_path", os.path.join(pkg_dir, "resource", "yolov8n.pt"))
        self.declare_parameter("blur_kernel", 21)      # 高斯核大小，必须为奇数
        self.declare_parameter("publish_period", 0.1)  # 发布周期，单位秒

        self.img_path = self.get_parameter("image_path").value
        self.model_path = self.get_parameter("model_path").value
        self.blur_kernel = int(self.get_parameter("blur_kernel").value)
        self.person_count = 0

        self.pub_raw = self.create_publisher(
            CompressedImage, "/image_raw/compressed", 10)
        self.pub_masked = self.create_publisher(
            CompressedImage, "/image_masked/compressed", 10)

        self.img_origin = self.load_image(self.img_path)
        self.model = self.load_model(self.model_path)
        self.img_masked = self.mask_pedestrians(self.img_origin)

        period = float(self.get_parameter("publish_period").value)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info(
            f"图片：{self.img_path}，模型：{self.model_path}，"
            f"行人目标数：{self.person_count}，发布周期：{period}s")

    def load_image(self, path):
        """读取测试图片，失败时直接抛异常，避免节点空转。"""
        img = cv2.imread(path)
        if img is None:
            raise RuntimeError(
                f"无法读取测试图片：{path}。请确认 resource/test.png 存在，"
                "并重新执行 colcon build 安装到 share 目录。")
        return img

    def load_model(self, path):
        """优先加载功能包内的本地模型；缺失时才回退到联网自动下载。"""
        if os.path.exists(path):
            return YOLO(path)
        self.get_logger().warning(
            f"未找到本地模型 {path}，回退为自动下载 yolov8n.pt（需要联网）")
        return YOLO("yolov8n.pt")

    def mask_pedestrians(self, img):
        """检测行人并对包围盒做高斯模糊，返回脱敏后的图像。"""
        masked = img.copy()
        height, width = masked.shape[:2]
        results = self.model(img, verbose=False)

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                if int(box.cls[0]) != PERSON_CLASS_ID:
                    continue
                # 关键修复：把坐标裁剪到图像范围内，否则越界坐标会产生错误 ROI
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(width, x2), min(height, y2)
                if x2 - x1 < 2 or y2 - y1 < 2:
                    continue
                roi = masked[y1:y2, x1:x2]
                masked[y1:y2, x1:x2] = cv2.GaussianBlur(
                    roi, (self.blur_kernel, self.blur_kernel), 0)
                self.person_count += 1
        return masked

    def to_msg(self, img):
        """把 OpenCV 图像编码为 sensor_msgs/CompressedImage。"""
        ok, buffer = cv2.imencode(".jpg", img)
        if not ok:
            raise RuntimeError("JPEG 编码失败，请检查图像是否为空")
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = "jpeg"
        msg.data = buffer.tobytes()
        return msg

    def timer_cb(self):
        self.pub_raw.publish(self.to_msg(self.img_origin))
        self.pub_masked.publish(self.to_msg(self.img_masked))

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PrivacyMaskNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        rclpy.logging.get_logger("privacy_mask_node").error(str(exc))
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
