import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO
import os

class PrivacyMaskNode(Node):
    def __init__(self):
        super().__init__("privacy_mask_node")
        # 1. 声明参数，不再写死路径
        self.declare_parameter("image_path", "")
        self.img_path = self.get_parameter("image_path").get_parameter_value().string_value

        self.pub_raw = self.create_publisher(CompressedImage, "/image_raw/compressed", 10)
        self.pub_masked = self.create_publisher(CompressedImage, "/image_masked/compressed", 10)

        self.img_origin = None
        self.img_blur = None
        self.load_ok = False

        # 加载YOLO，加try捕获
        try:
            self.model = YOLO("yolov8n.pt")
        except Exception as e:
            self.get_logger().error(f"YOLO模型加载失败：{e}")
            return

        # 校验图片是否存在
        if not os.path.exists(self.img_path):
            self.get_logger().error(f"图片不存在：{self.img_path}")
            return

        # 读取图片 + 只推理一次！
        self.img_origin = cv2.imread(self.img_path)
        if self.img_origin is None:
            self.get_logger().error("图片读取失败")
            return

        self.infer_once()
        self.load_ok = True

        # 定时器：只负责发图，不推理
        self.timer = self.create_timer(0.1, self.timer_cb)

    def infer_once(self):
        results = self.model(self.img_origin)
        self.img_blur = self.img_origin.copy()
        for res in results:
            boxes = res.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                if cls_id == 0:  # person
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    # 保护：合法roi才模糊
                    if x2 > x1 and y2 > y1:
                        roi = self.img_blur[y1:y2, x1:x2]
                        blur_roi = cv2.GaussianBlur(roi, (21, 21), 0)
                        self.img_blur[y1:y2, x1:x2] = blur_roi

    def timer_cb(self):
        if not self.load_ok:
            return
        # 发布原图
        msg_raw = CompressedImage()
        msg_raw.header.stamp = self.get_clock().now().to_msg()
        msg_raw.header.frame_id = "camera"
        msg_raw.format = "jpeg"
        _, buf_raw = cv2.imencode(".jpg", self.img_origin)
        msg_raw.data = buf_raw.tobytes()
        self.pub_raw.publish(msg_raw)

        # 发布脱敏图
        msg_mask = CompressedImage()
        msg_mask.header.stamp = self.get_clock().now().to_msg()
        msg_mask.header.frame_id = "camera"
        msg_mask.format = "jpeg"
        _, buf_mask = cv2.imencode(".jpg", self.img_blur)
        msg_mask.data = buf_mask.tobytes()
        self.pub_masked.publish(msg_mask)

def main(args=None):
    rclpy.init(args=args)
    node = PrivacyMaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
