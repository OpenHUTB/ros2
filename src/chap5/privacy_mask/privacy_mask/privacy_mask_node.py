#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from ultralytics import YOLO

class PrivacyMaskNode(Node):
    def __init__(self):
        super().__init__("privacy_mask_node")
        self.model = YOLO("yolov8n-seg.pt")
        self.img_path = "/home/user/test.jpg"
        self.img = cv2.imread(self.img_path)
        if self.img is None:
            self.get_logger().error("Image load failed!")
            return

        self.pub_raw = self.create_publisher(CompressedImage, "/image_raw/compressed", 10)
        self.pub_masked = self.create_publisher(CompressedImage, "/image_masked/compressed", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Privacy mask node started")

    def timer_callback(self):
        frame = self.img.copy()
        res = self.model(frame, conf=0.5)
        for r in res:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls[0])
                if cls == 0:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    roi = frame[y1:y2, x1:x2]
                    blur = cv2.GaussianBlur(roi, (25,25), 0)
                    frame[y1:y2, x1:x2] = blur

        _, buf_raw = cv2.imencode(".jpg", self.img)
        _, buf_mask = cv2.imencode(".jpg", frame)
        msg_raw = CompressedImage()
        msg_raw.format = "jpeg"
        msg_raw.data = buf_raw.tobytes()
        msg_mask = CompressedImage()
        msg_mask.format = "jpeg"
        msg_mask.data = buf_mask.tobytes()
        self.pub_raw.publish(msg_raw)
        self.pub_masked.publish(msg_mask)

def main(args=None):
    rclpy.init(args=args)
    node = PrivacyMaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
