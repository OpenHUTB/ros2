import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import cv2
import numpy as np

class FireDetectorNode(Node):
    def __init__(self):
        super().__init__('fire_detector_node')
        self.get_logger().info("火焰识别告警节点已启动，等待图像输入")
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )
        self.pub_img = self.create_publisher(
            Image,
            "/fire_detect/image_result",
            10
        )

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        # 两段HSV区间，修复审核指出的漏检问题
        lower_red1 = np.array([0, 120, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 80])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        fire_detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:
                fire_detected = True
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(cv_img, (x,y), (x+w,y+h), (0,0,255), 2)

        if fire_detected:
            self.get_logger().warn("【告警】检测到火焰目标！")

        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            self.pub_img.publish(out_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"发布图像失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，准备关闭节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
