#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class AuvVisionControlNode(Node):
    def __init__(self):
        super().__init__("auv_vision_control_node")
        self.bridge = CvBridge()
        # 发布速度指令话题，匹配Gazebo插件
        self.cmd_vel_pub = self.create_publisher(Twist, "/auv/cmd_vel", 10)
        # 订阅相机图像话题
        self.image_sub = self.create_subscription(
            Image,
            "/auv/camera/image_raw",
            self.image_callback,
            10
        )
        self.get_logger().info("ROS2 AUV视觉控制节点启动成功（简易版）")

    def image_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h,w = cv_img.shape[:2]

        # 简单视觉规则：检测画面中心障碍物，做避障逻辑
        roi = cv_img[:, int(w*0.3):int(w*0.7)]
        gray = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
        avg_bright = np.mean(gray)

        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        # 画面偏暗（障碍物），转向规避
        if avg_bright < 70:
            twist_msg.angular.z = 0.2
        else:
            twist_msg.angular.z = 0.0

        self.cmd_vel_pub.publish(twist_msg)
        cv2.imshow("AUV Camera View", cv_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AuvVisionControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
