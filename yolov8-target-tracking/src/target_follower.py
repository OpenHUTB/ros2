#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class TargetFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback, queue_size=1)
        self.target_class_id = 0  # 0 代表 COCO 数据集里的 'person'
        rospy.loginfo("目标跟随节点启动，等待目标...")

    def callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        height, width = cv_img.shape[:2]
        center_x = width // 2
        center_y = height // 2

        results = self.model(cv_img)
        target_found = False

        for res in results:
            for box in res.boxes:
                cls = int(box.cls[0])
                if cls == self.target_class_id:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    error_x = cx - center_x
                    error_y = cy - center_y

                    twist = Twist()
                    twist.angular.z = -float(error_x) / 300.0
                    twist.linear.x = -float(error_y) / 300.0

                    self.cmd_pub.publish(twist)
                    cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    target_found = True

        if not target_found:
            self.cmd_pub.publish(Twist())

        cv2.imshow("YOLOv8 Follow", cv_img)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("target_follower")
    node = TargetFollower()
    rospy.spin()
    cv2.destroyAllWindows()
