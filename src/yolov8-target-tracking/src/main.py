#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import rospkg
import os

class YoloDetector:
    def __init__(self):
        self.bridge = CvBridge()
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('yolo_detect')
        model_path = os.path.join(pkg_path, 'yolov8n.pt')
        
        self.model = YOLO(model_path)
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback, queue_size=1)
        rospy.loginfo("YOLO检测节点启动，等待图像...")

    def callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        results = self.model(cv_img)
        
        for res in results:
            boxes = res.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                name = self.model.names[cls]
                
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_img, f"{name} {conf:.2f}", (x1, y1 - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("YOLOv8 Detect", cv_img)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("yolo_detector")
    node = YoloDetector()
    rospy.spin()
    cv2.destroyAllWindows()
