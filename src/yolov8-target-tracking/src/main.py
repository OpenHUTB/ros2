#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        # 注意：这里已经修改为正确的 Gazebo 相机话题
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
                
                # 画框并显示类别和置信度
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
