#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
from std_msgs.msg import Float32MultiArray # 引入消息类型

class YoloDetector:
    def __init__(self):
        self.bridge = CvBridge()
        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(script_dir, 'yolov8n.pt')
        self.model = YOLO(model_path)
        self.model.to('cpu')
        
        # 核心改动：创建一个 Publisher，话题名叫 /target_position，用来发坐标
        self.pub = rospy.Publisher('/target_position', Float32MultiArray, queue_size=1)
        
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback, queue_size=1)
        rospy.loginfo("YOLO检测节点启动，正在发布目标位置...")

    def callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        height, width = cv_img.shape[:2]
        results = self.model(cv_img)
        
        # 找出置信度最高的目标
        best_box = None
        max_conf = 0

        for res in results:
            for box in res.boxes:
                conf = float(box.conf[0])
                # 我们假设目标是 person (类别 0) 或者只要有东西就追
                cls = int(box.cls[0])
                if conf > max_conf:
                    max_conf = conf
                    best_box = box

        # 如果找到了目标
        if best_box is not None:
            x1, y1, x2, y2 = map(int, best_box.xyxy[0])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            
            # 计算误差（目标中心相对于画面中心的偏差）
            error_x = cx - (width // 2)
            error_y = cy - (height // 2)

            # 把误差发布出去！
            data_to_send = Float32MultiArray()
            data_to_send.data = [float(error_x), float(error_y)]
            self.pub.publish(data_to_send)

            cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow("YOLOv8 Detect", cv_img)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("yolo_detector")
    node = YoloDetector()
    rospy.spin()
    cv2.destroyAllWindows()
