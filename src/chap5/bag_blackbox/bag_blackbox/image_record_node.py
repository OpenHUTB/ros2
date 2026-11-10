#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageRecorder:
    def __init__(self):
        self.bridge = CvBridge()
        self.save_path = rospy.get_param("~img_save_path", "./img_save/")
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        self.cnt = 0
        self.sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)

    def callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(os.path.join(self.save_path, f"img_{self.cnt:04d}.jpg"), cv_img)
            self.cnt += 1
            rospy.loginfo(f"save image {self.cnt}")
        except Exception as e:
            rospy.logerr(e)

if __name__ == "__main__":
    rospy.init_node("image_record_node")
    rec = ImageRecorder()
    rospy.spin()
