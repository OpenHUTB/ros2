#!/usr/bin/env python3
"""PilotNet 推理节点

订阅 /camera/image_raw，用 PilotNet CNN 推理，
发布 /pilotnet/control（steering + throttle）到 ROS 话题。
"""
import os
import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

# 从包内 src/ 导入模型
pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(pkg_root, 'src'))
from model import PilotNet

import torch


class PilotNetInference:
    def __init__(self):
        rospy.init_node('pilotnet_inference', anonymous=False)

        model_path = rospy.get_param('~model_path', 'pilotnet.pth')
        self.input_h = rospy.get_param('~input_h', 66)
        self.input_w = rospy.get_param('~input_w', 200)
        self.infer_every_n = rospy.get_param('~infer_every_n', 2)

        if not os.path.isabs(model_path):
            candidate = os.path.join(pkg_root, model_path)
            if os.path.exists(candidate):
                model_path = candidate

        self.model = PilotNet(self.input_h, self.input_w)
        if os.path.exists(model_path):
            self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
            rospy.loginfo("Loaded weights: " + model_path)
        else:
            rospy.logwarn("Weight file not found, using random init: " + model_path)
        self.model.eval()

        self.frame_count = 0

        self.ctrl_pub = rospy.Publisher('/pilotnet/control', Float32MultiArray, queue_size=1)
        self.annotated_pub = rospy.Publisher('/pilotnet/annotated_image', Image, queue_size=1)
        rospy.Subscriber('/camera/image_raw', Image, self.on_image, queue_size=1)

        rospy.loginfo("PilotNet inference node ready")

    @staticmethod
    def imgmsg_to_cv2(msg):
        n_channels = 3 if msg.encoding in ('bgr8', 'rgb8') else 1
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, n_channels)

    @staticmethod
    def cv2_to_imgmsg(frame):
        msg = Image()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = 3 * frame.shape[1]
        msg.data = frame.tobytes()
        return msg

    def on_image(self, msg):
        try:
            frame = self.imgmsg_to_cv2(msg)
        except Exception as e:
            rospy.logerr("decode error: " + str(e))
            return

        self.frame_count += 1
        if self.frame_count % self.infer_every_n != 0:
            return

        # 预处理：BGR -> RGB，resize，归一化到 [0,1]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.input_w, self.input_h))
        tensor = torch.from_numpy(resized).float().permute(2, 0, 1).unsqueeze(0) / 255.0

        with torch.no_grad():
            out = self.model(tensor)

        steering = float(out[0, 0].item())
        throttle = float(out[0, 1].item())

        ctrl_msg = Float32MultiArray()
        ctrl_msg.data = [steering, throttle]
        self.ctrl_pub.publish(ctrl_msg)

        rospy.loginfo_throttle(1.0,
            "steering=%.3f throttle=%.3f" % (steering, throttle))

        # 发布标注图像（原图 + 控制指令文本）
        annotated = frame.copy()
        cv2.putText(annotated,
                    "steer=%.2f thr=%.2f" % (steering, throttle),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0), 2)
        out_msg = self.cv2_to_imgmsg(annotated)
        out_msg.header = msg.header
        self.annotated_pub.publish(out_msg)


if __name__ == '__main__':
    try:
        PilotNetInference()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
