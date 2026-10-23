#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dataset.py —— 数据集：图像预处理 + 训练 Dataset + ROS 数据采集节点
"""
import os
import csv
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

IMG_H, IMG_W = 120, 160


def preprocess_image(img_bgr):
    """把 BGR 图像 resize -> 归一化 -> (3,H,W) float32 张量。"""
    img = cv2.resize(img_bgr, (IMG_W, IMG_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    img = (img - mean) / std
    tensor = torch.from_numpy(img).permute(2, 0, 1).contiguous()
    return tensor


class ImageDataset(Dataset):
    """从 data.csv 读取 (图像路径, [vx, vy, yaw_rate]) 样本。"""

    def __init__(self, csv_path):
        self.samples = []
        base_dir = os.path.dirname(csv_path)
        with open(csv_path, "r") as f:
            reader = csv.reader(f)
            header = next(reader, None)
            for row in reader:
                if not row:
                    continue
                img_rel = row[0]
                vx, vy, yaw = float(row[1]), float(row[2]), float(row[3])
                self.samples.append((os.path.join(base_dir, img_rel), [vx, vy, yaw]))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        img_path, action = self.samples[idx]
        img = cv2.imread(img_path)
        if img is None:
            img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
        tensor = preprocess_image(img)
        target = torch.tensor(action, dtype=torch.float32)
        return tensor, target


def dataset_collect():
    import rospy
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Twist
    from cv_bridge import CvBridge

    rospy.init_node("dataset_collector")
    bridge = CvBridge()
    save_dir = rospy.get_param("~save_dir", os.path.expanduser("~/perception_dataset"))
    img_dir = os.path.join(save_dir, "images")
    os.makedirs(img_dir, exist_ok=True)
    csv_path = os.path.join(save_dir, "data.csv")
    if not os.path.exists(csv_path):
        with open(csv_path, "w", newline="") as f:
            csv.writer(f).writerow(["image_path", "vx", "vy", "yaw"])

    state = {"latest_twist": Twist(), "count": 0}

    def on_twist(msg):
        state["latest_twist"] = msg

    def on_image(msg):
        twist = state["latest_twist"]
        moving = (abs(twist.linear.x) > 1e-3 or abs(twist.linear.y) > 1e-3
                  or abs(twist.angular.z) > 1e-3)
        if not moving:
            return
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        idx = state["count"]
        fname = f"img_{idx:06d}.jpg"
        cv2.imwrite(os.path.join(img_dir, fname), img)
        with open(csv_path, "a", newline="") as f:
            csv.writer(f).writerow(
                [os.path.join("images", fname),
                 round(twist.linear.x, 4),
                 round(twist.linear.y, 4),
                 round(twist.angular.z, 4)])
        state["count"] += 1
        rospy.loginfo_throttle(2.0, "已采集 %d 条样本", state["count"])

    rospy.Subscriber("/camera/image_raw", Image, on_image, queue_size=1)
    rospy.Subscriber("/uav/cmd_vel", Twist, on_twist, queue_size=1)
    rospy.loginfo("数据集采集节点已启动")
    rospy.spin()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--selfcheck":
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        t = preprocess_image(dummy)
        print(f"预处理输出形状: {tuple(t.shape)}")
        print("dataset.py 自检通过")
    else:
        dataset_collect()
