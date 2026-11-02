#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dataset.py —— 数据集：图像预处理 + 训练 Dataset（含数据增强）+ ROS 数据采集节点
"""
import os
import csv
import re
import random
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

IMG_H, IMG_W = 120, 160

# 部署安全限幅（与 perception_node 默认参数、安全层一致）。
# 训练标签按此尺度归一化到 [-1, 1]，使三通道 MSE 等权，
# 避免大量级的 vx 淹没小量级的 vy / yaw（转向指令被回归均值压缩）。
MAX_VX, MAX_VY, MAX_YAW = 2.0, 0.5, 0.4
ACTION_SCALE = np.array([MAX_VX, MAX_VY, MAX_YAW], dtype=np.float32)


def normalize_action(action):
    """原始指令 [vx, vy, yaw] -> 归一化 [-1, 1] 空间（训练/验证/诊断使用）。"""
    return np.asarray(action, dtype=np.float32) / ACTION_SCALE


def denormalize_action(action_norm):
    """归一化 [-1, 1] 空间 -> 原始指令 [vx, vy, yaw]（部署节点推理后使用）。"""
    return np.asarray(action_norm, dtype=np.float32) * ACTION_SCALE

# ImageNet 归一化均值/标准差（与 preprocess_image 保持一致）
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def preprocess_image(img_bgr):
    """把 BGR 图像 resize -> 归一化 -> (3,H,W) float32 张量（部署节点直接调用，不做增强）。"""
    img = cv2.resize(img_bgr, (IMG_W, IMG_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = (img - _MEAN) / _STD
    tensor = torch.from_numpy(img).permute(2, 0, 1).contiguous()
    return tensor


def _color_jitter(img_rgb):
    """随机亮度 / 对比度抖动（在 0~255 RGB 空间），提升对光照变化的泛化。"""
    # 对比度：围绕 1.0 上下浮动
    contrast = random.uniform(0.85, 1.15)
    mean = img_rgb.mean(axis=(0, 1), keepdims=True)
    img = (img_rgb - mean) * contrast + mean
    # 亮度：围绕 1.0 上下浮动
    brightness = random.uniform(0.85, 1.15)
    img = img * brightness
    return np.clip(img, 0, 255).astype(np.uint8)


def _augment(img_bgr, action):
    """训练期随机增强：水平翻转（动作对称翻转）+ 颜色抖动。

    水平镜像时：前后 vx 不变；左右 vy 取反；偏航 yaw 取反（左转 <-> 右转）。
    """
    img = cv2.resize(img_bgr, (IMG_W, IMG_H))
    action = list(action)

    if random.random() < 0.5:
        img = cv2.flip(img, 1)  # 水平翻转
        action[1] = -action[1]  # vy 左右镜像
        action[2] = -action[2]  # yaw 转向镜像

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_rgb = _color_jitter(img_rgb)
    img_norm = (img_rgb.astype(np.float32) / 255.0 - _MEAN) / _STD
    tensor = torch.from_numpy(img_norm).permute(2, 0, 1).contiguous()
    act_norm = normalize_action(action)
    return tensor, torch.tensor(act_norm, dtype=torch.float32)


class ImageDataset(Dataset):
    """从 data.csv 读取 (图像路径, [vx, vy, yaw_rate]) 样本。

    augment=True 时应用水平翻转 + 颜色抖动（仅训练集使用）；
    augment=False 时只做确定性预处理（验证集使用）。
    """

    def __init__(self, csv_path, augment=False):
        self.samples = []
        self.augment = augment
        base_dir = os.path.dirname(csv_path)
        with open(csv_path, "r") as f:
            reader = csv.reader(f)
            next(reader, None)  # 表头
            for row in reader:
                if not row:
                    continue
                img_rel = row[0]
                vx, vy, yaw = float(row[1]), float(row[2]), float(row[3])
                self.samples.append(
                    (os.path.join(base_dir, img_rel), [vx, vy, yaw]))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        img_path, action = self.samples[idx]
        img = cv2.imread(img_path)
        if img is None:
            img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
        if self.augment:
            return _augment(img, action)
        return preprocess_image(img), torch.tensor(
            normalize_action(action), dtype=torch.float32)


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

    # 续采：扫描现有图片最大编号，从其后续写，避免重启采集节点后覆盖旧样本。
    start_count = 0
    if os.path.isdir(img_dir):
        for fn in os.listdir(img_dir):
            m = re.match(r"img_(\d+)\.jpg", fn)
            if m:
                start_count = max(start_count, int(m.group(1)) + 1)

    state = {"latest_twist": Twist(), "count": start_count}

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
