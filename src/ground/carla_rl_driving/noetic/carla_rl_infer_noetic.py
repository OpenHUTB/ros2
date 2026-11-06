#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Noetic(ROS1) 版端到端推理节点：carla_rl_infer_noetic.py

与 carla_rl_driving/infer_node.py 功能等价，但基于 rospy，跑在 Ubuntu 20.04 + ROS Noetic。
闭环：订阅 /carla/ego_vehicle/rgb_view/image -> 预处理 -> 加载策略 -> 发布 ackermann 控制。

运行：
    source /opt/ros/noetic/setup.bash
    python noetic/carla_rl_infer_noetic.py --model models/ppo_cnn_town01.zip
"""

import argparse
import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import Image

try:
    from carla_msgs.msg import CarlaAckermannControl
except Exception:
    CarlaAckermannControl = None

# 允许直接以脚本方式运行时 import 到包内模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from carla_rl_driving.utils import preprocess_image  # noqa: E402


class InferNodeNoetic(object):
    def __init__(self, model_path, image_topic, control_topic, device='cpu'):
        self.model = None
        if os.path.exists(model_path):
            try:
                from stable_baselines3 import PPO
                self.model = PPO.load(model_path, device=device)
                rospy.loginfo('已加载策略模型：%s', model_path)
            except Exception as e:
                rospy.logwarn('加载模型失败（%s），将输出零控制', e)
        else:
            rospy.logwarn('模型不存在：%s，节点空转', model_path)

        self._img = None
        self.sub = rospy.Subscriber(image_topic, Image, self._on_image, queue_size=1)
        if CarlaAckermannControl is not None:
            self.pub = rospy.Publisher(control_topic, CarlaAckermannControl, queue_size=1)
        else:
            self.pub = None
            rospy.logwarn('未找到 carla_msgs，控制指令只打印日志')

    def _on_image(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        self._img = arr

    def step(self):
        if self._img is None or self.model is None:
            return
        obs = preprocess_image(self._img, target_h=80, target_w=160)
        obs = np.expand_dims(obs, 0)
        action, _ = self.model.predict(obs, deterministic=True)
        steer, accel = float(action[0]), float(action[1])
        if self.pub is not None:
            cmd = CarlaAckermannControl()
            cmd.steer = steer
            cmd.speed = max(accel, 0.0) * 10.0
            self.pub.publish(cmd)
        rospy.logdebug('steer=%.2f accel=%.2f', steer, accel)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='models/ppo_cnn_town01.zip')
    ap.add_argument('--image-topic', default='/carla/ego_vehicle/rgb_view/image')
    ap.add_argument('--control-topic', default='/carla/ego_vehicle/ackermann_cmd')
    ap.add_argument('--device', default='cpu')
    args, _ = ap.parse_known_args()

    rospy.init_node('carla_rl_infer_noetic')
    node = InferNodeNoetic(args.model, args.image_topic, args.control_topic, args.device)
    rate = rospy.Rate(20.0)
    rospy.loginfo('Noetic 推理节点启动，控制 20Hz')
    while not rospy.is_shutdown():
        node.step()
        rate.sleep()


if __name__ == '__main__':
    main()
