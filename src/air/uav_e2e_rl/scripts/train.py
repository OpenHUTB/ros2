#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端 RL 训练：相机图像 -> PPO -> 速度指令。

观测：camera 图像 (64x48 灰度, 归一化到 [0,1])
动作：[linear_x (0~2 m/s), angular_z (-1~1 rad/s)]
奖励：前进距离 * 0.5 - 碰撞惩罚 - 转向惩罚
"""
import os
import numpy as np
import rospy
import gymnasium as gym
from gymnasium import spaces
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import cv2


class UAVEndToEndEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self):
        super().__init__()
        self.img_w = 64
        self.img_h = 48
        self.observation_space = spaces.Box(
            low=0.0, high=1.0, shape=(1, self.img_h, self.img_w), dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([2.0, 1.0], dtype=np.float32))

        self._img = np.zeros((self.img_h, self.img_w), dtype=np.float32)
        self._scan = None
        self._prev_x = 0.0
        self._collision = False

        rospy.Subscriber("/camera/image_raw", Image, self._img_cb, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self._scan_cb, queue_size=1)
        self._cmd_pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)

    def _img_cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
        self._img = cv2.resize(gray, (self.img_w, self.img_h)).astype(np.float32) / 255.0

    def _scan_cb(self, msg):
        self._scan = np.array(msg.ranges, dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._prev_x = 0.0
        self._collision = False
        return self._get_obs(), {}

    def _get_obs(self):
        return self._img[np.newaxis, :, :].astype(np.float32)

    def step(self, action):
        vx, wz = float(action[0]), float(action[1])
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        self._cmd_pub.publish(tw)

        rospy.sleep(0.1)

        reward = vx * 0.5
        terminated = False
        if self._scan is not None:
            finite = self._scan[np.isfinite(self._scan)]
            if len(finite) > 0 and finite.min() < 0.6:
                reward -= 5.0
                terminated = True
        reward -= abs(wz) * 0.1
        truncated = False
        return self._get_obs(), reward, terminated, truncated, {}


def main():
    rospy.init_node("e2e_rl_train", anonymous=True)
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env

    env = UAVEndToEndEnv()
    model = PPO("CnnPolicy", env, verbose=1, n_steps=64, batch_size=32,
                learning_rate=3e-4,
                policy_kwargs=dict(normalize_images=False))
    model.learn(total_timesteps=20000)
    out = os.path.expanduser("~/catkin_ws/src/air/uav_e2e_rl/models/ppo_e2e.zip")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    model.save(out)
    rospy.loginfo(f"saved: {out}")


if __name__ == "__main__":
    main()
