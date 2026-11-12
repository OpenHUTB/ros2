#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端 RL 训练环境（ROS2/rclpy + gymnasium）。

观测：/camera/image_raw 灰度图 (1,48,64)
动作：[linear_x, angular_z]
奖励：实际前进距离（来自 /uav/odom）*0.5 - 碰撞惩罚 - 转向惩罚
碰撞：从 /camera/depth 前向扇区最小深度判断（不用 /scan，保持纯视觉）
截断：每 episode max_steps 步（默认 500）
"""
from __future__ import annotations

import numpy as np
import rclpy
from gymnasium import Env, spaces
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from . import e2e_logic as L


class UAVEndToEndEnv(Env):
    metadata = {"render_modes": []}

    def __init__(self, node=None):
        super().__init__()
        self._node = node or rclpy.create_node("e2e_rl_train")
        self.cfg = self._load_cfg()
        self.observation_space = spaces.Box(
            low=0.0, high=1.0,
            shape=(1, self.cfg["img_h"], self.cfg["img_w"]), dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([2.0, 1.0], dtype=np.float32))

        self._img = np.zeros((self.cfg["img_h"], self.cfg["img_w"]), dtype=np.float32)
        self._depth = None
        self._odom_xy = None
        self._prev_xy = None
        self._step = 0
        self._got_image = False

        self._cmd = self._node.create_publisher(Twist, "/uav/cmd_vel", 10)
        self._node.create_subscription(Image, "/camera/image_raw", self._img_cb, 10)
        self._node.create_subscription(Image, "/camera/depth", self._depth_cb, 10)
        self._node.create_subscription(Odometry, "/uav/odom", self._odom_cb, 10)
        self._rate = self._node.create_rate(10.0)

    def _load_cfg(self):
        """从 ROS 参数读配置，没声明的参数保持 DEFAULT_CFG 里的值。"""
        cfg = dict(L.DEFAULT_CFG)
        names = ("img_w", "img_h", "forward_gain", "collision_penalty",
                 "yaw_penalty", "collision_depth", "max_steps")
        for name in names:
            if not self._node.has_parameter(name):
                self._node.declare_parameter(name, cfg[name])
            cfg[name] = self._node.get_parameter(name).value
        return cfg

    def _img_cb(self, msg: Image):
        self._img = L.preprocess_image(
            msg.encoding, bytes(msg.data), msg.width, msg.height,
            self.cfg["img_w"], self.cfg["img_h"])
        self._got_image = True

    def _depth_cb(self, msg: Image):
        w, h = msg.width, msg.height
        self._depth = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(h, w)

    def _odom_cb(self, msg: Odometry):
        self._odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _spin_once(self):
        rclpy.spin_once(self._node, timeout_sec=0.01)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._prev_xy = self._odom_xy
        self._step = 0
        # 等一帧图
        for _ in range(50):
            self._spin_once()
            if self._got_image:
                break
        return self._get_obs(), {}

    def _get_obs(self):
        return self._img[np.newaxis, :, :].astype(np.float32)

    def step(self, action):
        vx, wz = L.clip_action(float(action[0]), float(action[1]))
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        self._cmd.publish(tw)

        for _ in range(5):
            self._spin_once()
        self._rate.sleep()

        dx = 0.0
        if self._odom_xy is not None and self._prev_xy is not None:
            dx = self._odom_xy[0] - self._prev_xy[0]
        self._prev_xy = self._odom_xy

        collided = False
        if self._depth is not None:
            collided = L.is_collision(self._depth, self.cfg["collision_depth"])

        reward = L.compute_reward(dx, wz, collided, self.cfg)
        self._step += 1
        terminated = bool(collided)
        truncated = self._step >= int(self.cfg["max_steps"])
        return self._get_obs(), reward, terminated, truncated, {}


def main():
    rclpy.init()
    from stable_baselines3 import PPO
    env = UAVEndToEndEnv()

    node = env._node
    node.declare_parameter("timesteps", 100000)
    node.declare_parameter("n_steps", 256)
    node.declare_parameter("batch_size", 64)
    node.declare_parameter("learning_rate", 3e-4)
    node.declare_parameter("seed", 42)
    node.declare_parameter("out", "/tmp/ppo_e2e.zip")

    timesteps = int(node.get_parameter("timesteps").value)
    n_steps = int(node.get_parameter("n_steps").value)
    batch_size = int(node.get_parameter("batch_size").value)
    lr = float(node.get_parameter("learning_rate").value)
    seed = int(node.get_parameter("seed").value)
    out = node.get_parameter("out").value

    model = PPO("CnnPolicy", env, verbose=1,
                n_steps=n_steps, batch_size=batch_size,
                learning_rate=lr, seed=seed,
                policy_kwargs=dict(normalize_images=False))
    model.learn(total_timesteps=timesteps)
    model.save(out)
    node.get_logger().info(f"saved: {out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
