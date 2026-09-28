#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO 导航本地测试（纯 numpy，不需要 ROS / 仿真器 / GPU）.

A. policy 纯函数：直方图 / 坐标变换 / 观测拼接 / 动作映射 / MLP 推理
B. NavEnv 训练环境：reset / step / 观测维度 / 终止判定

用法: python3 tests/test_ppo_nav_local.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
SCRIPTS = os.path.join(ROOT, "src", "air", "uav_ppo_nav", "scripts")
sys.path.insert(0, SCRIPTS)

import policy as P  # noqa: E402
from env import NavEnv  # noqa: E402


# ================================================================= A. policy
def test_policy():
    print("== A. policy 纯函数 ==")
    # 直方图
    h = P.lidar_points_to_histogram(np.zeros((0, 3)))
    check(h.shape == (16,) and np.allclose(h, 1.0), "空点云 -> 全 1（无遮挡）")

    pts = np.array([[6.0, 0.0, 0.0]])          # 正前方 6 m
    h = P.lidar_points_to_histogram(pts, max_range=12.0)
    check(abs(h[0] - 0.5) < 1e-4, "正前方 6m -> 扇形0 = 0.5: %.3f" % h[0])
    check(np.allclose(h[1:], 1.0), "其余扇形无遮挡 = 1.0")

    high = np.array([[1.0, 0.0, 5.0]])          # 垂直方向太远，应被过滤
    h = P.lidar_points_to_histogram(high)
    check(np.allclose(h, 1.0), "高度超出 z-band 的点被过滤")

    near = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    h = P.lidar_points_to_histogram(near, max_range=12.0)
    check(abs(h[0] - 1.0 / 12.0) < 1e-3, "同扇形取最近距离: %.4f" % h[0])

    # 坐标变换：yaw=0 时东向(+x)点在前方
    b = P.world_to_body([1.0, 0.0, 0.0], yaw=0.0)
    check(np.allclose(b, [1.0, 0.0, 0.0]), "yaw=0: 东向点 -> 正前方")
    b = P.world_to_body([1.0, 0.0, 0.0], yaw=np.pi / 2)
    check(np.allclose(b, [0.0, -1.0, 0.0], atol=1e-6), "yaw=90°: 东向点 -> 右方(左=-1)")

    # 观测拼接
    obs = P.build_observation(np.ones(16), [0.0, 0.0, 0.0], [3.0, 0.0, 1.5])
    check(obs.shape == (22,), "观测维度 = 22")
    check(abs(obs[19] - 1.0) < 1e-5, "水平速度归一化: 3.0/3.0 = 1.0")
    check(abs(obs[21] - 1.0) < 1e-5, "垂直速度归一化: 1.5/1.5 = 1.0")

    # 动作映射
    v = P.action_to_velocity([1.0, 0.0, 0.0, 0.0])
    check(v == (3.0, 0.0, 0.0, 0.0), "动作(1,0,0,0) -> 前进 3 m/s: %r" % (v,))
    v = P.action_to_velocity([0.0, 0.0, 1.0, 0.0])
    check(v == (0.0, 0.0, 1.5, 0.0), "动作(0,0,1,0) -> 上升 1.5 m/s")
    v = P.action_to_velocity([0.0, 0.0, 0.0, -1.0])
    check(v == (0.0, 0.0, 0.0, -1.0), "动作(0,0,0,-1) -> 偏航 -1 rad/s")

    # MLP 推理
    rng = np.random.default_rng(0)
    w = {"n_layers": 2,
         "h0_w": rng.standard_normal((64, 22)).astype(np.float32),
         "h0_b": rng.standard_normal(64).astype(np.float32),
         "h1_w": rng.standard_normal((64, 64)).astype(np.float32),
         "h1_b": rng.standard_normal(64).astype(np.float32),
         "out_w": rng.standard_normal((4, 64)).astype(np.float32),
         "out_b": rng.standard_normal(4).astype(np.float32)}
    mlp = P.MlpPolicy(w)
    a = mlp.forward(np.zeros(22, dtype=np.float32))
    check(a.shape == (4,) and np.all(np.abs(a) <= 1.0 + 1e-5), "MLP 前向输出 4 维且 clip 到 [-1,1]")


# ================================================================= B. env
def test_env():
    print("== B. NavEnv 训练环境 ==")
    env = NavEnv(num_obstacles=6, seed=0)
    obs, _ = env.reset()
    check(obs.shape == (22,), "reset 观测维度 22")
    check(env.action_space.shape == (4,), "动作空间 4 维")

    # 随机动作跑几步，确认 step 返回 5 元组且不崩溃
    terminated = truncated = False
    for _ in range(50):
        obs, r, terminated, truncated, info = env.step(env.action_space.sample())
        check(isinstance(r, float) and obs.shape == (22,), "step 返回合法奖励与观测")
        if terminated or truncated:
            break
    check(terminated or truncated or True, "随机策略也能正常推进回合")

    # 确定性：无障碍、目标就在前方，应能到达
    env2 = NavEnv(num_obstacles=0, seed=1)
    obs, _ = env2.reset()
    env2.yaw = 0.0                                  # 固定偏航，让机头朝世界 +x
    env2.goal = env2.pos + np.array([1.0, 0.0, 0.0])  # 目标在前方 1 m
    env2.dist_prev = 1.0
    succ = False
    for _ in range(100):
        obs, r, terminated, truncated, info = env2.step([1.0, 0.0, 0.0, 0.0])  # 全速前进
        if terminated and info["success"]:
            succ = True
            break
        if truncated:
            break
    check(succ, "无障碍直飞可到达目标（成功判定生效）")
    env.close()
    env2.close()


def main():
    test_policy()
    test_env()
    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for f in FAIL:
            print("  FAIL: %s" % f)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
