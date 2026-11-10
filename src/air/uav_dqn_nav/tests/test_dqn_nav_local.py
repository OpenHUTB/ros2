#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_dqn_nav 本地冒烟测试（无需 ROS / torch / gymnasium）.

运行：
    python3 tests/test_dqn_nav_local.py

覆盖：
1. 激光雷达扇形直方图（正东点云 -> 扇区 0 距离正确）
2. 观测构造（22 维）
3. 连续动作映射（与 PPO 一致）
4. DQN 离散动作表（35 个动作：16 巡航 + 16 慢速 + 升降 + 悬停，边界裁剪）
5. DqnPolicy 纯 numpy 推理（随机权重：动作索引与速度形状正确）
6. 与 PPO 模块的接口一致性（常量逐一相等）
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

import numpy as np

import policy
from policy import (ACTION_NAMES, N_ACTIONS, N_SECTORS, OBS_DIM,
                    DqnPolicy, act_index_to_velocity, action_to_velocity,
                    build_observation, lidar_points_to_histogram)

PASSED = 0
FAILED = 0


def check(name: str, cond: bool, detail: str = ""):
    global PASSED, FAILED
    if cond:
        PASSED += 1
        print("  [PASS] %s" % name)
    else:
        FAILED += 1
        print("  [FAIL] %s  %s" % (name, detail))


def make_random_policy_npz(path: str):
    """生成随机 Q 网络权重（2 隐藏层 + 1 输出层），模拟训练导出格式."""
    rng = np.random.default_rng(0)
    d = {
        "n_layers": 2,
        "h0_w": rng.standard_normal((64, OBS_DIM)).astype(np.float32),
        "h0_b": rng.standard_normal(64).astype(np.float32),
        "h1_w": rng.standard_normal((64, 64)).astype(np.float32),
        "h1_b": rng.standard_normal(64).astype(np.float32),
        "out_w": rng.standard_normal((N_ACTIONS, 64)).astype(np.float32),
        "out_b": rng.standard_normal(N_ACTIONS).astype(np.float32),
    }
    np.savez(path, **d)
    return path


def main():
    print("== 1. 激光雷达扇形直方图 ==")
    # 正东 5 m 处一个点 -> 扇区 0（世界系 0 = 正东）距离 5/12
    pts = np.array([[5.0, 0.0, 0.0]], dtype=np.float64)
    hist = lidar_points_to_histogram(pts, drone_xy=(0.0, 0.0), drone_z=0.0)
    check("点云在正东 -> 扇区0最近距离=5/12", abs(hist[0] - 5.0 / 12.0) < 1e-4,
          "hist[0]=%s" % hist[0])
    check("其余扇区无遮挡=1", float(hist[3]) == 1.0)
    # 垂直带外点不统计（z 差 > 1.5 m）
    pts2 = np.array([[5.0, 0.0, 5.0]], dtype=np.float64)
    hist2 = lidar_points_to_histogram(pts2, drone_xy=(0.0, 0.0), drone_z=0.0)
    check("高度带外点忽略", float(hist2[0]) == 1.0)

    print("== 2. 观测构造 ==")
    obs = build_observation(np.ones(16, dtype=np.float32),
                            np.array([8.0, 4.0, 2.0], dtype=np.float32),
                            np.array([3.0, 1.5, 0.0], dtype=np.float32))
    check("观测维度=22", obs.shape == (OBS_DIM,), "shape=%s" % (obs.shape,))
    check("目标归一化（8/8=1）", abs(obs[16] - 1.0) < 1e-4)
    check("速度归一化（3/3=1）", abs(obs[19] - 1.0) < 1e-4)

    print("== 3. 连续动作映射（与 PPO 一致） ==")
    vx, vy, vz, wz = action_to_velocity([1.0, 0.0, 0.0, 0.0])
    check("正向动作 -> 东向 3.0 m/s", abs(vx - 3.0) < 1e-4 and abs(vy) < 1e-4)
    vx, vy, vz, _ = action_to_velocity([0.0, -1.0, 1.0, 0.0])
    check("负北 -> 南向 3.0, 天向 1.5", abs(vy + 3.0) < 1e-4 and abs(vz - 1.5) < 1e-4)

    print("== 4. DQN 离散动作表 ==")
    check("动作数=35", N_ACTIONS == 35)
    check("动作名数=35", len(ACTION_NAMES) == 35)
    v0 = act_index_to_velocity(0)
    check("动作0=正东巡航", abs(v0[0] - 2.1) < 1e-3 and abs(v0[1]) < 1e-3)
    v16 = act_index_to_velocity(16)
    check("动作16=正东慢速逼近", abs(v16[0] - 1.05) < 1e-3 and abs(v16[1]) < 1e-3)
    v34 = act_index_to_velocity(34)
    check("动作34=悬停", all(abs(x) < 1e-6 for x in v34[:3]))
    v33 = act_index_to_velocity(33)
    check("动作33=下降", abs(v33[2] + 1.5) < 1e-4)
    v_clip = act_index_to_velocity(99)
    check("越界动作裁剪到 34", v_clip == v34)

    print("== 5. DqnPolicy 纯 numpy 推理 ==")
    npz = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_tmp_policy.npz")
    make_random_policy_npz(npz)
    p = DqnPolicy.from_npz(npz)
    act = p.act(obs)
    check("act 返回 0-34", 0 <= act < N_ACTIONS, "act=%d" % act)
    vel = p.forward(obs)
    check("forward 返回 4 维速度", vel.shape == (4,) and vel.dtype == np.float32,
          "shape=%s" % (vel.shape,))
    q = p.q_values(obs)
    check("q_values 长度=35", q.shape == (N_ACTIONS,))
    os.remove(npz)

    print("== 6. 与 PPO 模块接口一致性 ==")
    # 两个 policy.py 同名，按绝对路径加载 PPO 版本避免模块名冲突
    import importlib.util  # noqa: WPS433
    ppo_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "..", "uav_ppo_nav", "scripts", "policy.py")
    spec = importlib.util.spec_from_file_location("ppo_policy", ppo_path)
    ppo_policy = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ppo_policy)

    for name in ("N_SECTORS", "MAX_RANGE", "MAX_HORIZ_SPEED", "MAX_VERT_SPEED",
                 "MAX_YAW_RATE", "GOAL_SCALE", "OBS_DIM", "ACT_DIM"):
        a = getattr(policy, name)
        b = getattr(ppo_policy, name)
        check("常量 %s 与 PPO 一致 (%s)" % (name, a), a == b)

    print("\n结果: %d 通过, %d 失败" % (PASSED, FAILED))
    sys.exit(1 if FAILED else 0)


if __name__ == "__main__":
    main()
