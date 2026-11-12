#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""01 特征动态范围：状态明显变化时，池化图到底变了多少。

回答的问题：图像里有没有可学的信号，还是被相机配置/分辨率压没了。
若 64x64 原始分辨率的逐维 std 只比 16x16 池化大一点点，说明瓶颈在
"姿态变化在画面上本来就看不见"，而不是池化丢了信息。

用法: python3 diag/01_feature_dynamics.py
实测(云主机 MuJoCo 4.0.0): 16x16 逐维 std 均值 0.00277，64x64 原始 0.00354，
只差 1.3 倍；而这段时间倾角变化 2.6 度、骨盆位移 3.2 cm。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import MODEL_PATH, DEFAULT_STAND_TARGET  # noqa: E402

from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402
from humanoid_e2e_recovery.expert import expert_action  # noqa: E402
from humanoid_e2e_recovery.dataset import _img_to_vec  # noqa: E402


def main():
    sim = E2ESim(MODEL_PATH)
    vec16, raw64, states = [], [], []
    for trial in range(6):
        sim.reset()
        if trial > 0:
            ang = trial * 60
            f = 40 + trial * 10
            sim.apply_push((f * np.cos(np.radians(ang)), f * np.sin(np.radians(ang)), 0),
                           steps=40, target=DEFAULT_STAND_TARGET, balance=expert_action)
        for i in range(120):
            g = sim.render_front(gray=True)
            if i % 10 == 0:
                vec16.append(_img_to_vec(g))
                raw64.append(g.flatten())
                states.append((sim.pelvis_x, sim.pelvis_vx, sim.torso_tilt()))
            sim.step(target=DEFAULT_STAND_TARGET, torque_extra=expert_action(sim))
            if sim.fallen():
                break
    sim.close()

    V = np.stack(vec16)
    R = np.stack(raw64)
    S = np.array(states)

    print("样本数: %d" % len(V))
    print("=== 16x16 池化 (256 维) ===")
    print("  值域 {:.3f} ~ {:.3f}  逐维 std: 均值 {:.5f}  最大 {:.5f}".format(
        V.min(), V.max(), V.std(0).mean(), V.std(0).max()))
    print("=== 64x64 原始 (4096 维) ===")
    print("  值域 {:.3f} ~ {:.3f}  逐维 std: 均值 {:.5f}  最大 {:.5f}".format(
        R.min(), R.max(), R.std(0).mean(), R.std(0).max()))
    print("=== 状态变化范围（说明画面本应变化）===")
    print("  pelvis_x 范围 {:.4f} ~ {:.4f} (跨度 {:.4f} m)".format(
        S[:, 0].min(), S[:, 0].max(), np.ptp(S[:, 0])))
    print("  pelvis_vx 范围 {:.3f} ~ {:.3f}".format(S[:, 1].min(), S[:, 1].max()))
    print("  torso_tilt 范围 {:.2f}° ~ {:.2f}° (跨度 {:.2f}°)".format(
        np.degrees(S[:, 2].min()), np.degrees(S[:, 2].max()), np.degrees(np.ptp(S[:, 2]))))
    print("=== 信息量比 ===")
    print("  64x64 逐维 std 均值 / 16x16 = {:.1f} 倍".format(
        R.std(0).mean() / max(V.std(0).mean(), 1e-9)))


if __name__ == "__main__":
    main()
