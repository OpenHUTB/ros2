#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py —— 人形机器人端到端视觉平衡控制（standalone 入口）
============================================================
用法:
    python3 main.py --collect --episodes 100   # 采集专家数据
    python3 main.py --train                    # 训练端到端网络
    python3 main.py --demo --force 120         # 端到端演示
    python3 main.py --check                    # 自检
"""
from __future__ import annotations
import argparse, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from humanoid_e2e_recovery.sim import E2ESim, DEFAULT_STAND_TARGET
from humanoid_e2e_recovery.dataset import collect
from humanoid_e2e_recovery.train import train
from humanoid_e2e_recovery.predictor import E2EPredictor

MODEL_PATH = os.path.join(HERE, "models", "humanoid_camera.xml")
DATASET = os.path.join(HERE, "data", "e2e_dataset.npz")
WEIGHTS = os.path.join(HERE, "models_trained", "e2e_policy.npz")


def run_check():
    print("="*66); print("  端到端视觉平衡控制 自检"); print("="*66)
    sim = E2ESim(MODEL_PATH)
    sim.reset()
    for _ in range(300): sim.step()
    print(f"[PASS] 模型加载, 站姿根高={sim.root_z:.3f}m")
    img = sim.render_front()
    print(f"[PASS] 相机渲染 shape={img.shape}")
    sim.close()
    return 0


def run_demo(force=120, direction_deg=0):
    sim = E2ESim(MODEL_PATH)
    predictor = E2EPredictor(WEIGHTS)
    sim.reset()
    for _ in range(250): sim.step()
    rad = np.radians(direction_deg)
    sim.apply_push((force*np.cos(rad), force*np.sin(rad), 0), steps=40)
    max_tilt = 0; fell = None
    for i in range(1500):
        img = sim.render_front(gray=True)
        target = predictor.predict_from_image(img)
        sim.step(target=target)
        tilt = abs(sim.torso_tilt())
        max_tilt = max(max_tilt, tilt)
        if sim.fallen():
            fell = float(sim.data.time); break
        if i % 50 == 0:
            print(f"  t={sim.data.time:.2f}s 倾角={np.degrees(tilt):.1f}° 根高={sim.root_z:.3f}m")
    r = {"force": force, "dir": direction_deg, "max_tilt_deg": float(np.degrees(max_tilt)),
         "fell": fell, "final_z": sim.root_z}
    print(f"\n[结果] {'跌倒' if fell else '恢复'} | 最大倾角 {np.degrees(max_tilt):.1f}°")
    sim.close()
    return r


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--check", action="store_true")
    p.add_argument("--collect", action="store_true")
    p.add_argument("--train", action="store_true")
    p.add_argument("--demo", action="store_true")
    p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--force", type=float, default=120)
    p.add_argument("--direction", type=float, default=0)
    a = p.parse_args()
    if a.check: sys.exit(run_check())
    if a.collect:
        os.makedirs(os.path.dirname(DATASET), exist_ok=True)
        collect(MODEL_PATH, DATASET, n_episodes=a.episodes)
    elif a.train:
        os.makedirs(os.path.dirname(WEIGHTS), exist_ok=True)
        train(DATASET, WEIGHTS)
    elif a.demo:
        print(json.dumps(run_demo(a.force, a.direction), indent=2, default=str))
    else:
        p.print_help()


if __name__ == "__main__":
    main()
