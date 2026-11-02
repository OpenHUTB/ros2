#!/usr/bin/env python3
"""main.py —— 多模态跌倒预警 standalone 入口"""
from __future__ import annotations
import argparse, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from humanoid_fall_warning.sim import FallWarningSim, DEFAULT_STAND_TARGET
from humanoid_fall_warning.dataset import collect
from humanoid_fall_warning.train import train
from humanoid_fall_warning.predictor import FallWarningPredictor

MODEL = os.path.join(HERE, "models", "humanoid_warning.xml")
DATASET = os.path.join(HERE, "data", "fall_warning_dataset.npz")
WEIGHTS = os.path.join(HERE, "models_trained", "fall_warning.npz")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--collect", action="store_true")
    p.add_argument("--train", action="store_true")
    p.add_argument("--demo", action="store_true")
    p.add_argument("--samples", type=int, default=2000)
    a = p.parse_args()
    if a.collect:
        os.makedirs(os.path.dirname(DATASET), exist_ok=True)
        collect(MODEL, DATASET, n_samples=a.samples)
    elif a.train:
        os.makedirs(os.path.dirname(WEIGHTS), exist_ok=True)
        train(DATASET, WEIGHTS)
    elif a.demo:
        sim = FallWarningSim(MODEL)
        pred = FallWarningPredictor(WEIGHTS)
        sim.reset()
        for _ in range(200): sim.step()
        sim.apply_push((120, 0, 0), duration_steps=40)
        for i in range(1000):
            img = sim.render_front(gray=True)
            size = 16; bh, bw = img.shape[0]//size, img.shape[1]//size
            crop = img[:bh*size, :bw*size]
            feat = crop.reshape(size, bh, size, bw).mean(axis=(1,3)).flatten()
            pitch, roll = sim.torso_pose()
            p_normal, p_fall = pred.predict(feat, pitch, roll)
            target = DEFAULT_STAND_TARGET.copy()
            if p_fall > 0.6:
                target[1] = 0.3; target[6] = 0.3
                target[2] = -0.4; target[7] = -0.4
                print(f"  [{i}] FALL WARNING p_fall={p_fall:.2f} → squat")
            sim.step(target=target)
            if sim.fallen(): print(f"跌倒 @ {i}"); break
        sim.close()
    else:
        p.print_help()


if __name__ == "__main__":
    main()
