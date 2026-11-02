#!/usr/bin/env python3
"""main.py —— 多帧时序端到端平衡控制 standalone 入口"""
from __future__ import annotations
import argparse, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from humanoid_temporal_e2e.sim import TemporalE2ESim
from humanoid_temporal_e2e.dataset import collect
from humanoid_temporal_e2e.train import train
from humanoid_temporal_e2e.predictor import TemporalPredictor

MODEL = os.path.join(HERE, "models", "humanoid_temporal.xml")
DATASET = os.path.join(HERE, "data", "temporal_e2e_dataset.npz")
WEIGHTS = os.path.join(HERE, "models_trained", "temporal_e2e.npz")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--collect", action="store_true")
    p.add_argument("--train", action="store_true")
    p.add_argument("--demo", action="store_true")
    p.add_argument("--episodes", type=int, default=30)
    a = p.parse_args()
    if a.collect:
        os.makedirs(os.path.dirname(DATASET), exist_ok=True)
        collect(MODEL, DATASET, n_episodes=a.episodes)
    elif a.train:
        os.makedirs(os.path.dirname(WEIGHTS), exist_ok=True)
        train(DATASET, WEIGHTS)
    elif a.demo:
        sim = TemporalE2ESim(MODEL)
        pred = TemporalPredictor(WEIGHTS)
        sim.reset()
        for _ in range(200): sim.step()
        sim.apply_push((120, 0, 0), duration_steps=40)
        for i in range(1000):
            img = sim.render_front(gray=True)
            pred.push_frame(img)
            target = pred.predict()
            sim.step(target=target)
            if sim.fallen(): print(f"跌倒 @ {i}"); break
        sim.close()
    else:
        p.print_help()


if __name__ == "__main__":
    main()
