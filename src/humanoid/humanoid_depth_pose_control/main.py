#!/usr/bin/env python3
"""main.py —— 深度图姿态估计与平衡控制 standalone 入口"""
from __future__ import annotations
import argparse, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from humanoid_depth_pose_control.sim import DepthPoseSim
from humanoid_depth_pose_control.dataset import collect
from humanoid_depth_pose_control.train import train
from humanoid_depth_pose_control.predictor import DepthPosePredictor
from humanoid_depth_pose_control.controller import ProportionalController

MODEL = os.path.join(HERE, "models", "humanoid_depth.xml")
DATASET = os.path.join(HERE, "data", "depth_pose_dataset.npz")
WEIGHTS = os.path.join(HERE, "models_trained", "depth_pose_mlp.npz")
_CHECK_W = os.path.join(HERE, "data", "_check_w.npz")


def run_check():
    print("="*66); print("  深度图姿态估计与平衡控制 自检"); print("="*66)
    ok = True
    def check(name, passed, detail=""):
        nonlocal ok
        ok = ok and bool(passed)
        print(f"[{'PASS' if passed else 'FAIL'}] {name} {detail}")

    sim = DepthPoseSim(MODEL)
    check("模型加载", sim.model.nu == 15, f"(nu={sim.model.nu})")
    sim.reset()
    for _ in range(300): sim.step()
    check("站姿稳定", not sim.fallen(), f"(root_z={sim.root_z:.3f})")
    depth = sim.render_depth()
    check("深度图渲染", depth.shape == (64, 64), f"(shape={depth.shape})")

    os.makedirs(os.path.dirname(DATASET), exist_ok=True)
    collect(MODEL, DATASET, n_samples=200, verbose=False)
    check("数据集采集", os.path.isfile(DATASET))

    log = train(DATASET, _CHECK_W, epochs=10, verbose=False)
    check("训练收敛", log["final_val_mse"] < 0.01,
          f"(val_mse={log['final_val_mse']:.6f})")

    pred = DepthPosePredictor(_CHECK_W)
    p, r, y = pred.predict_from_depth(depth)
    check("推理输出", abs(p) < 0.5 and abs(r) < 0.5,
          f"(pitch={p:.3f}, roll={r:.3f})")

    ctrl = ProportionalController()
    t = ctrl.target_for(p, r, y)
    check("控制器输出", t.shape == (15,))

    sim.close()
    try: os.remove(_CHECK_W)
    except OSError: pass
    print(f"\n结论：自检{'通过' if ok else '存在问题'}。")
    return 0 if ok else 1


def run_demo(force=120, direction=0):
    sim = DepthPoseSim(MODEL)
    pred = DepthPosePredictor(WEIGHTS)
    ctrl = ProportionalController()
    sim.reset()
    for _ in range(200): sim.step()
    rad = np.radians(direction)
    sim.apply_push((force*np.cos(rad), force*np.sin(rad), 0), duration_steps=40)
    for i in range(1500):
        depth = sim.render_depth()
        p, r, y = pred.predict_from_depth(depth)
        target = ctrl.target_for(p, r, y)
        sim.step(target=target)
        if sim.fallen():
            print(f"  [demo] 跌倒 @ step {i}")
            break
        if i % 50 == 0:
            print(f"  t={sim.data.time:.2f}s pitch={p:.3f} roll={r:.3f} z={sim.root_z:.3f}")
    result = {"fallen": sim.fallen(), "final_z": sim.root_z}
    sim.close()
    print(json.dumps(result, indent=2))
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--check", action="store_true")
    p.add_argument("--collect", action="store_true")
    p.add_argument("--train", action="store_true")
    p.add_argument("--demo", action="store_true")
    p.add_argument("--samples", type=int, default=2000)
    p.add_argument("--force", type=float, default=120)
    p.add_argument("--direction", type=float, default=0)
    a = p.parse_args()
    if a.check: sys.exit(run_check())
    if a.collect:
        os.makedirs(os.path.dirname(DATASET), exist_ok=True)
        collect(MODEL, DATASET, n_samples=a.samples)
    elif a.train:
        os.makedirs(os.path.dirname(WEIGHTS), exist_ok=True)
        train(DATASET, WEIGHTS)
    elif a.demo:
        run_demo(a.force, a.direction)
    else:
        p.print_help()


if __name__ == "__main__":
    main()
