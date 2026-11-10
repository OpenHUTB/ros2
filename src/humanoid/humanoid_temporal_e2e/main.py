#!/usr/bin/env python3
"""main.py —— 多帧时序端到端视觉平衡控制 standalone 入口

用法::

    python3 main.py --check                     # 7 项自检
    python3 main.py --collect --samples 2000    # 采集行为克隆数据集
    python3 main.py --train --epochs 80         # 训练 + 常数基线对照
    python3 main.py --eval                      # 闭环评估（teacher / const / net）
    python3 main.py --demo --force 50 --gif demo.gif
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from humanoid_temporal_e2e.sim import (TemporalE2ESim, DEFAULT_STAND_TARGET,
                                       SQUAT_TARGET, INIT_QPOS, JOINT_NAMES)
from humanoid_temporal_e2e.dataset import collect, feat_from_gray, N_FRAMES
from humanoid_temporal_e2e.train import train, closed_loop_eval
from humanoid_temporal_e2e.predictor import TemporalPredictor

MODEL = os.path.join(HERE, "models", "humanoid_temporal.xml")
DATASET = os.path.join(HERE, "data", "temporal_e2e_dataset.npz")
WEIGHTS = os.path.join(HERE, "models_trained", "temporal_e2e.npz")


# --------------------------------------------------------------------------- 自检
def _check():
    ok = True

    def line(name, passed, detail="", skip=False):
        nonlocal ok
        tag = "SKIP" if skip else ("PASS" if passed else "FAIL")
        if not passed and not skip:
            ok = False
        print(f"  [{tag}] {name}" + (f" —— {detail}" if detail else ""))
        return passed

    print("[check] 7 项自检")
    sim = TemporalE2ESim(MODEL)
    line("1 mujoco 模型可加载", True, f"nq={sim.model.nq} nu={sim.model.nu} "
                                   f"dt={sim.timestep}")

    # 2 站姿：平衡开必须站住，平衡关必须露馅（否则说明平衡层没起作用）
    mx_on = mx_off = 0.0
    for balance in (True, False):
        sim.reset()
        mx = 0.0
        for _ in range(1500):
            sim.step(balance=balance)
            mx = max(mx, abs(np.degrees(sim.torso_pose()[0])))
        if balance:
            mx_on = mx
        else:
            mx_off = mx
    line("2 站姿 3 s 最大倾角 < 5°", mx_on < 5.0, f"{mx_on:.2f}°")
    line("   （对照）关平衡 > 20°", mx_off > 20.0, f"{mx_off:.2f}°")

    # 3 图像特征
    sim.reset()
    feats = [feat_from_gray(sim.render_front()) for _ in range(3)]
    f0 = np.stack(feats)
    line("3 图像特征 256 维且非恒定", f0.shape[1] == 256 and f0.std() > 1e-3,
         f"dim={f0.shape[1]} std={f0.std():.4f}")

    # 4 时序缓冲：3 帧后 ready，输出维度 15
    if os.path.exists(WEIGHTS):
        p = TemporalPredictor(WEIGHTS)
        p.reset()
        ready_after = None
        for i in range(N_FRAMES):
            p.push_frame(sim.render_front())
            if p.ready and ready_after is None:
                ready_after = i + 1
        out = p.predict()
        line("4 3 帧缓冲就绪且输出 15 维", ready_after == N_FRAMES and out.shape == (15,),
             f"ready_after={ready_after}, out={out.shape}")
        drift = float(np.abs(out - DEFAULT_STAND_TARGET).max())
        line("5 权重输出与站姿同量纲", drift < 1.5, f"max|out-stand|={drift:.3f} rad")
    else:
        line("4 3 帧缓冲就绪且输出 15 维", True, "缺权重，逻辑未验", skip=True)
        line("5 权重输出与站姿同量纲", True, "缺权重", skip=True)

    # 6 数据集
    if os.path.exists(DATASET):
        d = np.load(DATASET)
        y = d["y"]
        nvar = int((y.std(axis=0) > 1e-3).sum())
        line("6 数据集标签非恒定", nvar >= 4,
             f"X={d['X'].shape} 变化维 {nvar}/15")
    else:
        line("6 数据集标签非恒定", True, "未采集", skip=True)

    # 7 PD 力矩限幅
    sim.reset()
    tau = 60.0 * (DEFAULT_STAND_TARGET - sim.data.qpos[7:22])
    line("7 PD 力矩在 ±280 内", float(np.abs(tau).max()) <= 280.0,
         f"max={np.abs(tau).max():.1f}")
    sim.close()
    print("[check] 全部通过" if ok else "[check] 存在失败项")
    return 0 if ok else 1


# --------------------------------------------------------------------------- 演示
def _demo(force=50.0, duration=40, max_steps=700, gif=None, every=2,
          seconds_per_frame=0.05, assist=True, seed=0):
    """施力 → 网络逐帧输出 15 维关节目标 → overview 机位逐帧真实渲染录 gif。"""
    import imageio
    from PIL import Image, ImageDraw

    sim = TemporalE2ESim(MODEL)
    pred = TemporalPredictor(WEIGHTS)
    rng = np.random.default_rng(seed)

    sim.reset()
    sim.settle(300)          # 老师站定
    pred.reset()
    for _ in range(N_FRAMES):
        pred.push_frame(sim.render_front())

    frames = []

    def snapshot(tag, extra=""):
        rgb = sim.render_rgb(240, 320, camera="overview")
        im = Image.fromarray(rgb)
        d = ImageDraw.Draw(im)
        d.rectangle([0, 0, 178, 44], fill=(0, 0, 0))
        p, _ = sim.torso_pose()
        d.text((4, 4), f"t={tag}", fill=(200, 255, 200))
        d.text((4, 16), f"pitch={np.degrees(p):+.1f}deg", fill=(255, 220, 120))
        d.text((4, 28), extra or "policy: temporal net", fill=(190, 190, 190))
        frames.append(np.array(im))

    snapshot("before push")
    angle = float(rng.uniform(0, 360))
    fvec = np.array([force * np.cos(np.radians(angle)),
                     force * np.sin(np.radians(angle)), 0.0])
    print(f"[demo] push {force:.0f} N @ {angle:.0f}deg")

    fallen_at = None
    for i in range(max_steps):
        if i < duration:
            sim.set_push(fvec)
        else:
            sim.clear_push()
        img = sim.render_front()
        pred.push_frame(img)
        target = pred.predict()
        sim.step(target=target, balance=assist)
        if i % every == 0:
            snapshot(f"{i}", f"knee={target[2]:+.2f} ank={target[3]:+.2f}")
        if sim.fallen():
            fallen_at = i
            break
    sim.clear_push()
    snapshot("end", "fallen" if fallen_at is not None else "still standing")

    print(f"[demo] {'跌倒 @ ' + str(fallen_at) if fallen_at is not None else '未跌倒'}，"
          f"共 {len(frames)} 帧")

    if gif:
        os.makedirs(os.path.dirname(gif) or ".", exist_ok=True)
        imageio.mimsave(gif, frames, duration=seconds_per_frame, loop=0)
        print(f"[demo] gif -> {gif}  ({os.path.getsize(gif) / 1e6:.2f} MB)")
    sim.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--check", action="store_true", help="7 项自检")
    p.add_argument("--collect", action="store_true", help="采集数据集")
    p.add_argument("--train", action="store_true", help="训练")
    p.add_argument("--eval", action="store_true", help="闭环评估")
    p.add_argument("--demo", action="store_true", help="演示")
    p.add_argument("--samples", type=int, default=2000, help="目标样本数")
    p.add_argument("--episodes", type=int, default=200, help="采集最大 episode 数")
    p.add_argument("--epochs", type=int, default=80)
    p.add_argument("--force", type=float, default=50.0, help="演示推力 N")
    p.add_argument("--steps", type=int, default=700)
    p.add_argument("--gif", default=None)
    p.add_argument("--eval-episodes", type=int, default=3)
    p.add_argument("--no-assist", action="store_true", help="演示时关掉踝策略（纯网络闭环）")
    a = p.parse_args()

    if a.check:
        return _check()
    if a.collect:
        os.makedirs(os.path.dirname(DATASET), exist_ok=True)
        collect(MODEL, DATASET, n_samples=a.samples, max_episodes=a.episodes)
    elif a.train:
        os.makedirs(os.path.dirname(WEIGHTS), exist_ok=True)
        train(DATASET, WEIGHTS, epochs=a.epochs)
    elif a.eval:
        closed_loop_eval(WEIGHTS, MODEL, episodes=a.eval_episodes)
    elif a.demo:
        _demo(force=a.force, max_steps=a.steps, gif=a.gif, assist=not a.no_assist)
    else:
        p.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
