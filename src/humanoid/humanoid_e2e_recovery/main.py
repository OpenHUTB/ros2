#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py —— 人形机器人端到端视觉平衡控制（standalone 入口）
============================================================
用法:
    python3 main.py --collect --episodes 60   # 采集专家数据
    python3 main.py --train                   # 训练端到端网络
    python3 main.py --demo --force 50         # 端到端演示
    python3 main.py --check                   # 自检（含判别力对照）
"""
from __future__ import annotations
import argparse, json, os, shutil, sys, tempfile
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from humanoid_e2e_recovery.sim import E2ESim, DEFAULT_STAND_TARGET
from humanoid_e2e_recovery.dataset import collect
from humanoid_e2e_recovery.train import train
from humanoid_e2e_recovery.predictor import E2EPredictor
from humanoid_e2e_recovery.expert import expert_action

MODEL_PATH = os.path.join(HERE, "models", "humanoid_camera.xml")
DATASET = os.path.join(HERE, "data", "e2e_dataset.npz")
WEIGHTS = os.path.join(HERE, "models_trained", "e2e_policy.npz")

STAND_SECONDS = 10.0     # 站姿判据：必须稳住 10 s（评审要求）
CTRL_DT = 0.002


def _run_episode(sim, policy, seconds=STAND_SECONDS, render_every=2, enable_render=True):
    """用给定策略跑 seconds 秒。policy(sim) -> 15 维力矩增量 or None。"""
    steps = int(seconds / CTRL_DT)
    max_tilt = 0.0
    fell_at = None
    for i in range(steps):
        if policy is None:
            extra = None
        elif enable_render:
            extra = policy(sim, i % render_every == 0)
        else:
            extra = policy(sim, True)
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=extra)
        max_tilt = max(max_tilt, abs(sim.torso_tilt()))
        if sim.fallen():
            fell_at = sim.data.time
            break
    return fell_at, max_tilt


def _nn_policy(predictor):
    """端到端策略：只在需要时渲染图像，其余步沿用上一次输出。"""
    cache = {"u": np.zeros(15)}

    def policy(sim, refresh):
        if refresh:
            cache["u"] = predictor.predict_from_image(sim.render_front(gray=True))
        return cache["u"]
    return policy


def _zero_policy():
    def policy(sim, refresh):
        return np.zeros(15)
    return policy


def _expert_policy():
    def policy(sim, refresh):
        return expert_action(sim)
    return policy


def run_check():
    print("=" * 70)
    print("  端到端视觉平衡控制 自检（判据：站姿须稳 %.0f s；全零权重必须判 FAIL）" % STAND_SECONDS)
    print("=" * 70)
    ok = True

    def check(name, passed, detail=""):
        nonlocal ok
        ok = ok and bool(passed)
        print(f"[{'PASS' if passed else 'FAIL'}] {name} {detail}")

    # 1. 模型加载
    sim = E2ESim(MODEL_PATH)
    check("模型加载", sim.model.nq > 0 and sim.model.nu == 15,
          f"(nq={sim.model.nq}, nu={sim.model.nu}, 质量={sim.model.body_mass.sum():.2f}kg)")

    # 2. 相机渲染
    sim.reset()
    img = sim.render_front(gray=True)
    check("相机渲染", img.shape == (64, 64), f"(shape={img.shape})")

    # 3. 专家控制器站姿（teacher 本身必须先站得住）
    sim.reset()
    fell, tilt = _run_episode(sim, _expert_policy(), STAND_SECONDS, enable_render=False)
    check("专家控制器站姿 %.0fs" % STAND_SECONDS, fell is None,
          f"({'跌倒于 %.2fs' % fell if fell else '站稳'}，最大倾角 {np.degrees(tilt):.1f}°)")

    # 4. 权重文件
    check("权重文件存在", os.path.exists(WEIGHTS), f"({os.path.basename(WEIGHTS)})")
    if not os.path.exists(WEIGHTS):
        sim.close()
        print("=" * 70); print("  自检结果: 存在失败项"); print("=" * 70)
        return 1

    predictor = E2EPredictor(WEIGHTS)
    sim.reset()
    out = predictor.predict_from_image(sim.render_front(gray=True))
    check("MLP 推理输出", out.shape == (15,), f"(shape={out.shape})")
    check("推理非常量输出", float(np.std(out)) > 1e-6,
          f"(std={np.std(out):.3f} N·m，非全零/非死网络)")

    # 5. 判别力对照：全零权重必须站不住，否则说明自检无判别力
    tmpdir = tempfile.mkdtemp(prefix="e2e_zero_")
    zero_w = os.path.join(tmpdir, "zero.npz")
    d = dict(np.load(WEIGHTS))
    for k in ("W1", "W2", "W3"):
        d[k] = np.zeros_like(d[k])
    for k in ("b1", "b2", "b3"):
        d[k] = np.zeros_like(d[k])
    np.savez(zero_w, **d)
    zero_pred = E2EPredictor(zero_w)
    sim.reset()
    zfell, _ = _run_episode(sim, _nn_policy(zero_pred), STAND_SECONDS)
    check("全零权重必须失稳（判别力）", zfell is not None,
          f"(全零权重 {('%.2fs 即跌倒' % zfell) if zfell else '竟然也站满 %.0fs → 自检无判别力' % STAND_SECONDS})")
    shutil.rmtree(tmpdir, ignore_errors=True)

    # 6. 端到端站立
    sim.reset()
    fell, tilt = _run_episode(sim, _nn_policy(predictor), STAND_SECONDS)
    check("端到端站姿 %.0fs" % STAND_SECONDS, fell is None,
          f"({'跌倒于 %.2fs' % fell if fell else '站稳'}，最大倾角 {np.degrees(tilt):.1f}°)")

    sim.close()
    print("=" * 70)
    print(f"  自检结果: {'全部通过' if ok else '存在失败项'}")
    print("=" * 70)
    return 0 if ok else 1


def run_demo(force=50, direction_deg=0, seconds=STAND_SECONDS, gif_path=None):
    sim = E2ESim(MODEL_PATH)
    predictor = E2EPredictor(WEIGHTS)
    policy = _nn_policy(predictor)

    # 先空站，确认站姿本身没问题；再施加推力考察恢复
    sim.reset()
    fell0, tilt0 = _run_episode(sim, policy, 3.0)
    if fell0 is not None:
        sim.close()
        return {"stage": "stand", "fell": float(fell0), "max_tilt_deg": float(np.degrees(tilt0))}

    rad = np.radians(direction_deg)
    push = (force * np.cos(rad), force * np.sin(rad), 0)
    sim.apply_push(push, steps=40, target=DEFAULT_STAND_TARGET, balance=expert_action)

    frames = []
    max_tilt = 0.0
    fell = None
    steps = int(seconds / CTRL_DT)
    for i in range(steps):
        if i % 2 == 0:
            img = sim.render_front(gray=True)
            u = predictor.predict_from_image(img)
            if gif_path is not None and i % 6 == 0:
                frames.append(sim.render_front(gray=False).astype(np.uint8))
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
        tilt = abs(sim.torso_tilt())
        max_tilt = max(max_tilt, tilt)
        if sim.fallen():
            fell = float(sim.data.time)
            break
        if i % 100 == 0:
            print(f"  t={sim.data.time:.2f}s 倾角={np.degrees(tilt):.1f}° 根高={sim.root_z:.3f}m")

    sim.close()
    result = {"force": force, "dir": direction_deg, "max_tilt_deg": float(np.degrees(max_tilt)),
              "fell": fell, "final_z": None, "gif_frames": len(frames)}
    print(f"\n[结果] {'跌倒于 %.2fs' % fell if fell else '恢复并站稳 %.0fs' % seconds} "
          f"| 最大倾角 {np.degrees(max_tilt):.1f}°")
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--check", action="store_true")
    p.add_argument("--collect", action="store_true")
    p.add_argument("--train", action="store_true")
    p.add_argument("--demo", action="store_true")
    p.add_argument("--episodes", type=int, default=60)
    p.add_argument("--force", type=float, default=50)
    p.add_argument("--direction", type=float, default=0)
    p.add_argument("--seconds", type=float, default=STAND_SECONDS)
    a = p.parse_args()
    if a.check:
        sys.exit(run_check())
    if a.collect:
        os.makedirs(os.path.dirname(DATASET), exist_ok=True)
        collect(MODEL_PATH, DATASET, n_episodes=a.episodes)
    elif a.train:
        try:
            train(DATASET, WEIGHTS)
        except (FileNotFoundError, ValueError) as e:
            print(f"[错误] {e}", file=sys.stderr)
            sys.exit(2)
    elif a.demo:
        print(json.dumps(run_demo(a.force, a.direction, a.seconds), indent=2, default=str))
    else:
        p.print_help()


if __name__ == "__main__":
    main()
