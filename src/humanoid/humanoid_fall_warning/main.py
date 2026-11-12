#!/usr/bin/env python3
"""main.py —— 多模态跌倒预警 standalone 入口

子命令：
    --check                       7 项自检（模型/站姿/图像/数据集/权重/推理/控制器）
    --collect --samples N         采集 图像特征+IMU → 未来是否跌倒 的数据集
    --train --epochs N            训练并打印验证集混淆矩阵与基率对照
    --demo --force F --gif P      施加推力，逐帧真实渲染录 gif，预警时紧急下蹲
"""
from __future__ import annotations
import argparse, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from humanoid_fall_warning.sim import (FallWarningSim, DEFAULT_STAND_TARGET,
                                       SQUAT_TARGET, INIT_QPOS)
from humanoid_fall_warning.dataset import collect, image_feature, feat_from_gray
from humanoid_fall_warning.train import train, evaluate, format_report
from humanoid_fall_warning.predictor import FallWarningPredictor

MODEL = os.path.join(HERE, "models", "humanoid_warning.xml")
DATASET = os.path.join(HERE, "data", "fall_warning_dataset.npz")
WEIGHTS = os.path.join(HERE, "models_trained", "fall_warning.npz")
WARN_THRESHOLD = 0.6


def _check():
    ok = True
    def line(name, passed, detail="", skip=False):
        nonlocal ok
        if skip:
            # 数据集和权重属于"要先跑一步才存在"的产物，缺了不算失败，只提示
            print(f"[SKIP] {name} {detail}")
            return
        ok = ok and passed
        print(f"[{'PASS' if passed else 'FAIL'}] {name} {detail}")

    sim = FallWarningSim(MODEL)
    line("模型加载", sim.model.nu == 15, f"(nu={sim.model.nu}, nq={sim.model.nq})")

    # 站姿稳定：必须跑满 3 s。原自检只跑 0.6 s，正好落在"还没倒"的窗口里，
    # 所以旧站姿（3 s 内前倾 89.8°）也能拿到 PASS。
    sim.reset()
    max_tilt = 0.0
    for _ in range(1500):
        sim.step()
        p, r = sim.torso_pose()
        max_tilt = max(max_tilt, abs(np.degrees(p)), abs(np.degrees(r)))
    line("站姿稳定(3s)", (not sim.fallen()) and max_tilt < 5.0,
         f"(max_tilt={max_tilt:.2f}°, root_z={sim.root_z:.3f})")

    img = sim.render_front(gray=True)
    line("前端图像渲染", img.shape == (64, 64) and float(img.max()) > 0.1,
         f"(shape={img.shape}, max={float(img.max()):.3f})")
    feat = feat_from_gray(img)
    line("图像特征维度", feat.shape == (256,), f"(shape={feat.shape})")

    if os.path.isfile(DATASET):
        d = np.load(DATASET)
        X, y = d["X"], d["y"]
        uniq = len(np.unique(X[:, :256].round(4), axis=0))
        # 退化判据：输入常量或标签常量 —— 旧数据集正是这样（正例率 0、图像特征全同）
        line("数据集非退化", X.shape[1] == 258 and float(y.mean()) > 0.1
             and float(y.mean()) < 0.9 and uniq > 20,
             f"(X={X.shape}, pos_rate={float(y.mean()):.2f}, 唯一图像行={uniq})")
    else:
        line("数据集非退化", False, "(data/fall_warning_dataset.npz 不存在，先跑 --collect)", skip=True)

    if os.path.isfile(WEIGHTS):
        pred = FallWarningPredictor(WEIGHTS)
        pn, pf = pred.predict(feat, *sim.torso_pose())
        # 站立状态不应触发预警；旧权重对真实站立输入输出 p_fall=0.754（永远喊跌倒）
        line("推理输出", abs(pn + pf - 1.0) < 1e-3 and pf <= WARN_THRESHOLD,
             f"(p_normal={pn:.3f}, p_fall={pf:.3f})")
    else:
        line("推理输出", False, "(models_trained/fall_warning.npz 不存在，先跑 --train)", skip=True)
    line("控制器输出", SQUAT_TARGET.shape == (15,) and DEFAULT_STAND_TARGET.shape == (15,)
         and SQUAT_TARGET[3] > 0 and SQUAT_TARGET[8] > 0,
         f"(下蹲踝背屈={SQUAT_TARGET[3]:.2f})")

    sim.close()
    print("结论：" + ("自检通过。" if ok else "存在失败项。"))
    return 0 if ok else 1


def _demo(force=150.0, duration=40, max_steps=900, gif=None, every=3,
          seconds_per_frame=0.05):
    import imageio
    from PIL import Image, ImageDraw
    sim = FallWarningSim(MODEL)
    pred = FallWarningPredictor(WEIGHTS)
    sim.reset()
    for _ in range(300):
        sim.step()

    frames = []
    # 先录一帧"施力前"的站立画面，作为动图的对照起点
    if gif:
        rgb = sim.render_rgb(240, 320, camera="overview")
        im = Image.fromarray(rgb)
        d = ImageDraw.Draw(im)
        d.rectangle([0, 0, 150, 44], fill=(0, 0, 0))
        d.text((4, 4), "p_normal=--", fill=(180, 255, 180))
        d.text((4, 18), "p_fall=--", fill=(255, 220, 120))
        d.text((4, 30), "STAND (before push)", fill=(200, 200, 200))
        frames.append(np.array(im))

    sim.apply_push((force, 0.0, 0.0), duration_steps=duration)

    first_warn, fallen_at = None, None
    for i in range(max_steps):
        feat = image_feature(sim)
        pitch, roll = sim.torso_pose()
        p_normal, p_fall = pred.predict(feat, pitch, roll)
        warn = p_fall > WARN_THRESHOLD
        if warn and first_warn is None:
            first_warn = i
            print(f"  [demo] step {i}: FALL WARNING p_fall={p_fall:.2f} → 紧急下蹲")
        target = SQUAT_TARGET if warn else DEFAULT_STAND_TARGET
        sim.step(target=target)
        if gif and i % every == 0:
            rgb = sim.render_rgb(240, 320, camera="overview")
            im = Image.fromarray(rgb)
            d = ImageDraw.Draw(im)
            d.rectangle([0, 0, 150, 44], fill=(0, 0, 0))
            d.text((4, 4), f"p_normal={p_normal:.2f}", fill=(180, 255, 180))
            d.text((4, 18), f"p_fall={p_fall:.2f}", fill=(255, 220, 120))
            d.text((4, 30), "SQUAT" if warn else "STAND", fill=(255, 90, 90) if warn else (200, 200, 200))
            frames.append(np.array(im))
        if sim.fallen():
            fallen_at = i
            break
    sim.close()

    survived = (fallen_at if fallen_at is not None else max_steps) * sim.timestep
    print(f"  [demo] force={force:.0f} N / {duration} 步 | 预警步={first_warn} | "
          f"倒地步={fallen_at} | 存活 {survived:.2f} s | 录帧 {len(frames)}")
    if gif:
        os.makedirs(os.path.dirname(os.path.abspath(gif)) or ".", exist_ok=True)
        imageio.mimsave(gif, frames, duration=seconds_per_frame)
        print(f"  [demo] gif 已写出：{gif}（{os.path.getsize(gif)/1024:.0f} KB，"
              f"{len(frames)} 帧，画面为 sim.render_rgb 逐帧真实渲染）")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--check", action="store_true")
    p.add_argument("--collect", action="store_true")
    p.add_argument("--train", action="store_true")
    p.add_argument("--demo", action="store_true")
    p.add_argument("--samples", type=int, default=1500)
    p.add_argument("--epochs", type=int, default=80)
    p.add_argument("--force", type=float, default=150.0)
    p.add_argument("--gif", type=str, default=None)
    a = p.parse_args()
    if a.check:
        raise SystemExit(_check())
    elif a.collect:
        os.makedirs(os.path.dirname(DATASET), exist_ok=True)
        collect(MODEL, DATASET, n_samples=a.samples)
    elif a.train:
        os.makedirs(os.path.dirname(WEIGHTS), exist_ok=True)
        train(DATASET, WEIGHTS, epochs=a.epochs)
    elif a.demo:
        _demo(force=a.force, gif=a.gif)
    else:
        p.print_help()


if __name__ == "__main__":
    main()
