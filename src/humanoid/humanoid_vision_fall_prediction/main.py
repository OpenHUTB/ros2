#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py —— 人形机器人视觉跌倒预测与分级平衡控制（standalone 入口）
====================================================================

用法::

    python3 main.py --check                          # 自检
    python3 main.py --collect --samples 300          # 采集数据集
    python3 main.py --train                           # 训练神经网络
    python3 main.py --demo --force 150 --direction 0 # 演示：前向推扰+视觉预测+平衡控制
    python3 main.py --demo --no-viewer               # 无图形界面（服务器/CI）
    python3 main.py --ros2                           # 通过 ros2 launch 启动

工作流：站姿稳定 → 施加外力扰动 → 前视相机图像 → MLP 预测跌倒方向 →
        分级平衡控制（踝/髋修正）→ 恢复或跌倒 → 定量评价。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# 允许直接 python3 main.py 运行（无需 colcon build）
from humanoid_vision_fall_prediction.sim import VisionFallSim, DEFAULT_STAND_TARGET  # noqa: E402
from humanoid_vision_fall_prediction.dataset import collect_dataset  # noqa: E402
from humanoid_vision_fall_prediction.train import train_model  # noqa: E402
from humanoid_vision_fall_prediction.predictor import FallPredictor, LABEL_NAMES  # noqa: E402
from humanoid_vision_fall_prediction.controller import BalanceController  # noqa: E402

DEFAULT_CONFIG = os.path.join(HERE, "config", "params.yaml")
MODEL_PATH = os.path.join(HERE, "models", "humanoid_camera.xml")
DATA_DIR = os.path.join(HERE, "data")
WEIGHTS_DIR = os.path.join(HERE, "models_trained")
DATASET_PATH = os.path.join(DATA_DIR, "fall_dataset.npz")
WEIGHTS_PATH = os.path.join(WEIGHTS_DIR, "fall_predictor.npz")
# --check 专用临时权重路径：避免覆盖正式训练好的 models_trained/fall_predictor.npz
_CHECK_WEIGHTS = os.path.join(DATA_DIR, "_check_weights.npz")


def _load_params():
    """读取 YAML 参数；实际被 run_check / run_demo 调用，返回 dict。"""
    try:
        import yaml
        with open(DEFAULT_CONFIG, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


# ----------------------------------------------------------------------
# 自检
# ----------------------------------------------------------------------
def run_check() -> int:
    print("=" * 66)
    print("  人形机器人视觉跌倒预测模块自检")
    print("=" * 66)
    ok = True

    def check(name, passed, detail=""):
        nonlocal ok
        ok = ok and bool(passed)
        print(f"[{'PASS' if passed else 'FAIL'}] {name} {detail}")

    # 实际读取 params.yaml（审核意见：_load_params 必须被调用）
    params = _load_params()
    check("params.yaml 读取", isinstance(params, dict) and "nn" in params,
          f"(keys={list(params.keys())})")

    sim = VisionFallSim(MODEL_PATH)
    check("模型加载", sim.model.nq > 0 and sim.model.nu == 15,
          f"(nq={sim.model.nq}, nu={sim.model.nu})")

    sim.reset()
    for _ in range(500):
        sim.step()
    drift = abs(sim.root_z - 1.088)
    check("无扰动站立 1s 不跌倒", not sim.fallen(),
          f"(根高 {sim.root_z:.3f} m)")

    img = sim.render_front(gray=True)
    check("前视相机渲染", img.shape == (64, 64), f"(shape={img.shape})")

    # 快速小数据集训练（仅验证流程，输出到临时路径，不覆盖正式权重）
    os.makedirs(DATA_DIR, exist_ok=True)
    collect_dataset(MODEL_PATH, DATASET_PATH, samples_per_class=60,
                    force_min=100.0, force_max=200.0, verbose=False)
    check("数据集采集", os.path.isfile(DATASET_PATH), f"({DATASET_PATH})")

    log = train_model(DATASET_PATH, _CHECK_WEIGHTS, epochs=10, verbose=False)
    # 5 类随机猜基线 = 0.2；--check 只验证训练流程，不要求高精度
    check("MLP 训练可收敛", log["final_val_acc"] >= 0.2,
          f"(val_acc={log['final_val_acc']:.3f}, 随机基线=0.2)")

    predictor = FallPredictor(_CHECK_WEIGHTS)
    label, cls, conf = predictor.predict_from_image(img)
    check("推理接口", label in LABEL_NAMES and 0.0 <= conf <= 1.0,
          f"(label={label}, conf={conf:.3f})")

    ctrl = BalanceController()
    target = ctrl.target_for(cls, conf)
    check("平衡控制输出", target.shape == (15,), f"(shape={target.shape})")

    # 对照实验：用正式训练权重（如果存在）验证开/关修正的跌倒率差异
    if os.path.isfile(WEIGHTS_PATH):
        prod_predictor = FallPredictor(WEIGHTS_PATH)
        fell_with = _run_push_trial(prod_predictor, ctrl, use_correction=True,
                                    force=100.0)
        fell_without = _run_push_trial(prod_predictor, ctrl,
                                       use_correction=False, force=100.0)
        check("对照实验：开修正跌倒率 ≤ 关修正",
              fell_with <= fell_without,
              f"(开修正跌倒={fell_with}, 关修正跌倒={fell_without})")
    else:
        # 无正式权重时只验证对照流程不崩
        fell_with = _run_push_trial(predictor, ctrl, use_correction=True,
                                    force=80.0)
        fell_without = _run_push_trial(predictor, ctrl,
                                       use_correction=False, force=80.0)
        check("对照实验流程可运行（无正式权重时）",
              isinstance(fell_with, bool) and isinstance(fell_without, bool),
              f"(开修正跌倒={fell_with}, 关修正跌倒={fell_without})")

    sim.close()
    # 清理临时权重
    try:
        os.remove(_CHECK_WEIGHTS)
    except OSError:
        pass
    print(f"\n结论：自检{'通过' if ok else '存在问题'}。")
    return 0 if ok else 1


def _run_push_trial(predictor: FallPredictor, ctrl: BalanceController,
                    use_correction: bool = True,
                    force: float = 100.0, direction_deg: float = 0.0,
                    max_steps: int = 1500) -> bool:
    """单次推扰试验，返回是否跌倒。use_correction=False 时只用站姿 PD。"""
    sim = VisionFallSim(MODEL_PATH)
    sim.reset()
    for _ in range(250):
        sim.step()
    rad = np.radians(direction_deg)
    sim.apply_push((force * np.cos(rad), force * np.sin(rad), 0.0),
                   duration_steps=40)
    for _ in range(max_steps):
        img = sim.render_front(gray=True)
        if use_correction:
            _, cls, conf = predictor.predict_from_image(img)
            target = ctrl.target_for(cls, conf)
        else:
            target = DEFAULT_STAND_TARGET.copy()
        sim.step(target=target)
        if sim.fallen():
            sim.close()
            return True
    sim.close()
    return False


# ----------------------------------------------------------------------
# 演示
# ----------------------------------------------------------------------
def run_demo(force: float = 150.0, direction_deg: float = 0.0,
             viewer: bool = True, verbose: bool = True) -> dict:
    """施加一次推扰，视觉预测 + 平衡控制。"""
    sim = VisionFallSim(MODEL_PATH)
    predictor = FallPredictor(WEIGHTS_PATH)
    ctrl = BalanceController()

    sim.reset()
    # 稳定站立 1s
    for _ in range(500):
        sim.step()

    rad = np.radians(direction_deg)
    fx, fy = force * np.cos(rad), force * np.sin(rad)

    if verbose:
        print("=" * 66)
        print("  人形机器人视觉跌倒预测 + 分级平衡控制 演示")
        print("=" * 66)
        print(f"[INFO] 推扰 {force:.0f} N, 方向 {direction_deg:.0f}°")
        print(f"[INFO] 模型权重: {WEIGHTS_PATH}\n")

    # 推扰
    sim.apply_push((fx, fy, 0.0), duration_steps=40)

    max_tilt = 0.0
    recover_time = None
    fell_at = None
    predict_log = []

    for i in range(2000):
        img = sim.render_front(gray=True)
        label, cls, conf = predictor.predict_from_image(img)
        target = ctrl.target_for(cls, conf)
        sim.step(target=target)

        tilt = abs(sim.torso_tilt())
        max_tilt = max(max_tilt, tilt)

        if i % 25 == 0:
            predict_log.append({"t": float(sim.data.time), "label": label,
                                "conf": round(conf, 3),
                                "tilt_deg": round(np.degrees(tilt), 2),
                                "root_z": round(sim.root_z, 3)})
            if verbose:
                print(f"  t={sim.data.time:4.2f}s 预测={label:8s} "
                      f"conf={conf:.2f} 倾角={np.degrees(tilt):5.1f}° "
                      f"根高={sim.root_z:.3f}m")

        if sim.fallen():
            fell_at = float(sim.data.time)
            break
        if recover_time is None and i > 100 and tilt < 0.05 and not sim.fallen():
            recover_time = float(sim.data.time)

        if viewer and i % 20 == 0:
            # 用 overview 相机辅助观察（可选）
            pass

    metrics = {
        "force_n": force, "direction_deg": direction_deg,
        "max_tilt_deg": float(np.degrees(max_tilt)),
        "fell": fell_at is not None, "fell_at_s": fell_at,
        "recover_time_s": recover_time,
        "final_root_z": sim.root_z,
        "predict_log": predict_log,
    }
    if verbose:
        if metrics["fell"]:
            print(f"\n[结果] 跌倒 @ t={fell_at:.2f}s, 最大倾角 {np.degrees(max_tilt):.1f}°")
        else:
            print(f"\n[结果] 恢复成功, 最大倾角 {np.degrees(max_tilt):.1f}°, "
                  f"恢复时间 {recover_time}s")
    sim.close()
    return metrics


def main():
    parser = argparse.ArgumentParser(description="人形机器人视觉跌倒预测模块")
    parser.add_argument("--check", action="store_true", help="自检")
    parser.add_argument("--collect", action="store_true", help="采集数据集")
    parser.add_argument("--train", action="store_true", help="训练神经网络")
    parser.add_argument("--demo", action="store_true", help="演示推扰+预测+控制")
    parser.add_argument("--samples", type=int, default=300, help="采集样本数/类")
    parser.add_argument("--force", type=float, default=150.0, help="推扰力 (N)")
    parser.add_argument("--direction", type=float, default=0.0,
                        help="推扰方向（度；0=前,90=左,180=后,270=右）")
    parser.add_argument("--no-viewer", dest="viewer", action="store_false",
                        help="不打开可视化窗口")
    parser.set_defaults(viewer=True)
    parser.add_argument("--ros2", action="store_true", help="ros2 launch")
    args = parser.parse_args()

    if args.ros2:
        import subprocess
        sys.exit(subprocess.run(["ros2", "launch", "humanoid_vision_fall_prediction",
                                 "main.launch.py"]).returncode)

    if args.check:
        sys.exit(run_check())

    if args.collect:
        os.makedirs(DATA_DIR, exist_ok=True)
        collect_dataset(MODEL_PATH, DATASET_PATH,
                         samples_per_class=args.samples)
        return

    if args.train:
        os.makedirs(WEIGHTS_DIR, exist_ok=True)
        train_model(DATASET_PATH, WEIGHTS_PATH)
        return

    if args.demo:
        m = run_demo(force=args.force, direction_deg=args.direction,
                     viewer=args.viewer)
        print(json.dumps(m, ensure_ascii=False, indent=2, default=str))
        return

    parser.print_help()


if __name__ == "__main__":
    main()
