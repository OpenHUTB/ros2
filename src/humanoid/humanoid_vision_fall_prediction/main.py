#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py —— 人形机器人视觉跌倒预测与分级平衡控制（standalone 入口）
====================================================================

用法::

    python3 main.py --check                          # 自检
    python3 main.py --collect --samples 1500         # 采集数据集（总样本数）
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
# --check 专用临时权重/数据路径：避免覆盖正式训练好的
# models_trained/fall_predictor.npz 与 data/fall_dataset.npz
_CHECK_WEIGHTS = os.path.join(DATA_DIR, "_check_weights.npz")
_CHECK_DATASET = os.path.join(DATA_DIR, "_check_dataset.npz")


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
    stand_z0 = float(sim.root_z)
    for _ in range(500):
        sim.step()
    settle_cm = (stand_z0 - sim.root_z) * 100.0
    stand_tilt_deg = float(np.degrees(sim.torso_tilt()))
    # 早期版本只判断 not fallen，另算了一个 drift 却从未用于断言（死变量），
    # 因此“开局前倾 61°、1 s 内沉 14 cm”也被判为 PASS。这里同时约束躯干倾角。
    check("无扰动站立 1s 不跌倒且姿态稳定",
          (not sim.fallen()) and abs(stand_tilt_deg) < 10.0,
          f"(根高 {stand_z0:.3f}→{sim.root_z:.3f} m, 下沉 {settle_cm:.1f} cm, "
          f"躯干倾角 {stand_tilt_deg:.2f}°)")

    img = sim.render_front(gray=True)
    check("前视相机渲染", img.shape == (64, 64), f"(shape={img.shape})")

    # 采集流程验证：写到临时文件，绝不覆盖正式数据集
    # （审阅意见指出旧版 --check 直接把 60 个样本写进 data/fall_dataset.npz）
    collect_dataset(MODEL_PATH, _CHECK_DATASET, total_samples=40,
                    force_min=100.0, force_max=200.0, verbose=False)
    d = np.load(_CHECK_DATASET)
    check("数据集采集", len(d["y"]) == 40 and d["X"].shape[1] == 256,
          f"(n={len(d['y'])}, 分布={np.bincount(d['y']).tolist()})")

    # 收敛性用合成可分数据验证：真实数据集规模小，验证准确率受划分随机性
    # 影响很大，不适合做反向传播正确性的判据。旧断言只要求 val_acc ≥ 0.2
    # （恰好是 5 类随机基线），几乎不构成验证。
    rng = np.random.default_rng(0)
    centers = rng.normal(0.5, 0.25, size=(5, 256))
    syn_x = np.concatenate([centers[c] + rng.normal(0, 0.02, size=(200, 256))
                            for c in range(5)])
    syn_y = np.repeat(np.arange(5), 200)
    np.savez(_CHECK_DATASET, X=syn_x.astype(np.float32), y=syn_y)
    log = train_model(_CHECK_DATASET, _CHECK_WEIGHTS, epochs=30, verbose=False)
    check("MLP 反向传播可收敛（合成可分数据）", log["best_val_acc"] >= 0.9,
          f"(val_acc={log['best_val_acc']:.3f}, 随机基线=0.2)")

    predictor = FallPredictor(_CHECK_WEIGHTS)
    label, cls, conf = predictor.predict_from_image(img)
    check("推理接口", label in LABEL_NAMES and 0.0 <= conf <= 1.0,
          f"(label={label}, conf={conf:.3f})")

    pred_cfg = params.get("predict", {}) or {}
    ctrl = BalanceController(
        gain=float(pred_cfg.get("balance_tau_gain", 0.6)),
        confidence_threshold=float(pred_cfg.get("confidence_threshold", 0.70)))
    target = ctrl.target_for(cls, conf)
    check("平衡控制输出", target.shape == (15,), f"(shape={target.shape})")

    # 对照实验：正式权重下做 2 力 × 4 方向的开/关修正统计
    if os.path.isfile(WEIGHTS_PATH):
        prod_predictor = FallPredictor(WEIGHTS_PATH)
        stats = _ablation(prod_predictor, ctrl, max_steps=800)
        on, off = stats["on"], stats["off"]
        check("对照实验：开修正平均存活步数 ≥ 关修正",
              on["mean_survive"] >= off["mean_survive"],
              f"(开修正 {on['mean_survive']:.0f} 步/跌倒率 {on['fall_rate']*100:.0f}% | "
              f"关修正 {off['mean_survive']:.0f} 步/跌倒率 {off['fall_rate']*100:.0f}% | "
              f"n={on['n']})")
        delta = abs(on["mean_survive"] - off["mean_survive"])
        if delta < 0.05 * max(1.0, off["mean_survive"]):
            print(f"        注：开/关修正仅相差 {delta:.0f} 步（< 5%），说明视觉修正"
                  f"对跌倒结局没有显著影响，原因见 README「已知局限」。")
    else:
        stats = _ablation(predictor, ctrl, forces=(80.0,), max_steps=500)
        check("对照实验流程可运行（无正式权重时）",
              stats["on"]["n"] == stats["off"]["n"] == 4,
              f"(n={stats['on']['n']})")

    sim.close()
    # 清理临时权重与临时数据集
    for path in (_CHECK_WEIGHTS, _CHECK_DATASET):
        try:
            os.remove(path)
        except OSError:
            pass
    print(f"\n结论：自检{'通过' if ok else '存在问题'}。")
    return 0 if ok else 1


def _run_push_trial(predictor: FallPredictor, ctrl: BalanceController,
                    use_correction: bool = True,
                    force: float = 100.0, direction_deg: float = 0.0,
                    max_steps: int = 1500, return_steps: bool = False):
    """单次推扰试验。返回是否跌倒；return_steps=True 时返回存活步数。

    存活步数 ≈ max_steps 表示撑到上限没倒，可直接用于开/关修正的对照统计。
    """
    sim = VisionFallSim(MODEL_PATH)
    sim.reset()
    for _ in range(250):
        sim.step()
    rad = np.radians(direction_deg)
    sim.apply_push((force * np.cos(rad), force * np.sin(rad), 0.0),
                   duration_steps=40)
    for i in range(max_steps):
        img = sim.render_front(gray=True)
        if use_correction:
            _, cls, conf = predictor.predict_from_image(img)
            target = ctrl.target_for(cls, conf)
        else:
            target = DEFAULT_STAND_TARGET.copy()
        sim.step(target=target)
        if sim.fallen():
            sim.close()
            return i if return_steps else True
    sim.close()
    return max_steps if return_steps else False


def _ablation(predictor: FallPredictor, ctrl: BalanceController,
              forces=(80.0, 140.0),
              directions=(0.0, 180.0, 90.0, 270.0),
              max_steps: int = 1200) -> dict:
    """开/关视觉修正的多点对照统计。

    旧版只跑一次 push 并比较 `fell_with <= fell_without`，两边都跌时该条件
    恒成立（True <= True），属于空断言。这里改用“平均存活步数 + 跌倒率”：
    存活步数越大越好，撑满 max_steps 记满分。
    """
    result = {}
    for name, use in (("on", True), ("off", False)):
        survives = []
        for force in forces:
            for deg in directions:
                survives.append(_run_push_trial(
                    predictor, ctrl, use_correction=use, force=force,
                    direction_deg=deg, max_steps=max_steps, return_steps=True))
        fell = sum(1 for s in survives if s < max_steps)
        result[name] = {
            "mean_survive": float(np.mean(survives)),
            "fall_rate": fell / len(survives),
            "n": len(survives),
        }
    return result


# ----------------------------------------------------------------------
# 演示
# ----------------------------------------------------------------------
def run_demo(force: float = 150.0, direction_deg: float = 0.0,
             viewer: bool = True, verbose: bool = True) -> dict:
    """施加一次推扰，视觉预测 + 平衡控制。"""
    sim = VisionFallSim(MODEL_PATH)
    predictor = FallPredictor(WEIGHTS_PATH)
    pred_cfg = _load_params().get("predict", {}) or {}
    ctrl = BalanceController(
        gain=float(pred_cfg.get("balance_tau_gain", 0.6)),
        confidence_threshold=float(pred_cfg.get("confidence_threshold", 0.70)))

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

    # 真正打开 MuJoCo 被动可视化窗口。早期版本这里是 `if viewer: pass`，
    # 什么都没做，所以 --no-viewer 这个开关实际毫无意义。
    viewer_handle = None
    if viewer:
        try:
            import mujoco.viewer
            viewer_handle = mujoco.viewer.launch_passive(sim.model, sim.data)
        except Exception as exc:  # 无显示或该平台不支持 GUI 时降级
            print(f"[WARN] 可视化窗口启动失败（{exc}），改为无窗口运行")
            viewer_handle = None

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

        if viewer_handle is not None and i % 5 == 0:
            try:
                viewer_handle.sync()
            except Exception:
                viewer_handle = None

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
    if viewer_handle is not None:
        try:
            viewer_handle.close()
        except Exception:
            pass
    sim.close()
    return metrics


def _apply_standing_params() -> None:
    """把 params.yaml 的 standing.qpos_target 写入 DEFAULT_STAND_TARGET。

    DEFAULT_STAND_TARGET 是模块级 ndarray，controller 引用的是同一对象，
    因此原地赋值即可让配置全局生效。此前 params.yaml 的 standing / collect /
    nn / predict 四段都没有被任何代码读取，改 yaml 不会有任何效果。
    """
    st = (_load_params().get("standing", {}) or {}).get("qpos_target", {}) or {}
    idx = {"hip_roll_l": 0, "hip_pitch_l": 1, "knee_l": 2,
           "ankle_pitch_l": 3, "ankle_roll_l": 4, "hip_roll_r": 5,
           "hip_pitch_r": 6, "knee_r": 7, "ankle_pitch_r": 8,
           "ankle_roll_r": 9}
    for name, i in idx.items():
        if name in st:
            DEFAULT_STAND_TARGET[i] = float(st[name])


def main():
    parser = argparse.ArgumentParser(description="人形机器人视觉跌倒预测模块")
    parser.add_argument("--check", action="store_true", help="自检")
    parser.add_argument("--collect", action="store_true", help="采集数据集")
    parser.add_argument("--train", action="store_true", help="训练神经网络")
    parser.add_argument("--demo", action="store_true", help="演示推扰+预测+控制")
    parser.add_argument("--samples", type=int, default=1500,
                        help="采集的总样本数（normal 占 normal_ratio，其余均分到 4 个方向）")
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

    _apply_standing_params()

    if args.check:
        sys.exit(run_check())

    if args.collect:
        cfg = _load_params().get("collect", {}) or {}
        os.makedirs(DATA_DIR, exist_ok=True)
        collect_dataset(MODEL_PATH, DATASET_PATH,
                        total_samples=args.samples,
                        normal_ratio=float(cfg.get("normal_ratio", 0.2)),
                        force_min=float(cfg.get("force_min", 80.0)),
                        force_max=float(cfg.get("force_max", 220.0)))
        return

    if args.train:
        cfg = _load_params().get("nn", {}) or {}
        os.makedirs(WEIGHTS_DIR, exist_ok=True)
        train_model(DATASET_PATH, WEIGHTS_PATH,
                    epochs=int(cfg.get("epochs", 300)),
                    lr=float(cfg.get("learning_rate", 0.005)),
                    batch_size=int(cfg.get("batch_size", 64)))
        return

    if args.demo:
        m = run_demo(force=args.force, direction_deg=args.direction,
                     viewer=args.viewer)
        print(json.dumps(m, ensure_ascii=False, indent=2, default=str))
        return

    parser.print_help()


if __name__ == "__main__":
    main()
