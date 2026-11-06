#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mujoco_car_drive_fault 统一入口。

流程：
  1. --collect  三档拖滞程度 × 三档车速 × 左右两侧失效，真实跑出跑偏数据
  2. --train    用纯 numpy MLP 拟合"位姿 + 轮速 + 指令 -> 1.0 s 内是否跑偏出界"
  3. --eval     未参与训练的车速上对比 none / rule / nn 三组策略
  4. --demo     离屏渲染录制真实运行 gif

用法（满足仓库"入口以 main 开头、必须支持 launch"的约定）：
    python3 main.py --collect
    python3 main.py --train
    python3 main.py --eval
    python3 main.py --demo
    python3 main.py --test                  # 自检：纯逻辑单测 + 场景冒烟（需要 MuJoCo）
    python3 main.py --test --logic-only     # 只跑纯逻辑单测，未安装 MuJoCo 也可启动
    python3 main.py --launch                # ros2 launch 一键启动 ROS 2 闭环

退出码约定（失败必须返回非 0）：
    0 成功；1 运行失败或自检未通过；2 参数错误
"""

import argparse
import json
import subprocess
import sys
import unittest
import math
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mujoco_car_drive_fault import __version__  # noqa: E402
from mujoco_car_drive_fault.config import Config, RESULTS_DIR  # noqa: E402
from mujoco_car_drive_fault.model import RiskMLP, auc_score  # noqa: E402

# 只在顶层导入不依赖 MuJoCo 的纯逻辑模块；CarEnv / scenario 等仿真相关模块
# 延迟到各分支内部导入，使未安装 MuJoCo 时 `python3 main.py --test --logic-only`
# 也能正常启动并跑完纯逻辑单测。
TRAIN_SPEEDS = (0.6, 1.2)
EVAL_SPEEDS = (0.9,)
DATASET_PATH = RESULTS_DIR / "dataset.npz"
MODEL_PATH = RESULTS_DIR / "drive_fault_risk.npz"
METRICS_PATH = RESULTS_DIR / "eval_metrics.json"
TRAIN_METRICS_PATH = RESULTS_DIR / "train_metrics.json"
DEMO_GIF = RESULTS_DIR / "drive_fault_guard_demo.gif"


def _import_sim_modules():
    """延迟导入 MuJoCo 相关模块；未安装时给出明确提示并以退出码 1 退出。"""
    try:
        from mujoco_car_drive_fault.car_env import CarEnv
        from mujoco_car_drive_fault.scenario import run_episode, label_deviate
    except ImportError as exc:
        print(
            "[错误] 当前命令需要 MuJoCo，但导入失败：{}\n"
            "未安装仿真器时可先运行纯逻辑自检：python3 main.py --test --logic-only\n"
            "安装依赖见 README.md 与教程 5.2 节。".format(exc))
        raise SystemExit(1)
    return CarEnv, run_episode, label_deviate


# ---------------------------------------------------------------- 采集
def cmd_collect(cfg, args):
    CarEnv, run_episode, label_deviate = _import_sim_modules()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    X_all, y_all, ep_all = [], [], []
    scene_idx, speed_all, side_all, dev_all = [], [], [], []
    ep = 0
    for si, (scene, damping) in enumerate(cfg.scenes):
        for v_target in TRAIN_SPEEDS:
            for side in cfg.fault_sides:
                env = CarEnv(cfg, scene)
                result = run_episode(env, cfg, v_target, side=side, mode="none")
                y = label_deviate(result["feat"], result["deviate"],
                                  result["t_deviate"], cfg)
                X_all.append(result["feat"])
                y_all.append(y)
                ep_all.append(np.full(result["feat"].shape[0], ep, dtype=np.int32))
                scene_idx.append(np.full(result["feat"].shape[0], si, dtype=np.int32))
                speed_all.append(np.full(result["feat"].shape[0], v_target))
                side_all.append(np.full(result["feat"].shape[0], 1.0 if side == "left" else -1.0))
                dev_all.append(np.full(result["feat"].shape[0], result["deviate"]))
                print("[采集] 场景={} 拖滞={:.2f} v={:.1f} 失效侧={} 跑偏={} "
                      "跑偏时刻={} 最大偏差={:.2f}m 样本={}".format(
                          scene, damping, v_target, side, result["deviate"],
                          "-" if result["t_deviate"] is None else "{:.2f}s".format(result["t_deviate"]),
                          result["max_abs_y"], result["feat"].shape[0]))
                ep += 1
    X = np.concatenate(X_all, axis=0)
    y = np.concatenate(y_all, axis=0)
    np.savez_compressed(
        DATASET_PATH,
        X=X.astype(np.float32), y=y.astype(np.float32),
        ep=np.concatenate(ep_all), scene=np.concatenate(scene_idx),
        speed=np.concatenate(speed_all), side=np.concatenate(side_all),
        deviated=np.concatenate(dev_all))
    print("[采集] 共 {} 个工况，{} 个控制周期样本，正样本 {} 个（{:.2f}%），已写入 {}".format(
        ep, X.shape[0], int(y.sum()), 100.0 * y.mean(), DATASET_PATH.name))
    return 0


# ---------------------------------------------------------------- 训练
def _split_by_episode(ep):
    """按工况切分训练/验证集，避免同一段跑偏过程的相邻样本跨集泄漏。"""
    uniq = np.unique(ep)
    val_ids = set(int(e) for e in uniq[::4])
    mask_val = np.isin(ep, list(val_ids))
    return ~mask_val, mask_val


def cmd_train(cfg, args):
    if not DATASET_PATH.exists():
        print("[错误] 找不到数据集 {}，请先运行 python3 main.py --collect".format(DATASET_PATH))
        return 1
    z = np.load(DATASET_PATH)
    X = z["X"].astype(np.float64)
    y = z["y"].astype(np.float64)
    ep = z["ep"]

    tr, va = _split_by_episode(ep)
    if args.epochs:
        cfg.epochs = int(args.epochs)

    net = RiskMLP(cfg.feature_dim, cfg.hidden, cfg.seed)
    rng = np.random.default_rng(cfg.seed)
    pos = float(y[tr].sum())
    neg = max(1.0, float(tr.sum() - pos))
    pos_weight = float(np.clip(neg / max(1.0, pos), 1.0, 50.0))

    history = {"epoch": [], "loss": [], "val_auc": [], "val_loss": []}
    losses = []
    for epoch in range(cfg.epochs):
        perm = rng.permutation(np.count_nonzero(tr))
        idx_tr = np.flatnonzero(tr)[perm]
        ep_loss = []
        for s in range(0, idx_tr.size, cfg.batch):
            idx = idx_tr[s: s + cfg.batch]
            ep_loss.append(net.train_step(X[idx], y[idx], cfg.lr, pos_weight))
        losses.append(float(np.mean(ep_loss)))
        if epoch % 5 == 0 or epoch == cfg.epochs - 1:
            p_val = net.predict_proba(X[va])
            history["epoch"].append(epoch + 1)
            history["loss"].append(losses[-1])
            history["val_auc"].append(auc_score(p_val, y[va]))
            history["val_loss"].append(_bce(p_val, y[va]))

    net.save(MODEL_PATH)
    p_tr = net.predict_proba(X[tr])
    p_val = net.predict_proba(X[va])
    metrics = {
        "train_samples": int(tr.sum()),
        "val_samples": int(va.sum()),
        "train_episodes": int(len(np.unique(ep[tr]))),
        "val_episodes": int(len(np.unique(ep[va]))),
        "pos_weight": pos_weight,
        "final_loss": losses[-1],
        "train_auc": auc_score(p_tr, y[tr]),
        "val_auc": auc_score(p_val, y[va]),
        "train_acc": float(((p_tr >= 0.5) == (y[tr] >= 0.5)).mean()),
        "val_acc": float(((p_val >= 0.5) == (y[va] >= 0.5)).mean()),
        "epochs": cfg.epochs,
        "params": int(net.W1.size + net.b1.size + net.W2.size + net.b2.size),
        "history": history,
    }
    TRAIN_METRICS_PATH.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print("[训练] 隐层 {}、参数 {} 个、{} 轮".format(cfg.hidden, metrics["params"], cfg.epochs))
    print("[训练] 训练集 AUC {:.3f} 准确率 {:.3f}；验证集 AUC {:.3f} 准确率 {:.3f}".format(
        metrics["train_auc"], metrics["train_acc"], metrics["val_auc"], metrics["val_acc"]))
    print("[训练] 模型写入 {}".format(MODEL_PATH.name))
    return 0


def _bce(prob, label):
    eps = 1e-9
    prob = np.clip(prob, eps, 1.0 - eps)
    return float(-np.mean(label * np.log(prob) + (1.0 - label) * np.log(1.0 - prob)))


# ---------------------------------------------------------------- 评测
def cmd_eval(cfg, args):
    CarEnv, run_episode, label_deviate = _import_sim_modules()
    if not MODEL_PATH.exists():
        print("[错误] 找不到模型 {}，请先运行 python3 main.py --train".format(MODEL_PATH))
        return 1
    net = RiskMLP.load(MODEL_PATH)

    rows = []
    risk_prob, risk_label = [], []
    for scene, damping in cfg.scenes:
        for v_target in EVAL_SPEEDS:
            for side in cfg.fault_sides:
                for mode in ("none", "rule", "nn"):
                    env = CarEnv(cfg, scene)
                    r = run_episode(env, cfg, v_target, side=side, mode=mode, net=net)
                    rows.append({
                        "scene": scene, "damping": damping, "v_target": v_target,
                        "side": side, "mode": mode, "deviate": bool(r["deviate"]),
                        "t_deviate": r["t_deviate"], "t_end": r["t_end"],
                        "max_abs_y": r["max_abs_y"], "end_y": r["end_y"],
                        "max_abs_yaw": r["max_abs_yaw"],
                        "assist_frames": r["assist_frames"],
                        "slow_frames": r["slow_frames"],
                        "t_first_assist": r["t_first_assist"],
                        "domega_mean": float(np.mean(r["series"]["domega"])) if r["samples"] else 0.0,
                        "samples": r["samples"],
                    })
                    if mode == "none":
                        p = net.predict_proba(r["feat"]) if r["samples"] else np.zeros(0)
                        y = label_deviate(r["feat"], r["deviate"], r["t_deviate"], cfg)
                        risk_prob.append(p)
                        risk_label.append(y)
                    print("[评测] {} 拖滞={:.2f} v={:.1f} 失效侧={} 策略={} 跑偏={} "
                          "最大偏差={:.2f}m 末端偏差={:+.2f}m 补偿帧={}".format(
                              scene, damping, v_target, side, mode, r["deviate"],
                              r["max_abs_y"], r["end_y"], r["assist_frames"]))

    all_p = np.concatenate(risk_prob)
    all_y = np.concatenate(risk_label)
    auc = auc_score(all_p, all_y)
    acc = float(((all_p >= cfg.risk_threshold) == (all_y >= 0.5)).mean()) if all_p.size else float("nan")

    summary = {}
    for mode in ("none", "rule", "nn"):
        sel = [r for r in rows if r["mode"] == mode]
        dev = [r for r in sel if r["deviate"]]
        assisted = [r["t_first_assist"] for r in sel if r["t_first_assist"] is not None]
        summary[mode] = {
            "runs": len(sel),
            "deviate_runs": len(dev),
            "deviate_rate": float(len(dev) / max(1, len(sel))),
            "mean_max_abs_y": float(np.mean([r["max_abs_y"] for r in sel])),
            "mean_end_y": float(np.mean([r["end_y"] for r in sel])),
            "mean_max_abs_yaw": float(np.mean([r["max_abs_yaw"] for r in sel])),
            "mean_assist_frames": float(np.mean([r["assist_frames"] for r in sel])),
            "mean_t_first_assist": float(np.mean(assisted)) if assisted else None,
            "assisted_runs": len(assisted),
            "mean_abs_domega": float(np.mean([abs(r["domega_mean"]) for r in sel])),
        }

    metrics = {
        "eval_speeds": list(EVAL_SPEEDS),
        "risk_auc": auc,
        "risk_acc_at_threshold": acc,
        "risk_threshold": cfg.risk_threshold,
        "risk_samples": int(all_p.size),
        "risk_pos_samples": int(all_y.sum()),
        "summary": summary,
        "runs": rows,
    }
    METRICS_PATH.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print("[评测] 风险 AUC={:.3f} 阈值{:.2f}下准确率={:.3f}".format(auc, cfg.risk_threshold, acc))
    for mode in ("none", "rule", "nn"):
        s = summary[mode]
        first = "-" if s["mean_t_first_assist"] is None else "{:.2f}s".format(s["mean_t_first_assist"])
        print("[评测] 策略 {:<4} 跑偏率 {:.0%} ({}/{}), 平均最大偏差 {:.2f}m, "
              "平均末端偏差 {:+.2f}m, 平均补偿帧 {:.0f}, 首次介入 {}, 平均轮速差 {:.2f} rad/s".format(
                  mode, s["deviate_rate"], s["deviate_runs"], s["runs"],
                  s["mean_max_abs_y"], s["mean_end_y"], s["mean_assist_frames"],
                  first, s["mean_abs_domega"]))
    _plot_all(cfg, metrics)
    return 0


def _plot_all(cfg, metrics):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figdir = RESULTS_DIR / "figs"
    figdir.mkdir(parents=True, exist_ok=True)

    tm = json.loads(TRAIN_METRICS_PATH.read_text(encoding="utf-8"))
    h = tm["history"]
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    axes[0].plot(h["epoch"], h["loss"], color="#2f6fb3", linewidth=1.6)
    axes[0].set_xlabel("epoch")
    axes[0].set_ylabel("weighted BCE loss")
    axes[0].set_title("training loss")
    axes[0].grid(alpha=0.3)
    axes[1].plot(h["epoch"], h["val_auc"], color="#c8553d", linewidth=1.6)
    axes[1].set_xlabel("epoch")
    axes[1].set_ylabel("validation AUC")
    axes[1].set_ylim(0, 1.02)
    axes[1].set_title("validation AUC (held-out episodes)")
    axes[1].grid(alpha=0.3)
    fig.suptitle("drive fault risk MLP training, val AUC = {:.3f}".format(tm["val_auc"]))
    fig.tight_layout()
    fig.savefig(figdir / "train_curves.png", dpi=160)
    plt.close(fig)

    names = {"none": "none (no compensation)", "rule": "rule (pose threshold)",
             "nn": "nn (1.0s forecast)"}
    colors = {"none": "#8a8f98", "rule": "#d9a441", "nn": "#2f6fb3"}
    dampings = sorted({r["damping"] for r in metrics["runs"]})
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    width = 0.26
    for k, mode in enumerate(("none", "rule", "nn")):
        ys = []
        for dd in dampings:
            sel = [r for r in metrics["runs"] if r["damping"] == dd and r["mode"] == mode]
            ys.append(np.mean([r["max_abs_y"] for r in sel]) if sel else 0.0)
        xs = np.arange(len(dampings)) + (k - 1) * width
        ax.bar(xs, ys, width=width, color=colors[mode], label=names[mode])
        for x, v in zip(xs, ys):
            ax.text(x, v + 0.03, "{:.2f}".format(v), ha="center", fontsize=8)
    ax.axhline(cfg.deviate_m, color="#c8553d", linestyle="--", linewidth=1.0,
               label="deviation limit {:.2f} m".format(cfg.deviate_m))
    ax.set_xticks(np.arange(len(dampings)))
    ax.set_xticklabels(["damping {:.2f}".format(d) for d in dampings])
    ax.set_ylim(0.0, max(1.3, max(r["max_abs_y"] for r in metrics["runs"]) * 1.15))
    ax.set_ylabel("mean max lateral offset (m)")
    ax.set_title("lateral offset by fault severity and strategy")
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(figdir / "strategy_compare.png", dpi=160)
    plt.close(fig)

    worst = None
    for r in metrics["runs"]:
        if r["mode"] == "none" and r["deviate"]:
            if worst is None or r["max_abs_y"] > worst["max_abs_y"]:
                worst = r
    if worst is not None:
        _plot_timeline(cfg, worst, figdir)
    return figdir


def _plot_timeline(cfg, row, figdir):
    """重跑一次该工况，画横向偏差与风险概率随时间的变化。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    CarEnv, run_episode, _ = _import_sim_modules()
    net = RiskMLP.load(MODEL_PATH)
    env = CarEnv(cfg, row["scene"])
    r = run_episode(env, cfg, row["v_target"], side=row["side"], mode="none", net=net)
    p = net.predict_proba(r["feat"])
    t = np.asarray(r["series"]["t"])
    y = np.asarray(r["series"]["y"])
    fig, ax = plt.subplots(figsize=(7.4, 3.8))
    ax.plot(t, y, color="#c8553d", linewidth=1.5, label="lateral offset (m)")
    ax.axhline(cfg.deviate_m, color="#8a8f98", linestyle="--", linewidth=1.0,
               label="limit {:.2f} m".format(cfg.deviate_m))
    ax.set_xlabel("time (s)")
    ax.set_ylabel("lateral offset (m)")
    ax.grid(alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, p, color="#2f6fb3", linewidth=1.6, label="NN deviation risk (1.0 s ahead)")
    ax2.set_ylabel("risk probability")
    ax2.set_ylim(0, 1.05)
    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [l.get_label() for l in lines], fontsize=8, loc="upper left")
    ax.set_title("{} side={} v={:.1f} m/s, limit reached at {}s".format(
        row["scene"], row["side"], row["v_target"],
        "-" if row["t_deviate"] is None else "{:.2f}".format(row["t_deviate"])))
    fig.tight_layout()
    fig.savefig(figdir / "risk_timeline.png", dpi=160)
    plt.close(fig)


# ---------------------------------------------------------------- 演示
def cmd_demo(cfg, args):
    CarEnv, run_episode, _ = _import_sim_modules()
    if not MODEL_PATH.exists():
        print("[错误] 找不到模型 {}，请先运行 python3 main.py --train".format(MODEL_PATH))
        return 1
    import mujoco
    try:
        import imageio.v2 as imageio
    except ImportError as exc:
        print("[错误] 录制 gif 需要 imageio：{}".format(exc))
        return 1

    from mujoco_car_drive_fault.features import build
    from mujoco_car_drive_fault.guard import FaultGuard, comp_steer
    from mujoco_car_drive_fault.kinematics import differential_wheel_vels

    net = RiskMLP.load(MODEL_PATH)
    scene, damping = cfg.scenes[1]
    env = CarEnv(cfg, scene)
    renderer = mujoco.Renderer(env.model, height=360, width=480)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth = 135.0
    cam.elevation = -22.0
    cam.distance = 2.4

    env.reset(fault_side="left", fault_damping=damping)
    frames, t = [], 0.0
    guard = FaultGuard(cfg)
    from mujoco_car_drive_fault.scenario import hold_throttle
    v_cmd = 0.9
    while t < cfg.run_max_s:
        v = env.forward_speed()
        thr = hold_throttle(v, v_cmd, cfg)
        y = env.lateral_offset()
        yaw_deg = math.degrees(env.yaw())
        wl_cmd, wr_cmd = differential_wheel_vels(
            0.0, thr, cfg.v_max, cfg.omega_max, cfg.wheel_r, cfg.track,
            cfg.max_wheel_omega)
        _, x = build(env, wl_cmd, wr_cmd, cfg)
        risk = float(net.predict_proba(x))
        gain, action = guard.decide(risk, y, yaw_deg, mode="nn")
        integ = guard.update_integ(y) if gain > 0 else 0.0
        steer = comp_steer(y, env.yaw(), env.imu()["gyro"][2],
                           cfg, gain, integ) if gain > 0 else 0.0
        env.set_command(steer, thr * (0.5 if action == "slow" else 1.0))
        cam.lookat[:] = env.data.xpos[env.chassis_id]
        renderer.update_scene(env.data, camera=cam)
        if int(t / cfg.dt) % 3 == 0:
            frames.append(renderer.render())
        if abs(env.lateral_offset()) >= 1.35 or env.distance() >= cfg.distance_s:
            break
        t += cfg.dt
    renderer.close()

    fps = int(round(cfg.control_hz / 3.0))
    imageio.mimsave(str(DEMO_GIF), frames, duration=1000.0 / fps, loop=0)
    size_mb = DEMO_GIF.stat().st_size / 1e6
    print("[演示] 场景={} 帧数={} fps={} 文件 {:.2f} MB".format(scene, len(frames), fps, size_mb))
    if size_mb > 10.0:
        print("[警告] gif 超过 10MB，仓库要求 <10MB，请降低分辨率或帧数")
        return 1
    return 0


# ---------------------------------------------------------------- 自检
def cmd_test(cfg, args):
    test_dir = Path(__file__).resolve().parent / "test"
    suite = unittest.TestLoader().discover(str(test_dir))
    if args.logic_only:
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        return 0 if result.wasSuccessful() else 1

    ok = unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful()
    CarEnv, run_episode, _ = _import_sim_modules()
    for scene, damping in cfg.scenes:
        env = CarEnv(cfg, scene)
        r = run_episode(env, cfg, cfg.speeds[0], side="left", mode="none", max_s=6.0)
        print("[冒烟] {} 拖滞={:.2f} 6s 内最大偏差 {:.2f}m 跑偏={}".format(
            scene, damping, r["max_abs_y"], r["deviate"]))
    return 0 if ok else 1


def cmd_launch(cfg, args):
    import shutil
    if shutil.which("ros2") is None:
        print("[错误] 未找到 ros2 命令，请先安装 ROS 2 并在其环境中运行")
        return 1
    cmd = ["ros2", "launch", "mujoco_car_drive_fault", "main.launch.py"]
    print("[启动] {}".format(" ".join(cmd)))
    return subprocess.call(cmd)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="MuJoCo 差速小车单侧驱动失效检测与差速补偿")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--collect", action="store_true", help="采集跑偏数据")
    group.add_argument("--train", action="store_true", help="训练失效风险网络")
    group.add_argument("--eval", action="store_true", help="三组策略对比评测（默认）")
    group.add_argument("--demo", action="store_true", help="离屏录制演示 gif")
    group.add_argument("--test", action="store_true", help="运行单元测试与场景冒烟")
    group.add_argument("--launch", action="store_true", help="ros2 launch 启动 ROS 2 闭环")
    parser.add_argument("--logic-only", action="store_true", help="自检只跑纯逻辑单测")
    parser.add_argument("--epochs", type=int, default=None, help="训练轮数（默认取配置）")
    args = parser.parse_args(argv)

    print("mujoco_car_drive_fault v{}".format(__version__))
    cfg = Config()
    if args.collect:
        return cmd_collect(cfg, args)
    if args.train:
        return cmd_train(cfg, args)
    if args.demo:
        return cmd_demo(cfg, args)
    if args.test:
        return cmd_test(cfg, args)
    if args.launch:
        return cmd_launch(cfg, args)
    return cmd_eval(cfg, args)


if __name__ == "__main__":
    sys.exit(main())
