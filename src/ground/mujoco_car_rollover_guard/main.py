#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mujoco_car_rollover_guard 统一入口。

流程：
  1. --collect  三档载荷高度 × 三档车速 × 三条转向斜坡，真实跑出侧翻数据
  2. --train    用纯 numpy MLP 拟合"IMU + 车速 + 转向指令 -> 0.5s 内是否翻车"
  3. --eval     未参与训练的转向斜坡上对比 none / rule / nn 三组策略
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
import math
import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mujoco_car_rollover_guard import __version__  # noqa: E402
from mujoco_car_rollover_guard.config import Config, RESULTS_DIR  # noqa: E402
from mujoco_car_rollover_guard.model import RiskMLP, auc_score  # noqa: E402

# 只在顶层导入不依赖 MuJoCo 的纯逻辑模块；CarEnv / scenario 等仿真相关模块
# 延迟到各分支内部导入，使未安装 MuJoCo 时 `python3 main.py --test --logic-only`
# 也能正常启动并跑完纯逻辑单测。
TRAIN_RAMPS = (0.35, 0.50, 0.65)
EVAL_RAMP = 0.45
DATASET_PATH = RESULTS_DIR / "dataset.npz"
MODEL_PATH = RESULTS_DIR / "rollover_risk.npz"
METRICS_PATH = RESULTS_DIR / "eval_metrics.json"
TRAIN_METRICS_PATH = RESULTS_DIR / "train_metrics.json"
DEMO_GIF = RESULTS_DIR / "rollover_guard_demo.gif"


def _import_sim_modules():
    """延迟导入 MuJoCo 相关模块；未安装时给出明确提示并以退出码 1 退出。"""
    try:
        from mujoco_car_rollover_guard.car_env import CarEnv
        from mujoco_car_rollover_guard.scenario import run_episode, label_rollover
    except ImportError as exc:
        print(
            "[错误] 当前命令需要 MuJoCo，但导入失败：{}\n"
            "未安装仿真器时可先运行纯逻辑自检：python3 main.py --test --logic-only\n"
            "安装依赖见 README.md 与教程 5.2 节。".format(exc))
        raise SystemExit(1)
    return CarEnv, run_episode, label_rollover


# ---------------------------------------------------------------- 采集
def cmd_collect(cfg, args):
    CarEnv, run_episode, label_rollover = _import_sim_modules()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    X_all, y_all, ep_all = [], [], []
    scene_idx, speed_all, ramp_all, roll_all = [], [], [], []
    ep = 0
    for si, (scene, height) in enumerate(cfg.scenes):
        for v_target in cfg.speeds:
            for ramp in TRAIN_RAMPS:
                env = CarEnv(cfg, scene)
                result = run_episode(env, cfg, v_target, mode="none",
                                     steer_ramp=ramp, collect=True)
                y = label_rollover(result["feat"], result["rollover"],
                                   result["t_roll"], cfg)
                X_all.append(result["feat"])
                y_all.append(y)
                ep_all.append(np.full(result["feat"].shape[0], ep, dtype=np.int32))
                scene_idx.append(np.full(result["feat"].shape[0], si, dtype=np.int32))
                speed_all.append(np.full(result["feat"].shape[0], v_target))
                ramp_all.append(np.full(result["feat"].shape[0], ramp))
                roll_all.append(np.full(result["feat"].shape[0], result["rollover"]))
                print("[采集] 场景={} 载荷z={:.2f} 目标v={:.1f} 转向斜率={:.2f} "
                      "翻车={} 翻车时刻={} 样本={} 最长倾角={:.1f}°".format(
                          scene, height, v_target, ramp, result["rollover"],
                          "-" if result["t_roll"] is None else "{:.2f}s".format(result["t_roll"]),
                          result["feat"].shape[0], result["max_tilt"]))
                ep += 1
    X = np.concatenate(X_all, axis=0)
    y = np.concatenate(y_all, axis=0)
    np.savez_compressed(
        DATASET_PATH,
        X=X.astype(np.float32), y=y.astype(np.float32),
        ep=np.concatenate(ep_all), scene=np.concatenate(scene_idx),
        speed=np.concatenate(speed_all), ramp=np.concatenate(ramp_all),
        rolled=np.concatenate(roll_all))
    print("[采集] 共 {} 个工况，{} 个控制周期样本，正样本 {} 个（{:.2f}%），已写入 {}".format(
        ep, X.shape[0], int(y.sum()), 100.0 * y.mean(), DATASET_PATH.name))
    return 0


# ---------------------------------------------------------------- 训练
def _split_by_episode(ep):
    """按工况切分训练/验证集，避免同一段翻车过程的相邻样本跨集泄漏。"""
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
    neg = float(tr.sum() - pos)
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
    CarEnv, run_episode, label_rollover = _import_sim_modules()
    if not MODEL_PATH.exists():
        print("[错误] 找不到模型 {}，请先运行 python3 main.py --train".format(MODEL_PATH))
        return 1
    net = RiskMLP.load(MODEL_PATH)

    rows = []
    risk_prob, risk_label = [], []
    for si, (scene, height) in enumerate(cfg.scenes):
        for v_target in cfg.speeds:
            for mode in ("none", "rule", "nn"):
                env = CarEnv(cfg, scene)
                r = run_episode(env, cfg, v_target, mode=mode, net=net,
                                steer_ramp=EVAL_RAMP)
                rows.append({
                    "scene": scene, "height": height, "v_target": v_target,
                    "mode": mode, "rollover": bool(r["rollover"]),
                    "t_roll": r["t_roll"], "t_end": r["t_end"],
                    "max_roll": r["max_roll"], "max_tilt": r["max_tilt"],
                    "max_ay": r["max_ay"], "samples": r["samples"],
                })
                if mode == "none":
                    p = net.predict_proba(r["feat"]) if r["samples"] else np.zeros(0)
                    y = label_rollover(r["feat"], r["rollover"], r["t_roll"], cfg)
                    risk_prob.append(p)
                    risk_label.append(y)
                print("[评测] {} 载荷z={:.2f} v={:.1f} 策略={} 翻车={} 翻车时刻={} "
                      "最大横滚={:.1f}° 最大倾角={:.1f}°".format(
                          scene, height, v_target, mode, r["rollover"],
                          "-" if r["t_roll"] is None else "{:.2f}s".format(r["t_roll"]),
                          r["max_roll"], r["max_tilt"]))

    # 风险预测质量：在 none 组（未加干预、真实翻车）上逐样本评估
    all_p = np.concatenate(risk_prob)
    all_y = np.concatenate(risk_label)
    auc = auc_score(all_p, all_y)
    acc = float(((all_p >= cfg.risk_threshold) == (all_y >= 0.5)).mean()) if all_p.size else float("nan")

    summary = {}
    for mode in ("none", "rule", "nn"):
        sel = [r for r in rows if r["mode"] == mode]
        rolled = [r for r in sel if r["rollover"]]
        summary[mode] = {
            "runs": len(sel),
            "rollover_runs": len(rolled),
            "rollover_rate": float(len(rolled) / max(1, len(sel))),
            "mean_t_roll": float(np.mean([r["t_roll"] for r in rolled])) if rolled else None,
            "mean_max_roll": float(np.mean([r["max_roll"] for r in sel])),
            "mean_max_tilt": float(np.mean([r["max_tilt"] for r in sel])),
        }

    metrics = {
        "eval_ramp": EVAL_RAMP,
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
        print("[评测] 策略 {:<4} 翻车率 {:.0%} ({}/{}), 平均翻车时刻 {}, 平均最大横滚 {:.1f}°".format(
            mode, s["rollover_rate"], s["rollover_runs"], s["runs"],
            "-" if s["mean_t_roll"] is None else "{:.2f}s".format(s["mean_t_roll"]),
            s["mean_max_roll"]))
    _plot_all(cfg, metrics)
    return 0


def _plot_all(cfg, metrics):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figdir = RESULTS_DIR / "figs"
    figdir.mkdir(parents=True, exist_ok=True)

    # 1) 训练曲线
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
    fig.suptitle("rollover risk MLP training, val AUC = {:.3f}".format(tm["val_auc"]))
    fig.tight_layout()
    fig.savefig(figdir / "train_curves.png", dpi=160)
    plt.close(fig)

    # 2) 三组策略翻车率对比
    names = {"none": "none (no guard)", "rule": "rule (roll threshold)", "nn": "nn (0.5s forecast)"}
    colors = {"none": "#8a8f98", "rule": "#d9a441", "nn": "#2f6fb3"}
    heights = sorted({r["height"] for r in metrics["runs"]})
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    width = 0.26
    for k, mode in enumerate(("none", "rule", "nn")):
        rates = []
        for hh in heights:
            sel = [r for r in metrics["runs"] if r["height"] == hh and r["mode"] == mode]
            rates.append(100.0 * sum(r["rollover"] for r in sel) / max(1, len(sel)))
        xs = np.arange(len(heights)) + (k - 1) * width
        ax.bar(xs, rates, width=width, color=colors[mode], label=names[mode])
        for x, v in zip(xs, rates):
            ax.text(x, v + 2, "{:.0f}".format(v), ha="center", fontsize=9)
    ax.set_xticks(np.arange(len(heights)))
    ax.set_xticklabels(["payload z={:.2f} m".format(h) for h in heights])
    ax.set_ylabel("rollover rate over 3 speeds (%)")
    ax.set_ylim(0, 118)
    ax.set_title("rollover rate by payload height and strategy")
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(figdir / "strategy_compare.png", dpi=160)
    plt.close(fig)

    # 3) 典型工况的倾角与风险时序（none 组高分场景）
    worst = None
    for r in metrics["runs"]:
        if r["mode"] == "none" and r["rollover"]:
            if worst is None or r["max_tilt"] > worst["max_tilt"]:
                worst = r
    if worst is not None:
        _plot_timeline(cfg, worst, figdir)
    return figdir


def _plot_timeline(cfg, row, figdir):
    """重跑一次该工况（none），画倾角与风险概率随时间的变化。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    CarEnv, run_episode, _ = _import_sim_modules()
    net = RiskMLP.load(MODEL_PATH)
    env = CarEnv(cfg, row["scene"])
    r = run_episode(env, cfg, row["v_target"], mode="none", net=net, steer_ramp=EVAL_RAMP)
    p = net.predict_proba(r["feat"])
    t = np.asarray(r["series"]["t"])
    tilt = np.asarray(r["series"]["tilt"])
    roll = np.asarray(r["series"]["roll"])
    fig, ax = plt.subplots(figsize=(7.4, 3.8))
    ax.plot(t, tilt, color="#c8553d", linewidth=1.5, label="total tilt (deg)")
    ax.plot(t, np.abs(roll), color="#8a8f98", linewidth=1.2, label="|roll| (deg)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("attitude (deg)")
    ax.grid(alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, p, color="#2f6fb3", linewidth=1.6, label="NN rollover risk (0.5s ahead)")
    ax2.set_ylabel("risk probability")
    ax2.set_ylim(0, 1.05)
    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [l.get_label() for l in lines], fontsize=8, loc="upper left")
    ax.set_title("{} v={:.1f} m/s, rollover at {}s".format(
        row["scene"], row["v_target"],
        "-" if row["t_roll"] is None else "{:.2f}".format(row["t_roll"])))
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
    from mujoco_car_rollover_guard.guard import RolloverGuard
    from mujoco_car_rollover_guard.features import build

    try:
        import imageio.v2 as imageio
    except ImportError as exc:
        print("[错误] 录制 gif 需要 imageio：{}".format(exc))
        return 1

    net = RiskMLP.load(MODEL_PATH)
    cfg_demo_scene = cfg.scenes[2][0]
    cfg_demo_v = cfg.speeds[1]
    env = CarEnv(cfg, cfg_demo_scene)
    guard = RolloverGuard(cfg)

    renderer = mujoco.Renderer(env.model, height=360, width=480)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth = 135.0
    cam.elevation = -18.0
    cam.distance = 1.15

    env.reset()
    frames, t, steer, thr_cap = [], 0.0, 0.0, 0.0
    rolled = False
    while t < 9.0:
        v = env.forward_speed()
        thr_cap = min(1.0, thr_cap + cfg.throttle_ramp * cfg.dt)
        thr_cmd = min(0.55 + cfg.speed_keep_gain * (cfg_demo_v - v), thr_cap)
        thr_cmd = float(min(1.0, max(0.0, thr_cmd)))
        _, x = build(env, steer, cfg)
        risk = float(net.predict_proba(x))
        steer_i, thr, action = guard.gate(risk, steer, thr_cmd)
        env.set_command(steer, thr)
        cam.lookat[:] = env.data.xpos[env.chassis_id]
        renderer.update_scene(env.data, camera=cam)
        if int(t / cfg.dt) % 3 == 0:
            frames.append(renderer.render())
        if env.is_rollover():
            rolled = True
            break
        steer = min(1.0, steer + cfg.steer_ramp_rate * cfg.dt)
        t += cfg.dt
    renderer.close()

    fps = int(round(cfg.control_hz / 3.0))
    gif_path = DEMO_GIF
    imageio.mimsave(str(gif_path), frames, duration=1000.0 / fps, loop=0)
    size_mb = gif_path.stat().st_size / 1e6
    print("[演示] 场景={} 目标v={:.1f} 结果=翻车或结束({}) 帧数={} fps={} 文件 {:.2f} MB".format(
        cfg_demo_scene, cfg_demo_v, "翻车" if rolled else "未翻车", len(frames), fps, size_mb))
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
    for scene, height in cfg.scenes:
        env = CarEnv(cfg, scene)
        r = run_episode(env, cfg, cfg.speeds[0], mode="none", steer_ramp=0.5, max_s=4.0)
        print("[冒烟] {} 载荷z={:.2f} 4s 内最大倾角 {:.1f}° 翻车={}".format(
            scene, height, r["max_tilt"], r["rollover"]))
    return 0 if ok else 1


def cmd_launch(cfg, args):
    try:
        import shutil
        if shutil.which("ros2") is None:
            print("[错误] 未找到 ros2 命令，请先安装 ROS 2 并在其环境中运行")
            return 1
    except Exception:
        return 1
    cmd = ["ros2", "launch", "mujoco_car_rollover_guard", "main.launch.py"]
    print("[启动] {}".format(" ".join(cmd)))
    return subprocess.call(cmd)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="MuJoCo 差速小车侧倾/翻车风险预测与主动限速")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--collect", action="store_true", help="采集侧翻数据")
    group.add_argument("--train", action="store_true", help="训练风险网络")
    group.add_argument("--eval", action="store_true", help="三组策略对比评测（默认）")
    group.add_argument("--demo", action="store_true", help="离屏录制演示 gif")
    group.add_argument("--test", action="store_true", help="运行单元测试与场景冒烟")
    group.add_argument("--launch", action="store_true", help="ros2 launch 启动 ROS 2 闭环")
    parser.add_argument("--logic-only", action="store_true", help="自检只跑纯逻辑单测")
    parser.add_argument("--epochs", type=int, default=None, help="训练轮数（默认取配置）")
    args = parser.parse_args(argv)

    print("mujoco_car_rollover_guard v{}".format(__version__))
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
