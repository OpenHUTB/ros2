#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mujoco_car_nn_avoidance 统一入口。

流程：
  1. --collect  在多个 MuJoCo 场景真实采集"射线 -> 专家转向/油门"标签数据
  2. --train    用纯 numpy MLP 拟合"射线 -> 控制指令"的端到端模型
  3. --eval     加载模型，多场景闭环端到端避障评测，导出指标与曲线
  4. --demo     离屏渲染录制真实运行 gif（无窗口，Windows 也适用）

用法（满足仓库"入口以 main 开头、必须支持 launch"的约定）：
    python3 main.py --collect
    python3 main.py --train --epochs 300
    python3 main.py --eval
    python3 main.py --demo
    python3 main.py --test                  # 自检：纯逻辑单测 + 场景冒烟（需要 MuJoCo）
    python3 main.py --test --logic-only    # 只跑纯逻辑单测，未安装 MuJoCo 也可启动
    python3 main.py --launch                # ros2 launch 一键启动 ROS 2 闭环
    bash main.sh --eval                     # 同上，脚本入口

退出码约定（失败必须返回非 0）：
    0 成功；1 运行失败或自检未通过；2 参数错误
"""

import argparse
import csv
import math
import shutil
import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np

# 允许"在功能包根目录直接 python3 main.py"与"colcon 安装后 ros2 run"两种方式
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mujoco_car_nn_avoidance import __version__  # noqa: E402
# 只在顶层导入不依赖 MuJoCo 的纯逻辑模块；CarEnv / ray_scan 等仿真相关
# 导入延迟到 collect / evaluate / demo 分支内部，使未安装 MuJoCo 时
# `python3 main.py --test --logic-only` 也能正常启动并跑完纯逻辑单测。
from mujoco_car_nn_avoidance.config import Config  # noqa: E402
from mujoco_car_nn_avoidance.expert import expert_control  # noqa: E402
from mujoco_car_nn_avoidance.model import MLP  # noqa: E402


def _import_sim_modules():
    """延迟导入 MuJoCo 相关模块；未安装时给出明确提示并以退出码 1 退出。"""
    try:
        from mujoco_car_nn_avoidance.car_env import CarEnv
        from mujoco_car_nn_avoidance.sensors import ray_scan, normalize_rays
    except ImportError as exc:
        print(
            "[错误] 当前命令需要 MuJoCo，但导入失败：{}\n"
            "未安装仿真器时可先运行纯逻辑自检：python3 main.py --test --logic-only\n"
            "安装依赖见 README.md 与教程 5.2 节。".format(exc))
        raise SystemExit(1)
    return CarEnv, ray_scan, normalize_rays

PACKAGE_ROOT = Path(__file__).resolve().parent
VERSION_LINE = "mujoco_car_nn_avoidance {}".format(__version__)
RESULTS = PACKAGE_ROOT / "results"


def ensure_results():
    RESULTS.mkdir(parents=True, exist_ok=True)


def build_state(rays, yaw, cfg):
    """拼接网络输入：25 束归一化射线 + 1 维航向偏差 yaw/pi。"""
    from mujoco_car_nn_avoidance.sensors import normalize_rays
    x = normalize_rays(rays, cfg.lidar_range_m)
    yaw_norm = float(np.clip(yaw / math.pi, -1.0, 1.0))
    return np.concatenate([x, [yaw_norm]]).astype(np.float32)


# ----------------------------------------------------------------------
# 1. 数据采集
# ----------------------------------------------------------------------
def collect(cfg, seconds=6.0):
    """对每个场景真实跑若干秒，用探索式采集采样 射线 -> 专家标签 对。

    探索式采集：实际驱动加入零均值高斯噪声（rng 固定），让车主动探索更多状态
    （包括贴近障碍、偏航等）；训练标签仍用专家输出。这是模仿学习常用的
    "探索收集"策略，可避免纯专家 rollout 状态单一、缺少接近障碍样本。
    """
    ensure_results()
    CarEnv, ray_scan, _ = _import_sim_modules()
    rng = np.random.default_rng(cfg.seed)
    X, Y = [], []
    per_scene = []
    for scene in cfg.scenes:
        env = CarEnv(cfg, scene)
        env.reset()
        n = int(seconds * cfg.control_hz)
        cnt = 0
        for _ in range(n):
            rays = ray_scan(env)
            yaw = env.yaw()
            x = build_state(rays, yaw, cfg)
            steer, throttle = expert_control(rays, cfg, yaw)
            X.append(x)
            Y.append([steer, throttle])
            # 探索噪声：状态更丰富，标签仍为专家输出
            steer_drive = float(np.clip(steer + rng.normal(0.0, 0.15), -1.0, 1.0))
            wl, wr = env.desired_wheel_vel(steer_drive, throttle)
            env.step_vel(wl, wr)
            cnt += 1
        per_scene.append(cnt)
    X = np.asarray(X, dtype=np.float32)
    Y = np.asarray(Y, dtype=np.float32)
    np.savez(RESULTS / "dataset.npz", X=X, Y=Y)
    print("采集完成：共 {} 样本，各场景 {}（-> results/dataset.npz）".format(
        len(X), per_scene))
    return len(X) > 0


# ----------------------------------------------------------------------
# 2. 训练
# ----------------------------------------------------------------------
def train(cfg, epochs, lr):
    ensure_results()
    data = np.load(RESULTS / "dataset.npz")
    X, Y = data["X"], data["Y"]
    net = MLP(cfg.input_dim, cfg.hidden, cfg.output_dim, seed=cfg.seed)
    losses = net.fit(X, Y, epochs=epochs, batch=cfg.batch, lr=lr, seed=cfg.seed)
    net.save(RESULTS / "mlp_weights.npz")
    np.save(RESULTS / "train_loss.npy", np.array(losses))
    print("训练完成：共 {} epoch，末段平均 loss = {:.6f}".format(
        epochs, np.mean(losses[-20:])))
    print("权重保存 -> results/mlp_weights.npz；损失曲线 -> results/train_loss.npy")
    return np.isfinite(np.mean(losses[-20:]))


# ----------------------------------------------------------------------
# 3. 闭环评测（真实仿真，NN 端到端控制）
# ----------------------------------------------------------------------
def evaluate(cfg, max_sim_t=30.0):
    ensure_results()
    CarEnv, ray_scan, _ = _import_sim_modules()
    net = MLP.load(RESULTS / "mlp_weights.npz")
    rows = []
    for scene in cfg.scenes:
        env = CarEnv(cfg, scene)
        env.reset()
        sim_t = 0.0
        reached = False
        min_clearance = 1e9
        max_steer = 0.0
        while sim_t < max_sim_t:
            rays = ray_scan(env)
            x = build_state(rays, env.yaw(), cfg)
            steer, throttle = net.predict(x)
            steer = float(steer)
            max_steer = max(max_steer, abs(steer))
            min_clearance = min(min_clearance, float(np.min(rays)))
            wl, wr = env.desired_wheel_vel(steer, throttle)
            env.step_vel(wl, wr)
            # 一次 step_vel 推进一个控制周期（steps_per_control 个物理步）
            sim_t += cfg.steps_per_control * cfg.timestep
            pos = env.pos()
            if pos[0] > cfg.goal_x:
                reached = True
                break
        rows.append({
            "scene": scene,
            "success": reached,
            "time_s": round(sim_t, 2),
            "min_clearance_m": round(min_clearance, 3),
            "max_steer": round(max_steer, 3),
            "final_x": round(float(env.pos()[0]), 2),
        })
    header = list(rows[0].keys())
    with open(RESULTS / "eval_report.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print("[{}] success={} time={}s min_clearance={}m max_steer={}".format(
            r["scene"], r["success"], r["time_s"],
            r["min_clearance_m"], r["max_steer"]))
    print("评测报告 -> results/eval_report.csv")
    try:
        plot_eval(rows)
    except Exception as exc:  # matplotlib 不可用则跳过绘图
        print("（绘图跳过：{}）".format(exc))
    return all(r["success"] for r in rows)


def plot_eval(rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    scenes = [r["scene"].replace("car_scene_", "").replace(".xml", "") for r in rows]
    times = [r["time_s"] for r in rows]
    clearances = [r["min_clearance_m"] for r in rows]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    ax1.bar(scenes, times,
            color=["#4a90d9" if r["success"] else "#d9534f" for r in rows])
    ax1.set_title("Time to Goal (s)")
    ax1.set_ylabel("time / s")
    ax2.bar(scenes, clearances, color="#7fb800")
    ax2.set_title("Min Clearance (m)")
    ax2.set_ylabel("clearance / m")
    fig.tight_layout()
    fig.savefig(RESULTS / "eval_curves.png", dpi=110)
    plt.close(fig)
    print("评测曲线 -> results/eval_curves.png")


# ----------------------------------------------------------------------
# 4. 演示：真实运行并录制 gif
# ----------------------------------------------------------------------
def demo(cfg, gif_name="run_demo.gif", duration_s=18.0, every=2,
         width=480, height=360):
    """离屏渲染真实运行过程并合成 gif（无窗口，Windows 也适用）。"""
    ensure_results()
    CarEnv, ray_scan, _ = _import_sim_modules()
    import mujoco

    net = MLP.load(RESULTS / "mlp_weights.npz")
    scene = cfg.scenes[0]
    env = CarEnv(cfg, scene)
    env.reset()
    renderer = mujoco.Renderer(env.model, height=height, width=width)
    frames = []
    sim_t = 0.0
    step = 0
    while sim_t < duration_s:
        rays = ray_scan(env)
        x = build_state(rays, env.yaw(), cfg)
        steer, throttle = net.predict(x)
        wl, wr = env.desired_wheel_vel(steer, throttle)
        env.step_vel(wl, wr)
        sim_t += cfg.steps_per_control * cfg.timestep
        step += 1
        if step % every == 0:
            renderer.update_scene(env.data, "top")
            frames.append(renderer.render())
    out = RESULTS / gif_name
    import imageio.v2 as imageio
    # 每帧间隔 = every 个控制周期（真实仿真时长，单位秒）。
    # imageio 的 Pillow GIF writer 把 duration 按毫秒解释，这里必须传毫秒值，
    # 否则秒值会被四舍五入成 0 ms，浏览器兜底成 100 ms、整体播放变慢。
    frame_dt = every * cfg.steps_per_control * cfg.timestep
    frame_duration_ms = frame_dt * 1000.0
    imageio.mimsave(str(out), frames, duration=frame_duration_ms)
    total_s = len(frames) * frame_dt
    print("运行 gif -> results/{}（{} 帧，每帧 {:.0f} ms / {:.3f} s，"
          "总时长 {:.1f} s）".format(
              gif_name, len(frames), frame_duration_ms, frame_dt, total_s))
    return len(frames) > 0


# ----------------------------------------------------------------------
# 5. 自检：纯逻辑单测 + 场景冒烟
# ----------------------------------------------------------------------
def run_test(args) -> int:
    print("[自检] 1/2 纯逻辑测试（不需要 MuJoCo）")
    loader = unittest.TestLoader()
    suite = loader.discover(str(PACKAGE_ROOT / "test"), pattern="test_*.py")
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    ok = result.wasSuccessful()

    if args.logic_only:
        print("[自检] 已跳过场景冒烟测试（--logic-only）")
        return 0 if ok else 1

    print("[自检] 2/2 场景冒烟测试（需要 MuJoCo）")
    completed = subprocess.run(
        [sys.executable, str(PACKAGE_ROOT / "tools" / "check_scene.py")])
    ok = ok and completed.returncode == 0
    print("[自检] 结果：{}".format("通过" if ok else "未通过"))
    return 0 if ok else 1


# ----------------------------------------------------------------------
# 6. ROS 2 一键启动
# ----------------------------------------------------------------------
def run_launch() -> int:
    if shutil.which("ros2") is None:
        print("[错误] 未找到 ros2 命令。请先 source ROS 2 环境：\n"
              "  source /opt/ros/humble/setup.bash\n"
              "并确认已 colcon build 本功能包。")
        return 1
    command = ["ros2", "launch", "mujoco_car_nn_avoidance", "main.launch.py"]
    print("[启动] {}".format(" ".join(command)))
    # subprocess.run 透传真实退出码，不用 os.system 吞掉失败状态
    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


# ----------------------------------------------------------------------
# 入口
# ----------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="main.py",
        description="MuJoCo 地面差速小车神经网络端到端避障",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--collect", action="store_true", help="采集专家标签数据")
    mode.add_argument("--train", action="store_true", help="训练 MLP 端到端模型")
    mode.add_argument("--eval", action="store_true", help="多场景闭环评测")
    mode.add_argument("--demo", action="store_true", help="离屏录制运行 gif")
    mode.add_argument("--test", action="store_true", help="运行自检（单测 + 场景冒烟）")
    mode.add_argument("--launch", action="store_true", help="调用 ros2 launch 启动闭环")

    parser.add_argument("--seconds", type=float, default=6.0,
                        help="--collect 每个场景的采集时长 (s)")
    parser.add_argument("--epochs", type=int, default=None, help="--train 训练轮数")
    parser.add_argument("--lr", type=float, default=None, help="--train 学习率")
    parser.add_argument("--duration", type=float, default=18.0,
                        help="--demo 仿真时长上限 (s)")
    parser.add_argument("--logic-only", action="store_true",
                        help="配合 --test：只跑不依赖 MuJoCo 的纯逻辑测试")
    parser.add_argument("--version", action="version", version=VERSION_LINE,
                        help="打印版本号后退出")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    print("[版本] {}".format(VERSION_LINE))

    cfg = Config()
    if args.epochs is not None:
        cfg.epochs = args.epochs
    if args.lr is not None:
        cfg.lr = args.lr

    try:
        if args.test:
            return run_test(args)
        if args.launch:
            return run_launch()
        if args.collect:
            return 0 if collect(cfg, seconds=args.seconds) else 1
        if args.train:
            return 0 if train(cfg, cfg.epochs, cfg.lr) else 1
        if args.eval:
            return 0 if evaluate(cfg) else 1
        if args.demo:
            return 0 if demo(cfg, duration_s=args.duration) else 1
    except Exception as exc:  # noqa: BLE001
        print("[错误] {}".format(exc))
        return 1

    build_parser().print_help()
    print("\n请选择一个模式：--collect / --train / --eval / --demo / "
          "--test / --launch")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
