#!/usr/bin/env python3
"""uav_mujoco_depth_avoidance — 课程入口。

    python main.py --demo       训练 + 三场景闭环 + 出图出 GIF（默认）
    python main.py --teleop     键盘遥操作（pygame 窗口，任务 1）
    python main.py --test       单元测试
    python main.py --ros        ROS 2 节点（Humble / Noetic）

Windows 直接双击 main.bat，Linux 执行 ./main.sh，两者都等价于 `--demo`。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def run_demo(out_dir: str, epochs: int, n_train: int, n_val: int, scene: str,
             soak_steps: int = 3000) -> int:
    from uav_nav.env import DepthUavEnv
    from uav_nav.policy import PolicyConfig, run_episode, run_matrix, soak_run
    from uav_nav.train import plot_prediction, plot_training, save_gif, save_report, train

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    print("== 1/4 生成深度数据集并训练 CNN ==")
    env = DepthUavEnv(scene)
    net, history, (x_va, y_va) = train(env, epochs=epochs, n_train=n_train, n_val=n_val)

    print("== 2/4 训练曲线与预测对照图 ==")
    plot_training(history, out / "training_curve.png")
    plot_prediction(net, x_va, y_va, out / "prediction.png")
    weights = net.save(out / "policy.npz")
    print(f"训练好的权重已保存: {weights}（ROS 节点与 --teleop 可直接加载）")

    print("== 3/4 三种场景闭环飞行（含 GIF 录制）==")
    cfg = PolicyConfig()
    results = []
    for name in DepthUavEnv.SCENES:
        res = run_episode(net, name, seed=0, cfg=cfg, record=True)
        results.append(res)
        gif = out / f"flight_{name}.gif"
        size, frames = save_gif(res.frames, gif)
        print(f"[{name}] 成功={res.success} 碰撞={res.crashed} 步数={res.steps} "
              f"前进={res.traveled_x:.2f} m 最小余量={res.min_clearance:.3f} "
              f"预测误差={res.mean_pred_error:.3f} 路径长={res.path_length:.2f} m "
              f"-> {gif.name} ({size/1024:.0f} KB, {frames} 帧)")

    print("== 4/4 多种子稳定性统计、长时间运行与指标落盘 ==")
    matrix = run_matrix(net, seeds=(0, 1, 2, 3, 4))
    soak = soak_run(net, scene="clutter", total_steps=soak_steps)

    summary = save_report(history, {"matrix": matrix, "soak": soak, "scenes": [
        {"scene": r.scene, "success": r.success, "crashed": r.crashed,
         "steps": r.steps, "traveled_x": r.traveled_x, "min_clearance": r.min_clearance,
         "mean_pred_error": r.mean_pred_error, "mean_speed": r.mean_speed,
         "path_length": r.path_length}
        for r in results
    ]}, out)

    n_ok = sum(v["success_rate"] for v in matrix.values())
    print(f"\n参数数量            : {net.config()['params']}")
    print(f"最终 train MSE      : {history[-1]['train_mse']}")
    print(f"验证 MAE(左/中/右)  : {history[-1]['val_mae_left']:.3f} / "
          f"{history[-1]['val_mae_center']:.3f} / {history[-1]['val_mae_right']:.3f}")
    print("多实例成功率(3 场景 × 5 实例):")
    for name, row in matrix.items():
        print(f"  {name:8} 成功 {row['success_rate']*100:5.0f}%  碰撞 {row['collisions']}  "
              f"步数 {row['steps_mean']:5.1f}±{row['steps_std']:4.1f}  "
              f"最小余量 {row['min_clearance_mean']:.3f}(最差 {row['min_clearance_min']:.3f})")
    print(f"长时间运行          : {soak['total_steps']} 步 / {soak['simulated_seconds']} s，"
          f"{soak['episodes']} 次任务，碰撞 {soak['collisions']} 次")
    print(f"指标                : {out / 'metrics.json'}")
    print(f"GIF                 : {out}/flight_*.gif")
    return 0


def build_policy(env, weights: str = "", epochs: int = 8, n_train: int = 600, log=print):
    """构建用于部署的网络：优先加载训练好的权重，没有就现场训练一次。

    关键点：绝不返回随机初始化的网络。之前 `--teleop` / `--ros` 直接 `DepthCNN()`
    拿到的是未训练权重，控制律会给出错误转向（评审实测随机网络 slalom 只有 40% 成功率）。
    """
    from uav_nav.net import DepthCNN

    net = DepthCNN(seed=0)
    if weights:
        net.load(weights)
        log(f"已加载权重: {weights}（{net.config()['params']} 参数）")
        return net

    from uav_nav.train import train

    scene0, seed0 = env.scene, env.seed
    log(f"未提供权重，现场训练 {n_train} 样本 × {epochs} 轮（约 5 s）…")
    net, history, _ = train(env, epochs=epochs, n_train=n_train, n_val=150, log=lambda *a, **k: None)
    env.reset(scene=scene0, seed=seed0)     # 训练会改场景/位姿，这里显式还原（train 内部也会还原）
    log(f"训练完成：最终 train MSE {history[-1]['train_mse']:.5f}，"
        f"验证 MAE(左/中/右) {history[-1]['val_mae_left']:.3f}/"
        f"{history[-1]['val_mae_center']:.3f}/{history[-1]['val_mae_right']:.3f}；"
        f"场景已恢复为 {scene0}/seed={seed0}，起点 (0, 0, 1.5)")
    return net


def run_teleop(scene: str, weights: str = "") -> int:
    from uav_nav.env import DepthUavEnv
    from uav_nav.teleop import run_window

    env = DepthUavEnv(scene)
    print("键盘遥操作：W/S 调速度，A/D 偏航，空格悬停，R 重开，ESC 退出")
    print("（窗口里同时显示追尾视角、深度相机与网络预测，便于对照）")
    net = build_policy(env, weights)
    return run_window(env, net)


def run_tests() -> int:
    import pytest

    here = Path(__file__).resolve().parent
    return int(pytest.main(["-q", str(here / "test")]))


def run_ros() -> int:
    from uav_nav.node import main as node_main

    return node_main()


def main() -> int:
    parser = argparse.ArgumentParser(description="无人机深度相机神经网络避障（MuJoCo）")
    parser.add_argument("--demo", action="store_true", help="训练 + 闭环 + 出图（默认）")
    parser.add_argument("--teleop", action="store_true", help="键盘遥操作窗口")
    parser.add_argument("--test", action="store_true", help="运行单元测试")
    parser.add_argument("--ros", action="store_true", help="启动 ROS 2 节点")
    parser.add_argument("--scene", default="slalom", choices=["slalom", "gate", "clutter"])
    parser.add_argument("--weights", default="", help="训练好的 npz 权重（不给则现场训练）")
    parser.add_argument("--output-dir", default="results")
    parser.add_argument("--epochs", type=int, default=14)
    parser.add_argument("--n-train", type=int, default=900)
    parser.add_argument("--n-val", type=int, default=200)
    parser.add_argument("--soak-steps", type=int, default=3000, help="长时间运行测试的总步数")
    args = parser.parse_args()

    if args.test:
        return run_tests()
    if args.teleop:
        return run_teleop(args.scene, args.weights)
    if args.ros:
        return run_ros()
    return run_demo(args.output_dir, args.epochs, args.n_train, args.n_val, args.scene,
                    args.soak_steps)


if __name__ == "__main__":
    sys.exit(main())
