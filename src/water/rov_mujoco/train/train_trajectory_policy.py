#!/usr/bin/env python3
"""
任务 2 —— 神经网络轨迹跟踪策略训练 (Behavior Cloning)
======================================================
专家: LOSTrajectoryController (视线法 + 解耦 PID)
学生: 10 -> 64 -> 64 -> 4 全连接网络 (tanh)

流程:
  1. 在状态空间内采样 (位置误差 / 速度误差 / 偏航误差 / 期望速度)；
  2. 用 LOS 专家生成归一化推力标签 [fx/F, fy/F, fz/Fz, tau/T]；
  3. 用纯 NumPy 反向传播 + Adam 训练 MLP；
  4. 保存到 rov_mujoco/nn_weights.npz（ROS 运行时直接加载）。

运行:
    python3 train/train_trajectory_policy.py [--samples 60000] [--epochs 80]
"""

import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from rov_mujoco.nn_core import NumpyMLP, train_mlp, save_npz  # noqa: E402
from rov_mujoco.nn_trajectory_controller import STATE_SCALE  # noqa: E402

FORCE_SCALE = 350.0
DEPTH_SCALE = 500.0
TORQUE_SCALE = 80.0


def make_dataset(n, seed=0):
    """采样状态并计算 LOS 专家的归一化控制量标签"""
    rng = np.random.default_rng(seed)

    # 70% 全域均匀采样 + 30% 小误差聚焦采样（闭环大部分时间在零附近）
    n_far = int(n * 0.7)
    n_near = n - n_far

    def sample_pos(m):
        return np.concatenate([
            rng.uniform(-1.5, 1.5, (m, 2)),
            rng.uniform(-1.0, 1.0, (m, 1)),
        ], axis=1)

    def sample_pos_near(m):
        return np.clip(rng.normal(0.0, 0.12, (m, 3)), -0.6, 0.6)

    err_pos = np.vstack([sample_pos(n_far), sample_pos_near(n_near)])
    err_vel = rng.uniform(-0.8, 0.8, (n, 3))
    err_yaw = rng.uniform(-np.pi, np.pi, n)
    vel_d = rng.uniform(-0.6, 0.6, (n, 2))

    # 与 NNTrajectoryController.compute 完全一致的 10 维特征
    # 与在线推理共用 STATE_SCALE，训练/部署输入必须逐位一致
    state = (np.column_stack([
        err_pos, err_vel,
        np.sin(err_yaw), np.cos(err_yaw),
        vel_d,
    ]) * STATE_SCALE).astype(np.float32)

    # LOS 专家律（与 LOSTrajectoryController 相同增益与限幅）
    ex, ey, ez = err_pos[:, 0], err_pos[:, 1], err_pos[:, 2]
    evx, evy, evz = err_vel[:, 0], err_vel[:, 1], err_vel[:, 2]
    vxd, vyd = vel_d[:, 0], vel_d[:, 1]

    fx = np.clip(280.0 * ex + 85.0 * evx + 18.0 * vxd, -FORCE_SCALE, FORCE_SCALE)
    fy = np.clip(280.0 * ey + 85.0 * evy + 18.0 * vyd, -FORCE_SCALE, FORCE_SCALE)
    fz = np.clip(480.0 * ez + 140.0 * evz, -DEPTH_SCALE, DEPTH_SCALE)
    tau = np.clip(120.0 * np.sin(err_yaw), -TORQUE_SCALE, TORQUE_SCALE)

    action = np.column_stack([
        fx / FORCE_SCALE,
        fy / FORCE_SCALE,
        fz / DEPTH_SCALE,
        tau / TORQUE_SCALE,
    ]).astype(np.float32)

    return state, action


def main():
    parser = argparse.ArgumentParser(description="训练任务 2 神经网络轨迹跟踪策略")
    parser.add_argument('--samples', type=int, default=60000)
    parser.add_argument('--epochs', type=int, default=80)
    parser.add_argument('--batch-size', type=int, default=256)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--seed', type=int, default=2026)
    parser.add_argument('--out', type=str,
                        default=os.path.join(ROOT, 'rov_mujoco', 'nn_weights.npz'))
    args = parser.parse_args()

    print("=" * 66)
    print("  任务 2: 神经网络轨迹跟踪策略训练 (NumPy, BC)")
    print("=" * 66)
    print(f"  网络结构 : 10 -> 64 -> 64 -> 4 (tanh)")
    print(f"  样本数量 : {args.samples}")
    print(f"  优化器   : Adam lr={args.lr}, batch={args.batch_size}, epochs={args.epochs}")

    X, Y = make_dataset(args.samples, seed=args.seed)
    print(f"  输入范围 : [{X.min():.2f}, {X.max():.2f}] (已按 STATE_SCALE 归一化)")
    print(f"  标签均值 : {np.round(Y.mean(axis=0), 4)}")
    print(f"  标签标准差: {np.round(Y.std(axis=0), 4)}")

    # 按 8:2 划分训练/验证（train_mlp 内部固定划分）
    model, history = train_mlp(
        X, Y,
        sizes=[10, 64, 64, 4],
        hidden_act='tanh',
        output_act='tanh',
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        val_split=0.2,
        seed=args.seed,
    )

    # ── 验证集物理量误差 ───────────────────────────────
    rng = np.random.default_rng(args.seed + 1)
    Xv, Yv = make_dataset(4000, seed=args.seed + 7)
    pred = model.forward(Xv)
    mse = model.mse(pred, Yv)
    phys_scale = np.array([FORCE_SCALE, FORCE_SCALE, DEPTH_SCALE, TORQUE_SCALE])
    rmse_phys = np.sqrt(np.mean((pred - Yv) ** 2, axis=0)) * phys_scale
    print("\n  ── 验证集结果 ──")
    print(f"  归一化 MSE : {mse:.6f}")
    print(f"  物理量 RMSE: fx={rmse_phys[0]:.2f} N, fy={rmse_phys[1]:.2f} N, "
          f"fz={rmse_phys[2]:.2f} N, tau_z={rmse_phys[3]:.2f} N·m")
    print(f"  参数量     : {model.n_params()}")

    # 专家与学生的动作相关系数（逐维）
    corr = []
    for d in range(4):
        c = np.corrcoef(pred[:, d], Yv[:, d])[0, 1]
        corr.append(float(c))
    print(f"  动作相关系数: {np.round(corr, 4)}")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    save_npz(args.out, model, extra={
        'val_mse': np.float32(mse),
        'val_rmse_fx': np.float32(rmse_phys[0]),
        'val_rmse_fy': np.float32(rmse_phys[1]),
        'val_rmse_fz': np.float32(rmse_phys[2]),
        'val_rmse_tau': np.float32(rmse_phys[3]),
        'state_scale': STATE_SCALE,
    })
    hist_path = os.path.join(ROOT, 'models', 'trajectory_training_history.npz')
    os.makedirs(os.path.dirname(hist_path), exist_ok=True)
    np.savez(hist_path,
             train_loss=np.array(history['train_loss']),
             val_loss=np.array(history['val_loss']))
    print(f"\n  ✅ 权值已保存: {args.out}")
    print(f"  ✅ 训练曲线已保存: {hist_path}")

    # 训练必须是真的收敛，而不是随机初始化：验证 MSE 与初始值比较
    init_model = NumpyMLP([10, 64, 64, 4], seed=args.seed)
    init_mse = init_model.mse(init_model.forward(Xv), Yv)
    print(f"  随机初始化 MSE={init_mse:.6f} -> 训练后 MSE={mse:.6f} "
          f"(下降 {100.0 * (1.0 - mse / init_mse):.2f}%)")
    assert mse < init_mse * 0.05, "训练未显著优于随机初始化，请检查训练流程"

    print("=" * 66)


if __name__ == '__main__':
    main()
