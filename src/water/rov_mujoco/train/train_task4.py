#!/usr/bin/env python3
"""
任务 4 —— 端到端视觉伺服策略训练 (Behavior Cloning, 纯 NumPy)
==============================================================
数据: 在 MuJoCo 场景中随机摆放 ROV 于管线附近，渲染前视相机 forward_cam
      84x84 图像；专家标签由几何位姿解析给出（横向/航向纠偏 + 前向巡航）。
学生: mlp / small_cnn / nature_cnn 三种架构，全部用 NumPy 反向传播训练。

专家标签:
    vy_des = clip( 1.2 * (pipe_y - y), -0.35, 0.35 )
    wz_des = clip( -1.6 * yaw,          -0.60, 0.60 )
    vx_des = 0.45 * (1 - 0.55 * min(1, |y - pipe_y| / 0.6))
                    * (1 - 0.30 * min(1, |yaw| / 0.28))

运行:
    MUJOCO_GL=osmesa python3 train/train_task4.py [--samples 1200] [--epochs 30]
"""

import argparse
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from rov_mujoco.end_to_end_networks import (  # noqa: E402
    IMG_SIZE, NOMINAL_SPEED, VY_SCALE, WZ_SCALE,
    NumpyConvNet, expert_label_ref, rgb_to_gray_resize, train_network,
)

PIPE_Y = 3.2
PIPE_Z = -1.45
START_X = -3.5
GOAL_X = 3.0


def quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ], dtype=np.float64)


def expert_label(y, yaw):
    """几何专家：返回归一化动作 [out0, out1, out2]"""
    return expert_label_ref(y, yaw)


def collect_dataset(model, samples, seed=0, verbose=True):
    """在管线附近随机位姿渲染前视图像并生成专家标签"""
    import mujoco

    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, IMG_SIZE, IMG_SIZE)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")
    joint_id = model.body_jntadr[body_id]
    qpos_addr = model.jnt_qposadr[joint_id]
    dof_addr = model.jnt_dofadr[joint_id]

    rng = np.random.default_rng(seed)
    images = np.zeros((samples, IMG_SIZE, IMG_SIZE), dtype=np.float32)
    labels = np.zeros((samples, 3), dtype=np.float64)
    poses = np.zeros((samples, 5), dtype=np.float64)  # x, y, z, yaw, dy

    for i in range(samples):
        x = rng.uniform(START_X + 0.2, GOAL_X - 0.3)
        y = PIPE_Y + rng.uniform(-0.60, 0.60)
        z = PIPE_Z + rng.uniform(-0.16, 0.16)
        yaw = rng.uniform(-0.28, 0.28)
        quat = quat_from_euler(0.0, 0.0, yaw)

        data.qpos[:] = model.qpos0
        data.qpos[qpos_addr:qpos_addr + 3] = (x, y, z)
        data.qpos[qpos_addr + 3:qpos_addr + 7] = quat
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)

        renderer.update_scene(data, camera="forward_cam")
        images[i] = rgb_to_gray_resize(renderer.render(), IMG_SIZE)
        labels[i] = expert_label(y, yaw)
        poses[i] = (x, y, z, yaw, y - PIPE_Y)

        if verbose and (i + 1) % 200 == 0:
            print(f"    渲染样本 {i + 1}/{samples}")

    renderer.close()
    return images, labels, poses


def main():
    parser = argparse.ArgumentParser(description="任务 4 端到端视觉策略 BC 训练")
    parser.add_argument('--samples', type=int, default=1200)
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--batch-size', type=int, default=32)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--arch', type=str, default='all',
                        choices=['all', 'mlp', 'small_cnn', 'nature_cnn'])
    parser.add_argument('--seed', type=int, default=2026)
    parser.add_argument('--data-cache', type=str,
                        default=os.path.join(ROOT, 'models', 'task4_bc_dataset.npz'))
    parser.add_argument('--refresh-data', action='store_true',
                        help='忽略缓存，重新渲染数据集')
    args = parser.parse_args()

    import mujoco

    print("=" * 70)
    print("  任务 4: 端到端视觉伺服策略训练 (Behavior Cloning, 纯 NumPy)")
    print("=" * 70)

    model_path = os.path.join(ROOT, 'models', 'underwater_rov_with_arm.xml')
    print(f"  场景模型 : {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    print(f"  相机     : forward_cam, 输出 {IMG_SIZE}x{IMG_SIZE} 灰度")

    if os.path.exists(args.data_cache) and not args.refresh_data:
        with np.load(args.data_cache) as d:
            images, labels, poses = d['images'], d['labels'], d['poses']
        print(f"  数据集缓存: {args.data_cache} ({len(images)} 张)")
    else:
        images, labels, poses = collect_dataset(model, args.samples, seed=args.seed)
        np.savez_compressed(args.data_cache, images=images, labels=labels, poses=poses)
        print(f"  数据集已缓存: {args.data_cache}")
    img_mean = float(images.mean())
    img_std = float(images.std()) + 1e-6
    X = (images - img_mean) / img_std
    print(f"  样本数量 : {len(X)}")
    print(f"  图像统计 : mean={img_mean:.4f}, std={img_std:.4f}")
    print(f"  标签范围 : out0[{labels[:, 0].min():.3f},{labels[:, 0].max():.3f}] "
          f"out1[{labels[:, 1].min():.3f},{labels[:, 1].max():.3f}] "
          f"out2[{labels[:, 2].min():.3f},{labels[:, 2].max():.3f}]")

    archs = ['mlp', 'small_cnn', 'nature_cnn'] if args.arch == 'all' else [args.arch]
    results = {}

    for arch in archs:
        print(f"\n  ── 训练 {arch} ──")
        net, history, (Xva, Yva) = train_network(
            X, labels, architecture=arch, epochs=args.epochs,
            batch_size=args.batch_size, lr=args.lr, seed=args.seed)
        pred = net.forward(Xva)
        mse = float(np.mean((pred - Yva) ** 2))
        corr = [float(np.corrcoef(pred[:, d], Yva[:, d])[0, 1]) for d in range(3)]

        vx_rmse = float(np.sqrt(np.mean((NOMINAL_SPEED * (pred[:, 0] - Yva[:, 0]) / 2) ** 2)))
        vy_rmse = float(np.sqrt(np.mean((VY_SCALE * (pred[:, 1] - Yva[:, 1])) ** 2)))
        wz_rmse = float(np.sqrt(np.mean((WZ_SCALE * (pred[:, 2] - Yva[:, 2])) ** 2)))

        print(f"    验证 MSE : {mse:.6f}")
        print(f"    动作相关系数: {np.round(corr, 4)}")
        print(f"    物理量 RMSE: vx={vx_rmse:.4f} m/s, vy={vy_rmse:.4f} m/s, "
              f"wz={wz_rmse:.4f} rad/s")
        print(f"    参数量   : {net.n_params()}")

        # 闭环最关键的是横向纠偏（out1）与航向（out2），vx 随偏离平滑变化。
        # vx 的标注跨度小且与图像弱相关（主要取决于 |dy|），要求略低。
        assert corr[0] > 0.60, f"{arch}: vx 拟合不足 (corr={corr[0]:.3f})"
        assert corr[1] > 0.90, f"{arch}: vy 横向纠偏拟合不足 (corr={corr[1]:.3f})"
        assert corr[2] > 0.85, f"{arch}: wz 航向拟合不足 (corr={corr[2]:.3f})"

        state = dict(net.state_dict())
        state.update({
            'img_mean': np.float32(img_mean),
            'img_std': np.float32(img_std),
            'val_mse': np.float32(mse),
            'action_corr': np.asarray(corr, dtype=np.float32),
            'action_rmse': np.asarray([vx_rmse, vy_rmse, wz_rmse], dtype=np.float32),
            'arch': np.array(arch),
        })
        out = os.path.join(ROOT, 'models', f'task4_{arch}_bc.npz')
        np.savez(out, **state)
        print(f"    ✅ 权值已保存: {out}")
        results[arch] = {'mse': mse, 'corr': corr, 'n_params': net.n_params()}

    print("\n  ── 三架构对比 (验证集) ──")
    for arch, r in results.items():
        print(f"    {arch:<11} params={r['n_params']:>9d} | MSE={r['mse']:.6f} | "
              f"corr={np.round(r['corr'], 3)}")
    print("=" * 70)


if __name__ == '__main__':
    main()
