#!/usr/bin/env python3
"""
任务 3 —— 局部避障策略网络训练 (Behavior Cloning)
==================================================
专家: 声呐势场法 (目标吸引 + 障碍排斥 + 安全减速)
学生: 76 -> 128 -> 64 -> 3 MLP (tanh)

输入 (与 NeuralLocalAvoidancePolicy.compute_cmd 完全一致):
  [0:72]  72 束声呐距离 / 15 m
  [72]    目标距离 / 10 m
  [73]    sin(目标相对方位角)
  [74]    cos(目标相对方位角)
  [75]    当前前向速度 / max_speed

输出:
  [0] 归一化前向速度  vx = max_speed * (out0 + 1) / 2
  [1] 归一化横移速度  vy = 0.20 * out1
  [2] 归一化偏航角速度 wz = 0.55 * out2

运行:
    python3 train/train_local_policy.py [--samples 30000] [--epochs 50]
"""

import argparse
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from rov_mujoco.nn_core import Adam, NumpyMLP, save_npz  # noqa: E402

FOV = 2.0 * math.pi / 3.0
NUM_BEAMS = 72
BEAM_ANGLES = np.linspace(-FOV / 2.0, FOV / 2.0, NUM_BEAMS)
MAX_RANGE = 15.0
SAFE_MARGIN = 0.65
MAX_SPEED = 0.6
WZ_SCALE = 1.2          # 偏航输出归一化尺度 [rad/s]，与运行时共用


def expert_label(ranges, rel_angle):
    """声呐势场专家：返回归一化 (out0, out1, out2)"""
    r = np.clip(np.asarray(ranges, dtype=np.float64), 0.05, MAX_RANGE)
    min_dist = float(r.min())

    # 障碍排斥向量（机体坐标系；波束正角度为左舷 +y）。
    # 不做 w.sum() 归一化，使标签对每束距离近似可加，便于 MLP 学习。
    w = np.clip((SAFE_MARGIN - r) / SAFE_MARGIN, 0.0, 1.0)
    rep_x = -float(np.sum(w * np.cos(BEAM_ANGLES)) / 4.0)
    rep_y = -float(np.sum(w * np.sin(BEAM_ANGLES)) / 3.0)

    tx, ty = math.cos(rel_angle), math.sin(rel_angle)
    des_x = tx + rep_x
    des_y = ty + rep_y

    # 近障时按左右舷开阔度打破对称；正前方对称障碍也给出确定的转向偏好，
    # 避免专家标签在完全对称场景下恒为直行。
    if min_dist < SAFE_MARGIN:
        left_min = float(np.min(r[36:]))
        right_min = float(np.min(r[:36]))
        des_y += 0.35 * float(np.clip((left_min - right_min) / SAFE_MARGIN, -1.0, 1.0))
        # 固定左转偏好：让“完全对称的正前方障碍”也有确定标签，
        # 否则专家在对称场景下方向不确定，BC 会学到相互抵消的均值。
        des_y += 0.10

    vx_norm = float(np.clip(des_x, 0.0, 1.0))
    if min_dist < SAFE_MARGIN:
        vx_norm *= float(np.clip((min_dist - 0.25) / (SAFE_MARGIN - 0.25), 0.05, 1.0))
    if min_dist < 0.35:
        vx_norm = 0.0

    vy = 0.20 * float(np.clip(0.8 * rep_y, -1.0, 1.0))
    heading = math.atan2(des_y, des_x) if (abs(des_x) + abs(des_y)) > 1e-9 else 0.0
    wz = float(np.clip(1.5 * heading, -0.55, 0.55))

    out0 = 2.0 * (MAX_SPEED * vx_norm) / MAX_SPEED - 1.0
    out1 = vy / 0.20
    out2 = wz / WZ_SCALE
    return out0, out1, out2


def state_vector(ranges, dist_target, rel_angle, current_vx):
    state = np.zeros(76, dtype=np.float32)
    state[:NUM_BEAMS] = np.clip(np.asarray(ranges) / MAX_RANGE, 0.0, 1.0)
    state[72] = np.clip(dist_target / 10.0, 0.0, 1.0)
    state[73] = math.sin(rel_angle)
    state[74] = math.cos(rel_angle)
    state[75] = np.clip(current_vx / MAX_SPEED, -1.0, 1.0)
    return state


def make_dataset(n, seed=0):
    rng = np.random.default_rng(seed)
    X = np.zeros((n, 76), dtype=np.float32)
    Y = np.zeros((n, 3), dtype=np.float32)

    for i in range(n):
        # 35% 结构化样本：均匀背景 + 单个清晰障碍扇区，
        # 覆盖“正前方对称障碍”“单侧障碍”等安全关键场景；
        # 其余为随机背景 + 多障碍，增加多样性。
        structured = rng.random() < 0.35
        if structured:
            ranges = np.full(NUM_BEAMS, rng.uniform(5.0, MAX_RANGE))
        else:
            ranges = rng.uniform(2.0, MAX_RANGE, NUM_BEAMS)

        # 0~3 个障碍扇区（结构化样本至少 1 个）
        n_obs = 1 if structured else int(rng.integers(0, 4))
        for _ in range(n_obs):
            center = int(rng.integers(0, NUM_BEAMS))
            half = int(rng.integers(2, 10))
            lo, hi = max(0, center - half), min(NUM_BEAMS, center + half + 1)
            if structured and rng.random() < 0.5:
                d = float(rng.uniform(0.30, 1.20))
            else:
                d = float(rng.uniform(0.30, 4.0))
            if structured:
                ranges[lo:hi] = d
            else:
                ranges[lo:hi] = d + rng.normal(0.0, 0.03, hi - lo)

        # 8% 样本加入孤立近距杂波
        if rng.random() < 0.08:
            idx = rng.integers(0, NUM_BEAMS, 3)
            ranges[idx] = rng.uniform(0.25, 1.0, 3)

        ranges = np.clip(ranges, 0.2, MAX_RANGE)
        rel_angle = float(rng.uniform(-math.pi, math.pi))
        dist_target = float(rng.uniform(0.2, 10.0))
        current_vx = float(rng.uniform(-MAX_SPEED, MAX_SPEED))

        X[i] = state_vector(ranges, dist_target, rel_angle, current_vx)
        Y[i] = expert_label(ranges, rel_angle)

    return X, Y


def main():
    parser = argparse.ArgumentParser(description="训练局部避障策略网络")
    parser.add_argument('--samples', type=int, default=30000)
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--batch-size', type=int, default=256)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--seed', type=int, default=2026)
    parser.add_argument('--out', type=str,
                        default=os.path.join(ROOT, 'rov_mujoco', 'local_policy_weights.npz'))
    args = parser.parse_args()

    print("=" * 66)
    print("  任务 3: 局部避障策略网络训练 (NumPy, 势场专家蒸馏)")
    print("=" * 66)
    print("  网络结构 : 76 -> 128 -> 64 -> 3 (tanh)")

    X, Y = make_dataset(args.samples, seed=args.seed)
    print(f"  样本数量 : {X.shape[0]}")
    print(f"  标签均值 : {np.round(Y.mean(axis=0), 4)}")
    print(f"  标签标准差: {np.round(Y.std(axis=0), 4)}")

    rng = np.random.default_rng(args.seed)
    idx = rng.permutation(len(X))
    n_val = max(1, int(len(X) * 0.2))
    val_idx, tr_idx = idx[:n_val], idx[n_val:]
    Xtr, Ytr = X[tr_idx], Y[tr_idx]
    Xva, Yva = X[val_idx], Y[val_idx]

    # 近障样本过采样：安全关键状态在数据中占比不高，重复 2 倍以提升
    # 网络在 0.65 m 内的转向/减速精度（BC 的均方损失会向多数样本平均）。
    near = (Xtr[:, :72].min(axis=1) * MAX_RANGE) < SAFE_MARGIN
    if near.any():
        Xtr = np.vstack([Xtr, Xtr[near], Xtr[near]])
        Ytr = np.vstack([Ytr, Ytr[near], Ytr[near]])
        print(f"  近障样本过采样: {int(near.sum())} -> {int(near.sum()) * 3} "
              f"(总训练样本 {len(Xtr)})")

    model = NumpyMLP([76, 128, 64, 3], hidden_act='tanh',
                     output_act='tanh', seed=args.seed)
    opt = Adam(lr=args.lr)
    history = {'train_loss': [], 'val_loss': []}

    for epoch in range(args.epochs):
        perm = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(Xtr), args.batch_size):
            b = perm[start:start + args.batch_size]
            pred = model.forward(Xtr[b])
            losses.append(model.mse(pred, Ytr[b]))
            grads = model.backward(Ytr[b])
            opt.step(model.W + model.b,
                     [g[0] for g in grads] + [g[1] for g in grads])
        train_loss = float(np.mean(losses))
        val_loss = model.mse(model.forward(Xva), Yva)
        history['train_loss'].append(train_loss)
        history['val_loss'].append(val_loss)
        if epoch % 10 == 0 or epoch == args.epochs - 1:
            print(f"  epoch {epoch + 1:3d}/{args.epochs} | train MSE={train_loss:.6f} | "
                  f"val MSE={val_loss:.6f}")

    pred = model.forward(Xva)
    mse = model.mse(pred, Yva)
    corr = [float(np.corrcoef(pred[:, d], Yva[:, d])[0, 1]) for d in range(3)]
    print("\n  ── 验证集结果 ──")
    print(f"  归一化 MSE : {mse:.6f}")
    print(f"  动作相关系数: {np.round(corr, 4)}")
    print(f"  参数量     : {model.n_params()}")
    # vy 标签在无障碍时恒为 0（稀疏），相关系数天然偏低；主控制量 vx/wz 要求更高。
    assert corr[0] > 0.80 and corr[2] > 0.82, "局部策略网络对 vx/wz 的拟合不足"
    assert corr[1] > 0.25, "局部策略网络对横移量的拟合过低"

    # 安全行为抽查
    def probe(label, obstacle_slice, rel=0.0, dist=3.0):
        r = np.full(NUM_BEAMS, 10.0)
        if obstacle_slice is not None:
            r[obstacle_slice] = 0.5
        s = state_vector(r, dist, rel, 0.3)
        o = model.forward(s.reshape(1, -1))[0]
        e = expert_label(r, rel)
        print(f"    {label:<14} net out={np.round(o, 3)} -> vx={(o[0] + 1) * 0.5 * MAX_SPEED:.3f} m/s, "
              f"wz={WZ_SCALE * o[2]:+.3f} rad/s | expert wz={WZ_SCALE * e[2]:+.3f}")
        return o

    print("\n  ── 行为抽查 (目标正前方 3 m) ──")
    o_free = probe("无遮挡", None)
    o_front = probe("正前方 0.5 m", slice(30, 42))
    o_left = probe("左前方 0.5 m", slice(54, 66))
    o_right = probe("右前方 0.5 m", slice(6, 18))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    save_npz(args.out, model, extra={
        'val_mse': np.float32(mse),
        'action_corr': np.asarray(corr, dtype=np.float32),
    })
    print(f"\n  ✅ 权值已保存: {args.out}")

    assert float(o_front[0]) < float(o_free[0]), "近障时前向速度未降低"
    # 完全对称的正前方障碍是 BC 的最难点（方向不确定）；这里只要求网络
    # 产生同向转向，0.5 m 内的强转向由运行时安全层兜底。
    assert float(o_front[2]) > 0.05, "正前方近障时未学到绕障转向"
    assert float(o_left[2]) < 0.0 and float(o_right[2]) > 0.0, "左右障碍的转向方向不明确"
    print("=" * 66)


if __name__ == '__main__':
    main()
