"""dataset.py —— 采集「3 帧时序图像 → 老师关节目标角」的行为克隆数据

原实现有三处会让数据集完全失效的问题，这里逐条改掉：

1. ``if sim.fallen:`` —— ``fallen`` 是方法，不加括号恒为真，循环第一轮就 break，
   导致 ``samples=0``、``np.stack([])`` 直接抛异常（实测 ``--collect`` 必崩）。
2. 标签 ``target = DEFAULT_STAND_TARGET.copy()`` —— 所有样本的 y 完全相同，
   网络最优解就是输出常数，学不到任何东西（MSE 还会“收敛得很好”）。
3. 输入取自扰动之前（站定状态），图像恒定。

现在的口径：老师（``sim.teacher_target()``）用踝策略 + 大倾角下蹲闭环站住，
数据集记录「老师在该状态下实际会给出的 15 维目标角」，这才是可学的端到端映射。

防泄漏：帧间隔 ``FRAME_STRIDE``=10 步（0.02 s），样本间隔 ``SAMPLE_STRIDE``=30 步，
相邻样本的 3 帧窗口不重叠。
"""
from __future__ import annotations

import os
import numpy as np
from .sim import TemporalE2ESim

POOL = 16            # 每帧 64×64 → 16×16 池化 → 256 维
N_FRAMES = 3         # 时序窗口长度
FRAME_STRIDE = 10    # 每 10 步取一帧（0.02 s）
SAMPLE_STRIDE = 30   # 每 30 步产出一个样本 → 相邻样本的 3 帧窗口不重叠
SETTLE_STEPS = 300   # 采集前先让老师站定
EPISODE_STEPS = 900  # 每 episode 最长 900 步（1.8 s）


def feat_from_gray(img, size=POOL):
    """64×64 灰度 → 16×16 均值池化 → 256 维。训练/推理/节点共用同一份实现。"""
    h, w = img.shape
    bh, bw = h // size, w // size
    crop = img[:bh * size, :bw * size]
    return crop.reshape(size, bh, size, bw).mean(axis=(1, 3)).flatten().astype(np.float32)


# 兼容旧调用名
_downsample = feat_from_gray


def collect(model_path, out_path, n_samples=2000, seed=0, max_episodes=200,
            verbose=True):
    """采集行为克隆数据集。

    Args:
        n_samples: 目标样本数（到量即停）
        max_episodes: 兜底的最大 episode 数（防止一直采不满时死循环）
    """
    rng = np.random.default_rng(seed)
    sim = TemporalE2ESim(model_path)
    X, y, meta = [], [], []
    ep = 0
    while len(X) < n_samples and ep < max_episodes:
        sim.reset()
        sim.settle(SETTLE_STEPS)      # 老师闭环站定

        # 1~3 次随机推力，方向 0~360°，力 20~70 N，开始时刻散布在 0~700 步
        # 上限 70 N 的依据：本机型老师闭环在 50 N 仍能站住，80 N 起必倒
        # （15 关节、无迈步能力，纯 PD + 踝策略的能力边界就在这里）。
        # 数据集只覆盖“老师能救回”的区间，否则采到的是必然跌倒的轨迹。
        n_push = int(rng.integers(1, 4))
        plan = []
        for _ in range(n_push):
            start = int(rng.integers(0, 700))
            mag = float(rng.uniform(20.0, 70.0))
            ang = float(rng.uniform(0.0, 360.0))
            plan.append((start, start + 40,
                        np.array([mag * np.cos(np.radians(ang)),
                                  mag * np.sin(np.radians(ang)), 0.0])))

        frames, ep_n = [], 0
        for t in range(EPISODE_STEPS):
            # 叠加当前所有激活的推力
            f_sum = np.zeros(3)
            for start, end, f in plan:
                if start <= t < end:
                    f_sum += f
            if np.any(f_sum):
                sim.set_push(f_sum)
            else:
                sim.clear_push()

            if t % FRAME_STRIDE == 0:
                frames.append(feat_from_gray(sim.render_front(gray=True)))

            if (t % SAMPLE_STRIDE == SAMPLE_STRIDE - 1 and len(frames) >= N_FRAMES):
                x_seq = np.concatenate(frames[-N_FRAMES:]).astype(np.float32)
                y_seq = sim.teacher_target().astype(np.float32)
                X.append(x_seq)
                y.append(y_seq)
                ep_n += 1

            sim.step(balance=True)
            if sim.fallen():
                break
        sim.clear_push()

        if verbose and ep % 10 == 0:
            print(f"  [collect] ep {ep} | +{ep_n} samples | total={len(X)}")
        ep += 1

    if not X:
        raise RuntimeError("采集到 0 个样本：请检查 sim.teacher_target() 是否可用")
    X = np.stack(X).astype(np.float32)
    y = np.array(y, dtype=np.float32)

    # 数据体检：标签若恒定，说明又退化成“常数站姿”了
    y_std = y.std(axis=0)
    uniq_rows = len(np.unique(X, axis=0))

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    np.savez(out_path, X=X, y=y,
             frame_stride=np.array(FRAME_STRIDE),
             sample_stride=np.array(SAMPLE_STRIDE))

    sim.close()
    print(f"[collect] saved {out_path}")
    print(f"  X={X.shape} y={y.shape} episodes={ep}")
    print(f"  图像唯一行 {uniq_rows}/{len(X)} | 标签逐维 std 中位 {np.median(y_std):.4f}"
          f" (min {y_std.min():.4f}, max {y_std.max():.4f})")
    if np.median(y_std) < 1e-3:
        raise RuntimeError("标签几乎恒定：数据集退化，拒绝输出")
    return out_path
