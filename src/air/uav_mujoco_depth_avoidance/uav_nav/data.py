"""数据集生成：随机位置/朝向 + 随机场景，用 MuJoCo 渲染深度图并给出扇区标签。

标签不经过任何手工规则，直接来自物理渲染的深度图：

    y_k = min(depth[第 k 个扇区]) / D_MAX,  k ∈ {左, 中, 右}

因此网络学的目标是"从 64×64 深度图里, 判断三个方向各还有多少余量"。
"""

from __future__ import annotations

import numpy as np

from .env import D_MAX, DepthUavEnv

NET_HW = 32          # 网络输入边长（64×64 深度图 2×2 平均池化）
Y_MIN, Y_MAX = -2.0, 2.0
X_MIN, X_MAX = 1.0, 13.0


def to_net_input(depth: np.ndarray) -> np.ndarray:
    """64×64 米制深度 -> 32×32 通行度 (1 - d/D_MAX)，2×2 平均池化。"""
    d = np.clip(depth, 0.0, D_MAX) / D_MAX
    h, w = d.shape
    pooled = d.reshape(h // 2, 2, w // 2, 2).mean(axis=(1, 3))
    return (1.0 - pooled).astype(np.float32)


def sample_scene(env: DepthUavEnv, rng: np.random.Generator) -> None:
    """把飞机随机放到场景里（不撞障碍、朝向大致朝 +x）。"""
    for _ in range(50):
        x = float(rng.uniform(X_MIN, X_MAX))
        y = float(rng.uniform(Y_MIN, Y_MAX))
        if np.min(np.abs(env.model.geom_pos[env.post_ids, 0] - x)
                  + np.abs(env.model.geom_pos[env.post_ids, 1] - y)) > 0.7:
            break
    yaw = float(rng.uniform(-0.5, 0.5))
    env.reset(scene=env.scene, seed=env.seed, pos=(x, y, 1.5), yaw=yaw)


def make_dataset(env: DepthUavEnv, n: int, seed: int = 0, scenes=None,
                 with_images: bool = True):
    """返回 (X, Y)。X 形状 (n, 32, 32, 1)，Y 形状 (n, 3)。"""
    rng = np.random.default_rng(seed)
    scenes = scenes or list(DepthUavEnv.SCENES)
    xs, ys = [], []
    for i in range(n):
        env.set_scene(scenes[i % len(scenes)], seed=seed + i)
        sample_scene(env, rng)
        depth = env.depth()
        xs.append(to_net_input(depth))
        ys.append(env.sector_clearance(depth))
    x = np.stack(xs)[..., None].astype(np.float32) if with_images else None
    y = np.stack(ys).astype(np.float32)
    return x, y
