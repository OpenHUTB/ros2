"""示范数据：专家沿“到目标距离图”下降，按 episode 切分。"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np

from .encoder import N_OBS, encode
from .expert import LOOKAHEAD, steer, target_point
from .grid import build_grid, planning_grid, sample_free_pose, to_cell
from .sim import collides, step
from .value import descend_path, distance_map

GOAL_TOL = 0.25
MIN_START_GOAL = 2.0

__all__ = ["collect", "dagger", "merge", "split", "policy_action", "Dataset"]


class Dataset:
    def __init__(self, x, y, episode, stats: Dict | None = None):
        self.x = np.asarray(x, np.float32)
        self.y = np.asarray(y, np.float32)
        self.episode = np.asarray(episode, np.int32)
        self.stats = stats or {}

    def __len__(self):
        return int(len(self.x))


def policy_action(net, obs, lookahead: float = LOOKAHEAD):
    """把网络的归一化前视点输出转成 (v, ω)。

    DAgger 采样和闭环评测必须走同一条转换路径，否则采到的状态
    不是策略真实运行时会到达的状态。
    """
    p = net.predict(obs)
    return steer(float(p[0]) * lookahead, float(p[1]) * lookahead)


def _plan(g, start_xy, goal_xy):
    """目标距离图 + 从起点下降的路径。"""
    dist = distance_map(g, goal_xy)
    ci, cj = to_cell(g, goal_xy[0], goal_xy[1])
    if not np.isfinite(dist[cj, ci]):
        return None, None
    path = descend_path(dist, g, start_xy)
    return dist, path


def collect(episodes: int = 120, seed: int = 0, max_steps: int = 400,
            verbose: bool = True) -> Dataset:
    """每个 episode 从随机起点导航到随机目标；只保留专家成功到达的片段。"""
    g_raw = build_grid()        # 碰撞检测用原始地图
    g = planning_grid()         # 规划与观测用膨胀地图
    rng = np.random.default_rng(seed)

    xs: List[np.ndarray] = []
    ys: List[np.ndarray] = []
    eps: List[int] = []
    done = 0
    attempts = 0

    while done < episodes and attempts < episodes * 20:
        attempts += 1
        sx, sy, sth = sample_free_pose(g, rng)
        gx, gy, _ = sample_free_pose(g, rng)
        if collides(g, (sx, sy, sth)):
            continue
        if math.hypot(gx - sx, gy - sy) < MIN_START_GOAL:
            continue

        dist, path = _plan(g, (sx, sy), (gx, gy))
        if path is None:
            continue

        pose = (sx, sy, sth)
        buf_x, buf_y = [], []
        ok = False
        for _ in range(max_steps):
            buf_x.append(encode(g, pose, (gx, gy), dist))
            fwd, lat = target_point(path, pose)
            buf_y.append((fwd / LOOKAHEAD, lat / LOOKAHEAD))
            v, w = steer(fwd, lat)
            pose = step(pose, v, w)
            if collides(g_raw, pose):
                break
            if math.hypot(pose[0] - gx, pose[1] - gy) < GOAL_TOL:
                ok = True
                break

        if not ok or len(buf_x) < 10:
            continue

        xs.extend(buf_x)
        ys.extend(buf_y)
        eps.extend([done] * len(buf_x))
        done += 1
        if verbose and done % 25 == 0:
            print("  已采集 %d/%d 个 episode" % (done, episodes))

    if not xs:
        raise RuntimeError("没有采到任何样本，检查地图或专家策略")

    return Dataset(np.stack(xs), np.stack(ys), np.array(eps),
                   {"episodes": done, "samples": len(xs)})


def dagger(net, episodes: int = 25, seed: int = 1,
           max_steps: int = 400) -> Dataset:
    """DAgger：按策略自己的轨迹采样，每个状态都问专家该输出什么。

    行为克隆只在专家轨迹上训练，策略一旦偏离就进入没见过的状态；
    这里用策略自己的状态分布重新标注，把偏差纠正回来。
    """
    g_raw = build_grid()
    g = planning_grid()
    rng = np.random.default_rng(seed)

    xs: List[np.ndarray] = []
    ys: List[np.ndarray] = []
    eps: List[int] = []
    done = 0
    attempts = 0

    while done < episodes and attempts < episodes * 20:
        attempts += 1
        sx, sy, sth = sample_free_pose(g, rng)
        gx, gy, _ = sample_free_pose(g, rng)
        if collides(g, (sx, sy, sth)):
            continue
        if math.hypot(gx - sx, gy - sy) < MIN_START_GOAL:
            continue

        dist, path = _plan(g, (sx, sy), (gx, gy))
        if path is None:
            continue

        pose = (sx, sy, sth)
        kept = 0
        for _ in range(max_steps):
            obs = encode(g, pose, (gx, gy), dist)
            cur = descend_path(dist, g, pose[:2])      # 当前状态问专家
            if cur is None:
                break
            fwd, lat = target_point(cur, pose)
            xs.append(obs)
            ys.append((fwd / LOOKAHEAD, lat / LOOKAHEAD))
            eps.append(done)
            kept += 1

            v, w = policy_action(net, obs)
            v += float(rng.normal(0.0, 0.10))       # 探索噪声加在速度上
            w += float(rng.normal(0.0, 0.15))
            pose = step(pose, max(0.0, v), w)
            if collides(g_raw, pose):
                break
            if math.hypot(pose[0] - gx, pose[1] - gy) < GOAL_TOL:
                break

        if kept:
            done += 1

    if not xs:
        return Dataset(np.zeros((0, N_OBS), np.float32),
                       np.zeros((0, 2), np.float32), np.zeros((0,), np.int32))
    return Dataset(np.stack(xs), np.stack(ys), np.array(eps),
                   {"dagger_samples": len(xs)})


def merge(base: Dataset, extra: Dataset, offset: int = 100000) -> Dataset:
    """把 DAgger 样本接到原数据集后面，episode 编号加偏移避免混在一起。"""
    if len(extra) == 0:
        return base
    stats = dict(base.stats)
    stats.update(extra.stats)
    return Dataset(
        np.concatenate([base.x, extra.x]),
        np.concatenate([base.y, extra.y]),
        np.concatenate([base.episode, extra.episode + offset]),
        stats)


def split(ds: Dataset, val_frac: float = 0.2,
          seed: int = 0) -> Tuple[Dataset, Dataset]:
    """按 episode 切分，避免相邻帧泄漏。"""
    ids = np.unique(ds.episode)
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(ids))
    n_val = max(1, int(round(len(ids) * val_frac)))
    val_ids = set(ids[order[:n_val]].tolist())

    va = np.array([int(e) in val_ids for e in ds.episode])
    tr = ~va
    trs = dict(ds.stats)
    trs.update({"train_samples": int(tr.sum()), "val_samples": int(va.sum())})
    return (Dataset(ds.x[tr], ds.y[tr], ds.episode[tr], trs),
            Dataset(ds.x[va], ds.y[va], ds.episode[va], trs))
