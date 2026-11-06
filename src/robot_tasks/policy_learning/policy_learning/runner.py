"""闭环评测：网络策略与专家策略在同一批起点/目标上的表现。"""
from __future__ import annotations

import math
import time
from typing import Dict, List

import numpy as np

from .encoder import encode
from .dataset import policy_action
from .expert import pure_pursuit
from .grid import build_grid, planning_grid, sample_free_pose, to_cell
from .sim import collides, step
from .value import descend_path, distance_map

GOAL_TOL = 0.25

__all__ = ["sample_pair", "run_episode", "evaluate", "latency"]


def sample_pair(g, rng, min_dist=2.0, tries=300):
    for _ in range(tries):
        sx, sy, sth = sample_free_pose(g, rng)
        gx, gy, _ = sample_free_pose(g, rng)
        if collides(g, (sx, sy, sth)):
            continue
        if math.hypot(gx - sx, gy - sy) < min_dist:
            continue
        ci, cj = to_cell(g, gx, gy)
        if np.isfinite(distance_map(g, (gx, gy))[cj, ci]):
            return (sx, sy, sth), (gx, gy)
    return None


def run_episode(g_raw, controller, start, goal, max_steps=400) -> Dict:
    pose = start
    length = 0.0
    for step_i in range(max_steps):
        v, w = controller(pose)
        prev = pose
        pose = step(pose, v, w)
        length += math.hypot(pose[0] - prev[0], pose[1] - prev[1])
        if collides(g_raw, pose):
            return {"success": False, "reason": "collision",
                    "steps": step_i + 1, "length": length}
        if math.hypot(pose[0] - goal[0], pose[1] - goal[1]) < GOAL_TOL:
            return {"success": True, "reason": "arrived",
                    "steps": step_i + 1, "length": length}
    return {"success": False, "reason": "timeout", "steps": max_steps,
            "length": length}


def evaluate(net, n_episodes=50, seed=5000, max_steps=400) -> Dict:
    """网络策略与专家策略用完全相同的起点和终点。"""
    g_raw = build_grid()
    g = planning_grid()
    rng = np.random.default_rng(seed)

    eps = []
    while len(eps) < n_episodes:
        pair = sample_pair(g, rng)
        if pair is None:
            break
        start, goal = pair
        dist = distance_map(g, goal)
        path = descend_path(dist, g, start[:2])
        if path is None:
            continue
        eps.append((start, goal, dist, path))

    out = {}
    for kind in ("expert", "net"):
        ok = col = to = 0
        lengths: List[float] = []
        steps: List[int] = []
        for start, goal, dist, path in eps:
            if kind == "expert":
                controller = lambda pose, p=path: pure_pursuit(p, pose)
            else:
                controller = lambda pose, gg=goal, dd=dist: policy_action(
                    net, encode(g, pose, gg, dd))
            r = run_episode(g_raw, controller, start, goal, max_steps)
            ok += int(r["success"])
            col += int(r["reason"] == "collision")
            to += int(r["reason"] == "timeout")
            lengths.append(r["length"])
            steps.append(r["steps"])
        n = max(len(lengths), 1)
        out[kind] = {
            "episodes": len(lengths),
            "success_rate": ok / n,
            "collision_rate": col / n,
            "timeout_rate": to / n,
            "length_mean": float(np.mean(lengths)) if lengths else 0.0,
            "steps_mean": float(np.mean(steps)) if steps else 0.0,
        }
    return out


def latency(net, sample, n=500) -> Dict[str, float]:
    """单帧推理延迟（毫秒）。"""
    x = np.asarray(sample, np.float32)
    for _ in range(50):
        net.predict(x)
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        net.predict(x)
        ts.append((time.perf_counter() - t0) * 1e3)
    return {"mean_ms": float(np.mean(ts)),
            "p95_ms": float(np.percentile(ts, 95))}
