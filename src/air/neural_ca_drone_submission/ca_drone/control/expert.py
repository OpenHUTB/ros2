"""Classical geometric controller used as the imitation expert and baseline."""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from ..sim.world import World


def lookahead_point(path: np.ndarray, pos: np.ndarray, lookahead: float = 2.0) -> np.ndarray:
    """Project ``pos`` on the polyline and walk ``lookahead`` metres ahead.

    Projecting onto every segment (rather than snapping to the nearest vertex)
    is what prevents the classic "corner cutting" failure in narrow gaps.
    """
    if path is None or len(path) == 0:
        return pos
    if len(path) == 1:
        return path[0]
    best_d, best_j, best_t = float("inf"), 0, 0.0
    p = np.asarray(pos[:2], dtype=float)
    for j in range(len(path) - 1):
        a = np.asarray(path[j, :2], dtype=float)
        b = np.asarray(path[j + 1, :2], dtype=float)
        ab = b - a
        L2 = float(ab @ ab)
        t = 0.0 if L2 < 1e-12 else float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
        d = float(np.linalg.norm(p - (a + t * ab)))
        if d < best_d:
            best_d, best_j, best_t = d, j, t
    a = np.asarray(path[best_j, :2], dtype=float)
    b = np.asarray(path[best_j + 1, :2], dtype=float)
    seg_len = float(np.linalg.norm(b - a))
    rem = (1.0 - best_t) * seg_len
    if rem >= lookahead:
        return a + (best_t * seg_len + lookahead) / max(seg_len, 1e-9) * (b - a)
    need = lookahead - rem
    for k in range(best_j + 1, len(path) - 1):
        a = np.asarray(path[k, :2], dtype=float)
        b = np.asarray(path[k + 1, :2], dtype=float)
        L = float(np.linalg.norm(b - a))
        if need <= L:
            return a + need / max(L, 1e-9) * (b - a)
        need -= L
    return np.asarray(path[-1], dtype=float)


def repulsion(world: World, pos: np.ndarray, desired: np.ndarray, radius: float = 2.8,
              gain: float = 1.5) -> np.ndarray:
    """Tangential avoidance field: steer around, never cancel forward motion."""
    vec = np.zeros(2)
    dn = float(np.linalg.norm(desired))
    dirn = desired / dn if dn > 1e-6 else np.array([1.0, 0.0])
    for o in world.obstacles():
        d = float(o.distance(pos))
        if d >= radius:
            continue
        away = pos[:2] - o.center[:2]
        n = float(np.linalg.norm(away))
        if n < 1e-3:
            away = -dirn
            n = 1.0
        away = away / n
        tang = np.array([-away[1], away[0]])
        if float(np.dot(tang, dirn)) < 0:
            tang = -tang
        w = gain * (1.0 / max(d, 0.25) - 1.0 / radius)
        vec += w * (0.85 * tang + 0.35 * away)
    return vec


def command_to_point(world: World, pos: np.ndarray, target: np.ndarray,
                     speed: float = 1.35, final_slowdown: bool = False,
                     radius: float = 2.8, gain: float = 1.5) -> np.ndarray:
    """Pure-pursuit command in the world frame that steers to an arbitrary point.

    ``radius``/``gain`` shape the tangential avoidance field; the end-to-end
    dataset uses a longer, stronger field so that obstacles 2-4.5 m away already
    produce a visible lateral command (anticipatory avoidance).
    """
    delta = np.asarray(target[:2], dtype=float) - np.asarray(pos[:2], dtype=float)
    dist = float(np.linalg.norm(delta))
    desired = np.zeros(2) if dist < 1e-6 else delta / dist * speed
    if final_slowdown:
        goal_d = float(np.linalg.norm(world.goal[:2] - pos[:2]))
        if goal_d < 3.0:
            desired *= max(0.25, goal_d / 3.0)
    cmd = desired + repulsion(world, pos, desired, radius=radius, gain=gain)
    n = float(np.linalg.norm(cmd))
    if n > 1.45:
        cmd *= 1.45 / n
    return cmd


def expert_command(world: World, pos: np.ndarray, vel_body: np.ndarray, yaw: float,
                   path: np.ndarray, speed: float = 1.35,
                   lookahead: float = 2.2) -> np.ndarray:
    target = lookahead_point(path, pos, lookahead)
    cmd = command_to_point(world, pos, target, speed, final_slowdown=True)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * cmd[0] + s * cmd[1], -s * cmd[0] + c * cmd[1]])
