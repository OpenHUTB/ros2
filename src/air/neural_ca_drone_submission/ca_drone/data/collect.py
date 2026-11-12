"""Dataset generation for the four learning modules.

All data comes from the CA-Sim ray-casting sensor and the geometric expert, so
the training pipeline is fully reproducible without any external simulator.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..control.expert import expert_command
from ..mapping import GridMap, patch_clearance
from ..models.control import control_features
from ..planning.astar import astar, cells_to_world, simplify_path
from ..render.scene import depth_image
from ..sim.drone import Drone
from ..sim.world import World, make_world

DEPTH_W, DEPTH_H = 64, 48
MAX_RANGE = 12.0
FOV = 90.0
RES = 0.5


def _sample_pose(world: World, rng: np.random.Generator) -> Optional[np.ndarray]:
    for _ in range(60):
        x = rng.uniform(2.0, 46.0)
        y = rng.uniform(-9.0, 9.0)
        yaw = float(rng.uniform(-0.7, 0.7))
        p = np.array([x, y, world.flight_z])
        if world.clearance(p) > 0.75 and world.inside(p, margin=1.0):
            return p
    return None


def perception_dataset(n: int = 1200, seed: int = 1) -> Dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    names = ["sparse", "dense", "dynamic", "corridor"]
    depths = np.zeros((n, DEPTH_H, DEPTH_W), dtype=np.float32)
    masks = np.zeros((n, DEPTH_H, DEPTH_W), dtype=np.uint8)
    i = 0
    while i < n:
        world = make_world(names[int(rng.integers(0, len(names)))])
        world.t = float(rng.uniform(0, 6))
        p = _sample_pose(world, rng)
        if p is None:
            continue
        yaw = float(rng.uniform(-0.7, 0.7))
        d, ids = depth_image(world, p, yaw, DEPTH_W, DEPTH_H, FOV, MAX_RANGE)
        mask = (ids >= 0).astype(np.uint8)
        if mask.sum() < 6:
            continue
        # sensor noise: range noise, dropout and a few flying pixels
        d = d * (1.0 + rng.normal(0, 0.012, size=d.shape))
        drop = rng.random(d.shape) < 0.012
        d[drop] = MAX_RANGE
        fly = rng.random(d.shape) < 0.004
        d[fly] = rng.uniform(0.6, MAX_RANGE, size=int(fly.sum()))
        d = np.clip(d, 0.3, MAX_RANGE).astype(np.float32)
        depths[i] = d
        masks[i] = mask
        i += 1
    return {"depth": depths, "mask": masks}


def _patch(occ: np.ndarray, ix: int, iy: int, k: int) -> np.ndarray:
    r = k // 2
    p = np.ones((k, k), dtype=np.float32)
    x0, x1 = max(0, ix - r), min(occ.shape[0], ix + r + 1)
    y0, y1 = max(0, iy - r), min(occ.shape[1], iy + r + 1)
    p[x0 - (ix - r):x1 - (ix - r), y0 - (iy - r):y1 - (iy - r)] = occ[x0:x1, y0:y1]
    return p


def planner_dataset(n_maps: int = 50, per_map: int = 130, seed: int = 2,
                    patch: int = 11) -> Dict[str, np.ndarray]:
    from ..planning.astar import dijkstra
    rng = np.random.default_rng(seed)
    names = ["sparse", "dense", "dynamic", "corridor"]
    X, y = [], []
    for m in range(n_maps):
        if m % 3 == 0:
            occ = rng.random((104, 88)) < 0.18
            occ[0, :] = occ[-1, :] = True
            occ[:, 0] = occ[:, -1] = True
        else:
            world = make_world(names[int(rng.integers(0, len(names)))])
            gm = GridMap.from_world(world, RES, inflate=0.45)
            occ = gm.occ.copy()
        free = np.argwhere(~occ)
        if len(free) < 50:
            continue
        goal = tuple(free[int(rng.integers(0, len(free)))])
        dist = dijkstra(occ, goal, RES)
        cand = np.argwhere(np.isfinite(dist))
        if len(cand) < 20:
            continue
        pick = cand[rng.integers(0, len(cand), size=per_map)]
        for ix, iy in pick:
            p = _patch(occ, int(ix), int(iy), patch)
            dx = (goal[0] - ix) * RES / 25.0
            dy = (goal[1] - iy) * RES / 25.0
            d = math.hypot(goal[0] - ix, goal[1] - iy) * RES / 50.0
            X.append(np.concatenate([p.reshape(-1), [dx, dy, d]]))
            y.append(dist[ix, iy] / 50.0)
    return {"x": np.asarray(X, dtype=np.float32), "y": np.asarray(y, dtype=np.float32).reshape(-1, 1)}


def _oracle_path(world: World) -> Optional[np.ndarray]:
    gm = GridMap.from_world(world, RES, inflate=0.45)
    s = gm.to_cell(world.start)
    g = gm.to_cell(world.goal)
    r = astar(gm.occ, s, g, RES, weight=1.0)
    if not r["success"]:
        return None
    cells = simplify_path(gm.occ, r["path_cells"])
    return cells_to_world(cells, gm.x0, gm.y0, RES, world.flight_z)


def control_dataset(episodes: int = 36, seed: int = 3, max_steps: int = 320,
                    patch: int = 9) -> Dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    names = ["sparse", "dense", "dynamic", "corridor"]
    X, Y = [], []
    for ep in range(episodes):
        world = make_world(names[ep % len(names)])
        world.t = float(rng.uniform(0, 5))
        path = _oracle_path(world)
        if path is None:
            continue
        gm = GridMap.from_world(world, RES, inflate=0.45)
        drone = Drone(world)
        for step in range(max_steps):
            cmd = expert_command(world, drone.state.pos, drone.body_velocity(),
                                 drone.state.yaw, path)
            if step % 2 == 0:
                patch_occ = gm.local_patch(drone.state.pos, patch)
                from ..control.expert import lookahead_point, command_to_point
                tgt = lookahead_point(path, drone.state.pos, 2.0)
                # DAgger-style augmentation: relabel off-path targets with the
                # expert so the policy learns to recover from large cross-track
                # errors (which the on-path data never contains).
                targets = [tgt]
                for _ in range(2):
                    off = rng.uniform(-2.8, 2.8, size=2)
                    t = np.array([tgt[0] + off[0], tgt[1] + off[1], world.flight_z])
                    if not world.inside(t, margin=1.0):
                        continue
                    if world.clearance(t) < 0.6:
                        continue
                    targets.append(t)
                for t in targets:
                    delta = t[:2] - drone.state.pos[:2]
                    c, s = math.cos(drone.state.yaw), math.sin(drone.state.yaw)
                    gb = np.array([c * delta[0] + s * delta[1],
                                   -s * delta[0] + c * delta[1]])
                    world_cmd = command_to_point(world, drone.state.pos, t, speed=1.35)
                    body = np.array([c * world_cmd[0] + s * world_cmd[1],
                                     -s * world_cmd[0] + c * world_cmd[1]])
                    X.append(control_features(patch_occ, gb, drone.body_velocity(),
                                              patch_clearance(patch_occ, RES)))
                    Y.append(body)
            drone.step(cmd)
            if drone.reached() or drone.collided() or drone.out_of_bounds():
                break
    return {"x": np.asarray(X, dtype=np.float32), "y": np.asarray(Y, dtype=np.float32)}


def e2e_dataset(episodes: int = 30, seed: int = 4, max_steps: int = 320,
                limit: int = 7200) -> Dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    names = ["sparse", "dense", "dynamic", "corridor"]
    X, G, Y = [], [], []
    from ..control.expert import command_to_point
    from ..models.e2e import goal_features
    for ep in range(episodes):
        world = make_world(names[ep % len(names)])
        world.t = float(rng.uniform(0, 5))
        path = _oracle_path(world)
        if path is None:
            continue
        drone = Drone(world)
        for step in range(max_steps):
            cmd = expert_command(world, drone.state.pos, drone.body_velocity(),
                                 drone.state.yaw, path)
            if step % 3 == 0:
                d, _ = depth_image(world, drone.state.pos, drone.state.yaw,
                                   DEPTH_W, DEPTH_H, FOV, MAX_RANGE)
                depth_in = np.clip(d / MAX_RANGE, 0, 1).astype(np.float32)
                c, s = math.cos(drone.state.yaw), math.sin(drone.state.yaw)
                # Goal augmentation: the goal is a pure coordinate, so we can
                # relabel the same observation with synthetic goals (including
                # goals behind the drone).  This is what teaches the reactive
                # policy to turn and to slow down, instead of always flying
                # straight at full speed.
                goals = [world.goal[:2]]
                for _ in range(2):
                    cand = drone.state.pos[:2] + rng.uniform(-6.0, 6.0, size=2)
                    p3 = np.array([cand[0], cand[1], world.flight_z])
                    if world.inside(p3, margin=1.0) and world.clearance(p3) >= 0.8:
                        goals.append(cand)
                for gi, g in enumerate(goals):
                    delta = np.asarray(g, dtype=float) - drone.state.pos[:2]
                    dist = float(np.linalg.norm(delta))
                    gb = np.array([c * delta[0] + s * delta[1],
                                   -s * delta[0] + c * delta[1]])
                    wc = command_to_point(world, drone.state.pos,
                                          np.array([g[0], g[1], world.flight_z]),
                                          speed=1.35, final_slowdown=(gi == 0),
                                          radius=4.5, gain=2.6)
                    body = np.array([c * wc[0] + s * wc[1],
                                     -s * wc[0] + c * wc[1]])
                    X.append(depth_in)
                    G.append(goal_features(gb, dist))
                    Y.append(body)
            drone.step(cmd)
            if drone.reached() or drone.collided() or drone.out_of_bounds():
                break
        # Recovery samples: poses close to an obstacle (0.9-2.2 m clearance)
        # with an arbitrary heading.  A purely reactive policy must already be
        # steering there; otherwise "fly straight until the wall fills the
        # image" is a sufficient explanation of the training data.
        for _ in range(8):
            for _try in range(40):
                p3 = np.array([rng.uniform(1.5, 48.0), rng.uniform(-16.0, 16.0),
                               world.flight_z])
                clr = float(world.clearance(p3))
                if world.inside(p3, margin=1.0) and 0.9 <= clr <= 2.2:
                    break
            else:
                continue
            yaw = float(rng.uniform(-math.pi, math.pi))
            d, _ = depth_image(world, p3, yaw, DEPTH_W, DEPTH_H, FOV, MAX_RANGE)
            depth_in = np.clip(d / MAX_RANGE, 0, 1).astype(np.float32)
            c, s = math.cos(yaw), math.sin(yaw)
            delta = world.goal[:2] - p3[:2]
            gb = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])
            wc = command_to_point(world, p3, world.goal, speed=1.35,
                                  final_slowdown=True, radius=4.5, gain=2.6)
            body = np.array([c * wc[0] + s * wc[1], -s * wc[0] + c * wc[1]])
            X.append(depth_in)
            G.append(goal_features(gb, float(np.linalg.norm(delta))))
            Y.append(body)
    X = np.asarray(X, dtype=np.float32)
    if len(X) > limit:
        idx = rng.choice(len(X), size=limit, replace=False)
        X, G, Y = X[idx], np.asarray(G)[idx], np.asarray(Y)[idx]
    return {"depth": X, "goal": np.asarray(G, dtype=np.float32),
            "cmd": np.asarray(Y, dtype=np.float32)}
