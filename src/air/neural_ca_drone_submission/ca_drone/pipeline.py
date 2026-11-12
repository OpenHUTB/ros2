"""Closed-loop runners: classical baseline, full neural stack and end-to-end policy.

The same runner is used by the GIF recorder, the batch evaluation and the ROS
nodes, so the numbers in the report and the videos come from one code path.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .control.expert import expert_command, lookahead_point
from .mapping import GridMap, patch_clearance
from .models.control import ControlNet, control_features, predict_command
from .models.e2e import E2ENet, goal_features
from .models.perception import PerceptionNet, normalize_depth, predict_mask
from .models.planner import NeuralHeuristic, predict_field
from .nn.autograd import Tensor
from .planning.astar import astar, cells_to_world, simplify_path
from .render.scene import compose_frame, depth_image
from .sim.drone import Drone
from .sim.world import World, make_world

RES = 0.5
PLAN_MARGIN = 2
MAX_CMD = 1.2
DEPTH_W, DEPTH_H = 64, 48
MAX_RANGE = 12.0
FOV = 90.0


@dataclass
class EpisodeResult:
    scene: str
    mode: str
    success: bool
    collision: bool
    steps: int
    duration: float
    path_length: float
    min_clearance: float
    replans: int = 0
    plan_ms: float = 0.0
    infer_ms: float = 0.0
    frames: List = field(default_factory=list)

    def as_dict(self) -> Dict:
        return {"scene": self.scene, "mode": self.mode, "success": self.success,
                "collision": self.collision, "steps": self.steps,
                "duration_s": round(self.duration, 2),
                "path_length_m": round(self.path_length, 2),
                "min_clearance_m": round(self.min_clearance, 3),
                "replans": self.replans, "plan_ms": round(self.plan_ms, 2),
                "infer_ms": round(self.infer_ms, 2)}


class NeuralStack:
    """Lazy loader for the four checkpoints (missing files are tolerated)."""

    def __init__(self, weights: str = "weights"):
        self.weights = weights
        self._perception = None
        self._planner = None
        self._control = None
        self._e2e = None
        self.load_ms = 0.0

    def _load(self, name, cls):
        path = os.path.join(self.weights, f"{name}.npz")
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"missing checkpoint {path}; run tools/train.py --module all first")
        t = time.time()
        model, _meta = cls.load(path)
        self.load_ms += (time.time() - t) * 1e3
        return model

    @property
    def perception(self) -> PerceptionNet:
        if self._perception is None:
            self._perception = self._load("perception", PerceptionNet)
        return self._perception

    @property
    def planner(self) -> NeuralHeuristic:
        if self._planner is None:
            self._planner = self._load("planner", NeuralHeuristic)
        return self._planner

    @property
    def control(self) -> ControlNet:
        if self._control is None:
            self._control = self._load("control", ControlNet)
        return self._control

    @property
    def e2e(self) -> E2ENet:
        if self._e2e is None:
            self._e2e = self._load("e2e", E2ENet)
        return self._e2e


def _plan(stack: Optional[NeuralStack], gm: GridMap, start, goal,
          use_neural: bool = True) -> Dict:
    # Two-cell (1 m) inflation keeps the planned path away from the walls: the
    # learned controller tracks within ~0.4 m, so a thinner margin would clip
    # boxes even when the global path itself is collision free.
    occ = gm.plan_grid(margin=PLAN_MARGIN)
    s, g = gm.to_cell(start), gm.to_cell(goal)
    t = time.time()
    if use_neural and stack is not None:
        field = predict_field(stack.planner, occ, g, RES)
        r = astar(occ, s, g, RES, heuristic=field, weight=1.15)
    else:
        r = astar(occ, s, g, RES, weight=1.0)
    ms = (time.time() - t) * 1e3
    r["plan_ms"] = ms
    if not r["success"]:
        return r
    cells = simplify_path(occ, r["path_cells"])
    pts = cells_to_world(cells, gm.x0, gm.y0, RES, goal[2])
    pts[-1] = np.asarray(goal, dtype=float)
    r["world_path"] = pts
    r["expanded"] = r.get("expanded", 0)
    return r


def run_episode(scene: str = "sparse", mode: str = "full", stack: Optional[NeuralStack] = None,
                max_steps: int = 700, render_every: int = 0, seed: int = 0,
                speed: float = 1.35, verbose: bool = False,
                record: Optional[List] = None,
                trace: Optional[List] = None,
                goal: Optional[np.ndarray] = None,
                on_frame=None) -> EpisodeResult:
    """Run one closed-loop episode.

    ``mode``:
      * ``expert`` - true map + A* + geometric controller (classical baseline)
      * ``control`` - true map + A* + **neural** controller (control module test)
      * ``full``   - depth -> CNN mask -> online grid -> neural A* -> neural control
      * ``e2e``    - depth + goal -> CNN -> body velocity
      * ``dagger`` - the e2e policy flies, the full stack labels every visited
        state (on-policy distillation for DAgger)
    """
    world = make_world(scene)
    if goal is not None:
        world.goal = np.asarray(goal, dtype=float)
    world.t = float(seed % 5) * 0.7
    drone = Drone(world)
    gm = GridMap(res=RES, x0=0.0, y0=-22.0, nx=104, ny=88)  # empty onboard map
    path = None
    frames: List = []
    replans = 0
    plan_ms = 0.0
    infer_ms = 0.0
    last_plan_step = -10 ** 9
    replan_every = 10 if mode in ("full", "dagger") else 10 ** 9
    events: List[str] = []

    for step in range(max_steps):
        if trace is not None:
            trace.append(drone.state.pos.copy())
        depth, _ids = depth_image(world, drone.state.pos, drone.state.yaw,
                                  DEPTH_W, DEPTH_H, FOV, MAX_RANGE)
        if mode == "e2e":
            delta = world.goal[:2] - drone.state.pos[:2]
            c, s = math.cos(drone.state.yaw), math.sin(drone.state.yaw)
            gb = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])
            feats_goal = goal_features(gb, float(np.linalg.norm(delta)))
            t = time.time()
            cmd = stack.e2e(Tensor(np.clip(depth / MAX_RANGE, 0, 1)[None, None]),
                            Tensor(feats_goal[None])) .data[0]
            infer_ms += (time.time() - t) * 1e3
            if world.clearance(drone.state.pos) < 0.8:
                cmd = cmd * 0.85
        else:
            if mode == "expert":
                if path is None:
                    gm = GridMap.from_world(world, RES, inflate=0.45)
                    r = _plan(None, gm, world.start, world.goal, use_neural=False)
                    path = r.get("world_path")
                    replans += 1
                    plan_ms += r.get("plan_ms", 0.0)
            elif mode == "control":
                # Isolate the control network: the map and the global path are
                # ground truth, so any failure here is a tracking failure.
                if path is None:
                    gm = GridMap.from_world(world, RES, inflate=0.9)
                    r = _plan(None, gm, world.start, world.goal, use_neural=False)
                    path = r.get("world_path")
                    replans += 1
                    plan_ms += r.get("plan_ms", 0.0)
            else:
                prob = predict_mask(stack.perception, depth, MAX_RANGE)
                gm.integrate_depth(depth, prob, drone.state.pos, drone.state.yaw,
                                   FOV, MAX_RANGE, threshold=0.55)
                need = path is None or (step - last_plan_step) >= replan_every
                if need:
                    r = _plan(stack, gm, drone.state.pos, world.goal, use_neural=True)
                    if r["success"]:
                        path = r["world_path"]
                        plan_ms += r.get("plan_ms", 0.0)
                        replans += 1
                        last_plan_step = step
                    elif verbose:
                        events.append("planner: no path, holding last plan")
            if path is None or len(path) < 2:
                cmd = np.array([0.0, 0.0])
            else:
                target = lookahead_point(path, drone.state.pos, 2.0)
                delta = target[:2] - drone.state.pos[:2]
                c, s = math.cos(drone.state.yaw), math.sin(drone.state.yaw)
                gb = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])
                patch = gm.local_patch(drone.state.pos, 9)
                clear = patch_clearance(patch, RES)
                feats = control_features(patch, gb, drone.body_velocity(), clear)
                t = time.time()
                cmd = predict_command(stack.control, feats)
                infer_ms += (time.time() - t) * 1e3
                n = float(np.linalg.norm(cmd))
                if n > MAX_CMD:
                    cmd = cmd * (MAX_CMD / n)
                if clear < 0.6:
                    cmd = cmd * 0.75
                g_world = world.goal[:2] - drone.state.pos[:2]
                c2, s2 = math.cos(drone.state.yaw), math.sin(drone.state.yaw)
                g_body = np.array([c2 * g_world[0] + s2 * g_world[1],
                                   -s2 * g_world[0] + c2 * g_world[1]])
                if mode == "dagger":
                    # DAgger: the student flies, the teacher labels the state it
                    # actually visits; this is what fixes the "goal behind me"
                    # failure the on-policy data never contained.
                    teacher = cmd.copy()
                    t = time.time()
                    cmd = stack.e2e(
                        Tensor(np.clip(depth / MAX_RANGE, 0, 1)[None, None]),
                        Tensor(goal_features(g_body, float(np.linalg.norm(g_world)))[None]),
                    ).data[0]
                    infer_ms += (time.time() - t) * 1e3
                    if world.clearance(drone.state.pos) < 0.8:
                        cmd = cmd * 0.85
                    if record is not None:
                        record.append((depth.copy(), goal_features(
                            g_body, float(np.linalg.norm(g_world))), teacher))
                elif record is not None and (step % 2 == 0):
                    # Distillation hook: (depth, goal features, executed command)
                    # lets us train the end-to-end policy on the full stack.
                    record.append((depth.copy(), goal_features(
                        g_body, float(np.linalg.norm(g_world))), cmd.copy()))
        drone.step(cmd)
        world.t = drone.t  # advance time-driven obstacles (dynamic scene)
        if render_every and (step % render_every == 0):
            info = {"mode": mode.upper(), "progress": min(1.0, step / max_steps),
                    "plan_ms": plan_ms / max(replans, 1),
                    "infer_ms": infer_ms / max(step + 1, 1),
                    "events": events, "replans": replans}
            frame = compose_frame(world, drone.state.pos, drone.state.vel,
                                  drone.state.yaw, path, info)
            frames.append(frame)
            if on_frame is not None:
                on_frame(frame)
        if drone.collided():
            events.append(f"collision at t={drone.t:.1f}s")
            break
        if drone.reached():
            events.append(f"goal reached in {drone.t:.1f}s")
            break
        if drone.out_of_bounds():
            events.append("out of bounds")
            break
    return EpisodeResult(scene=scene, mode=mode, success=drone.reached(),
                         collision=drone.collided(), steps=step + 1,
                         duration=float(drone.t), path_length=float(drone.path_length),
                         min_clearance=float(drone.min_clearance),
                         replans=replans,
                         plan_ms=plan_ms / max(replans, 1),
                         infer_ms=infer_ms / max(step + 1, 1), frames=frames)


def save_gif(frames: List, path: str, duration_ms: int = 80, max_width: int = 640,
             colors: int = 64) -> str:
    """Write an optimised GIF (palette, 64 colours, no dithering) for the report (<10 MB)."""
    from PIL import Image
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    if not frames:
        raise ValueError("no frames to save")
    try:
        dither = Image.Dither.NONE
    except AttributeError:  # Pillow 7 on Ubuntu 20.04
        dither = getattr(Image, "NONE", 0)
    imgs = []
    for f in frames:
        if f.width > max_width:
            f = f.resize((max_width, int(f.height * max_width / f.width)), Image.LANCZOS)
        imgs.append(f.convert("RGB").quantize(colors=colors, method=Image.MEDIANCUT,
                                              dither=dither))
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=duration_ms,
                 loop=0, optimize=True, disposal=2)
    return path
