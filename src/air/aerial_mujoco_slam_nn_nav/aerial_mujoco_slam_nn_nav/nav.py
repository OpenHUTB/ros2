"""无人机导航栈：SLAM 建图 -> 代价地图 -> A* -> 速度矢量控制 -> 飞控。

和地面版的区别：无人机可以侧移，所以不用"纯跟踪的转向 + 前进"，
而是把期望速度直接分解到机体系 $(v_x, v_y)$，偏航只做缓慢对齐。
这样贴路径更紧，也不会因为"边转边冲"撞上障碍。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

import mujoco

from .mapping import OccupancyGrid, Slam, local_patch
from .planning import CostMap, PurePursuit, astar, simplify
from .sensors import DepthCamera, Lidar, Odometry


@dataclass
class NavConfig:
    goal_tol: float = 0.40
    replan_period: float = 0.8
    scan_period: float = 0.1
    inflation: float = 0.45
    max_time: float = 90.0
    v_max: float = 0.65
    altitude: float = 1.2
    cmd_filter: float = 6.0


class Navigation:
    def __init__(self, sim, cfg: Optional[NavConfig] = None,
                 grid: Optional[OccupancyGrid] = None, with_depth: bool = False,
                 policy: str = "classical"):
        self.sim = sim
        self.cfg = cfg or NavConfig()
        self.policy = policy
        self.net = None
        if policy in ("neural", "e2e"):
            from .neural import NeuralPolicy
            self.net = NeuralPolicy()
        sx, sy, syaw = sim.layout.start
        self.start = (sx, sy, syaw)
        self.goal = np.array(sim.layout.goal, dtype=float)
        self.grid = grid or OccupancyGrid(size_x=8.0, size_y=6.0, res=0.05)
        self.slam = Slam(self.grid)
        self.slam.reset((sx, sy, syaw))
        self.lidar = Lidar(sim.model, n_beams=180, max_range=8.0)
        self.depth = DepthCamera(sim.model, 32, 24) if (with_depth or policy == "e2e") else None
        self.odom = Odometry()
        self.odom.reset(sx, sy, syaw)
        self.pp = PurePursuit(lookahead=0.8, v_max=self.cfg.v_max, w_max=1.0)
        self.path: List = []
        self.path_idx = 0
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.mode = "init"
        self.collisions = 0
        self.travel = 0.0
        self.t = 0.0
        self.ate: List[float] = []
        self._last_scan = None
        self._last_scan_t = -1.0
        self._last_plan_t = -1.0
        self._last_odom = (sx, sy, syaw)
        self._prev_pos = np.array([sx, sy])
        self._last_collision_t = -10.0
        self._recover_until = -1.0
        self._pos_check_t = 0.0
        self._pos_check = np.array([sx, sy])

    # ------------------------------------------------------------------
    def _perceive(self):
        st = self.sim.state()
        ranges = self.lidar.scan(self.sim.data)
        o = self.odom.update(st["vel"][0], st["vel"][1], 0.0, self.cfg.scan_period)
        dx = o["x"] - self._last_odom[0]
        dy = o["y"] - self._last_odom[1]
        dyaw = math.atan2(math.sin(o["yaw"] - self._last_odom[2]),
                          math.cos(o["yaw"] - self._last_odom[2]))
        self._last_odom = (o["x"], o["y"], o["yaw"])
        self._last_scan = ranges
        pose = self.slam.update((dx, dy, dyaw), ranges, self.lidar.angles, self.cfg.scan_period)
        self.ate.append(math.hypot(pose[0] - st["pos"][0], pose[1] - st["pos"][1]))
        return pose

    def _plan(self, pose):
        for inflation in (self.cfg.inflation, 0.26, 0.20, 0.15):
            cm = CostMap(self.grid, inflation=inflation)
            path = astar(cm, (pose[0], pose[1]), tuple(self.goal))
            if path:
                self.path = simplify(path)
                self.path_idx = 0
                return True
        return False

    # ------------------------------------------------------------------
    def _path_command(self, pose):
        """位置式前视点跟踪：世界系速度目标 -> 机体系速度矢量。"""
        if not self.path:
            return 0.0, 0.0, 0.0
        target, self.path_idx = self.pp.lookahead_point(pose, self.path, self.path_idx)
        dx, dy = target[0] - pose[0], target[1] - pose[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0, 0.0, 0.0
        speed = min(self.cfg.v_max, 1.6 * dist)
        if self._last_scan is not None:
            n = len(self._last_scan)
            front = float(self._last_scan[n // 2 - 10: n // 2 + 10].min())
            if front < 0.9:
                speed *= max(0.35, (front - 0.35) / 0.55)
        ux, uy = dx / dist, dy / dist
        wx, wy = ux * speed, uy * speed
        c, s = math.cos(pose[2]), math.sin(pose[2])
        vx = c * wx + s * wy
        vy = -s * wx + c * wy
        w = 0.6 * math.atan2(math.sin(math.atan2(wy, wx) - pose[2]),
                             math.cos(math.atan2(wy, wx) - pose[2]))
        return vx, vy, w

    def _neural_command(self, pose, st):
        """神经规划给方向，位置式跟踪负责执行。"""
        patch = local_patch(self.grid, pose, size=21, stride=2)
        gx = self.goal[0] - pose[0]
        gy = self.goal[1] - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        body = (c * gx + s * gy, -s * gx + c * gy)
        phi = float(self.net.plan_direction(patch, body, float(np.hypot(gx, gy))))
        if self.path:
            li = min(self.path_idx + 3, len(self.path) - 1)
            hx, hy = self.path[li][0] - pose[0], self.path[li][1] - pose[1]
            phi_path = math.atan2(-s * hx + c * hy, c * hx + s * hy)
            phi = 0.5 * phi + 0.5 * phi_path
        scan = self._last_scan if self._last_scan is not None else np.full(180, 8.0)
        n = len(scan)
        front = float(scan[n // 2 - 12:n // 2 + 12].min())
        speed = self.cfg.v_max * max(0.3, min(1.0, (front - 0.35) / 0.8))
        world = pose[2] + phi
        wx, wy = speed * math.cos(world), speed * math.sin(world)
        vx = c * wx + s * wy
        vy = -s * wx + c * wy
        w = 0.8 * phi
        return vx, vy, w

    # ------------------------------------------------------------------
    def step(self, dt: float = 0.01) -> None:
        st = self.sim.state()
        if self.t - self._last_scan_t >= self.cfg.scan_period:
            self._last_scan_t = self.t
            pose = self._perceive()
            if self.t - self._last_plan_t >= self.cfg.replan_period or not self.path:
                self._last_plan_t = self.t
                self._plan(pose)
        pose = tuple(self.slam.pose)
        dist_goal = math.hypot(self.goal[0] - st["pos"][0], self.goal[1] - st["pos"][1])

        if dist_goal < self.cfg.goal_tol:
            self.mode = "done"
            vx, vy, w = 0.0, 0.0, 0.0
        elif self.policy == "e2e" and self.depth is not None:
            self.mode = "e2e"
            depth = self.depth.depth(self.sim.data)
            vx, vy, w = self.net.end_to_end(depth)
        elif self.policy == "neural":
            self.mode = "neural"
            vx, vy, w = self._neural_command(pose, st)
        elif self.path:
            self.mode = "fly"
            vx, vy, w = self._path_command(pose)
        else:
            self.mode = "blocked"
            vx, vy, w = 0.0, 0.0, 0.0

        if self.t - self._pos_check_t > 1.2:
            moved = float(np.linalg.norm(np.array(st["pos"][:2]) - self._pos_check))
            if self.mode in ("fly", "neural", "e2e") and moved < 0.12 and (abs(vx) + abs(vy)) > 0.1:
                self._recover_until = self.t + 1.5
                self.path = []
            self._pos_check_t = self.t
            self._pos_check = np.array(st["pos"][:2])
        if self.t < self._recover_until:
            vx, vy, w = -0.35, 0.0, 0.0
            self.mode = "recover"

        a = min(1.0, self.cfg.cmd_filter * dt)
        self.vx += (vx - self.vx) * a
        self.vy += (vy - self.vy) * a
        self.w += (w - self.w) * a

        self.sim.set_command(vx=self.vx, vy=self.vy, yaw_rate=self.w, z_ref=self.cfg.altitude)
        self.sim.step(int(round(dt / self.sim.model.opt.timestep)))

        pos = np.array(self.sim.base_pos()[:2])
        self.travel += float(np.linalg.norm(pos - self._prev_pos))
        self._prev_pos = pos
        self.t += dt
        if self._check_collision() and self.t - self._last_collision_t > 0.6:
            self.collisions += 1
            self._last_collision_t = self.t

    def _check_collision(self) -> bool:
        m, d = self.sim.model, self.sim.data
        for i in range(d.ncon):
            c = d.contact[i]
            for gid in (c.geom1, c.geom2):
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
                if name.startswith("obst") or name.startswith("wall"):
                    return True
        return False

    # ------------------------------------------------------------------
    def run(self, max_time: Optional[float] = None) -> dict:
        max_time = max_time or self.cfg.max_time
        while self.t < max_time:
            self.step()
            if self.mode == "done" or self.sim.is_fallen():
                break
        return self.summary()

    def summary(self) -> dict:
        st = self.sim.state()
        ate = float(np.sqrt(np.mean(np.square(self.ate)))) if self.ate else 0.0
        return {
            "success": bool(self.mode == "done" and not self.sim.is_fallen()),
            "fallen": bool(self.sim.is_fallen()),
            "time_s": round(self.t, 2),
            "travel_m": round(self.travel, 3),
            "collisions": int(self.collisions),
            "final_dist_m": round(float(np.hypot(self.goal[0] - st["pos"][0],
                                                 self.goal[1] - st["pos"][1])), 3),
            "ate_rmse_m": round(ate, 4),
            "map_coverage": round(self.grid.coverage(), 3),
            "mode": self.mode,
        }
