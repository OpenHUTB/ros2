"""完整导航栈：SLAM 建图 -> 代价地图 -> A* -> 纯跟踪 -> 步态 -> 关节。

ROS 节点只是这一层的薄封装，离线评测脚本也直接复用，保证"仿真里跑通"与
"ROS 里跑通"是同一份控制代码。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

import mujoco

from . import kinematics as K
from .gait import GaitParams, Walker
from .mapping import OccupancyGrid, Slam, local_patch
from .planning import CostMap, PurePursuit, astar, simplify
from .sensors import DepthCamera, Lidar, Odometry


@dataclass
class NavConfig:
    goal_tol: float = 0.35
    replan_period: float = 0.8
    scan_period: float = 0.1
    inflation: float = 0.38
    max_time: float = 120.0
    v_max: float = 0.26
    w_max: float = 0.65
    collision_dist: float = 0.22
    settle_period: float = 9.0
    settle_time: float = 1.4
    tilt_limit: float = 0.30
    cmd_filter: float = 2.5
    gait: GaitParams = field(default_factory=lambda: GaitParams(
        pelvis_mode="feet", pelvis_height=0.70, duty=0.78, period=1.15,
        side_shift=0.02, foot_spread=0.07, step_height=0.08,
        capture_gain=0.35, capture_max=0.08, com_kp=1.0, com_kd=0.1,
        com_max=0.04, roll_gain=0.0, pitch_gain=0.0, ankle_kp=0.0, ankle_kd=0.0))


class Navigation:
    """闭环导航：输入仿真器，输出关节目标。"""

    def __init__(self, sim, cfg: Optional[NavConfig] = None, grid: Optional[OccupancyGrid] = None,
                 with_depth: bool = False, policy: str = "classical"):
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
        self.grid = grid or OccupancyGrid(size_x=10.5, size_y=7.5, res=0.05)
        self.slam = Slam(self.grid)
        self.lidar = Lidar(sim.model, n_beams=180, max_range=8.0)
        self.depth = DepthCamera(sim.model, 32, 24) if with_depth else None
        self.odom = Odometry()
        self.pp = PurePursuit(lookahead=0.55, v_max=self.cfg.v_max, w_max=self.cfg.w_max)
        self.walker = Walker(self.cfg.gait, sx, sy, syaw)
        self.path: List = []
        self.path_idx = 0
        self.v = 0.0
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
        self._contact_flag = False
        self._next_settle_t = 0.0
        self._settle_until = -1.0
        self._last_collision_t = -10.0
        self.slam.reset((sx, sy, syaw))
        self.odom.reset(sx, sy, syaw)

    # ------------------------------------------------------------------
    def reset(self):
        self.sim.reset()
        sx, sy, syaw = self.start
        self.slam.reset((sx, sy, syaw))
        self.odom.reset(sx, sy, syaw)
        self.walker = Walker(self.cfg.gait, sx, sy, syaw)
        self.path = []
        self.path_idx = 0
        self.mode = "init"
        self.collisions = 0
        self.travel = 0.0
        self.t = 0.0
        self.ate = []
        self._last_scan = None
        self._last_scan_t = -1.0
        self._last_plan_t = -1.0
        self._prev_pos = np.array([sx, sy])
        self._next_settle_t = 0.0
        self._settle_until = -1.0
        self._last_collision_t = -10.0

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
        pose = self.slam.update((dx, dy, dyaw), ranges, self.lidar.angles,
                                self.cfg.scan_period)
        self.ate.append(math.hypot(pose[0] - st["pos"][0], pose[1] - st["pos"][1]))
        return pose, ranges

    def _plan(self, pose):
        # 依次降低膨胀半径重试：起始点被膨胀层包住时也能规划出来
        for inflation in (self.cfg.inflation, 0.30, 0.22, 0.16):
            costmap = CostMap(self.grid, inflation=inflation)
            path = astar(costmap, (pose[0], pose[1]), tuple(self.goal))
            if path:
                self.path = simplify(path)
                self.path_idx = 0
                return True
        return False

    # ------------------------------------------------------------------
    def _neural_command(self, pose, st):
        """神经规划 + 神经控制：局部栅格 -> 方向 -> 速度指令。"""
        patch = local_patch(self.grid, pose, size=21, stride=2)
        gx = self.goal[0] - pose[0]
        gy = self.goal[1] - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        body = (c * gx + s * gy, -s * gx + c * gy)
        phi = self.net.plan_direction(patch, body, float(np.hypot(gx, gy)))
        # 安全层：若网络给出的方向 0.35 m 内被占据，则退回 A* 路径方向
        if not self._direction_free(patch, phi, 0.35) and self.path:
            li = min(self.path_idx + 3, len(self.path) - 1)
            px, py = self.path[li]
            hx, hy = px - pose[0], py - pose[1]
            phi = math.atan2(-s * hx + c * hy, c * hx + s * hy)
        scan = self._last_scan if self._last_scan is not None else np.full(180, 8.0)
        n = len(scan)
        sectors = np.array([scan[n // 2 - 15:n // 2 + 15].min(),
                            scan[n // 2 + 15:n // 2 + 50].min(),
                            scan[n // 2 - 50:n // 2 - 15].min()])
        v_nn, w_nn = self.net.control(body, phi, sectors, self.v)
        # 残差/混合控制：神经输出与经典纯跟踪各占一半，兼顾学习策略与步态稳定性
        if self.path:
            v_pp, w_pp, self.path_idx = self.pp.control(pose, self.path, self.path_idx)
            return 0.5 * (v_nn + v_pp), 0.5 * (w_nn + w_pp)
        return v_nn, w_nn

    def _smooth(self, v: float, w: float, dt: float):
        """网络输出的一阶低通：抑制逐帧抖动，双足对速度突变非常敏感。"""
        a = min(1.0, self.cfg.cmd_filter * dt)
        self.v += (v - self.v) * a
        self.w += (w - self.w) * a
        if self.v < 0.06 and abs(self.w) > 0.25:
            self.v = 0.08          # 双足原地转向需要保持最小步频
        return self.v, self.w

    def _direction_free(self, patch, phi, dist: float) -> bool:
        """在局部栅格上沿 phi 方向检查 dist 米内是否可通行。"""
        half = patch.shape[0] // 2
        steps = max(2, int(dist / 0.1))
        for k in range(1, steps + 1):
            r = k * dist / steps / 0.2      # 栅格步长 0.1 m ≈ 1 格（stride=2, res=0.05）
            cx = int(round(half + r * math.cos(phi)))
            cy = int(round(half - r * math.sin(phi)))
            cx = min(max(cx, 0), patch.shape[1] - 1)
            cy = min(max(cy, 0), patch.shape[0] - 1)
            if patch[cy, cx] > 0.5:
                return False
        return True

    # ------------------------------------------------------------------
    def step(self, dt: float = 0.01) -> Dict[str, float]:
        cfg = self.cfg
        st = self.sim.state()
        if self.t - self._last_scan_t >= cfg.scan_period:
            self._last_scan_t = self.t
            pose, _ = self._perceive()
            if self.t - self._last_plan_t >= cfg.replan_period or not self.path:
                self._last_plan_t = self.t
                self._plan(pose)
        pose = tuple(self.slam.pose)

        dist_goal = math.hypot(self.goal[0] - st["pos"][0], self.goal[1] - st["pos"][1])
        # 定期/受扰时"原地踏步"让身体重新站直，抑制姿态误差累积
        if self.t >= self._next_settle_t:
            self._settle_until = self.t + cfg.settle_time
            self._next_settle_t = self.t + cfg.settle_period
        tilt = max(abs(st["roll"]), abs(st["pitch"]))
        if tilt > cfg.tilt_limit:
            self._settle_until = max(self._settle_until, self.t + cfg.settle_time)
            self._next_settle_t = self.t + cfg.settle_period
        settling = self.t < self._settle_until

        if dist_goal < cfg.goal_tol:
            self.mode = "done"
            self.v, self.w = 0.0, 0.0
        elif settling:
            self.mode = "settle"
            self.v, self.w = 0.0, 0.0
        elif self.policy == "e2e" and self.depth is not None:
            self.mode = "e2e"
            depth = self.depth.depth(self.sim.data)
            v_raw, w_raw = self.net.end_to_end(depth)
            self.v, self.w = self._smooth(v_raw, w_raw, dt)
        elif self.policy == "neural":
            self.mode = "neural"
            v_raw, w_raw = self._neural_command(pose, st)
            self.v, self.w = self._smooth(v_raw, w_raw, dt)
        elif self.path:
            self.mode = "walk"
            self.v, self.w, self.path_idx = self.pp.control(pose, self.path, self.path_idx)
            if self.v < 0.05 and abs(self.w) > 0.3:
                self.v = 0.08  # 原地转向时保持最小步频
        else:
            self.mode = "blocked"
            self.v, self.w = 0.0, 0.0

        self.walker.command(self.v, self.w)
        feet = {"l": tuple(self.sim.ankle_pos("l")[:2]), "r": tuple(self.sim.ankle_pos("r")[:2])}
        q = self.walker.update(dt, actual_pos=st["pos"], actual_yaw=st["yaw"],
                               roll=st["roll"], pitch=st["pitch"], vel=st["vel"], feet_xy=feet)
        self.sim.set_targets(q)
        self.sim.step(int(round(dt / self.sim.model.opt.timestep)))

        # 指标
        pos = np.array(self.sim.base_pos()[:2])
        self.travel += float(np.linalg.norm(pos - self._prev_pos))
        self._prev_pos = pos
        self.t += dt
        if self._check_collision():
            if self.t - self._last_collision_t > 0.6:
                self.collisions += 1
                self._last_collision_t = self.t
                self.path = []          # 撞到东西立刻重规划
        return q

    def _check_collision(self) -> bool:
        m, d = self.sim.model, self.sim.data
        for i in range(d.ncon):
            c = d.contact[i]
            for gid in (c.geom1, c.geom2):
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
                if name.startswith("obst") or name.startswith("wall"):
                    if abs(c.dist) > 0.002:
                        return True
        return False

    # ------------------------------------------------------------------
    def run(self, max_time: Optional[float] = None) -> dict:
        """跑完一个 episode，返回统计结果。"""
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
