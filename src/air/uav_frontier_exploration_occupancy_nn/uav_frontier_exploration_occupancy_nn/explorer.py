#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""前沿探索闭环：建图 -> 前沿提取 -> 网络选点 -> 控制器导航。

不依赖 ROS，离线仿真可直接驱动；ROS 节点在 explorer_node.py 中按节拍调用 step。
"""

from __future__ import annotations

import math

import numpy as np

from .occupancy_map import OccupancyMap, OCCUPIED
from .frontier import extract_candidates
from .frontier_policy import FrontierScoreNetwork
from .controller import ExplorerController

SENSOR_RAYS = 18
SENSOR_RANGE = 2.6
SENSOR_FOV = math.radians(240.0)
DT = 1.0 / 20.0
MAX_STEPS = 600


class FrontierExplorer:
    """前沿探索闭环控制器。"""

    def __init__(self) -> None:
        self.map = OccupancyMap(width=40, height=30, resolution=0.25)
        self._build_world()
        self.policy = FrontierScoreNetwork()
        self.ctrl = ExplorerController(x=0.5, y=0.5, yaw=0.0)
        self.trajectory: list[tuple[float, float]] = [(self.ctrl.x, self.ctrl.y)]
        self.target: dict | None = None
        self.scores: np.ndarray = np.zeros(0)
        self.n_candidates = 0
        self.blocked_steps = 0
        self.step_count = 0
        self.done = False

    def _build_world(self) -> None:
        m = self.map
        m.add_obstacle_rect(3.0, 1.5, 3.6, 2.1)
        m.add_obstacle_rect(6.0, 3.5, 6.6, 4.1)
        m.add_obstacle_rect(4.0, 5.0, 4.6, 5.6)

    def _sensor_angles(self) -> list[float]:
        half = SENSOR_FOV / 2.0
        return [self.ctrl.yaw - half + SENSOR_FOV * i / (SENSOR_RAYS - 1)
                for i in range(SENSOR_RAYS)]

    def _clearance(self) -> dict:
        def free_at(angle: float, dist: float) -> bool:
            x = self.ctrl.x + dist * math.cos(angle)
            y = self.ctrl.y + dist * math.sin(angle)
            c, r = self.map.world_to_cell(x, y)
            return self.map.grid[r, c] != OCCUPIED

        return {
            "ahead": free_at(self.ctrl.yaw, 0.55),
            "left": free_at(self.ctrl.yaw + math.pi / 4, 0.55),
            "right": free_at(self.ctrl.yaw - math.pi / 4, 0.55),
        }

    def step(self) -> dict:
        if self.done:
            return self.status()

        self.map.sense(self.ctrl.x, self.ctrl.y, self._sensor_angles(), SENSOR_RANGE)
        candidates = extract_candidates(self.map, self.ctrl.x, self.ctrl.y, self.ctrl.yaw)
        self.n_candidates = len(candidates)

        decision = self.policy.forward(candidates)
        self.scores = decision["scores"]
        self.target = decision["target"]

        if self.target is None:
            self.done = True
            return self.status()

        clearance = self._clearance()
        if not clearance["ahead"]:
            self.blocked_steps += 1
        prev_x, prev_y = self.ctrl.x, self.ctrl.y
        ctrl = self.ctrl.update(self.target["x"], self.target["y"], DT, clearance)

        # 安全护盾：新位姿落入占据格则回退，不驶入障碍。
        cc, rr = self.map.world_to_cell(self.ctrl.x, self.ctrl.y)
        if self.map.grid[rr, cc] == OCCUPIED:
            self.ctrl.x, self.ctrl.y = prev_x, prev_y
            self.ctrl.vx, self.ctrl.vy = 0.0, 0.0

        self.trajectory.append((self.ctrl.x, self.ctrl.y))
        self.step_count += 1

        # 到达当前前沿后继续重规划，仅在无候选或达到步数上限时结束。
        if self.step_count >= MAX_STEPS:
            self.done = True
        return self.status()

    def status(self) -> dict:
        return {
            "x": self.ctrl.x,
            "y": self.ctrl.y,
            "yaw": self.ctrl.yaw,
            "target": self.target,
            "scores": self.scores,
            "n_candidates": self.n_candidates,
            "free": self.map.explored_free_count(),
            "unknown": self.map.unknown_count(),
            "frontiers": len(self.map.frontier_cells()),
            "blocked_steps": self.blocked_steps,
            "steps": self.step_count,
            "done": self.done,
        }

    def run(self) -> list[dict]:
        history: list[dict] = []
        while not self.done:
            history.append(self.step())
        return history
