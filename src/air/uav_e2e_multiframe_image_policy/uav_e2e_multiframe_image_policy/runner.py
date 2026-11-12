#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端多帧策略闭环：下视图像 -> 历史堆叠 -> 策略网络 -> 控制指令。

不依赖 ROS，离线仿真可直接驱动；ROS 节点在 e2e_node.py 中按节拍调用 step。
"""

from __future__ import annotations

import numpy as np

from .camera import render_patch, FrameHistory
from .policy import E2EImagePolicy
from .controller import ImagePolicyController
from . import scene

DT = 1.0 / 20.0
MAX_STEPS = 420


class E2ERunner:
    """端到端策略闭环仿真器。"""

    def __init__(self) -> None:
        self.controller = ImagePolicyController(
            x=0.5, y=scene.centerline_y(0.5), yaw=0.0)
        self.history = FrameHistory()
        self.policy = E2EImagePolicy()
        self.trajectory: list[tuple[float, float]] = [
            (self.controller.x, self.controller.y)]
        self.metrics = {"max_abs_lateral": 0.0, "final_lateral": 0.0,
                        "x_reached": 0.0, "max_speed": 0.0, "steps": 0}
        self.step_count = 0
        self.done = False
        self.last_decision: dict = {"forward": 0.0, "yaw_cmd": 0.0}

    def step(self) -> dict:
        if self.done:
            return self.status()
        frame = render_patch(self.controller.x, self.controller.y, self.controller.yaw)
        self.history.update(frame)
        stack = self.history.stack()
        decision = self.policy.forward(stack)
        self.last_decision = decision
        self.controller.update(decision["forward"], decision["yaw_cmd"], DT)

        off = scene.lateral_offset(self.controller.x, self.controller.y)
        self.metrics["max_abs_lateral"] = max(
            self.metrics["max_abs_lateral"], abs(off))
        self.metrics["x_reached"] = max(self.metrics["x_reached"], self.controller.x)
        self.metrics["max_speed"] = max(self.metrics["max_speed"], decision["forward"])
        self.trajectory.append((self.controller.x, self.controller.y))
        self.step_count += 1

        if self.controller.x >= scene.X_MAX or self.step_count >= MAX_STEPS:
            self.done = True
            self.metrics["final_lateral"] = off
            self.metrics["steps"] = self.step_count
        return self.status()

    def status(self) -> dict:
        return {
            "x": self.controller.x,
            "y": self.controller.y,
            "yaw": self.controller.yaw,
            "forward": self.last_decision["forward"],
            "yaw_cmd": self.last_decision["yaw_cmd"],
            "offset": scene.lateral_offset(self.controller.x, self.controller.y),
            "steps": self.step_count,
            "done": self.done,
        }

    def run(self) -> list[dict]:
        history: list[dict] = []
        while not self.done:
            history.append(self.step())
        return history
