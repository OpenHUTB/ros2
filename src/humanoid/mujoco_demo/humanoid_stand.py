#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人形机器人基础示例：MuJoCo 里让一个 10 关节的人形"站住".

从 falling_ball.py 再往前走一步：这次不是一个自由落体的小球，而是一个有质量、
有关节、会被重力压弯的人形。脚本做两件事：

    1. 逐个关节做 PD 位置控制（目标 = 站立姿态，所有关节角为 0）；
    2. 用 mj_rne 的偏置力做重力补偿，让 PD 只负责修小误差。

两种跑法：

    python3 src/humanoid/mujoco_demo/humanoid_stand.py            # 可视化
    python3 src/humanoid/mujoco_demo/humanoid_stand.py --headless # 无头，站住返回 0
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as ctrl  # noqa: E402
from scene_humanoid import JOINTS, XML  # noqa: E402

# 每个关节一组 PD 增益：腿要硬（撑体重），胳膊可以软一点
GAINS = {
    "hip": (300.0, 25.0),
    "knee": (300.0, 25.0),
    "ankle": (150.0, 12.0),
    "shoulder": (60.0, 5.0),
    "elbow": (40.0, 3.0),
}
TORQUE_LIMIT = 200.0


def gains_for(name):
    for prefix, pair in GAINS.items():
        if name.startswith(prefix):
            return pair
    return (50.0, 5.0)


class Humanoid(object):
    """把模型、关节索引、PD 目标打包在一起的小控制器."""

    def __init__(self, model, data, joint_names=JOINTS):
        self.model = model
        self.data = data
        self.names = list(joint_names)
        self.qadr = [model.jnt_qposadr[model.joint(n).id] for n in self.names]
        self.vadr = [model.jnt_dofadr[model.joint(n).id] for n in self.names]
        self.act = [model.actuator("m_" + n).id for n in self.names]
        self.kp = [gains_for(n)[0] for n in self.names]
        self.kd = [gains_for(n)[1] for n in self.names]
        self.q_des = [0.0] * len(self.names)

    # ------------------------------------------------------------------ 状态
    def ground(self):
        """把骨盆高度调到"脚底刚好贴地"，避免开局穿模或悬空."""
        self.data.qpos[:] = 0.0
        self.data.qpos[3] = 1.0          # 单位四元数
        mujoco.mj_forward(self.model, self.data)
        lowest = min(self.data.geom_xpos[g][2] - self.model.geom_size[g][2]
                     for g in range(self.model.ngeom) if self.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
                     and self.model.geom_bodyid[g] != 0)
        self.data.qpos[2] = -lowest + 0.002   # 脚底刚好落在 z=0
        mujoco.mj_forward(self.model, self.data)

    def height(self):
        return float(self.data.qpos[2])

    def quat(self):
        return tuple(float(v) for v in self.data.qpos[3:7])

    def step(self):
        q = [float(self.data.qpos[i]) for i in self.qadr]
        qd = [float(self.data.qvel[i]) for i in self.vadr]
        bias = [float(self.data.qfrc_bias[i]) for i in self.vadr]
        tau = ctrl.saturate(ctrl.pd_torque(q, qd, self.q_des, self.kp, self.kd, bias), TORQUE_LIMIT)
        for idx, value in zip(self.act, tau):
            self.data.ctrl[idx] = value
        mujoco.mj_step(self.model, self.data)


def run(headless=False, seconds=5.0, max_steps=None):
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    robot = Humanoid(model, data)
    robot.ground()
    base_height = robot.height()
    steps = max_steps or int(seconds / model.opt.timestep)
    print("初始骨盆高度 %.3f m，关节 %d 个，控制步数 %d" % (base_height, len(JOINTS), steps))

    worst = base_height
    if headless:
        for _ in range(steps):
            robot.step()
            worst = min(worst, robot.height())
        stats = {
            "height": round(robot.height(), 3),
            "base_height": round(base_height, 3),
            "min_height": round(worst, 3),
            "tilt_deg": round(ctrl.tilt_deg(robot.quat()), 2),
            "upright": ctrl.is_standing(robot.height(), base_height, quat_wxyz=robot.quat()),
        }
        print("结果: %s" % stats)
        return stats

    import importlib
    viewer_mod = importlib.import_module("mujoco.viewer")   # 只有开窗口时才需要
    with viewer_mod.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.2
        viewer.cam.elevation = -12
        viewer.cam.azimuth = 120
        for _ in range(steps):
            if not viewer.is_running():
                break
            robot.step()
            viewer.sync()
            time.sleep(model.opt.timestep)
    return {
        "height": round(robot.height(), 3),
        "base_height": round(base_height, 3),
        "upright": ctrl.is_standing(robot.height(), base_height, quat_wxyz=robot.quat()),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="MuJoCo 人形机器人站立示例")
    parser.add_argument("--headless", action="store_true", help="不开窗口，站住返回 0")
    parser.add_argument("--seconds", type=float, default=5.0, help="仿真时长（默认 5 s）")
    args = parser.parse_args(argv)
    stats = run(headless=args.headless, seconds=args.seconds)
    return 0 if stats["upright"] else 1


if __name__ == "__main__":
    sys.exit(main())
