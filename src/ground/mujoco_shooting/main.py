#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 小车自动打靶：车固定，炮塔自动识别立着的靶子并转动瞄准，依次击倒.

两种跑法：

    # 1) 打开 MuJoCo 可视化窗口（默认）
    python3 src/ground/mujoco_shooting/main.py

    # 2) 无头跑一局（不开窗口，跑完打印结果，返回码 0 = 全部击倒）
    python3 src/ground/mujoco_shooting/main.py --headless --max-steps 12000

无头模式是这次重构的重点：把"能不能打中"从"用眼睛看"变成"可断言的结果"，
这样既能进 CI，也能在没有显示器的机器上验证。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as ctrl  # noqa: E402
from scene import XML  # noqa: E402


def build_model():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    return model, data


class Shooter(object):
    """打靶状态机：扫描 -> 选靶 -> 瞄准 -> 开火 -> 冷却，直到全部击倒/弹尽/超时."""

    MAX_BALLS = 12
    SCAN_STEPS = 400
    COOLDOWN_STEPS = 300

    def __init__(self, model, data, targets=ctrl.TARGETS):
        self.model = model
        self.data = data
        self.targets = list(targets)
        self.n = 0
        self.shot = 0
        self.cool = 0
        self.scan = 1.0
        self.scan_t = 0
        self.last_tid = -1
        self.hit = [0] * len(self.targets)

    # ------------------------------------------------------------------ 查询
    def yaw(self):
        return float(self.data.qpos[ctrl.TURRET_YAW_QPOS])

    def yaw_rate(self):
        return float(self.data.qvel[ctrl.TURRET_YAW_QVEL])

    def down(self):
        return ctrl.down_flags(self.data.qpos, len(self.targets))

    def stats(self):
        return {
            "steps": self.n,
            "shots": self.shot,
            "down": sum(self.down()),
            "targets": len(self.targets),
            "all_down": ctrl.all_down(self.data.qpos, len(self.targets)),
            "time_s": round(float(self.data.time), 2),
        }

    # ------------------------------------------------------------------ 单步
    def step(self):
        """推进一个物理步；返回 (running, tid)，tid 是当前锁定目标（-1 表示没有）."""
        if ctrl.all_down(self.data.qpos, len(self.targets)):
            self.data.ctrl[:] = 0
            return False, -1
        if self.shot >= self.MAX_BALLS:
            self.data.ctrl[:] = 0
            return False, -1

        tid = -1
        if self.n < self.SCAN_STEPS:
            # 开场雷达扫描：炮塔左右扫场，激光跟着摆
            self.data.ctrl[0] = 5.0 * self.scan
            self.scan_t += 1
            if self.scan_t > 130:
                self.scan = -self.scan
                self.scan_t = 0
        elif self.cool > 0:
            self.cool -= 1
            self.data.ctrl[0] = 0.0
        else:
            tid = ctrl.select_target(self.yaw(), self.targets, self.down())
            if tid == -1:
                self.data.ctrl[0] = 0.0
            else:
                target = self.targets[tid]
                self.data.ctrl[0] = ctrl.turret_command(self.yaw(), self.yaw_rate(), target)
                if ctrl.is_aligned(self.yaw(), self.yaw_rate(), target):
                    self._fire(tid)

        mujoco.mj_step(self.model, self.data)
        self.n += 1
        return True, tid

    def _fire(self, tid):
        bq = ctrl.BALL_Q0 + self.shot * ctrl.BALL_DOF_Q
        bv = ctrl.BALL_V0 + self.shot * ctrl.BALL_DOF_V
        muzzle = ctrl.muzzle_position(self.yaw())
        boost = 1.25 if self.hit[tid] > 0 else 1.0
        vx, vy, vz, dist = ctrl.launch_velocity(muzzle, self.targets[tid], boost=boost)

        self.data.qpos[bq:bq + 3] = [muzzle[0], muzzle[1], ctrl.MUZZLE_Z]
        self.data.qpos[bq + 3:bq + 7] = [0.0, 0.0, 0.0, 1.0]
        self.data.qvel[bv:bv + 3] = [vx, vy, vz]
        self.data.qvel[bv + 3:bv + 6] = [0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, self.data)

        if tid != self.last_tid:
            print("锁定靶%d 距离%.2fm" % (tid, dist))
            self.last_tid = tid
        print("第%d发 发射 力度%.1f" % (self.shot + 1, math.hypot(vx, vy)))
        self.hit[tid] += 1
        self.cool = self.COOLDOWN_STEPS
        self.shot += 1


def paint_crosshair(model, tid):
    """被锁定的靶子上方亮起白色准星（其余透明）."""
    for i in range(len(ctrl.TARGETS)):
        alpha = 1.0 if i == tid else 0.0
        for prefix in ("cx", "cy"):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "%s%d" % (prefix, i))
            if gid >= 0:
                model.geom_rgba[gid] = [1.0, 1.0, 1.0, alpha]


def run_headless(max_steps=12000):
    """不开窗口地跑完整一局，返回统计（供测试 / CI 断言）."""
    model, data = build_model()
    shooter = Shooter(model, data)
    tid = -1
    for _ in range(max_steps):
        running, tid = shooter.step()
        if not running:
            break
    paint_crosshair(model, tid)
    stats = shooter.stats()
    print("结果: %s" % stats)
    return stats


def run_viewer(max_steps=12000):
    import mujoco.viewer  # import lazily: headless machines never touch it
    model, data = build_model()
    shooter = Shooter(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.distance = 9.0
        viewer.cam.elevation = -40
        viewer.cam.azimuth = 90
        tid = -1
        while viewer.is_running():
            running, tid = shooter.step()
            if not running or shooter.n >= max_steps:
                break
            paint_crosshair(model, tid)
            viewer.cam.lookat[0] = 0.6 * math.cos(shooter.yaw())
            viewer.cam.lookat[1] = 0.6 * math.sin(shooter.yaw())
            viewer.sync()
            time.sleep(0.002)
    stats = shooter.stats()
    print("结果: %s" % stats)
    return stats


def main(argv=None):
    parser = argparse.ArgumentParser(description="MuJoCo 小车自动打靶")
    parser.add_argument("--headless", action="store_true", help="不开可视化窗口，跑完打印结果")
    parser.add_argument("--max-steps", type=int, default=12000, help="最大物理步数（0.002 s/步）")
    args = parser.parse_args(argv)

    stats = run_headless(args.max_steps) if args.headless else run_viewer(args.max_steps)
    return 0 if stats["all_down"] else 1


if __name__ == "__main__":
    sys.exit(main())
