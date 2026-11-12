#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 小车自动打靶：车固定，炮塔自动识别立着的靶子并转动瞄准，依次击倒.

三种跑法：

    # 1) 打开 MuJoCo 可视化窗口（默认）
    python3 src/ground/mujoco_shooting/main.py

    # 2) 无头跑一局（不开窗口，跑完打印结果，返回码 0 = 全部击倒）
    python3 src/ground/mujoco_shooting/main.py --headless --max-steps 12000

    # 3) 无头 + 侧风 + 成绩单（逐发记录写 CSV）
    python3 src/ground/mujoco_shooting/main.py --headless --wind 0.8 0.3 --report out.csv

无头模式是这次重构的重点：把"能不能打中"从"用眼睛看"变成"可断言的结果"，
这样既能进 CI，也能在没有显示器的机器上验证。

``--wind`` 是水平风加速度（m/s²），由 ``ballistics.solve_launch_velocity``
在发射时补偿，仿真里按 ``F = m·a`` 每步作用在飞行中的小球上。
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as ctrl  # noqa: E402
from ballistics import score_shots  # noqa: E402
from scene import XML  # noqa: E402

#: 小球质量（与 scene.py 里的 geom mass 一致），用于把风加速度换算成力
BALL_MASS = 0.1


def build_model():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    return model, data


class Shooter(object):
    """打靶状态机：扫描 -> 选靶 -> 瞄准 -> 开火 -> 冷却，直到全部击倒/弹尽/超时."""

    SCAN_STEPS = 400
    COOLDOWN_STEPS = 300

    def __init__(self, model, data, targets=ctrl.TARGETS, wind=(0.0, 0.0),
                 max_shots=12):
        self.model = model
        self.data = data
        self.targets = list(targets)
        self.wind = (float(wind[0]), float(wind[1]))
        self.max_shots = int(max_shots)
        self.n = 0
        self.shot = 0
        self.cool = 0
        self.scan = 1.0
        self.scan_t = 0
        self.last_tid = -1
        self.hit = [0] * len(self.targets)
        self.shots_log = []
        self.reason = "超时"

    # ------------------------------------------------------------------ 查询
    def yaw(self):
        return float(self.data.qpos[ctrl.TURRET_YAW_QPOS])

    def yaw_rate(self):
        return float(self.data.qvel[ctrl.TURRET_YAW_QVEL])

    def down(self):
        return ctrl.down_flags(self.data.qpos, len(self.targets))

    def stats(self):
        summary = score_shots(self.shots_log, sum(self.down()),
                              float(self.data.time), self.reason,
                              total_targets=len(self.targets))
        summary.update({
            "steps": self.n,
            "targets": len(self.targets),
            "all_down": ctrl.all_down(self.data.qpos, len(self.targets)),
            "time_s": round(float(self.data.time), 2),
            "wind": self.wind,
            # 兼容旧字段名：原统计用的是 down，成绩汇总里叫 downed
            "down": summary["downed"],
        })
        return summary

    # ------------------------------------------------------------------ 单步
    def step(self):
        """推进一个物理步；返回 (running, tid)，tid 是当前锁定目标（-1 表示没有）."""
        before = self.down()
        if ctrl.all_down(self.data.qpos, len(self.targets)):
            self.reason = "全部击倒"
            self.data.ctrl[:] = 0
            return False, -1
        if self.shot >= self.max_shots:
            self.reason = "球用尽"
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

        # 风：对已经打出去的球施加恒定水平力（F = m·a）
        if self.wind[0] or self.wind[1]:
            for k in range(self.shot):
                body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ball%d" % k)
                if body >= 0:
                    self.data.xfrc_applied[body, 0] = BALL_MASS * self.wind[0]
                    self.data.xfrc_applied[body, 1] = BALL_MASS * self.wind[1]

        mujoco.mj_step(self.model, self.data)
        self.n += 1

        # 击倒判定：从"立着"变成"倒下"的那一步，归因给最近一次打它的那发
        for i, is_down in enumerate(self.down()):
            if is_down and not before[i]:
                for shot in reversed(self.shots_log):
                    if shot["target"] == i and not shot["hit"]:
                        shot["hit"] = True
                        break
        return True, tid

    def _fire(self, tid):
        bq = ctrl.BALL_Q0 + self.shot * ctrl.BALL_DOF_Q
        bv = ctrl.BALL_V0 + self.shot * ctrl.BALL_DOF_V
        muzzle = ctrl.muzzle_position(self.yaw())
        boost = 1.25 if self.hit[tid] > 0 else 1.0
        vx, vy, vz, dist = ctrl.launch_velocity(
            muzzle, self.targets[tid], boost=boost, wind=self.wind)

        self.data.qpos[bq:bq + 3] = [muzzle[0], muzzle[1], ctrl.MUZZLE_Z]
        self.data.qpos[bq + 3:bq + 7] = [0.0, 0.0, 0.0, 1.0]
        self.data.qvel[bv:bv + 3] = [vx, vy, vz]
        self.data.qvel[bv + 3:bv + 6] = [0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, self.data)

        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        self.shots_log.append({
            "shot": self.shot + 1,
            "time": float(self.data.time),
            "target": tid,
            "distance": dist,
            "speed": speed,
            "hit": False,
        })

        if tid != self.last_tid:
            print("锁定靶%d 距离%.2fm" % (tid, dist))
            self.last_tid = tid
        print("第%d发 发射 力度%.1f" % (self.shot + 1, speed))
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


def write_report(path, shots, stats):
    """把逐发记录写成 CSV（UTF-8 BOM，Excel 可直接打开），末尾附一行汇总."""
    folder = os.path.dirname(os.path.abspath(path))
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    with open(path, "w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.writer(fh)
        writer.writerow(["shot", "sim_time_s", "target", "distance_m", "speed_m_s", "hit"])
        for shot in shots:
            writer.writerow([shot["shot"], "%.3f" % shot["time"], shot["target"],
                             "%.3f" % shot["distance"], "%.3f" % shot["speed"],
                             1 if shot["hit"] else 0])
        writer.writerow([])
        writer.writerow(["summary", "shots=%d" % stats["shots"],
                         "downed=%d" % stats["downed"],
                         "hit_rate=%.2f" % stats["hit_rate"],
                         "duration=%.2fs" % stats["duration"],
                         "wind=(%.2f,%.2f)" % tuple(stats["wind"]),
                         "reason=%s" % stats["reason"]])


def format_summary(stats):
    """把成绩排成一段可读文本."""
    avg = stats["avg_shots_per_down"]
    lines = [
        "=" * 46,
        "打靶成绩",
        "=" * 46,
        "  结束原因    : %s" % stats["reason"],
        "  发射发数    : %d" % stats["shots"],
        "  击倒靶数    : %d / %d" % (stats["downed"], stats["targets"]),
        "  命中率      : %.1f%%" % (stats["hit_rate"] * 100.0),
        "  一发一个靶  : %d 个" % stats["first_shot_hits"],
        "  平均用弹    : %s" % ("%.2f 发/靶" % avg if avg is not None else "—"),
        "  耗时        : %.2f s" % stats["duration"],
        "  风加速度    : (%.2f, %.2f) m/s²" % tuple(stats["wind"]),
        "=" * 46,
    ]
    return "\n".join(lines)


def run_headless(max_steps=12000, wind=(0.0, 0.0), max_shots=12,
                 report=None, verbose=True):
    """不开窗口地跑完整一局，返回统计（供测试 / CI 断言）."""
    model, data = build_model()
    shooter = Shooter(model, data, wind=wind, max_shots=max_shots)
    tid = -1
    for _ in range(max_steps):
        running, tid = shooter.step()
        if not running:
            break
    paint_crosshair(model, tid)
    stats = shooter.stats()
    if report:
        write_report(report, shooter.shots_log, stats)
    if verbose:
        print("结果: %s" % stats)
        print()
        print(format_summary(stats))
    return stats


def run_viewer(max_steps=12000, wind=(0.0, 0.0), max_shots=12):
    import mujoco.viewer  # import lazily: headless machines never touch it
    model, data = build_model()
    shooter = Shooter(model, data, wind=wind, max_shots=max_shots)
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
    parser.add_argument("--time-limit", type=float, default=None,
                        help="仿真时长上限（秒）；给定时覆盖 --max-steps")
    parser.add_argument("--wind", nargs=2, type=float, default=(0.0, 0.0),
                        metavar=("AX", "AY"), help="水平风加速度 m/s²，默认 0 0")
    parser.add_argument("--max-shots", type=int, default=12, help="最多发射几发，默认 12")
    parser.add_argument("--report", default=None, help="逐发记录写成 CSV")
    args = parser.parse_args(argv)

    max_steps = args.max_steps
    if args.time_limit is not None:
        max_steps = max(1, int(args.time_limit / 0.002))

    if args.headless:
        stats = run_headless(max_steps, wind=tuple(args.wind),
                             max_shots=args.max_shots, report=args.report)
    else:
        stats = run_viewer(max_steps, wind=tuple(args.wind),
                           max_shots=args.max_shots)
        if args.report:
            print("提示：--report 只在无头模式下生效")
    return 0 if stats["all_down"] else 1


if __name__ == "__main__":
    sys.exit(main())
