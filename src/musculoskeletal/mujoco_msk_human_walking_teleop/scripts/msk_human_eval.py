#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线多场景性能评测（headless，不需要 ROS 与图形界面）。

用法：
  python3 msk_human_eval.py [--out eval_results.json]

场景与指标：
  平地站立 3 次 x 6 s   成功率 / 末态俯仰 / 位置漂移
  平地蹲起 15 s         完成周期数 / 最大蹲深 / 是否摔倒
  平地踢腿 3 次         脚尖峰值高度 / 踢后 2 s 是否摔倒
  平地行走 20 s         距离 / 平均速度 / 步频能量(激活努力) / 是否摔倒
  5 度坡行走 15 s       距离 / 是否摔倒
  负载(10kg)行走 15 s   距离 / 是否摔倒
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from msk_human_core import (Composer, MskHumanSim, load_params)


def run(sim, comp, duration, dt=0.002, on_step=None):
    """统一仿真循环，返回 (摔倒时刻 or None, effort_acc)。"""
    n = int(duration / dt)
    acc = {"act": 0.0, "work": 0.0}
    for i in range(n):
        u = comp.update(sim, dt)
        sim.step(u)
        e = sim.effort(u, dt)
        acc = e
        if on_step is not None:
            on_step(i * dt)
        if sim.fallen():
            return i * dt, acc
    return None, acc


def trial_stand(sim, p, seed=None):
    comp = Composer(p)
    comp.set_action("stand")
    sim.reset()
    x0 = sim.pelvis_x()
    t_fall, acc = run(sim, comp, 6.0)
    ok = t_fall is None and abs(sim.pitch()) < 0.20 and abs(sim.pelvis_x() - x0) < 0.20
    return {"ok": ok, "pitch": round(float(sim.pitch()), 4),
            "drift": round(float(sim.pelvis_x() - x0), 4),
            "z_end": round(float(sim.pelvis_z()), 4)}


def trial_squat(sim, p):
    comp = Composer(p)
    comp.set_action("squat")
    sim.reset()
    z_min = [1.0]
    cycles = [0]
    in_low = [False]

    def cb(_t):
        z = sim.pelvis_z()
        z_min[0] = min(z_min[0], z)
        # 实际完成周期数：骨盆高度越过下蹲阈值以下再回到其上计 1 次
        if z < 0.78:
            in_low[0] = True
        elif in_low[0] and z > 0.83:
            in_low[0] = False
            cycles[0] += 1

    t_fall, acc = run(sim, comp, 15.0, on_step=cb)
    return {"ok": t_fall is None and cycles[0] >= 5, "cycles": cycles[0],
            "z_min": round(z_min[0], 4), "depth": round(1.0 - z_min[0], 3),
            "work_J": round(acc["work"], 1)}


def trial_kick(sim, p):
    comp = Composer(p)
    comp.set_action("stand")
    sim.reset()
    for _ in range(1000):            # 先站稳 2 s
        sim.step(comp.update(sim, 0.002))
    peak = [0.0]

    def cb(_t):
        peak[0] = max(peak[0], float(sim.toe_pos("r")[2]))

    comp.set_action("kick")
    t_fall, _ = run(sim, comp, 3.0, on_step=cb)
    return {"ok": t_fall is None and peak[0] > 0.30,
            "toe_peak_m": round(peak[0], 3)}


def trial_walk(sim, p, duration=20.0):
    comp = Composer(p)
    comp.set_action("stand")
    sim.reset()
    for _ in range(500):             # 站稳 1 s
        sim.step(comp.update(sim, 0.002))
    x0 = sim.pelvis_x()
    comp.set_action("walk")
    t_fall, acc = run(sim, comp, duration)
    dist = sim.pelvis_x() - x0
    speed = dist / max(duration, 1e-6)
    return {"ok": t_fall is None and dist > 1.0, "dist_m": round(float(dist), 3),
            "speed_mps": round(float(speed), 3),
            "act_effort": round(acc["act"], 1),
            "work_J": round(acc["work"], 1),
            "work_per_m": round(acc["work"] / max(dist, 1e-6), 1),
            "fell_at": t_fall}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "eval_results.json"))
    args = ap.parse_args()

    p = load_params()
    results = {}
    sims = {sc: MskHumanSim(sc) for sc in ("flat", "slope5", "load")}

    print("== 平地站立 x3 ==")
    results["stand_flat"] = [trial_stand(sims["flat"], p) for _ in range(3)]

    print("== 平地蹲起 ==")
    results["squat_flat"] = trial_squat(sims["flat"], p)

    print("== 平地踢腿 x3 ==")
    results["kick_flat"] = [trial_kick(sims["flat"], p) for _ in range(3)]

    print("== 平地行走 20s ==")
    results["walk_flat"] = trial_walk(sims["flat"], p, 20.0)

    print("== 5 度坡行走 15s ==")
    results["walk_slope5"] = trial_walk(sims["slope5"], p, 15.0)

    print("== 负载 10kg 行走 15s ==")
    results["walk_load"] = trial_walk(sims["load"], p, 15.0)

    for s in sims.values():
        s.close()

    def ok_rate(trials):
        return sum(1 for t in trials if t["ok"]) / len(trials)

    summary = [
        ("平地站立成功率", "%d/3" % round(ok_rate(results["stand_flat"]) * 3)),
        ("蹲起周期数(15s)", str(results["squat_flat"]["cycles"])),
        ("踢腿脚尖峰值(m)", str(max(t["toe_peak_m"]
                                    for t in results["kick_flat"]))),
        ("平地行走距离(m)/速度(m/s)", "%s / %s" % (
            results["walk_flat"]["dist_m"], results["walk_flat"]["speed_mps"])),
        ("下坡行走距离(m)", str(results["walk_slope5"]["dist_m"])),
        ("负载行走距离(m)", str(results["walk_load"]["dist_m"])),
    ]
    print("\n================= 性能汇总 =================")
    for k, v in summary:
        print("%-26s %s" % (k, v))
    print("=============================================")

    with open(args.out, "w") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print("结果已写入", args.out)


if __name__ == "__main__":
    main()
