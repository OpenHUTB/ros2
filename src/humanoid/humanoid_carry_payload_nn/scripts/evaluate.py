#!/usr/bin/env python3
"""evaluate.py —— 三种负载场景下的控制器对照评测。

对 0 / 2 / 4 kg 三种负载分别运行三个控制器：
    open : 纯开环步态（无平衡补偿，基线）
    pd   : PD 姿态反馈专家
    nn   : 神经网络残差控制器（models/balance_mlp.npz）
每个场景跑固定时长，统计躯干俯仰的均值/RMS/峰值、是否跌倒与等效搬运里程，
最后打印对照表并写出一份 CSV（默认 results/eval_<时间戳>.csv）。

用法：
    python3 scripts/evaluate.py
    python3 scripts/evaluate.py --masses 0 2 4 --seconds 20 --speed 0.20
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import BalanceMLP, PDExpert                     # noqa: E402
from sim_core import CarryPayloadSim, package_path         # noqa: E402

DEFAULT_MASSES = (0.0, 2.0, 4.0)


def scenario_command(t, speed):
    """评测用指令曲线：0.8 s 内加速到目标速度，4~7 s 轻微转向，随后匀速搬运。"""
    v = speed * min(1.0, t / 0.8)
    w = 0.15 * math.sin(0.5 * t) if 4.0 < t < 7.0 else 0.0
    return float(v), float(w)


def run_episode(mass, controller, seconds, speed, dt_sample=0.01):
    sim = CarryPayloadSim(payload_mass=mass)
    sim.reset(mass)
    n = int(seconds / sim.dt)
    every = max(1, int(dt_sample / sim.dt))
    pitches, fallen_at = [], None
    for i in range(n):
        t = i * sim.dt
        v_cmd, w_cmd = scenario_command(t, speed)
        sim.step(v_cmd=v_cmd, w_cmd=w_cmd, controller=controller)
        if i % every == 0:
            _, pitch, _, _ = sim.posture()
            pitches.append(pitch)
            if fallen_at is None and not sim.standing():
                fallen_at = round(t, 2)
                break
    dist = sim.distance
    sim.close()
    arr = np.degrees(np.asarray(pitches)) if pitches else np.zeros(1)
    return {
        "pitch_mean_deg": float(arr.mean()),
        "pitch_rms_deg": float(np.sqrt(np.mean(arr ** 2))),
        "pitch_max_deg": float(np.abs(arr).max()),
        "fell_at_s": fallen_at,
        "distance_m": float(dist),
        "survived": fallen_at is None,
    }


def main():
    ap = argparse.ArgumentParser(description="负载场景控制器对照评测")
    ap.add_argument("--masses", type=float, nargs="+", default=list(DEFAULT_MASSES))
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--speed", type=float, default=0.20)
    ap.add_argument("--weights", default=package_path("models/balance_mlp.npz"))
    ap.add_argument("--output", default="")
    args = ap.parse_args()

    controllers = [("open", None), ("pd", PDExpert())]
    if os.path.isfile(args.weights):
        controllers.append(("nn", BalanceMLP.load(args.weights)))
    else:
        print("[eval] 未找到权重 %s，跳过神经网络对照（先运行 train_mlp.py）"
              % args.weights)

    rows = []
    print("\n%-12s %-6s %8s %8s %8s %8s %6s" %
          ("场景", "控制器", "俯仰均值", "俯仰RMS", "俯仰峰值", "里程(m)", "跌倒"))
    for mass in args.masses:
        for name, ctrl in controllers:
            r = run_episode(mass, ctrl, args.seconds, args.speed)
            rows.append(dict(scene="%.0fkg" % mass, controller=name,
                             speed=args.speed, seconds=args.seconds, **r))
            print("%-12s %-6s %8.2f %8.2f %8.2f %8.2f %6s" %
                  ("%.0f kg" % mass, name, r["pitch_mean_deg"], r["pitch_rms_deg"],
                   r["pitch_max_deg"], r["distance_m"],
                   "-" if r["survived"] else "%.2fs" % r["fell_at_s"]))
        print()

    out = args.output or package_path(
        os.path.join("results", "eval_%s.csv" % time.strftime("%Y%m%d_%H%M%S")))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print("[eval] 指标已写入 %s" % out)


if __name__ == "__main__":
    main()
