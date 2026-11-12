#!/usr/bin/env python3
"""纯物理仿真：不渲染画面，直接出曲线图与数据（不需要 GPU / EGL）。

用法：
    python3 scripts/physics_only_demo.py [--duration 24] [--out DIR]
    python3 scripts/main.py               # 等价（另有 --headless 走无头渲染）
产物：
    offline_curves.png    三联图（轨迹叠地形等高线+速度着色 / IMU 姿态 / 四悬架行程）
    offline_log.csv       逐帧数据
    offline_metrics.json  指标（路径长度、最终距离、roll/pitch RMS 等）
输出目录：--out > 环境变量 OFFROAD_OUT_DIR > 源码树 docs/img > ~/.cache/<pkg>/output
（装到 <ws>/lib/<pkg>/ 后向上两级是 <ws>/lib，不能按脚本位置硬算，见 resolve_out_dir()）
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from offroad_sim_core import OffroadSim, pose_summary, resolve_out_dir  # noqa: E402
from plot_curves import make_curves  # noqa: E402


def main(duration=24.0, out_dir=None, target=(18.0, 2.0), start=(-2.0, -0.5),
         tilt_assist=0.0, verbose=True):
    out = resolve_out_dir(out_dir)

    sim = OffroadSim(target=target, start=start, tilt_assist_gain=tilt_assist)
    samples = sim.run(duration=duration)

    metrics = pose_summary(samples, sim.target)
    metrics["settings"] = {
        "duration_s": duration,
        "start": list(start),
        "target": list(target),
        "tilt_assist_gain": tilt_assist,
        "sample_dt": 0.02,
        "terrain": sim.terrain_info,
    }
    with open(os.path.join(out, "offline_metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)

    with open(os.path.join(out, "offline_log.csv"), "w") as f:
        f.write("t,x,y,z,roll_deg,pitch_deg,yaw_deg,v_body,sus_fl,sus_fr,sus_rl,sus_rr\n")
        for s in samples:
            f.write("%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%.4f,%.4f,%.4f,%.4f\n" % (
                s["t"], s["x"], s["y"], s["z"], np.degrees(s["roll"]),
                np.degrees(s["pitch"]), np.degrees(s["yaw"]), s["v_body"], *s["sus"]))

    make_curves(samples, sim.target, os.path.join(out, "offline_curves.png"))
    if verbose:
        print("out: %s (%d 帧)" % (out, len(samples)))
        print("path=%.2fm final_dist=%.2fm roll_rms=%.2fdeg pitch_rms=%.2fdeg" % (
            metrics["path_length_m"], metrics["final_dist_to_target_m"],
            metrics["roll_rms_deg"], metrics["pitch_rms_deg"]))
    return metrics


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=24.0)
    ap.add_argument("--out", default=None, help="输出目录（默认源码树 docs/img，装到空间后退到 ~/.cache）")
    ap.add_argument("--start", type=float, nargs=2, default=(-2.0, -0.5))
    ap.add_argument("--target", type=float, nargs=2, default=(18.0, 2.0))
    ap.add_argument("--tilt-assist", type=float, default=0.0)
    args = ap.parse_args()
    main(duration=args.duration, out_dir=args.out, target=tuple(args.target),
         start=tuple(args.start), tilt_assist=args.tilt_assist)
