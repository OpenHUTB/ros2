# -*- coding: utf-8 -*-
"""
record_demo.py —— 加载训练好的 CNN，端到端推理演示并录制素材与性能指标。

流程：跟随视角相机图像 → E2ECNN → (v, w) → 仿真步进；采集第三方视角 GIF、
关键帧 PNG，并记录站立率、碰撞/跌倒次数、指令分布、推理帧率、轨迹等指标。

运行（包根目录）：
    python3 scripts/record_demo.py
    python3 scripts/record_demo.py --duration 25 --no-viewer
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import torch

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PKG_ROOT)

from humanoid_camera_e2e_control.sim_core import HumanoidSim, open_viewer, calibrate_command
from humanoid_camera_e2e_control.network import E2ECNN
from humanoid_camera_e2e_control.data_io import preprocess, direction_label


def run(weights, out_dir, duration, max_reset=0, viewer=True, seed=42):
    np.random.seed(seed)
    torch.manual_seed(seed)
    sim = HumanoidSim()
    net = E2ECNN.load(weights)
    net.eval()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    net.to(device)

    view = open_viewer(sim) if viewer else None
    os.makedirs(out_dir, exist_ok=True)

    # 控制周期：每 0.05 s 推理一次（25 个物理步长 @ dt=0.002）
    control_period = 0.05
    steps_per_ctrl = int(control_period / sim.dt)
    total_ctrl = int(duration / control_period)

    frames, snapshots = [], []
    traj = []
    standing_steps = contact_count = fall_count = reset_count = 0
    contact_now = False
    infer_times = []
    cmd_dist = {"forward": 0, "left": 0, "right": 0, "stop": 0}
    x_max = 0.0
    reach_target = False

    # 关键帧目标位置（起点、1/3、2/3、终点）；提前结束时用最后一帧补齐
    snap_marks = {0, total_ctrl // 3, 2 * total_ctrl // 3, total_ctrl - 1}
    snap_taken = set()

    sim.reset(0.0, 0.0, 0.0)
    fall_time = None
    t0 = time.time()

    for ci in range(total_ctrl):
        n = len(traj)
        # ---- 感知 + 推理 ----
        img = sim.render_front()
        x = preprocess(img).unsqueeze(0).to(device)
        ti = time.time()
        with torch.no_grad():
            pred = net(x)[0]
        v, w = calibrate_command(float(pred[0]), float(pred[1]))
        infer_times.append(time.time() - ti)

        # 指令分布统计
        d = direction_label(v, w)
        if d == 0:
            cmd_dist["stop"] += 1
        elif d == 1:
            cmd_dist["left"] += 1
        elif d == 2:
            cmd_dist["right"] += 1
        if v > 0.03 and abs(w) < 0.05:
            cmd_dist["forward"] += 1

        # ---- 物理步进 ----
        for _ in range(steps_per_ctrl):
            sim.step(v_lin=v, w_yaw=w)
            if sim.standing():
                standing_steps += 1
            else:
                fall_count += 1
            if sim.contact_obstacle():
                if not contact_now:
                    contact_count += 1
                    contact_now = True
            else:
                contact_now = False

        od = sim.read_odom()
        traj.append((od["x"], od["y"], od["yaw"]))
        x_max = max(x_max, od["x"])
        if od["x"] > 4.0:
            reach_target = True

        # ---- 采集演示帧（第三方视角，约 6~10 FPS）----
        if n % 3 == 0:
            frames.append(sim.render_overview())
        # 关键帧：到达目标步数位置即抓取
        for mark in snap_marks:
            if mark not in snap_taken and n >= mark:
                snapshots.append(sim.render_overview())
                snap_taken.add(mark)
                break

        # ---- 跌倒 / 碰撞处理：默认不重置，结束第一段行程以免素材位置跳变 ----
        if (not sim.standing()) or contact_now:
            if reset_count < max_reset:
                reset_count += 1
                sim.reset(x=np.random.uniform(-0.3, 0.3),
                          y=np.random.uniform(-0.3, 0.3),
                          yaw=np.random.uniform(-0.2, 0.2))
                contact_now = False
                print(f"[演示] 触发重置第 {reset_count}/{max_reset} 次")
            else:
                print(f"[演示] 行程结束：跌倒 {fall_count} 步 / 碰撞 {contact_count} 次，"
                      f"录制第一段连续行程 {len(traj)} 个控制周期。")
                break

        if view is not None:
            view.sync()
            if not view.is_running():
                print("\n[INFO] 可视化窗口已关闭，结束演示。")
                break

    # 补齐关键帧：提前结束时把最后一帧作为终点帧
    if snapshots:
        while len(snapshots) < 4:
            snapshots.append(sim.render_overview())

    wall = time.time() - t0

    # ---------------- 保存 GIF / 关键帧 ----------------
    gif_path = os.path.join(out_dir, "humanoid_e2e_demo.gif")
    png_names = ["scene_start.png", "scene_mid1.png", "scene_mid2.png", "scene_end.png"]
    try:
        import imageio
        imageio.mimsave(gif_path, frames, fps=12)
        gif_kb = os.path.getsize(gif_path) / 1024
    except ImportError:
        gif_path, gif_kb = None, 0.0
        print("[WARN] 未安装 imageio，跳过 GIF。")

    for name, img in zip(png_names, snapshots):
        p = os.path.join(out_dir, name)
        try:
            from PIL import Image
            Image.fromarray(img).save(p)
        except ImportError:
            imageio.imwrite(p, img)

    total_steps = standing_steps + fall_count
    metrics = {
        "duration_s": round(wall, 2),
        "control_period_s": control_period,
        "control_steps": len(traj),
        "standing_ratio": round(standing_steps / max(total_steps, 1), 4),
        "contact_count": contact_count,
        "fall_count": fall_count,
        "reset_count": reset_count,
        "max_x_m": round(x_max, 3),
        "reach_target_x4": reach_target,
        "cmd_distribution": cmd_dist,
        "infer_avg_ms": round(float(np.mean(infer_times)) * 1000, 2),
        "infer_fps": round(1.0 / float(np.mean(infer_times)), 1),
        "wall_time_s": round(wall, 2),
        "gif_frames": len(frames),
        "gif_size_kb": round(gif_kb, 1),
        "trajectory": [[round(x, 3), round(y, 3)] for x, y, _ in traj],
    }
    with open(os.path.join(out_dir, "metrics_demo.json"), "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)

    # 轨迹点单独导出（供文档画轨迹图）
    with open(os.path.join(out_dir, "trajectory.csv"), "w", encoding="utf-8") as f:
        f.write("x,y,yaw\n")
        for x, y, yaw in traj:
            f.write(f"{x:.3f},{y:.3f},{yaw:.3f}\n")

    if view is not None:
        try:
            view.close()
        except Exception:                                         # noqa: BLE001
            pass

    print("\n================ 端到端演示指标 ================")
    print(f"运行时长(s)        : {metrics['duration_s']}")
    print(f"站立步数占比       : {metrics['standing_ratio']*100:.1f}%")
    print(f"碰撞次数           : {contact_count}")
    print(f"跌倒步数           : {fall_count}")
    print(f"最大前进位移(m)    : {x_max:.2f}")
    print(f"到达 x=4.0 终点    : {'是' if reach_target else '否'}")
    print(f"指令分布           : {cmd_dist}")
    print(f"单帧推理耗时       : {metrics['infer_avg_ms']} ms")
    print(f"推理 FPS           : {metrics['infer_fps']}")
    print(f"GIF 帧数 / 大小    : {len(frames)} 帧 / {gif_kb:.0f} KB")
    print(f"[保存] 素材目录    : {out_dir}")
    return metrics


def main():
    ap = argparse.ArgumentParser(description="人形端到端 CNN 推理演示与素材录制")
    here = _PKG_ROOT
    ap.add_argument("--weights", default=os.path.join(here, "weights", "humanoid_cnn.pt"))
    ap.add_argument("--out-dir", default=os.path.join(os.getcwd(), "demo_assets"))
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--max-reset", type=int, default=0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--no-viewer", dest="viewer", action="store_false")
    ap.set_defaults(viewer=True)
    args = ap.parse_args()
    run(args.weights, args.out_dir, args.duration, max_reset=args.max_reset,
        viewer=args.viewer, seed=args.seed)


if __name__ == "__main__":
    main()
