# -*- coding: utf-8 -*-
"""
collect_data.py —— 键盘驾驶采集图像—指令数据集（standalone，无 ROS）。

人在环操控人形机器人在避障通道内绕行，同步记录跟随视角相机图像与当时速度指令，
供 train_e2e.py 训练端到端 CNN。

运行（包根目录）：
    python3 scripts/collect_data.py                 # 开窗，默认 2000 帧
    python3 scripts/collect_data.py --max-frames 3000
    python3 scripts/collect_data.py --no-viewer     # 无图形环境

按键：
    W/S 前进/后退   A/D 左转/右转   空格 停止   ESC 退出并落盘
"""

import argparse
import os
import sys
import time

import numpy as np

# 定位包根（脚本位于 包根/scripts/ 下）
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PKG_ROOT)

from humanoid_camera_e2e_control.sim_core import HumanoidSim, open_viewer, IMG_H, IMG_W
from humanoid_camera_e2e_control.keyboard import KeyReader
from humanoid_camera_e2e_control.data_io import save_dataset, clear_dataset, direction_label

HELP = """
============================================================
  图像—指令数据集采集（键盘驾驶）
------------------------------------------------------------
  W/S  前进/后退      A/D  左转/右转
  空格 停止           ESC  退出并落盘
  松开按键 回到低速前进（连续采集前进帧）
============================================================
"""


def collect(data_dir, max_frames, viewer=True, sample_step=50, reset_every=400):
    clear_dataset(data_dir)
    sim = HumanoidSim()
    reader = KeyReader()
    view = open_viewer(sim) if viewer else None

    images, vels, yaws = [], [], []
    step_count = 0
    epoch = 0
    collected = 0
    dist = {"forward": 0, "left": 0, "right": 0, "stop": 0}

    print(HELP)
    print(f"[采集] 目标 {max_frames} 帧 | 采样间隔 {sample_step} 步 | 输出目录 {data_dir}")
    t0 = time.time()
    try:
        while collected < max_frames:
            key = reader.get_key()
            v, w = 0.15, 0.0  # 默认低速前进，保证画面里有持续前进帧
            if key in ('w', 'W', 'up'):
                v, w = sim.V_MAX * 0.75, 0.0
            elif key in ('s', 'S', 'down'):
                v, w = -sim.V_MAX * 0.4, 0.0
            elif key in ('a', 'A', 'left'):
                v, w = 0.08, sim.W_MAX * 0.6
            elif key in ('d', 'D', 'right'):
                v, w = 0.08, -sim.W_MAX * 0.6
            elif key == ' ':
                v, w = 0.0, 0.0
            elif key == '\x1b' or key == 'q':
                break

            sim.step(v_lin=v, w_yaw=w)
            step_count += 1

            # 跌倒或碰撞后随机重置，避免单条轨迹反复摔倒
            if (not sim.standing()) or sim.contact_obstacle():
                sim.reset(x=np.random.uniform(0, 1.5),
                          y=np.random.uniform(-0.7, 0.7),
                          yaw=np.random.uniform(-0.4, 0.4))

            if step_count % sample_step == 0:
                img = sim.render_front()
                images.append(img)
                vels.append(v)
                yaws.append(w)
                collected += 1
                if v < 0.03 and abs(w) < 0.05:
                    dist["stop"] += 1
                elif w > 0.05:
                    dist["left"] += 1
                elif w < -0.05:
                    dist["right"] += 1
                else:
                    dist["forward"] += 1

                # 周期性落盘，防止异常退出丢数据
                if len(images) % reset_every == 0:
                    save_dataset(np.array(images), np.array(vels), np.array(yaws),
                                 data_dir, batch=epoch)
                    # 落盘后清空内存缓冲（历史批次已独立存盘）
                    images, vels, yaws = [], [], []
                    epoch += 1
                    sim.reset(x=np.random.uniform(0, 1.5),
                              y=np.random.uniform(-0.7, 0.7),
                              yaw=np.random.uniform(-0.4, 0.4))
                    print(f"[采集] 已落盘第 {epoch} 批，方向分布 {dist}")

            if view is not None:
                view.sync()
                if not view.is_running():
                    print("\n[INFO] 可视化窗口已关闭，结束采集。")
                    break
    finally:
        reader.close()
        if view is not None:
            try:
                view.close()
            except Exception:                                     # noqa: BLE001
                pass

    if images:
        save_dataset(np.array(images), np.array(vels), np.array(yaws),
                     data_dir, batch=epoch)

    used = time.time() - t0
    print(f"\n[采集] 完成 | 用时 {used:.1f}s | 方向分布 {dist}")
    print(f"[采集] 下一步：python3 scripts/train_e2e.py --data-dir {data_dir}")


def main():
    ap = argparse.ArgumentParser(description="人形机器人图像—指令数据集采集")
    here = _PKG_ROOT
    ap.add_argument("--data-dir", default=os.path.join(here, "data", "training"))
    ap.add_argument("--max-frames", type=int, default=2000)
    ap.add_argument("--sample-step", type=int, default=50)
    ap.add_argument("--reset-every", type=int, default=400)
    ap.add_argument("--no-viewer", dest="viewer", action="store_false")
    ap.set_defaults(viewer=True)
    args = ap.parse_args()
    collect(args.data_dir, args.max_frames, viewer=args.viewer,
            sample_step=args.sample_step, reset_every=args.reset_every)


if __name__ == "__main__":
    main()
