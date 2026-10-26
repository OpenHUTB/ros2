# -*- coding: utf-8 -*-
"""
teacher_collect.py —— 规则教师自动驾驶采集数据集（standalone，无键盘、无 ROS）。

教师依据"前方最近障碍物相对机器人的横向方位"打转向标签，使视觉观测与动作指令
形成稳定因果对应：
  1. 遍历场景立柱 / 散块，取机器人前方（x_rel ∈ [0.1, 1.2] m）内最近障碍；
  2. 横向偏移 dy = py − y：障碍在右侧(dy > 0.12) → 顺时针避让 w = −0.22；
     障碍在左侧(dy < −0.12) → 逆时针避让 w = +0.22；前方无近障碍 → 直行 w = 0；
  3. 线速度 v = 0.10 m/s；约 5% 帧随机停车，补充停止样本。

教师规则本身不进入最终模块，最终决策完全由训练得到的 E2ECNN 完成。
跌倒 / 碰撞 / 到达通道末端后随机重置位姿继续采集。

运行（包根目录）：
    python3 scripts/teacher_collect.py --max-frames 3000
"""

import argparse
import os
import sys
import time

import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PKG_ROOT)

from humanoid_camera_e2e_control.sim_core import HumanoidSim
from humanoid_camera_e2e_control.data_io import save_dataset, clear_dataset


def _nearest_obstacle_offset(sim):
    """返回前方最近障碍相对机器人的横向偏移 dy；前方 1.2 m 内无近障碍返回 None。

    感知相机为运行时 ``MjvCamera``（free 模式）第三人称跟随视角，画面左右与障碍
    相对机器人的横向方位直接对应，因此标签依据相对偏移 dy 生成，保证视觉观测与
    转向指令因果对齐。
    """
    od = sim.read_odom()
    x, y = od["x"], od["y"]
    best_dy, best_d = None, 1.2
    for gid in sim._obstacle_geom_ids:
        px, py = sim.data.geom_xpos[gid][0], sim.data.geom_xpos[gid][1]
        dx = px - x
        if dx < 0.1 or dx > 1.2:
            continue
        dist = (dx * dx + (py - y) ** 2) ** 0.5
        if dist < best_d:
            best_d, best_dy = dist, py - y
    return best_dy


def collect(data_dir, max_frames, sample_step=50, reset_every=400, seed=42):
    rng = np.random.default_rng(seed)
    sim = HumanoidSim()
    clear_dataset(data_dir)
    images, vels, yaws = [], [], []
    dist = {"forward": 0, "left": 0, "right": 0, "stop": 0}
    step_count = 0
    batch = 0
    t0 = time.time()
    collected = 0  # 累计已采集帧数（上限按累计计数，不随批次清空重置）

    def reset_random():
        sim.reset(x=float(rng.uniform(0, 1.2)),
                  y=float(rng.uniform(-0.3, 0.3)),
                  yaw=float(rng.uniform(-0.2, 0.2)))

    reset_random()
    print(f"[采集] 目标 {max_frames} 帧 | 采样间隔 {sample_step} 步 | 输出 {data_dir}")

    while collected < max_frames:
        od = sim.read_odom()
        dy = _nearest_obstacle_offset(sim)

        # 依据最近障碍相对方位生成三档避让指令（跟随视角下画面左右与 dy 对应；
        # 障碍在哪一侧，机器人转向另一侧避让）
        if dy is None:
            w = 0.0
        elif dy > 0.12:
            w = -0.22      # 障碍在机器人 +y 侧（画面右侧），向 -y 侧避让（右转）
        elif dy < -0.12:
            w = 0.22       # 障碍在机器人 -y 侧（画面左侧），向 +y 侧避让（左转）
        else:
            w = 0.0
        v = 0.10

        # 随机停车样本（约 5%）
        if rng.random() < 0.05:
            v, w = 0.0, 0.0

        sim.step(v_lin=v, w_yaw=w)
        step_count += 1

        if (not sim.standing()) or sim.contact_obstacle() or od["x"] > 6.2:
            reset_random()

        if step_count % sample_step == 0:
            images.append(sim.render_front())
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

            if len(images) % reset_every == 0:
                save_dataset(np.array(images), np.array(vels), np.array(yaws),
                             data_dir, batch=batch)
                images, vels, yaws = [], [], []
                batch += 1
                reset_random()
                print(f"[采集] 第 {batch} 批落盘 | 累计方向分布 {dist}")

    if images:
        save_dataset(np.array(images), np.array(vels), np.array(yaws),
                     data_dir, batch=batch)

    print(f"\n[采集] 完成 | 用时 {time.time()-t0:.1f}s | 方向分布 {dist}")
    print(f"[采集] 下一步：python3 scripts/train_e2e.py --data-dir {data_dir}")


def main():
    ap = argparse.ArgumentParser(description="规则教师自动采集图像—指令数据集")
    here = _PKG_ROOT
    ap.add_argument("--data-dir", default=os.path.join(here, "data", "training"))
    ap.add_argument("--max-frames", type=int, default=3000)
    ap.add_argument("--sample-step", type=int, default=50)
    ap.add_argument("--reset-every", type=int, default=400)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()
    collect(args.data_dir, args.max_frames, args.sample_step, args.reset_every, args.seed)


if __name__ == "__main__":
    main()
