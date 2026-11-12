#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景批量实验：单场景闭环 + 工况扫描 + CSV 导出。

三个场景覆盖 AEB 最该被回答的问题：

1. ``baseline``  —— 5 m/s 接近静止前车，AEB 开启：应当**停住且不撞**；
2. ``no_aeb``    —— 同样工况但增益为 0（相当于关掉 AEB）：**必然追尾**，
   这个场景是"刹车到底有没有用"的对照组，没有它就无法说明 AEB 的价值；
3. ``high_speed``—— 12 m/s：用来暴露系统边界，撞不撞都如实记录。

工况扫描 ``sweep``：初速度 × 增益，输出每格的最小间距/是否碰撞，
用来回答"这套参数能覆盖到多快的接近速度"。
"""
from __future__ import annotations

import csv
import os
from dataclasses import dataclass, field

import numpy as np

from aeb_controller import AEBController
from aeb_env import DEFAULT_MAX_FORCE, AEBEnv
from aeb_metrics import summarize


@dataclass
class AEBConfig:
    """一次仿真的全部参数（集中在这里，CLI 只是覆盖）。"""

    ttc_threshold: float = 5.0      # TTC 阈值 (s)，低于它开始制动
    gain: float = 1.4               # 比例增益
    v_stop: float = 0.1             # 低速保护阈值 (m/s)
    max_force: float = DEFAULT_MAX_FORCE  # 制动力上限 (N)
    ego_v: float = 5.0              # 自车初速度 (m/s)
    lead_v: float = 0.0             # 前车初速度 (m/s)
    lead_x: float = 35.0            # 前车初始位置 (m)
    sim_time: float = 15.0          # 仿真时长 (s)
    control_period: float = 0.01    # 控制周期 (s)：控制不必每步都算，省时间
    latch: bool = True              # 制动保持：触发后制动力不低于 latch_floor
    latch_floor: float = 0.35       # 保持下限（0~1）
    release_ratio: float = 1.5      # TTC 恢复到 阈值×该倍数 才解除保持
    extra: dict = field(default_factory=dict)


class Track:
    """一条仿真轨迹的记录。"""

    def __init__(self):
        self.t, self.x, self.v, self.gap, self.ttc, self.brake, self.force = ([] for _ in range(7))

    def push(self, env, ttc, brake, force):
        self.t.append(env.time)
        self.x.append(env.ego_x)
        self.v.append(env.ego_v)
        self.gap.append(env.gap)
        self.ttc.append(ttc)
        self.brake.append(brake)
        self.force.append(force)

    def arrays(self):
        return {k: np.asarray(getattr(self, k), dtype=np.float64)
                for k in ("t", "x", "v", "gap", "ttc", "brake", "force")}

    def metrics(self, cfg: AEBConfig):
        a = self.arrays()
        return summarize(a["t"], a["gap"], a["v"], a["ttc"], a["brake"],
                         x=a["x"], v_stop=cfg.v_stop)


def run_scenario(cfg: AEBConfig, name="baseline", xml_path=None):
    """跑一次闭环仿真，返回 (Track, metrics)。"""
    env = AEBEnv(xml_path=xml_path)
    env.reset(ego_x=0.0, ego_v=cfg.ego_v, lead_x=cfg.lead_x, lead_v=cfg.lead_v)
    ctrl = AEBController(ttc_threshold=cfg.ttc_threshold, gain=cfg.gain,
                         v_stop=cfg.v_stop, latch=cfg.latch,
                         latch_floor=cfg.latch_floor,
                         release_ratio=cfg.release_ratio)

    track = Track()
    dt = env.timestep
    n_steps = max(1, int(round(cfg.sim_time / dt)))
    # 控制周期：每控制周期算一次制动量，中间各步保持（零阶保持器）
    ctrl_every = max(1, int(round(cfg.control_period / dt)))
    brake = 0.0
    ttc = AEBController.INF_TTC
    force = 0.0

    for k in range(n_steps):
        if k % ctrl_every == 0:
            ttc = ctrl.compute_ttc(env.gap, env.ego_v, env.lead_v, cfg.v_stop)
            brake = ctrl.get_brake(ttc, env.ego_v, env.closing_speed)
            force = env.apply_brake(brake, cfg.max_force)
        track.push(env, ttc, brake, force)
        env.step()
        # 已经停住：驻车保持（本模型无静摩擦，否则会以低速一直滑）
        if abs(env.ego_v) < cfg.v_stop and ctrl.triggered:
            env.hold_still()
            track.push(env, ttc, 0.0, 0.0)
            break

    return track, track.metrics(cfg)


def sweep(cfg: AEBConfig, speeds=(3.0, 5.0, 8.0, 12.0, 16.0),
          gains=(0.0, 0.7, 1.4, 2.1), xml_path=None):
    """初速度 × 增益扫描，返回每格的 (min_gap, collided)。"""
    rows = []
    for v in speeds:
        for g in gains:
            c = AEBConfig(**{**cfg.__dict__, "ego_v": float(v), "gain": float(g)})
            _track, m = run_scenario(c, xml_path=xml_path)
            rows.append({"ego_v": float(v), "gain": float(g),
                         "min_gap": m["min_gap"], "collided": m["collided"],
                         "stop_distance": m["stop_distance"],
                         "brake_onset": m["brake_onset"],
                         "ttc_min": m["ttc_min"]})
    return rows


# ----------------------------------------------------------------------------
# 导出
# ----------------------------------------------------------------------------
def _fmt(v):
    if isinstance(v, float):
        return "" if not np.isfinite(v) else f"{v:.6f}"
    return v


def write_track_csv(track, save_dir, name):
    path = os.path.join(save_dir, f"aeb_{name}_series.csv")
    a = track.arrays()
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "ego_x", "ego_v", "gap", "ttc", "brake", "force"])
        for i in range(len(a["t"])):
            w.writerow([f"{a[k][i]:.6f}" for k in
                        ("t", "x", "v", "gap", "ttc", "brake", "force")])
    return path


def write_metrics_csv(results, sweep_rows, save_dir):
    path = os.path.join(save_dir, "aeb_metrics.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "metric", "value"])
        for name, m in results.items():
            for k, v in m.items():
                w.writerow([name, k, _fmt(v)])
        for row in sweep_rows:
            for k, v in row.items():
                if k in ("ego_v", "gain"):
                    continue
                w.writerow([f"sweep@v{row['ego_v']:g}_g{row['gain']:g}", k, _fmt(v)])
    return path
