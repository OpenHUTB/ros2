#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景批量实验：同一场景分别用 baseline / escape 两种控制器跑，输出对照指标。

三个场景（对应文档里的三种画面）：
  * ``chase``   空旷场地追移动目标（目标随机游走，固定 seed 可复现）
  * ``gate``    两障碍之间留一个缝，要求从左侧穿到右侧（文档里的"穿越障碍间隙"）
  * ``blocked`` 障碍**正压在**目标方向上——原实现"原地打转"的典型卡死场景
"""
from __future__ import annotations

import csv
import os

import numpy as np

from car_2d_target_tracker.car import Car
from car_2d_target_tracker.config import Scenario, TrackerConfig
from car_2d_target_tracker.controller import Tracker
from car_2d_target_tracker.geometry import nearest_obstacle
from car_2d_target_tracker.metrics import summarize
from car_2d_target_tracker.target import Target


def default_scenarios(cfg: TrackerConfig = None):
    cfg = cfg or TrackerConfig()
    w, h = cfg.width, cfg.height
    gate_obstacles = ((380.0, 210.0, 60.0), (380.0, 430.0, 60.0))   # 中间约 100 px 的缝
    return [
        Scenario("chase", car=(w / 2, h / 2, 0.0), target=(700.0, 150.0),
                 target_motion="wander", obstacles=(),
                 sim_time=30.0, note="空旷场地追移动目标"),
        Scenario("gate", car=(120.0, 300.0, 0.0), target=(820.0, 300.0),
                 target_motion="static", obstacles=gate_obstacles,
                 sim_time=30.0, note="从两障碍之间的缝穿过"),
        Scenario("blocked", car=(120.0, 300.0, 0.0), target=(800.0, 300.0),
                 target_motion="static",
                 obstacles=((460.0, 300.0, 90.0),),
                 sim_time=30.0, note="障碍正压在目标方向上（原地打转会卡死）"),
    ]


def run_scenario(cfg: TrackerConfig, scenario: Scenario, mode: str = "escape"):
    """跑一个场景，返回 (series dict, metrics dict)。"""
    car = Car(scenario.car[0], scenario.car[1], scenario.car[2], radius=cfg.car_radius)
    target = Target(scenario.target[0], scenario.target[1], speed=cfg.target_speed,
                    width=cfg.width, height=cfg.height, margin=cfg.target_margin,
                    motion=scenario.target_motion, seed=cfg.seed)
    obstacles = tuple(scenario.obstacles) if scenario.obstacles else ()
    ctrl = Tracker(cfg, mode=mode)

    t_list, x_list, y_list, v_list, w_list = [], [], [], [], []
    dist_list, clr_list = [], []
    dt = cfg.dt
    n = int(round(scenario.sim_time / dt))
    arrived_at = None

    for k in range(n):
        v, omega, arrived = ctrl.compute(car, target)
        t_list.append(k * dt)
        x_list.append(car.x)
        y_list.append(car.y)
        v_list.append(car.v if k else 0.0)
        w_list.append(car.omega if k else 0.0)
        dist_list.append(car.distance_to(target.x, target.y))
        near = nearest_obstacle(car.x, car.y, obstacles)
        clr_list.append(float("inf") if near is None else near[3] - cfg.car_radius)
        if arrived:
            arrived_at = k * dt
            break
        car.step(v, omega, dt, obstacles)
        target.step(dt)

    series = {"t": t_list, "x": x_list, "y": y_list, "v": v_list,
              "omega": w_list, "dist": dist_list, "clearance": clr_list,
              "obstacles": obstacles, "target_track": (target.x, target.y)}
    metrics = summarize(t_list, x_list, y_list, v_list, dist_list, clr_list,
                        arrive_radius=cfg.arrive_radius, speed_eps=cfg.stuck_speed_eps)
    return series, metrics


def compare(cfg: TrackerConfig = None, scenarios=None, verbose=True):
    """对所有场景跑 baseline 与 escape，返回 {scenario: {mode: (series, metrics)}}。"""
    cfg = cfg or TrackerConfig()
    scenarios = scenarios or default_scenarios(cfg)
    out = {}
    for sc in scenarios:
        out[sc.name] = {}
        for mode in ("baseline", "escape"):
            out[sc.name][mode] = run_scenario(cfg, sc, mode=mode)
        if verbose:
            b = out[sc.name]["baseline"][1]
            e = out[sc.name]["escape"][1]
            print(f"\n[{sc.name}] {sc.note}")
            for tag, m in (("baseline", b), ("escape", e)):
                print("  %-8s reached=%-5s arrive=%6.2fs path=%7.1fpx "
                      "min_clear=%7.1fpx stuck=%5.1f%% final_dist=%6.1fpx"
                      % (tag, m["reached"],
                         m["arrive_time"] if m["reached"] else float("nan"),
                         m["path_length"], m["min_clearance"],
                         m["stuck_fraction"] * 100.0, m["final_dist"]))
    return out


# ----------------------------------------------------------------------------
# 导出
# ----------------------------------------------------------------------------
def write_series_csv(series, path):
    keys = ("t", "x", "y", "v", "omega", "dist", "clearance")
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for i in range(len(series["t"])):
            w.writerow([f"{series[k][i]:.6f}" for k in keys])
    return path


def write_metrics_csv(results, path):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "mode", "metric", "value"])
        for name, per_mode in results.items():
            for mode, (_series, m) in per_mode.items():
                for k, v in m.items():
                    w.writerow([name, mode, k,
                                "" if isinstance(v, float) and not np.isfinite(v) else v])
    return path


def export_all(results, save_dir, cfg=None):
    """导出 CSV（以及可选的 PNG）。返回生成的文件列表。"""
    cfg = cfg or TrackerConfig()
    os.makedirs(save_dir, exist_ok=True)
    out = []
    for name, per_mode in results.items():
        for mode, (series, _m) in per_mode.items():
            out.append(write_series_csv(series, os.path.join(save_dir, f"{name}_{mode}.csv")))
    out.append(write_metrics_csv(results, os.path.join(save_dir, "tracker_metrics.csv")))
    try:
        from car_2d_target_tracker import plotting
        out.extend(plotting.export_all(results, save_dir, cfg))
    except Exception as exc:  # noqa: BLE001  —— 绘图是可选功能，失败不影响主流程
        print(f"[提示] 跳过 PNG 绘图：{exc}")
    return out
