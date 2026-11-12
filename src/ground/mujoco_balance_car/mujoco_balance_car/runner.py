#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""批量场景实验：把"能跑"变成"能测"。

三个场景 + 一次参数扫描，覆盖控制品质的不同侧面：

1. `balance`      初始倾角恢复：只考验内环，看多快能把摆杆扶正。
2. `position`     位置阶跃跟踪：考验外环，看能不能"边保持平衡边走到 1 m"。
3. `disturbance`  中途被推一把：考验抗扰，看偏离后多久回到原位。
4. `lean_sweep`   初始倾角扫描：看可恢复倾角的边界，给出成功率与稳定时间曲线。

每个场景都导出时间序列 CSV，指标可由序列独立复算；`plotting.py` 在装了
matplotlib 时额外导出 PNG，没装则跳过（不影响主流程，也不影响退出码）。
"""

import csv
import os

import numpy as np

from mujoco_balance_car.config import BalanceConfig, ScenarioConfig
from mujoco_balance_car.controller import BalanceController
from mujoco_balance_car.env import BalanceEnv
from mujoco_balance_car import metrics


class ScenarioResult:
    """一个场景的完整记录：时间序列 + 指标。"""

    def __init__(self, name, t, angle, x, target_lean, torque, extra=None):
        self.name = name
        self.t = np.asarray(t, dtype=float)
        self.angle = np.asarray(angle, dtype=float)
        self.x = np.asarray(x, dtype=float)
        self.target_lean = np.asarray(target_lean, dtype=float)
        self.torque = np.asarray(torque, dtype=float)
        self.extra = extra or {}
        self.metrics = {}

    def to_rows(self):
        """导出用：一行一个采样点。"""
        return zip(self.t, self.angle, self.x, self.target_lean, self.torque)


def _simulate(env, ctrl, duration, step_time=None, step_target=None,
              disturb_time=None, disturb_velocity=0.0, record_every=1):
    """跑一段闭环仿真，返回时间序列。

    事件（位置阶跃、速度扰动）在**进入该时刻的那一步之前**触发，保证记录到的
    信号与文档里"第 1 s 阶跃 / 第 2 s 被推"的描述一致。
    """
    dt = env.timestep
    n = max(1, int(round(float(duration) / dt)))
    t_list, ang_list, x_list, lean_list, trq_list = [], [], [], [], []
    stepped = disturbed = False

    for k in range(n):
        now = k * dt
        if step_time is not None and not stepped and now + 1e-12 >= step_time:
            ctrl.set_target(step_target)
            stepped = True
        if disturb_time is not None and not disturbed and now + 1e-12 >= disturb_time:
            env.disturb(disturb_velocity)
            disturbed = True

        if k % record_every == 0:
            t_list.append(now)
            ang_list.append(env.angle)
            x_list.append(env.x)

        torque, target_lean = ctrl.compute(env.x, env.angle, dt)
        trq_list.append(torque)
        lean_list.append(target_lean)
        env.step(torque)

    # 补最后一个点（控制量的记录长度与状态对齐，绘图时不会错位）
    t_list.append(n * dt)
    ang_list.append(env.angle)
    x_list.append(env.x)
    return (np.asarray(t_list), np.asarray(ang_list), np.asarray(x_list),
            np.asarray(lean_list), np.asarray(trq_list))


def _fresh(cfg, xml_path=None):
    env = BalanceEnv(xml_path=xml_path, timestep=cfg.control_dt)
    ctrl = BalanceController(cfg, target_position=cfg.target_position)
    return env, ctrl


def run_balance(cfg=None, xml_path=None, duration=None):
    """场景 1：给定初始倾角，只做平衡（位置目标 0）。"""
    cfg = cfg or BalanceConfig()
    env, ctrl = _fresh(cfg, xml_path)
    env.reset(lean=cfg.initial_lean, x=0.0)
    t, ang, x, lean, trq = _simulate(env, ctrl, duration or cfg.duration)
    res = ScenarioResult("balance", t, ang, x, lean, trq,
                         {"initial_lean": cfg.initial_lean})
    res.metrics["angle_settle"] = metrics.settle_time(t, ang, 0.0, cfg.angle_tol)
    res.metrics["angle_max_abs"] = metrics.max_abs_deviation(ang, 0.0)
    res.metrics["angle_steady_err"] = metrics.steady_state_error(t, ang, 0.0, cfg.steady_frac)
    res.metrics["pos_steady_err"] = metrics.steady_state_error(t, x, 0.0, cfg.steady_frac)
    res.metrics["final_angle"] = float(ang[-1])
    res.metrics["upright"] = bool(abs(float(ang[-1])) < 0.15)
    return res


def run_position_step(cfg=None, sc=None, xml_path=None):
    """场景 2：第 1 s 把位置目标阶跃到 `step_position`，看能否边平衡边到位。"""
    cfg = cfg or BalanceConfig()
    sc = sc or ScenarioConfig()
    env, ctrl = _fresh(cfg, xml_path)
    ctrl.set_target(0.0)
    env.reset(lean=cfg.initial_lean, x=0.0)
    t, ang, x, lean, trq = _simulate(
        env, ctrl, max(cfg.duration, sc.settle_window),
        step_time=sc.step_time, step_target=sc.step_position)
    res = ScenarioResult("position_step", t, ang, x, lean, trq,
                         {"target": sc.step_position, "step_time": sc.step_time})
    settle_abs = metrics.settle_time(t, x, sc.step_position, cfg.pos_tol)
    # 稳定时间一律换算成"相对阶跃时刻"，否则读者会把 1 s 的等待时间算进去
    res.metrics["pos_settle"] = (settle_abs - sc.step_time
                                 if np.isfinite(settle_abs) else float("nan"))
    res.metrics["pos_overshoot_pct"] = metrics.peak_overshoot(
        x[len(x) // 2:], sc.step_position, start_value=0.0)
    res.metrics["pos_steady_err"] = metrics.steady_state_error(t, x, sc.step_position, cfg.steady_frac)
    res.metrics["final_pos"] = float(x[-1])
    res.metrics["angle_max_abs"] = metrics.max_abs_deviation(ang, 0.0)
    res.metrics["reached"] = bool(abs(float(x[-1]) - sc.step_position) < max(cfg.pos_tol,
                                                                            0.05 * sc.step_position))
    return res


def run_disturbance(cfg=None, sc=None, xml_path=None):
    """场景 3：第 2 s 给底盘一个速度脉冲，看偏离与恢复时间。"""
    cfg = cfg or BalanceConfig()
    sc = sc or ScenarioConfig()
    env, ctrl = _fresh(cfg, xml_path)
    env.reset(lean=cfg.initial_lean, x=0.0)
    t, ang, x, lean, trq = _simulate(
        env, ctrl, max(cfg.duration, sc.settle_window),
        disturb_time=sc.disturbance_time, disturb_velocity=sc.disturbance_velocity)
    res = ScenarioResult("disturbance", t, ang, x, lean, trq,
                         {"disturb_time": sc.disturbance_time,
                          "disturb_velocity": sc.disturbance_velocity})
    res.metrics["pos_max_dev"] = metrics.max_abs_deviation(x, 0.0)
    rec = metrics.settle_time(t, x, 0.0, cfg.pos_tol)
    res.metrics["pos_recover_time"] = (rec - sc.disturbance_time
                                       if np.isfinite(rec) else float("nan"))
    a_set = metrics.settle_time(t, ang, 0.0, cfg.angle_tol)
    res.metrics["angle_recover_time"] = (a_set - sc.disturbance_time
                                         if np.isfinite(a_set) else float("nan"))
    res.metrics["angle_max_abs"] = metrics.max_abs_deviation(ang, 0.0)
    res.metrics["final_pos"] = float(x[-1])
    res.metrics["recovered"] = bool(abs(float(x[-1])) < 0.05 and abs(float(ang[-1])) < 0.15)
    return res


def run_lean_sweep(cfg=None, sc=None, xml_path=None):
    """场景 4：扫描初始倾角，统计成功率与稳定时间（可恢复倾角的边界）。"""
    cfg = cfg or BalanceConfig()
    sc = sc or ScenarioConfig()
    rows = []
    for lean in sc.lean_sweep:
        env, ctrl = _fresh(cfg, xml_path)
        env.reset(lean=float(lean), x=0.0)
        t, ang, x, lean_cmd, trq = _simulate(env, ctrl, cfg.duration)
        settle = metrics.settle_time(t, ang, 0.0, cfg.angle_tol)
        final_angle = float(ang[-1])
        rows.append({
            "initial_lean": float(lean),
            "angle_settle": settle,
            "angle_max_abs": metrics.max_abs_deviation(ang, 0.0),
            "final_angle": final_angle,
            "success": bool(np.isfinite(settle) and abs(final_angle) < cfg.angle_tol * 2.0),
            "t": t, "angle": ang, "x": x,
        })
    return rows


# ----------------------------------------------------------------------------
# 导出
# ----------------------------------------------------------------------------
def _fmt(value):
    if isinstance(value, float):
        return "" if not np.isfinite(value) else f"{value:.6f}"
    return value


def write_series_csv(result, save_dir):
    path = os.path.join(save_dir, f"{result.name}_series.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "angle_rad", "x_m", "target_lean_rad", "torque_nm"])
        for row in result.to_rows():
            w.writerow([f"{v:.6f}" for v in row])
    return path


def write_metrics_csv(results, sweep_rows, save_dir):
    """指标汇总：一行一个场景（扫描按各自倾角一行）。"""
    path = os.path.join(save_dir, "balance_metrics.csv")
    keys = ["scenario", "metric", "value"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for res in results:
            for k, v in res.metrics.items():
                w.writerow([res.name, k, _fmt(v)])
        for row in sweep_rows:
            for k, v in row.items():
                if k in ("t", "angle", "x"):
                    continue
                w.writerow([f"lean_sweep@{row['initial_lean']:.3f}", k, _fmt(v)])
    return path


def run_all(save_dir=None, cfg=None, sc=None, xml_path=None, verbose=True):
    """跑完全部场景，导出 CSV（以及可选的 PNG），返回汇总字典。"""
    cfg = cfg or BalanceConfig()
    sc = sc or ScenarioConfig()
    results = [run_balance(cfg, xml_path=xml_path),
               run_position_step(cfg, sc, xml_path=xml_path),
               run_disturbance(cfg, sc, xml_path=xml_path)]
    sweep = run_lean_sweep(cfg, sc, xml_path=xml_path)

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        for res in results:
            write_series_csv(res, save_dir)
        csv_path = write_metrics_csv(results, sweep, save_dir)
    else:
        csv_path = None

    summary = {"results": results, "sweep": sweep, "csv": csv_path,
               "config": cfg, "scenario": sc}

    if save_dir:
        from mujoco_balance_car import plotting
        pngs = plotting.export_all(results, sweep, cfg, save_dir)
        summary["pngs"] = pngs

    if verbose:
        print_table(summary)
    return summary


def print_table(summary):
    """把指标打成终端表格（评审/复现时直接贴日志即可）。"""
    print("\n=== 场景指标 ===")
    header = f"{'scenario':16s} {'metric':22s} {'value':>12s}"
    print(header)
    print("-" * len(header))
    for res in summary["results"]:
        for k, v in res.metrics.items():
            if isinstance(v, float):
                shown = "nan" if not np.isfinite(v) else f"{v:.5f}"
            else:
                shown = str(v)
            print(f"{res.name:16s} {k:22s} {shown:>12s}")

    print("\n=== 初始倾角扫描 ===")
    print(f"{'lean(rad)':>10s} {'lean(deg)':>10s} {'settle(s)':>11s} {'max|ang|':>10s} {'success':>8s}")
    for row in summary["sweep"]:
        st = row["angle_settle"]
        st_txt = "nan" if not np.isfinite(st) else f"{st:.3f}"
        print(f"{row['initial_lean']:10.3f} {np.degrees(row['initial_lean']):10.2f} "
              f"{st_txt:>11s} {row['angle_max_abs']:10.4f} {str(row['success']):>8s}")
    ok = sum(1 for r in summary["sweep"] if r["success"])
    print(f"\n可恢复倾角：{ok}/{len(summary['sweep'])} 档成功"
          f"（最大成功倾角 "
          f"{max([r['initial_lean'] for r in summary['sweep'] if r['success']], default=0):.3f} rad）")
    if summary.get("csv"):
        print(f"指标 CSV：{summary['csv']}")
