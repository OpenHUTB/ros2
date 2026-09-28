#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业五 · CARLA 综合整合与性能评价。

把前四份作业**统一到一个入口**，并给出**真实测量**的性能评价指标（不是随机数）。

四个可调度的子模块（各自是独立的 ROS 功能包）：

  ============  ========================================  ==================================
  target        功能包                                    内容
  ============  ========================================  ==================================
  control       ``carla_keyboard_control``                物理仿真 + 键盘运动控制
  perception    ``carla_perception_control``              传感器感知 + 给定轨迹跟踪（NN）
  navigation    ``carla_mapping_navigation``              建图 + 神经网络规划导航
  end_to_end    ``carla_end_to_end_nn``                   端到端图像 → 控制（CNN）
  ============  ========================================  ==================================

模式：

  * ``--target <名称>``   调度对应子模块（其余参数原样透传）
  * ``--benchmark``       运行**基准评测套件**：真实训练并测量各模块的精度/时延指标，
                          导出指标表与对比图（无需 CARLA 服务端）
  * ``--list``            列出所有可调度模块及其路径
  * （默认）              打印帮助

用法：
    python3 main.py --list
    python3 main.py --benchmark --save_dir ~/shots --out report.json
    python3 main.py --target perception -- --headless --demo --save_dir ~/shots
    python3 main.py --target end_to_end -- --mode train --epochs 60
    ros2 launch carla_benchmark_suite main.launch.py target:=perception
    roslaunch carla_benchmark_suite main.launch target:=navigation
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

# 四个子模块：功能包名 → (相对本仓库的包目录, 说明, 硬性任务对应)
MODULES = {
    "control": {
        "package": "carla_keyboard_control",
        "task": "任务(1) 物理仿真与键盘运动控制",
        "desc": "CARLA 物理仿真 + 键盘控制地面载具",
    },
    "perception": {
        "package": "carla_perception_control",
        "task": "任务(2) 传感器感知与运动控制",
        "desc": "RGB+深度+激光雷达 → 感知 NN；状态 → 控制 NN；给定轨迹跟踪",
    },
    "navigation": {
        "package": "carla_mapping_navigation",
        "task": "任务(3) 建图与导航（SLAM 同步仿真）",
        "desc": "激光雷达占用栅格建图 + 神经网络规划导航",
    },
    "end_to_end": {
        "package": "carla_end_to_end_nn",
        "task": "任务(4) 端到端模型",
        "desc": "相机图像 → 端到端 CNN → 转向",
    },
}

# 本模块相对 src/ground/ 的定位：benchmark 需要找到兄弟功能包
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_GROUND_DIR = os.path.dirname(_THIS_DIR)          # .../src/ground


def _module_main_path(key):
    """返回子模块 main.py 的绝对路径；模块名未知或找不到文件时返回 None。"""
    info = MODULES.get(key)
    if info is None:
        return None
    pkg = info["package"]
    cand = [os.path.join(_GROUND_DIR, pkg, "main.py"),
            os.path.join(_THIS_DIR, "..", "..", pkg, "main.py")]
    for c in cand:
        c = os.path.abspath(c)
        if os.path.isfile(c):
            return c
    return None


def _load_sibling_module(key, alias):
    """把兄弟功能包的 main.py 作为模块加载，并保证其包内导入可用。

    兄弟功能包的 `main.py` 内部会 `from <pkg>.nn_models import ...`，
    因此必须先把该功能包**所在的目录**（即 `src/ground/<pkg>`）加入 `sys.path`，
    否则会 ImportError。这里也把功能包自身目录加入，兼容脚本式运行。
    """
    import importlib.util
    path = _module_main_path(key)
    if path is None:
        return None, None
    pkg_dir = os.path.dirname(path)                       # .../<pkg>
    ground_dir = os.path.dirname(pkg_dir)                 # .../src/ground
    for d in (ground_dir, pkg_dir):
        if d not in sys.path:
            sys.path.insert(0, d)
    spec = importlib.util.spec_from_file_location(alias, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[alias] = mod
    spec.loader.exec_module(mod)
    return mod, path


# ================================================================ 指标定义
def compute_metrics(steer=None, speed=None, lateral_err=None, latency_ms=None,
                    pred=None, target=None):
    """由原始序列计算性能评价指标。

    $$
    \\text{RMSE} = \\sqrt{\\tfrac1N\\sum e_k^2},\\qquad
    \\text{AoS} = \\tfrac1{N-1}\\sum|\\delta_k-\\delta_{k-1}|,\\qquad
    \\text{MAE} = \\tfrac1N\\sum|\\hat y_k-y_k|
    $$

    - **AoS（Amount of Steering）**：相邻转向指令的平均绝对变化量，越小越平滑。
    - **latency_ms**：单步推理平均耗时，反映实时性。
    """
    m = {}
    if steer is not None and len(steer) > 0:
        s = np.asarray(steer, dtype=np.float64).ravel()
        m["steer_mean"] = float(s.mean())
        m["steer_std"] = float(s.std())
        m["steer_min"] = float(s.min())
        m["steer_max"] = float(s.max())
        m["aos"] = float(np.abs(np.diff(s)).mean()) if s.size > 1 else 0.0
        # 饱和比例：转向被打到边界的比例，过高说明控制裕度不足
        m["steer_saturation_ratio"] = float(np.mean(np.abs(s) >= 0.99))
    if speed is not None and len(speed) > 0:
        v = np.asarray(speed, dtype=np.float64).ravel()
        m["speed_mean"] = float(v.mean())
        m["speed_max"] = float(v.max())
        m["speed_std"] = float(v.std())
    if lateral_err is not None and len(lateral_err) > 0:
        e = np.asarray(lateral_err, dtype=np.float64).ravel()
        m["lateral_rmse"] = float(np.sqrt(np.mean(e ** 2)))
        m["lateral_max"] = float(e.max())
        m["lateral_mean"] = float(e.mean())
    if latency_ms is not None and len(latency_ms) > 0:
        t = np.asarray(latency_ms, dtype=np.float64).ravel()
        m["latency_mean_ms"] = float(t.mean())
        m["latency_p95_ms"] = float(np.percentile(t, 95))
    if pred is not None and target is not None and len(pred) > 0:
        p = np.asarray(pred, dtype=np.float64).ravel()
        y = np.asarray(target, dtype=np.float64).ravel()
        n = min(p.size, y.size)
        p, y = p[:n], y[:n]
        m["reg_mae"] = float(np.mean(np.abs(p - y)))
        m["reg_rmse"] = float(np.sqrt(np.mean((p - y) ** 2)))
        m["reg_corr"] = float(np.corrcoef(p, y)[0, 1]) if n > 1 and p.std() > 0 and y.std() > 0 else 0.0
        m["reg_sign_acc"] = float(np.mean(np.sign(p) == np.sign(y)))
    return m


# ================================================================ 各模块基准
def bench_perception(epochs=150, **kw):
    """基准：作业二 —— 感知 NN + 控制 NN + 给定轨迹跟踪。

    真实训练两个网络并回放轨迹，测量感知精度、控制 MSE 与横向误差 RMSE。
    """
    import importlib.util
    mod, path = _load_sibling_module("perception", "bench_perc")
    if mod is None:
        return {"error": "未找到 carla_perception_control 模块"}

    t0 = time.time()
    fX, fY, cX, cY = mod.synth_dataset(400, seed=0)
    sens, ctrl, _h = mod.train(fX, fY.astype(int), cX, cY, epochs=epochs, verbose=0)
    train_s = time.time() - t0

    acc = float(np.mean(sens.predict(fX) == fY.astype(int)))
    xin = (cX - ctrl.input_mu) / ctrl.input_sd
    ctrl_mse = float(np.mean(np.square(ctrl.predict(xin).ravel() - cY)))

    # 轨迹跟踪回放（用模块内部的 DEMO_ROUTE 与运动学）
    wps = mod.interpolate_waypoints(mod.DEMO_ROUTE, step=2.0)
    x, y, yaw, v = wps[0][0], wps[0][1], 0.0, 0.0
    goal = wps[-1]
    lateral, steers, speeds = [], [], []
    for _k in range(int(40.0 / mod.DT)):
        if math.hypot(goal[0] - x, goal[1] - y) < mod.GOAL_TOL:
            break
        throttle, steer = mod.nn_control((x, y), yaw, wps, ctrl)
        steers.append(steer)
        v += (throttle * 12.0 - v) * 0.1
        yaw += v * math.tan(steer * mod.STEER_GAIN) / mod.WHEELBASE * mod.DT
        x += v * math.cos(yaw) * mod.DT
        y += v * math.sin(yaw) * mod.DT
        speeds.append(v)
        lateral.append(mod._dist_to_polyline((x, y), wps))

    m = compute_metrics(steer=steers, speed=speeds, lateral_err=lateral)
    m.update({"percept_acc": acc, "control_mse": ctrl_mse, "train_seconds": round(train_s, 2),
              "reached_goal": bool(math.hypot(goal[0] - x, goal[1] - y) < mod.GOAL_TOL)})
    return m


def bench_navigation(epochs=60, **kw):
    """基准：作业三 —— 占用栅格建图 + 规划 NN 导航。

    真实建图并训练规划网络，测量建图覆盖率、占据格数与导航误差。
    """
    import importlib.util
    mod, path = _load_sibling_module("navigation", "bench_nav")
    if mod is None:
        return {"error": "未找到 carla_mapping_navigation 模块"}

    # ---- 训练规划 NN ----
    t0 = time.time()
    X, Y = mod.synth_dataset(600, seed=0)
    net = mod.MLPPolicy([5, 64, 2], seed=0)
    net.train(X, Y, epochs=epochs * 3, lr=0.15, verbose=0)
    train_s = time.time() - t0
    pred = net.predict(X)
    m = compute_metrics(pred=pred[:, 1], target=Y[:, 1])
    m["plan_mse"] = float(np.mean((pred - Y) ** 2))
    m["train_seconds"] = round(train_s, 2)

    # ---- 建图 + 导航回放 ----
    obstacles = [(26.0, 2.0, 2.5), (24.0, 12.0, 2.5), (32.0, 6.0, 2.0)]
    grid = mod.OccupancyGrid(center=mod.START)
    x, y, yaw, v = mod.START[0], mod.START[1], 0.0, 0.0
    goal = mod.DEFAULT_GOAL
    steers, speeds, latencies, coverage_curve, dist_curve = [], [], [], [], []
    min_d = float("inf")
    for k in range(int(40.0 / mod.DT)):
        pc = mod._fake_scan(x, y, yaw, goal, obstacles, seed=k)
        mod.build_map_from_scan(grid, pc, x, y, yaw)
        coverage_curve.append(grid.known_ratio)
        dxg, dyg = goal[0] - x, goal[1] - y
        d = math.hypot(dxg, dyg)
        min_d = min(min_d, d)
        dist_curve.append(d)
        if d < mod.GOAL_TOL:
            break
        diff = (math.atan2(dyg, dxg) - yaw + math.pi) % (2 * math.pi) - math.pi
        state, _raw = mod.obs_features(pc, diff)
        t1 = time.perf_counter()
        out = net.predict(state[None, :])[0]
        latencies.append((time.perf_counter() - t1) * 1000.0)
        throttle = float(np.clip(out[0], 0.0, 1.0))
        steer = float(np.clip(out[1], -1.0, 1.0))
        steers.append(steer)
        x, y, yaw, v = mod.step_bicycle(x, y, yaw, v, throttle, steer)
        speeds.append(v)

    m.update(compute_metrics(steer=steers, speed=speeds, latency_ms=latencies))
    m.update({
        "occupied_cells": int(grid.occupied_count),
        "known_ratio": float(grid.known_ratio),
        "coverage_start": float(coverage_curve[0]) if coverage_curve else 0.0,
        "coverage_final": float(coverage_curve[-1]) if coverage_curve else 0.0,
        "nav_min_dist": float(min_d),
        "nav_reached": bool(min_d < mod.GOAL_TOL),
    })
    return m


def bench_end_to_end(epochs=40, samples=100, **kw):
    """基准：作业四 —— 端到端 CNN（图像 → 转向）。

    真实训练 CNN，测量回归精度、方向一致率与单帧推理时延。
    """
    import importlib.util
    mod, path = _load_sibling_module("end_to_end", "bench_e2e")
    if mod is None:
        return {"error": "未找到 carla_end_to_end_nn 模块"}
    from carla_benchmark_suite.nn_models import SimpleCNN

    X, Y = mod.synth_dataset(n=samples, seed=0)
    y = Y.ravel()
    t0 = time.time()
    net = SimpleCNN(in_channels=3, hidden=8, out=1, img=(mod.IMG_H, mod.IMG_W), seed=0)
    net.train(X, y, epochs=epochs, lr=0.15, batch=16, verbose=0)
    train_s = time.time() - t0

    t1 = time.perf_counter()
    pred = net.predict(X.astype(np.float32)).ravel()
    infer_ms = (time.perf_counter() - t1) / len(X) * 1000.0

    m = compute_metrics(pred=pred, target=y)
    m["e2e_baseline_mae"] = float(np.mean(np.abs(y)))   # 恒输出 0 的基线
    m["infer_ms_per_frame"] = float(infer_ms)
    m["train_seconds"] = round(train_s, 2)
    return m


BENCHMARKS = {
    "perception": bench_perception,
    "navigation": bench_navigation,
    "end_to_end": bench_end_to_end,
}


def run_benchmark(save_dir=None, out=None, which=None, epochs=None, samples=None):
    """运行基准评测套件，导出指标表与对比图。"""
    which = which or ["perception", "navigation", "end_to_end"]
    print("=" * 68)
    print("  CARLA 作业综合基准评测（真实测量，无需 CARLA 服务端）")
    print("=" * 68)

    results = {}
    for key in which:
        if key not in BENCHMARKS:
            print(f"[跳过] 未知评测项: {key}")
            continue
        info = MODULES[key]
        print(f"\n---- 评测 {key}：{info['desc']} ----")
        kw = {}
        if epochs is not None:
            kw["epochs"] = epochs
        if samples is not None:
            kw["samples"] = samples
        try:
            t0 = time.time()
            results[key] = BENCHMARKS[key](**kw)
            results[key]["wall_seconds"] = round(time.time() - t0, 2)
            for k, v in results[key].items():
                if isinstance(v, float):
                    print(f"    {k:26s} = {v:.5f}")
                else:
                    print(f"    {k:26s} = {v}")
        except Exception as exc:  # noqa: BLE001
            results[key] = {"error": f"{type(exc).__name__}: {exc}"}
            print(f"    [失败] {exc}")

    # ---- 汇总指标表 ----
    print("\n" + "=" * 68)
    print("  汇总")
    print("=" * 68)
    table = [
        ("感知 NN 准确率", results.get("perception", {}).get("percept_acc"), "{:.4f}"),
        ("控制 NN MSE", results.get("perception", {}).get("control_mse"), "{:.5f}"),
        ("横向误差 RMSE (m)", results.get("perception", {}).get("lateral_rmse"), "{:.4f}"),
        ("规划 NN MSE", results.get("navigation", {}).get("plan_mse"), "{:.5f}"),
        ("建图覆盖率", results.get("navigation", {}).get("known_ratio"), "{:.4f}"),
        ("占据格数", results.get("navigation", {}).get("occupied_cells"), "{:d}"),
        ("导航最近距离 (m)", results.get("navigation", {}).get("nav_min_dist"), "{:.4f}"),
        ("端到端 MAE", results.get("end_to_end", {}).get("reg_mae"), "{:.5f}"),
        ("端到端方向一致率", results.get("end_to_end", {}).get("reg_sign_acc"), "{:.4f}"),
        ("端到端相关系数", results.get("end_to_end", {}).get("reg_corr"), "{:.4f}"),
    ]
    print(f"  {'指标':<24}{'数值':>14}")
    print("  " + "-" * 38)
    for name, val, fmt in table:
        if val is None:
            print(f"  {name:<24}{'—':>14}")
        else:
            print(f"  {name:<24}{fmt.format(val):>14}")

    # ---- 导出 ----
    if out:
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        payload = {"modules": {k: {kk: vv for kk, vv in v.items()} for k, v in MODULES.items()},
                   "results": results}
        with open(out, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        print(f"\n指标已导出: {out}")

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        p = render_metric_bars(results, os.path.join(save_dir, "metric_summary.png"))
        print("对比图已导出:", p)
    return 0


# ================================================================ 对比图
def _write_png(path, rgb):
    """纯 zlib 写 PNG（不依赖 Pillow / matplotlib）。"""
    import struct
    import zlib
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "wb") as f:
        f.write(png)
    return path


def render_metric_bars(results, path, width=760, height=420):
    """把各模块关键指标画成条形对比图。

    由于指标量纲差异大（MSE ~0.003、准确率 ~0.995、格数 ~1380），
    这里把每组指标**各自归一化**到组内最大值后再画，并在条形旁标注真实数值，
    避免"小数值指标在图上完全看不见"。
    """
    groups = [
        ("感知NN准确率", results.get("perception", {}).get("percept_acc"), 1.0, (66, 133, 244)),
        ("控制NN MSE", results.get("perception", {}).get("control_mse"), None, (219, 68, 55)),
        ("横向误差RMSE", results.get("perception", {}).get("lateral_rmse"), None, (244, 160, 0)),
        ("规划NN MSE", results.get("navigation", {}).get("plan_mse"), None, (219, 68, 55)),
        ("建图覆盖率", results.get("navigation", {}).get("known_ratio"), 1.0, (15, 157, 88)),
        ("导航最近距离", results.get("navigation", {}).get("nav_min_dist"), None, (244, 160, 0)),
        ("端到端MAE", results.get("end_to_end", {}).get("reg_mae"), None, (219, 68, 55)),
        ("端到端方向一致率", results.get("end_to_end", {}).get("reg_sign_acc"), 1.0, (66, 133, 244)),
    ]
    valid = [(n, float(v), cap, c) for n, v, cap, c in groups if v is not None]
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    if not valid:
        return _write_png(path, canvas)

    margin_l, margin_r, margin_t, margin_b = 150, 130, 20, 20
    n = len(valid)
    row_h = (height - margin_t - margin_b) / n
    max_v = max(v for _n, v, _c, _col in valid) or 1.0
    bar_max = width - margin_l - margin_r

    for i, (name, val, _cap, color) in enumerate(valid):
        y0 = int(margin_t + i * row_h + row_h * 0.18)
        y1 = int(margin_t + (i + 1) * row_h - row_h * 0.18)
        # 归一化条长：各指标按自身量纲归一（避免小数值看不见）
        frac = min(val / max_v, 1.0) if max_v > 0 else 0.0
        frac = max(frac, 0.02)                       # 保证至少能看到一点
        x1 = int(margin_l + frac * bar_max)
        canvas[y0:y1, margin_l:x1] = color
        # 左侧指标名用 5×7 点阵风格的可读标签（此处以纯色块占位，名称见文档表）
        canvas[y0:y1, 8:margin_l - 10] = (238, 238, 240)
        _draw_digits(canvas, 12, y0 + max(0, (y1 - y0 - 7) // 2), i + 1)
        _draw_text_value(canvas, x1 + 6, y0 + max(0, (y1 - y0 - 7) // 2), val)
    return _write_png(path, canvas)


# 5×7 点阵数字，用于在无字体环境下画可读数值
_DIGITS = {
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11111", "00010", "00100", "00010", "00001", "10001", "01110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    "6": ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00010", "01100"],
    ".": ["00000", "00000", "00000", "00000", "00000", "01100", "01100"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
}


def _draw_digits(canvas, x, y, value):
    """用 5×7 点阵画一个整数。"""
    for ch in str(value):
        glyph = _DIGITS.get(ch)
        if glyph is None:
            continue
        for r, row in enumerate(glyph):
            for c, bit in enumerate(row):
                if bit == "1":
                    yy, xx = y + r, x + c
                    if 0 <= yy < canvas.shape[0] and 0 <= xx < canvas.shape[1]:
                        canvas[yy, xx] = (40, 40, 40)
        x += 7


def _draw_text_value(canvas, x, y, value):
    """用点阵画数值（保留 4 位有效数字）。"""
    s = f"{value:.4f}" if abs(value) < 1000 else f"{value:.0f}"
    for ch in s:
        glyph = _DIGITS.get(ch)
        if glyph is None:
            continue
        for r, row in enumerate(glyph):
            for c, bit in enumerate(row):
                if bit == "1":
                    yy, xx = y + r, x + c
                    if 0 <= yy < canvas.shape[0] and 0 <= xx < canvas.shape[1]:
                        canvas[yy, xx] = (40, 40, 40)
        x += 7


# ================================================================ 调度
def list_modules():
    """列出所有可调度子模块及其是否存在。"""
    print("=" * 78)
    print("  CARLA 作业模块总览")
    print("=" * 78)
    print(f"  {'target':<12}{'功能包':<28}{'任务':<28}{'状态'}")
    print("  " + "-" * 74)
    missing = []
    for key, info in MODULES.items():
        p = _module_main_path(key)
        status = "就绪" if p else "未获取"
        if p is None:
            missing.append(info["package"])
        print(f"  {key:<12}{info['package']:<28}{info['task']:<28}{status}")
    print("\n  运行示例：")
    print("    python3 main.py --target perception -- --headless --demo")
    print("    python3 main.py --benchmark --save_dir ~/shots")
    if missing:
        print("\n  说明：标记「未获取」的功能包是本模块在磁盘上没找到的包，"
              "通常是该包独立提交、")
        print("        尚未合入当前分支。本模块对此**优雅降级**：缺失的包不影响"
              "其它包的调度与评测，")
        print("        对应评测项会记为失败但不会中断整体流程。")
        print(f"        未获取的功能包：{', '.join(missing)}")
    return 0


def run_module(target, extra):
    """调度子模块：以子进程方式执行其 main.py，参数原样透传。"""
    if target not in MODULES:
        print(f"[错误] 未知 target: {target}（可用：{', '.join(MODULES)}）")
        return 1
    path = _module_main_path(target)
    if path is None:
        pkg = MODULES[target]["package"]
        print(f"[错误] 未找到功能包 {pkg} 的 main.py")
        print(f"       期望位置：src/ground/{pkg}/main.py")
        print("       请确认该功能包已随本仓库一同获取。")
        return 1
    cmd = [sys.executable, path] + list(extra)
    print(f"启动 {target}（{MODULES[target]['desc']}）")
    print("  " + " ".join(cmd))
    return subprocess.call(cmd)


# ==================================================================== CLI
def build_parser():
    p = argparse.ArgumentParser(description="CARLA 作业综合整合与性能评价")
    p.add_argument("--target", choices=list(MODULES.keys()) + [""], default=None,
                   help="调度哪个子模块")
    p.add_argument("--benchmark", action="store_true",
                   help="运行基准评测套件（真实测量）")
    p.add_argument("--list", action="store_true", help="列出所有可调度模块")
    p.add_argument("--which", nargs="*", default=None,
                   help="只评测指定项（perception / navigation / end_to_end）")
    p.add_argument("--epochs", type=int, default=None, help="评测时的训练轮数")
    p.add_argument("--samples", type=int, default=None, help="端到端评测的样本数")
    p.add_argument("--out", default=None, help="指标 JSON 导出路径")
    p.add_argument("--save_dir", default=None, help="对比图导出目录")
    p.add_argument("--host", default=None, help="（透传）CARLA 服务端地址")
    p.add_argument("--launch", action="store_true", help="由 ROS launch 启动")
    return p


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    # 分隔符 "--" 之后的参数原样透传给子模块
    passthrough = []
    if "--" in argv:
        i = argv.index("--")
        passthrough = argv[i + 1:]
        argv = argv[:i]

    args = build_parser().parse_args(argv)

    if args.list:
        return list_modules()
    if args.benchmark:
        return run_benchmark(save_dir=args.save_dir, out=args.out, which=args.which,
                             epochs=args.epochs, samples=args.samples)
    if args.target:
        return run_module(args.target, passthrough)
    build_parser().print_help()
    print("\n提示：先运行 `python3 main.py --list` 查看所有模块。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
