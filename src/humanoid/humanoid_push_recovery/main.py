#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py —— 人形机器人平衡与外力抗扰恢复（standalone 入口，无需 ROS）
=====================================================================

用法::

    python3 main.py                        # 演示：一次前向推扰 + 平衡恢复（打开可视化窗口）
    python3 main.py --force 180            # 指定扰动力 (N)
    python3 main.py --direction 90         # 扰动方向（0°=前向, 90°=侧向, 180°=后向）
    python3 main.py --check                # 模型/控制器/判据 自检
    python3 main.py --sweep                # 测定各方向临界冲量（踝 vs 踝+髋 对照）
    python3 main.py --capture assets_out   # 采集关键帧/动图/曲线/metrics.json
    python3 main.py --no-viewer            # 无图形环境：仅命令行输出
    python3 main.py --test                 # 运行自动化测试
    python3 main.py --ros2                 # 通过 ros2 launch 启动

工作流：站姿稳定 → 施加外力扰动 → 踝/髋策略恢复（或跌倒判定）→ 定量评价。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from humanoid_push_recovery.balance import (                              # noqa: E402
    BalanceController, capture_point, convex_hull, distance_to_boundary)
from humanoid_push_recovery.evaluate import (                             # noqa: E402
    compare_strategies, critical_force, run_trial, sweep_forces)
from humanoid_push_recovery.sim_core import (                             # noqa: E402
    JOINT_NAMES, ROLL_JOINTS, BalanceSim, load_params, open_viewer)

DEFAULT_CONFIG = os.path.join(HERE, "config", "params.yaml")


# ----------------------------------------------------------------------
# 模式 1：自检
# ----------------------------------------------------------------------
def run_check(params, model_path=None) -> int:
    """模型 / 控制器 / 平衡判据 可用性自检。"""
    ok = True

    def check(name, passed, detail=""):
        nonlocal ok
        ok = ok and bool(passed)
        print(f"[{'PASS' if passed else 'FAIL'}] {name} {detail}")

    print("=" * 66)
    print("  人形机器人平衡与抗扰恢复模块自检")
    print("=" * 66)
    sim = BalanceSim(model_path=model_path, params=params)
    check("模型加载", sim.model.nq > 0 and sim.model.nu == len(JOINT_NAMES),
          f"(nq={sim.model.nq}, nv={sim.model.nv}, nu={sim.model.nu}, 关节={len(JOINT_NAMES)})")
    check("roll 关节索引", len(sim.roll_idx) == len(ROLL_JOINTS),
          f"(roll 关节={list(ROLL_JOINTS)})")

    sim.controller = BalanceController(params)
    sim.reset()
    corners = sim.foot_corners()
    check("支撑多边形（脚底 8 角点）", corners.shape == (8, 3), f"({corners.shape})")
    hull = convex_hull(corners[:, :2])
    check("支撑多边形凸包", len(hull) >= 3, f"({len(hull)} 个顶点)")

    # 质心投影应落在支撑多边形内
    com = sim.com_position()
    margin = distance_to_boundary(com[:2], hull)
    check("静立时质心投影在支撑多边形内", margin > 0,
          f"(稳定裕度 {margin * 1000:.1f} mm)")

    xi = capture_point(com, sim.com_velocity(), com[2])
    check("捕获点计算", np.all(np.isfinite(xi)), f"(捕获点 {np.round(xi, 4)})")

    # 自由站立稳定性（无扰动 5 s）
    sim.reset()
    ref = sim.com_position()
    for _ in range(int(5.0 / sim.dt)):
        sim.step()
    drift = float(np.linalg.norm(sim.com_position()[:2] - ref[:2]))
    check("无扰动自由站立 5 s 不跌倒", not sim.fallen() and drift < 0.02,
          f"(质心漂移 {drift * 1000:.2f} mm, 根高 {sim.root_height():.3f} m)")

    r = run_trial(params, force=float(params.get("push_force", 120.0)), direction_deg=0.0)
    check("典型扰动可恢复", r["recovered"],
          f"({r['force']:.0f} N 推扰, 最大偏移 {r['max_offset_m'] * 1000:.0f} mm, "
          f"恢复 {r['recover_time_s']:.2f} s)" if r["recovered"]
          else f"({r['force']:.0f} N 推扰未能恢复)")

    r0 = run_trial(params, force=float(params.get("push_force", 120.0)),
                   direction_deg=0.0, balance=False)
    check("关闭平衡控制器会跌倒（对照成立）", r0["fell"],
          f"(无平衡时 {r0['force']:.0f} N 推扰即跌倒)")

    sim.close()
    print()
    print(f"结论：自检{'通过' if ok else '存在问题'}。")
    return 0 if ok else 1


# ----------------------------------------------------------------------
# 模式 2：演示
# ----------------------------------------------------------------------
def run_demo(params, force=None, direction=None, hip_enable=None, capture_dir=None,
             viewer=True, verbose=True, model_path=None):
    """施加一次扰动并记录恢复过程；可选打开可视化窗口与采集素材。"""
    p = dict(params)
    if hip_enable is not None:
        p["hip_enable"] = bool(hip_enable)
    force = float(force if force is not None else p.get("push_force", 120.0))
    # 方向未显式指定时读取 params.yaml 的 push_direction_deg（默认 0° = 前向）
    direction = float(direction if direction is not None
                      else p.get("push_direction_deg", 0.0))
    duration = float(p.get("push_duration", 0.08))

    sim = BalanceSim(model_path=model_path, params=p)
    sim.controller = BalanceController(p)
    sim.reset()
    if capture_dir is not None:
        # 采集素材时把地面参考线放到初始质心正下方（固定侧视相机下位移可见）
        sim.set_com_reference_marker()
    view = open_viewer(sim) if viewer else None
    pacer = _Pacer(sim.dt) if view is not None else None

    ref = sim.com_position()[:2].copy()
    rad = np.radians(float(direction))
    fx, fy = force * np.cos(rad), force * np.sin(rad)
    ns = int(1.0 / sim.dt)

    if verbose:
        print("=" * 66)
        print("  人形机器人平衡与外力抗扰恢复演示")
        print("=" * 66)
        print(f"[INFO] 扰动力 {force:.0f} N × {duration:.2f} s "
              f"(冲量 {force * duration:.2f} N·s, 方向 {direction:.0f}°) | "
              f"髋策略 {'开' if p.get('hip_enable', True) else '关'}")
        print(f"[INFO] 平衡参数 | 踝 kp={p.get('kp_ankle')} kd={p.get('kd_ankle')} "
              f"| 髋 kp={p.get('kp_hip')} kd={p.get('kd_hip')}\n")

    frames, series = [], []
    max_offset = 0.0
    recover_time = None
    fell_at = None

    for i in range(int(8.0 / sim.dt)):
        if i == ns:
            sim.apply_push(force=(fx, fy, 0.0), duration=duration)
        sim.step()
        offset = float(np.linalg.norm(sim.com_position()[:2] - ref))
        max_offset = max(max_offset, offset)
        speed = float(np.linalg.norm(sim.com_velocity()[:2]))
        if capture_dir is not None and i % 20 == 0 and len(frames) < 400:
            # 固定侧视相机：机器人不随相机移动，位移与倾角都能看出来
            frames.append(sim.render(camera="demo_side", height=360, width=480))
        if i % 5 == 0:
            series.append({"t": float(sim.data.time), "offset": offset,
                           "tilt": sim.torso_tilt(), "zmp": sim.zmp().tolist(),
                           "ankle_tau": sim.controller.last.get("ankle_tau"),
                           "hip_tau": sim.controller.last.get("hip_tau")})
        if sim.fallen():
            fell_at = float(sim.data.time)
            break
        if i > ns + 75 and recover_time is None and offset < 0.005 and speed < 0.02:
            recover_time = (i - ns) * sim.dt
        if view is not None:
            view.sync()
            if not view.is_running():
                if verbose:
                    print("[INFO] 可视化窗口已关闭，提前结束演示。")
                break
        if i % 500 == 0 and verbose:
            print(f"[演示] t={sim.data.time:4.2f}s 质心偏移 {offset * 1000:6.1f} mm "
                  f"倾角 {np.degrees(sim.torso_tilt()):5.1f}° 根高 {sim.root_height():.3f} m")
        # 有可视化窗口时把仿真节流到实时
        if pacer is not None:
            pacer.wait()

    if view is not None:
        try:
            view.close()
        except Exception:                                                 # noqa: BLE001
            pass

    metrics = {
        "force_n": force, "duration_s": duration, "impulse_ns": force * duration,
        "direction_deg": float(direction),
        "hip_enable": bool(p.get("hip_enable", True)),
        "max_offset_mm": max_offset * 1000.0,
        "recover_time_s": recover_time,
        "fell": fell_at is not None,
        "fell_at_s": fell_at,
        "final_tilt_deg": float(np.degrees(sim.torso_tilt())),
        "final_root_height_m": sim.root_height(),
        "ankle_torque_limit_nm": p.get("ankle_torque_limit"),
        "hip_torque_limit_nm": p.get("hip_torque_limit"),
        "hip_enable_param": bool(p.get("hip_enable", True)),
    }

    if verbose:
        print()
        if metrics["fell"]:
            print(f"[结果] ❌ 未能恢复：t={fell_at:.2f}s 判为跌倒 "
                  f"(最大偏移 {max_offset * 1000:.0f} mm)")
        else:
            print(f"[结果] ✅ 已恢复：最大偏移 {max_offset * 1000:.0f} mm | "
                  f"恢复时间 {recover_time if recover_time is not None else float('nan'):.2f} s | "
                  f"末态倾角 {metrics['final_tilt_deg']:.2f}°")

    if capture_dir is not None:
        # 推扰发生在第 ns 步，采集帧率为每 20 步一帧 → 推扰时刻的帧序号 ns//20
        metrics["capture"] = capture_assets(frames, series, capture_dir, metrics,
                                            push_frame=ns // 20)
    sim.close()
    return metrics


class _Pacer:
    """把仿真步进节流到实时（仅在打开可视化窗口时使用）。"""

    def __init__(self, dt):
        import time
        self._time = time
        self.dt = dt
        self._next = time.perf_counter()

    def wait(self):
        self._next += self.dt
        delay = self._next - self._time.perf_counter()
        if delay > 0:
            self._time.sleep(delay)
        else:
            self._next = self._time.perf_counter()


def capture_assets(frames, series, capture_dir, metrics, push_frame=None):
    """保存关键帧、动图、时序原始数据 (CSV) 与 metrics.json。

    说明：本模块**不生成数据曲线图**（matplotlib 绘图）。所有实测数值以
    ``main.py --test`` / ``--sweep`` 的表格输出、``metrics.json`` 与这里的
    ``recovery_series.csv`` 为准，需要曲线时请用这些原始数据自行绘制。

    关键帧取自推扰时刻前后（``push_frame`` 为施加推扰时的帧序号）：
    静立 → 被推瞬间 → 恢复中 → 恢复稳定。
    """
    os.makedirs(capture_dir, exist_ok=True)
    saved = []
    if frames:
        n = len(frames)
        push_frame = 0 if push_frame is None else int(push_frame)
        picks = (
            (max(push_frame - 1, 0), "scene_stand"),
            (min(push_frame + 2, n - 1), "scene_push"),
            (min(push_frame + 10, n - 1), "scene_recover"),
            (n - 1, "scene_settled"),
        )
        try:
            import imageio.v2 as imageio
            gif = os.path.join(capture_dir, "push_recovery_demo.gif")
            imageio.mimsave(gif, frames, fps=25)
            saved.append(gif)
            for idx, label in picks:
                png = os.path.join(capture_dir, f"{label}.png")
                imageio.imwrite(png, frames[idx])
                saved.append(png)
        except ImportError:
            print("[采集] 未安装 imageio，跳过关键帧/动图")
    if series:
        import csv
        csv_path = os.path.join(capture_dir, "recovery_series.csv")
        keys = ["t", "offset", "tilt", "ankle_tau", "hip_tau"]
        with open(csv_path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["time_s", "com_offset_m", "torso_tilt_deg",
                             "ankle_torque_nm", "hip_torque_nm"])
            for s in series:
                writer.writerow([
                    f"{s['t']:.4f}",
                    f"{s['offset']:.6f}",
                    f"{np.degrees(s['tilt']):.4f}",
                    "" if s.get("ankle_tau") is None else f"{s['ankle_tau']:.4f}",
                    "" if s.get("hip_tau") is None else f"{s['hip_tau']:.4f}",
                ])
        saved.append(csv_path)
    metrics_path = os.path.join(capture_dir, "metrics.json")
    with open(metrics_path, "w", encoding="utf-8") as handle:
        json.dump(metrics, handle, ensure_ascii=False, indent=2, default=str)
    saved.append(metrics_path)
    print(f"[采集] 素材已写入 {capture_dir}: {[os.path.basename(p) for p in saved]}")
    return saved


# ----------------------------------------------------------------------
# 模式 3：临界冲量扫描
# ----------------------------------------------------------------------
def run_sweep(params, capture_dir=None, verbose=True):
    """测定前向/侧向/后向的临界力，并对比踝策略与踝+髋策略。"""
    p = dict(params)
    dur = float(p.get("push_duration", 0.08))
    results = compare_strategies(p, directions=(0.0, 90.0, 180.0))

    if verbose:
        print("=" * 66)
        print("  最大可恢复冲量测定（扰动持续 %.2f s）" % dur)
        print("=" * 66)
        print(f"{'方向':<12}{'仅踝策略':>22}{'踝+髋策略':>22}{'提升':>10}")
        names = {"0": "前向 (+x)", "90": "侧向 (+y)", "180": "后向 (-x)"}
        for key in ("0", "90", "180"):
            a = results["ankle_only"][key]
            b = results["ankle_plus_hip"][key]
            print(f"{names[key]:<12}"
                  f"{a['critical_force_n']:>10.1f} N / {a['critical_impulse_ns']:>5.2f} N·s"
                  f"{b['critical_force_n']:>10.1f} N / {b['critical_impulse_ns']:>5.2f} N·s"
                  f"{results['gain_percent'][key]:>9.1f}%")
        print()

    # 逐点曲线：以「仅踝」与「踝+髋」各 8 个力档位
    forces = np.linspace(20.0, 320.0, 8)
    curves = {}
    for label, hip in (("ankle_only", False), ("ankle_plus_hip", True)):
        curves[label] = sweep_forces(p, forces, direction_deg=0.0, hip_enable=hip)
        if verbose:
            print(f"[{label}] 前向扰动力 → 结果：")
            for r in curves[label]:
                tag = "跌倒" if r["fell"] else f"最大偏移 {r['max_offset_m'] * 1000:5.0f} mm"
                print(f"    {r['force']:6.0f} N ({r['impulse']:5.2f} N·s) → {tag}")

    metrics = {"push_duration_s": dur, "critical": results, "curves": curves}
    if capture_dir is not None:
        os.makedirs(capture_dir, exist_ok=True)
        # 逐点原始数据写入 sweep_metrics.json（不生成数据曲线图：绘图由使用者
        # 依据这些实测数据自行完成）
        with open(os.path.join(capture_dir, "sweep_metrics.json"), "w", encoding="utf-8") as handle:
            json.dump(metrics, handle, ensure_ascii=False, indent=2, default=str)
    return metrics


# ----------------------------------------------------------------------
# 主入口
# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="人形机器人平衡与外力抗扰恢复（ZMP/捕获点 + 踝/髋策略）")
    parser.add_argument("--config", default=None, help="参数文件 (YAML)")
    parser.add_argument("--model", default=None, help="MJCF 模型路径")
    parser.add_argument("--force", type=float, default=None, help="扰动力大小 (N)")
    parser.add_argument("--direction", type=float, default=None,
                        help="扰动方向（度；0=前向, 90=侧向, 180=后向）；"
                             "不指定时读取 params.yaml 的 push_direction_deg")
    parser.add_argument("--no-hip", dest="hip", action="store_false",
                        help="关闭髋策略（仅踝策略）")
    parser.set_defaults(hip=None)
    parser.add_argument("--check", action="store_true", help="自检")
    parser.add_argument("--sweep", action="store_true", help="测定临界冲量并对比策略")
    parser.add_argument("--capture", type=str, default=None, help="采集素材到指定目录")
    parser.add_argument("--no-viewer", dest="viewer", action="store_false",
                        help="不打开可视化窗口（无图形环境时使用）")
    parser.set_defaults(viewer=True)
    parser.add_argument("--test", action="store_true", help="运行自动化测试")
    parser.add_argument("--ros2", action="store_true", help="通过 ros2 launch 启动")
    args = parser.parse_args()

    if args.test:
        import subprocess
        test_path = os.path.join(HERE, "test", "test_standalone.py")
        sys.exit(subprocess.run([sys.executable, test_path]).returncode)
    if args.ros2:
        import subprocess
        sys.exit(subprocess.run(["ros2", "launch", "humanoid_push_recovery",
                                 "main.launch.py"]).returncode)

    params = load_params(args.config or DEFAULT_CONFIG)
    if args.model:
        params["model_path"] = args.model

    if args.check:
        sys.exit(run_check(params, model_path=args.model))
    if args.sweep:
        run_sweep(params, capture_dir=args.capture)
        return

    metrics = run_demo(params, force=args.force, direction=args.direction,
                       hip_enable=args.hip, capture_dir=args.capture,
                       viewer=args.viewer, model_path=args.model)
    if args.capture is None:
        print(json.dumps(metrics, ensure_ascii=False, indent=2, default=str))


if __name__ == "__main__":
    main()
