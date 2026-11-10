#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 交通信号灯识别与响应 —— 入口。

两种运行方式::

    python3 main.py               # 离线仿真（不需要 CARLA，能出图）
    python3 main.py --carla       # 连 CARLA 跑真车（需要先启动 CarlaUE4.sh）

离线仿真用的是同一套决策逻辑和纵向控制器，只是把 CARLA 换成简单的运动学
积分，因此可以在任何机器上复现判决过程与曲线。

参数：::

    --carla              连接 CARLA（默认 127.0.0.1:2000）
    --host / --port      指定 CARLA 地址
    --duration SEC       仿真时长（默认 20 s）
    --save-dir DIR       CARLA 截图目录（默认 screenshots/）
    --light RED|YELLOW|GREEN   离线仿真里用的灯色（默认 RED）
    --outdir DIR         曲线输出目录（默认 demo_out/）
"""

from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from longitudinal import LongitudinalController  # noqa: E402
from simple_plot import plot_series  # noqa: E402
from traffic_light_logic import (  # noqa: E402
    GO,
    GREEN,
    LightParams,
    RED,
    SLOW,
    STOP,
    YELLOW,
    braking_distance,
    decide,
    target_speed_for_stop,
)

BLUE = (40, 110, 220)
RED_C = (215, 60, 55)
GREEN_C = (35, 165, 90)
ORANGE = (235, 150, 30)
GRAY = (150, 150, 150)


def hr(title=""):
    line = "=" * 72
    if title:
        print(line)
        print(title)
        print(line)
    else:
        print(line)


# ---------------------------------------------------------------------------
def load_params(path):
    """读 config/tl_params.yaml。

    只解析最简单的 ``key: value`` 形式，避免额外引入 PyYAML。
    """
    if not path or not os.path.isfile(path):
        return {}, LightParams()
    light_kv, ctrl_kv = {}, {}
    light_keys = {"comfort_decel", "max_decel", "yellow_min_gap",
                  "stop_margin", "min_speed"}
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            k, v = line.split(":", 1)
            k, v = k.strip(), v.strip()
            try:
                val = float(v)
            except ValueError:
                continue
            if k in light_keys:
                light_kv[k] = val
            else:
                ctrl_kv[k] = val
    return ctrl_kv, LightParams(**light_kv)


def make_controller(kv):
    """按参数构造纵向控制器，缺省用默认值。"""
    allowed = ("kp", "ki", "integral_max", "throttle_gain",
               "brake_gain", "deadband")
    kw = {k: v for k, v in kv.items() if k in allowed}
    return LongitudinalController(**kw)


def offline_sim(light=RED, duration=20.0, dt=0.05, start_distance=60.0,
                start_speed=12.0, params=None, outdir=None, ctrl_kw=None):
    """离线仿真：自车匀速接近信号灯，按决策逻辑制动。

    返回 dict 里含时间序列，便于出图。
    """
    params = params if params is not None else LightParams()
    ctrl = LongitudinalController(**(ctrl_kw or {}))

    x = 0.0                      # 已行驶距离
    v = start_speed
    t = 0.0
    hist = {"t": [], "v": [], "gap": [], "v_target": [], "action": [],
            "throttle": [], "brake": []}

    print()
    hr("离线仿真：灯色 = %s，初速 %.1f m/s，初始距离 %.1f m" % (
        light, start_speed, start_distance))
    print("  参数：舒适减速 %.2f m/s²，最大减速 %.2f m/s²，停车余量 %.2f m"
          % (params.comfort_decel, params.max_decel, params.stop_margin))
    print()
    print("  %6s %8s %8s %9s %8s %8s  %s" % (
        "t(s)", "v(m/s)", "距离(m)", "目标v", "油门", "刹车", "动作"))
    print("  " + "-" * 72)

    passed = False
    last_print = -1.0
    while t <= duration:
        gap = start_distance - x
        if gap < 0 and not passed:
            passed = True

        action, v_target, note = decide(light, gap, v, params,
                                        passed_stop_line=passed)
        if v_target is None:
            v_target = start_speed          # 不限速时保持巡航速度
        else:
            # 停车曲线在离得远时允许很高的速度，但没必要为此先加速，
            # 所以上限压到巡航速度。
            v_target = min(v_target, start_speed)

        # 前馈：停车曲线的设计减速度就是 comfort_decel，直接喂给控制器，
        # 否则 PI 跟不上，速度一降刹车变弱会冲过停止线。
        a_ff = 0.0 if (action == GO and v_target >= start_speed - 1e-9) \
            else -params.comfort_decel

        throttle, brake = ctrl.update(v, v_target, dt, a_feedforward=a_ff)

        hist["t"].append(t)
        hist["v"].append(v)
        hist["gap"].append(gap)
        hist["v_target"].append(v_target)
        hist["action"].append(action)
        hist["throttle"].append(throttle)
        hist["brake"].append(brake)

        if t - last_print >= 1.0:
            print("  %6.2f %8.3f %8.2f %9.3f %8.3f %8.3f  %-5s %s" % (
                t, v, gap, v_target, throttle, brake, action, note))
            last_print = t

        # 简单的纵向动力学：油门给 3 m/s²，刹车给 -6 m/s²
        acc = 3.0 * throttle - 6.0 * brake
        v = max(0.0, v + acc * dt)
        x += v * dt
        t += dt

        if v <= 1e-3 and gap < 0.5 and action == STOP:
            print()
            print("  已在 %.2f s 停住，距停止线 %.3f m" % (t, gap))
            break

    # 汇总
    final_gap = start_distance - x
    stopped = v < 0.05
    print()
    print("  结果：末速 %.3f m/s，距停止线 %.3f m，%s"
          % (v, final_gap, "已停车" if stopped else "仍在行驶"))
    if light == RED and not passed:
        ok = stopped and final_gap > -1e-6
        print("  红灯判定：%s" % ("✓ 停在停止线前" if ok else "✗ 未正确停车"))

    if outdir:
        _plot(hist, outdir)
    return hist


def _plot(hist, outdir):
    if not os.path.isdir(outdir):
        os.makedirs(outdir)
    plot_series(
        os.path.join(outdir, "speed_profile.png"),
        [("actual v", hist["t"], hist["v"], BLUE),
         ("target v", hist["t"], hist["v_target"], GRAY)],
        title="SPEED PROFILE",
    )
    plot_series(
        os.path.join(outdir, "distance_to_stop_line.png"),
        [("gap (m)", hist["t"], hist["gap"], RED_C)],
        title="DISTANCE TO STOP LINE",
    )
    plot_series(
        os.path.join(outdir, "pedal.png"),
        [("throttle", hist["t"], hist["throttle"], GREEN_C),
         ("brake", hist["t"], hist["brake"], ORANGE)],
        title="THROTTLE AND BRAKE",
    )
    print()
    print("  曲线已保存: %s/speed_profile.png, distance_to_stop_line.png, "
          "pedal.png" % outdir)


# ---------------------------------------------------------------------------
def carla_main(args, light_params=None, ctrl_kv=None):
    """连 CARLA 跑真车。"""
    try:
        from carla_runner import TrafficLightRunner
    except Exception as e:
        print("无法导入 carla_runner: %s" % e)
        return 1

    print()
    hr("CARLA 模式")
    print("  连接 %s:%d" % (args.host, args.port))
    runner = TrafficLightRunner(
        params=light_params,
        ctrl=make_controller(ctrl_kv or {}),
        host=args.host, port=args.port)
    try:
        print("  生成自车...")
        runner.spawn_ego()
        print("  挂载相机...")
        runner.attach_camera(args.save_dir)
        print("  开始运行 %.1f s（Ctrl+C 可中断）" % args.duration)
        stats = runner.run(duration=args.duration, dt=0.05, save_every=10)
        print()
        hr("CARLA 结果")
        print("  步数          : %d" % stats["steps"])
        print("  停车决策次数  : %d" % stats["stopped"])
        print("  减速决策次数  : %d" % stats["slow"])
        print("  通过决策次数  : %d" % stats["go"])
        print("  最近距离      : %.2f m" % stats["min_gap"])
        print("  末速          : %.2f m/s" % stats["final_speed"])
        print("  末态灯色      : %s" % stats["final_state"])
        print("  截图数量      : %d（%s）" % (stats["saved_images"], args.save_dir))
    finally:
        runner.cleanup()
    return 0


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="CARLA 交通信号灯识别与响应")
    ap.add_argument("--carla", action="store_true", help="连接 CARLA 真机")
    ap.add_argument("--offline", action="store_true",
                    help="强制离线模式（与 --carla 互斥，默认即离线）")
    ap.add_argument("--params", default=None,
                    help="参数文件，默认用模块内 config/tl_params.yaml")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--save-dir", default=os.path.join(HERE, "screenshots"))
    ap.add_argument("--outdir", default=os.path.join(HERE, "demo_out"))
    ap.add_argument("--light", default=RED, choices=[RED, YELLOW, GREEN])
    ap.add_argument("--speed", type=float, default=12.0)
    ap.add_argument("--distance", type=float, default=60.0)
    ap.add_argument("--all-lights", action="store_true",
                    help="离线模式依次跑红/黄/绿三种灯色")
    # roslaunch 启动 <node> 时会在命令行末尾追加 __name:= / __log:=，
    # 用 parse_known_args 忽略掉，否则 launch 一启动就退出。
    args, _unknown = ap.parse_known_args()

    print()
    hr("CARLA 交通信号灯识别与响应")

    # 参数文件：默认找模块里的 config
    params_path = args.params
    if params_path is None:
        cand = os.path.join(os.path.dirname(HERE), "config", "tl_params.yaml")
        if os.path.isfile(cand):
            params_path = cand
    ctrl_kv, light_params = load_params(params_path)
    if params_path:
        print()
        print("  参数文件: %s" % params_path)
        print("  %s" % light_params)

    if args.carla and not args.offline:
        return carla_main(args, light_params, ctrl_kv)

    # 离线模式
    print()
    print("  离线仿真模式（不连接 CARLA）。加 --carla 可跑真车。")

    if args.all_lights:
        for light in (RED, YELLOW, GREEN):
            offline_sim(light=light, duration=args.duration,
                        start_distance=args.distance, start_speed=args.speed,
                        params=light_params, ctrl_kw=ctrl_kv,
                        outdir=args.outdir if light == RED else None)
        print()
        hr("完成")
        print("  三种灯色的判决都已跑完，曲线在 %s" % args.outdir)
    else:
        offline_sim(light=args.light, duration=args.duration,
                    start_distance=args.distance, start_speed=args.speed,
                    params=light_params, ctrl_kw=ctrl_kv,
                    outdir=args.outdir)
        print()
        hr("完成")
        print("  曲线目录: %s" % os.path.abspath(args.outdir))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
