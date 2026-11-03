"""命令行入口：离线演示、单车速估算与 ROS 2 节点三种模式。

``python main.py`` / ``python main.py demo`` 不需要 ROS 2；``ros2 launch`` 会把带
``--ros-args`` 的启动参数交给 ROS 2 节点。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

from .disc import BrakeDisc, BrakeDiscSpec, FrictionFadeCurve
from .scenario import (
    DESCENT_SPEEDS_KMH,
    STOP_COOLDOWN_S,
    STOP_CYCLES,
    STOP_SPEED_KMH,
    build_demo,
    kmh_to_mps,
    simulate_descent,
    simulate_stop_cycles,
    summary,
)
from .vehicle import BrakingSpec, per_disc_power_w, stopping_distance_m

DEFAULT_OUT_DIR = "results"


def _tolerant_stdout() -> None:
    """窄编码控制台（如 GBK）下不因个别字符中断；只改错误处理，不改编码。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass


def _print_header(disc: BrakeDisc, braking: BrakingSpec) -> None:
    spec = disc.spec
    curve = disc.curve
    print(
        f"制动盘：单盘 {spec.disc_mass_kg:.1f} kg，热容 "
        f"{spec.heat_capacity_j_per_k / 1000.0:.1f} kJ/K，散热 {spec.cooling_w_per_k:.0f} W/K，"
        f"环境 {spec.ambient_c:.0f} °C"
    )
    print(
        f"摩擦：常温 μ={curve.mu_cold:.2f}，{curve.fade_start_c:.0f}–{curve.fade_end_c:.0f} °C "
        f"内衰退到 μ={curve.mu_hot:.2f}"
    )
    print(
        f"整车：{braking.mass_kg:.0f} kg，{braking.brake_count} 个制动盘，"
        f"常温减速度 {braking.reference_decel_mps2:.1f} m/s^2"
    )


def run_demo(argv) -> int:
    parser = argparse.ArgumentParser(prog="main.py demo", description="生成运行效果图与指标")
    parser.add_argument("--out", default=DEFAULT_OUT_DIR, help="输出目录（默认 results）")
    parser.add_argument("--grade", type=float, default=0.07, help="长下坡坡度（小数，默认 0.07）")
    parser.add_argument("--duration", type=float, default=600.0, help="长下坡仿真时长（s）")
    parser.add_argument("--cycles", type=int, default=STOP_CYCLES, help="连续制动次数")
    parser.add_argument("--cooldown", type=float, default=STOP_COOLDOWN_S, help="制动间隔（s）")
    args = parser.parse_args(argv)

    bundle = build_demo(grade=args.grade, duration_s=args.duration,
                        cycles=args.cycles, cooldown_s=args.cooldown)
    _print_header(bundle.disc, bundle.braking)

    print("\n恒速长下坡（{:.0f}% 坡度，{:.0f} s）：".format(args.grade * 100, args.duration))
    header = f"{'车速(km/h)':>10} {'单盘功率(kW)':>13} {'平衡温度(C)':>12} " \
             f"{'进入衰退(s)':>12} {'100→0 停车(m)':>14}"
    print(header)
    print("-" * len(header))
    for run in bundle.descents:
        equilibrium = "—" if run.equilibrium_c is None else f"{run.equilibrium_c:.0f}"
        onset = "未衰退" if run.fade_onset_s is None else f"{run.fade_onset_s:.0f}"
        print(
            f"{run.speed_kmh:>10.0f} {run.power_per_disc_w / 1000.0:>13.2f} "
            f"{equilibrium:>12} {onset:>12} {run.stop_distance_m:>14.1f}"
        )

    cycles = bundle.cycles
    print(
        f"\n连续 {cycles.cycle.size} 次 {cycles.speed_kmh:.0f}→0 km/h 急刹"
        f"（间隔 {cycles.cooldown_s:.0f} s）："
    )
    header2 = f"{'次序':>4} {'起温(C)':>9} {'终温(C)':>9} {'停车距离(m)':>12}"
    print(header2)
    print("-" * len(header2))
    for cycle, before, after, distance in zip(
        cycles.cycle, cycles.temperature_before_c,
        cycles.temperature_after_c, cycles.distance_m
    ):
        flag = "  已衰退" if before >= bundle.disc.curve.fade_start_c else ""
        print(f"{cycle:>4} {before:>9.0f} {after:>9.0f} {distance:>12.1f}{flag}")
    print(
        f"\n冷态 100→0 停车距离 {cycles.distance_cold_m:.1f} m；"
        f"末次 {cycles.distance_m[-1]:.1f} m"
        f"（{(cycles.distance_m[-1] / cycles.distance_cold_m - 1.0) * 100.0:+.1f}%）"
    )

    from .visualize import render

    os.makedirs(args.out, exist_ok=True)
    figure_path = os.path.join(args.out, "demo_screenshot.png")
    render(bundle, figure_path)

    payload = summary(bundle)
    payload["options"] = {
        "grade": args.grade,
        "duration_s": args.duration,
        "cycles": args.cycles,
        "cooldown_s": args.cooldown,
    }
    payload["emergency_stop_kmh"] = 100.0
    metrics_path = os.path.join(args.out, "metrics.json")
    with open(metrics_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    print(f"\n运行效果图：{figure_path}")
    print(f"指标文件：{metrics_path}")
    return 0


def run_check(argv) -> int:
    parser = argparse.ArgumentParser(prog="main.py check", description="单车速/单温度速查")
    parser.add_argument("--speed", type=float, default=60.0, help="车速（km/h）")
    parser.add_argument("--grade", type=float, default=0.07, help="坡度（小数）")
    parser.add_argument("--temperature", type=float, default=None,
                        help="给定盘温直接算停车距离（°C），缺省按环境温度")
    args = parser.parse_args(argv)

    disc, braking = BrakeDisc(), BrakingSpec()
    _print_header(disc, braking)
    power = per_disc_power_w(kmh_to_mps(args.speed), args.grade, braking)
    equilibrium = disc.equilibrium_temperature_c(power)
    onset = disc.time_to_reach_c(power, disc.curve.fade_start_c)
    temperature = disc.spec.ambient_c if args.temperature is None else args.temperature
    print(
        f"\n车速 {args.speed:.0f} km/h、坡度 {args.grade * 100:.0f}%：单盘功率 "
        f"{power / 1000.0:.2f} kW，平衡温度 "
        f"{'—（超出散热能力）' if equilibrium is None else f'{equilibrium:.0f} °C'}，"
        f"进入衰退 {'不会发生' if onset is None else f'{onset:.0f} s'}"
    )
    print(
        f"盘温 {temperature:.0f} °C 时：μ={float(disc.curve.mu(temperature)):.3f}，"
        f"100→0 km/h 停车距离 "
        f"{stopping_distance_m(kmh_to_mps(100.0), temperature, braking, disc.curve):.1f} m"
    )
    return 0


def run_ros(argv) -> int:
    from .ros_node import main as ros_main

    ros_main(argv)
    return 0


def main(argv=None) -> int:
    _tolerant_stdout()
    args = list(sys.argv[1:] if argv is None else argv)
    commands = {"demo": run_demo, "check": run_check, "ros": run_ros}
    if args and args[0] in commands:
        return commands[args[0]](args[1:])
    if any(arg.startswith("--ros-args") or arg.startswith("__") for arg in args):
        return run_ros(args)
    return run_demo(args)


if __name__ == "__main__":
    raise SystemExit(main())
