"""命令行入口：离线演示、单车速扫查、ROS 2 节点三种模式。

``python main.py`` / ``python main.py demo`` 不需要 ROS 2；``ros2 launch`` 或
``ros2 run speed_bump_ride_comfort main`` 会把带 ``--ros-args`` 的启动参数交给
ROS 2 节点。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

from .analysis import (
    DEFAULT_LIMITS_MPS2,
    DEFAULT_SPEEDS_MPS,
    DEFAULT_SWEEP,
    build_demo,
    recommend_max_speed,
    speed_curve,
)
from .bump import SpeedBump
from .comfort import passage_metrics
from .vehicle import QuarterCar

DEFAULT_OUT_DIR = "results"


def _tolerant_stdout() -> None:
    """让控制台在 GBK 等窄编码下不因个别字符（如 ``²``）中断。

    只改错误处理策略、不改编码，所以中文在这种控制台上仍能正常显示。控制台文案
    统一用 ``m/s^2`` 之类的写法，``²`` 只出现在图片里。
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):  # 非 TextIOWrapper 时直接忽略
            pass


def _print_header(vehicle: QuarterCar, bump: SpeedBump) -> None:
    print(
        f"减速带：{bump.height_m * 1000:.0f} mm 高 × {bump.width_m:.1f} m 宽"
        f"（{'半正弦' if bump.kind == 'half_sine' else '梯形'}，"
        f"截面面积 {bump.reference_area_m2():.4f} m^2）"
    )
    print(
        f"车辆：车身 {vehicle.sprung_mass_kg:.0f} kg / 非悬挂 "
        f"{vehicle.unsprung_mass_kg:.0f} kg，车身偏频 "
        f"{vehicle.body_natural_frequency_hz:.2f} Hz，车轮偏频 "
        f"{vehicle.wheel_natural_frequency_hz:.2f} Hz，阻尼比 {vehicle.damping_ratio:.3f}"
    )


def _print_rows(rows) -> None:
    header = f"{'车速(km/h)':>10} {'峰值(m/s^2)':>11} {'RMS(m/s^2)':>10} " \
             f"{'VDV(m/s^1.75)':>14} {'最小轮荷比':>11} {'动行程(mm)':>11}"
    print(header)
    print("-" * len(header))
    for row in rows:
        flag = "  >阈值" if row["exceeds_comfort_limit"] else ""
        lift = "  离地" if row["tire_load_min_ratio"] <= 0.0 else ""
        print(
            f"{row['speed_kmh']:>10.1f} {row['peak_accel_mps2']:>11.2f} "
            f"{row['rms_accel_mps2']:>10.2f} {row['vdv_mps1_75']:>14.2f} "
            f"{row['tire_load_min_ratio']:>11.2f} {row['suspension_travel_mm']:>11.1f}"
            f"{flag}{lift}"
        )


def _print_advice(advice) -> None:
    print("\n建议通过车速（自定义阈值，非标准限值）：")
    for entry in advice:
        if entry.get("speed_mps") is None:
            print(
                f"  · {entry['label']}：扫速范围内无满足项"
                f"（{entry['first_speed_mps']:.1f} m/s 已超阈值）"
            )
            continue
        print(
            f"  · {entry['label']}：≤ {entry['speed_mps'] * 3.6:.1f} km/h"
            f"（{entry['speed_mps']:.1f} m/s，峰值 {entry['peak_accel_mps2']:.2f} m/s^2，"
            f"阈值 {entry['limit_mps2']:.1f} m/s^2）"
        )


def run_demo(argv) -> int:
    parser = argparse.ArgumentParser(prog="main.py demo", description="生成运行效果图与指标")
    parser.add_argument("--out", default=DEFAULT_OUT_DIR, help="输出目录（默认 results）")
    parser.add_argument("--dt", type=float, default=1.0e-3, help="积分步长（s）")
    parser.add_argument(
        "--speeds",
        type=float,
        nargs="+",
        default=list(DEFAULT_SPEEDS_MPS),
        help="对比用的车速（m/s）",
    )
    args = parser.parse_args(argv)

    demo = build_demo(speeds_mps=tuple(args.speeds), dt=args.dt)
    _print_header(demo["vehicle"], demo["bump"])
    _print_rows(demo["rows"])
    _print_advice(demo["advice"])

    from .visualize import render

    os.makedirs(args.out, exist_ok=True)
    figure_path = os.path.join(args.out, "demo_screenshot.png")
    render(demo, figure_path)

    payload = {
        "vehicle": demo["vehicle_summary"],
        "bump": demo["bump_summary"],
        "dt_s": demo["dt_s"],
        "limits_mps2": demo["limits_mps2"],
        "primary_limit_mps2": demo["primary_limit_mps2"],
        "metrics": demo["rows"],
        "advice": demo["advice"],
        "sweep": {
            "speeds_mps": [round(float(v), 3) for v in demo["curve"]["speeds_mps"]],
            "peak_accel_mps2": [round(float(v), 4) for v in demo["curve"]["peak_accel_mps2"]],
            "tire_load_min_ratio": [
                round(float(v), 4) for v in demo["curve"]["tire_load_min_ratio"]
            ],
            "suspension_travel_mm": [
                round(float(v), 4) for v in demo["curve"]["suspension_travel_mm"]
            ],
        },
    }
    metrics_path = os.path.join(args.out, "metrics.json")
    with open(metrics_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    print(f"\n运行效果图：{figure_path}")
    print(f"指标文件：{metrics_path}")
    return 0


def run_sweep(argv) -> int:
    parser = argparse.ArgumentParser(prog="main.py sweep", description="单车速或扫速计算")
    parser.add_argument("--speed", type=float, default=None, help="单车速（m/s），缺省则扫速")
    parser.add_argument("--dt", type=float, default=1.0e-3, help="积分步长（s）")
    parser.add_argument("--limit", type=float, default=DEFAULT_LIMITS_MPS2[0][1],
                        help="舒适阈值（m/s^2）")
    args = parser.parse_args(argv)

    vehicle = QuarterCar()
    bump = SpeedBump()
    _print_header(vehicle, bump)
    if args.speed is not None:
        passage = vehicle.simulate(bump, args.speed, dt=args.dt)
        _print_rows([passage_metrics(passage, limit_mps2=args.limit,
                                     travel_limit_mm=vehicle.travel_limit_m * 1000.0)])
        return 0
    curve = speed_curve(vehicle, bump, dt=args.dt, **DEFAULT_SWEEP)
    advice = recommend_max_speed(curve, args.limit)
    print(f"{'车速(m/s)':>10} {'车速(km/h)':>11} {'峰值(m/s^2)':>11} {'最小轮荷比':>11}")
    for speed, peak, load in zip(
        curve["speeds_mps"], curve["peak_accel_mps2"], curve["tire_load_min_ratio"]
    ):
        print(f"{speed:>10.1f} {speed * 3.6:>11.1f} {peak:>11.2f} {load:>11.2f}")
    if advice["speed_mps"] is not None:
        print(
            f"\n阈值 {args.limit:.1f} m/s^2 下，建议最高车速 "
            f"{advice['speed_mps'] * 3.6:.1f} km/h（{advice['speed_mps']:.1f} m/s）"
        )
    else:
        print(f"\n阈值 {args.limit:.1f} m/s^2 下，扫速范围内无满足项")
    return 0


def run_ros(argv) -> int:
    from .ros_node import main as ros_main

    ros_main(argv)
    return 0


def main(argv=None) -> int:
    _tolerant_stdout()
    args = list(sys.argv[1:] if argv is None else argv)
    commands = {"demo": run_demo, "sweep": run_sweep, "ros": run_ros}
    if args and args[0] in commands:
        return commands[args[0]](args[1:])
    if any(arg.startswith("--ros-args") or arg.startswith("__") for arg in args):
        return run_ros(args)
    return run_demo(args)


if __name__ == "__main__":
    raise SystemExit(main())
