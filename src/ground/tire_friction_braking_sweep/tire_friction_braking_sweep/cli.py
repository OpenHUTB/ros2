"""命令行入口：离线演示、模拟器实测与重新拟合。

``python main.py demo`` 只用到已有数据文件（没有就跑合成数据），不需要模拟器；
``python main.py sim`` 会连模拟器跑真实扫掠。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

from .analyze import BrakingPoint, SweepDataset, summary
from .model import GRAVITY, BrakingSpec, braking_distance_m, saturated_decel_mps2
from .sim import DEFAULT_FRICTIONS, DEFAULT_VEHICLE, run_sweep

DEFAULT_OUT_DIR = "results"
SIM_DATA = "sim_sweep.json"
METRICS = "metrics.json"
FIGURE = "demo_screenshot.png"


def _tolerant_stdout() -> None:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass


def _print_summary(payload: dict) -> None:
    spec = payload["spec"]
    print(
        f"试验条件：{spec['reference_speed_kmh']:.0f} km/h 全力制动，"
        f"{spec['mass_kg']:.0f} kg，制动踏板 {spec['brake_ratio']:.0%}"
    )
    source = "模拟器实测" if payload.get("source") == "carla" else "合成数据（未连模拟器）"
    print(f"数据来源：{source}")
    env = payload.get("environment") or {}
    if env:
        print(f"环境：{env.get('map')} / {env.get('vehicle')} / 服务端 {env.get('server_version')}")
        if env.get("failures"):
            print(f"失败点：{env['failures']}")
    fit = payload["fit"]
    print(
        f"\n饱和拟合：a_max={fit['a_max_mps2']:.2f} m/s²    "
        f"μ½={fit['mu_half']:.2f}    R²={fit['r2']:.4f}    RMSE={fit['rmse_mps2']:.3f} m/s²"
    )
    header = f"{'μ':>6} {'距离(m)':>10} {'a(m/s²)':>10} {'a/μg':>8} {'弹性':>9}  判定"
    print("\n" + header)
    print("-" * len(header))
    for point in payload["points"]:
        elastic = point["elasticity"]
        elastic_text = "—" if elastic is None else f"{elastic:.3f}"
        print(
            f"{point['tire_friction']:>6.2f} {point['distance_m']:>10.2f} "
            f"{point['decel_mps2']:>10.2f} {point['distance_ratio']:>8.2f} "
            f"{elastic_text:>9}  {point['bottleneck']}"
        )
    span = payload["range"]
    print(
        f"\nμ {span['mu_min']:.2f} → {span['mu_max']:.2f}：制动距离 "
        f"{span['distance_at_mu_min_m']:.2f} m 缩短到 {span['distance_at_mu_max_m']:.2f} m"
        f"（-{span['distance_reduction_percent']:.1f}%）"
    )


def _synthetic_dataset(
    frictions=DEFAULT_FRICTIONS,
    speed_kmh: float = 50.0,
    a_max: float = 6.4,
    mu_half: float = 0.45,
) -> SweepDataset:
    """没有模拟器数据时，用饱和模型生成一组示例点（明确标注为合成数据）。"""
    speed = speed_kmh / 3.6
    points = []
    for friction in frictions:
        decel = float(saturated_decel_mps2(friction, a_max, mu_half))
        distance = float(braking_distance_m(speed, decel))
        points.append(
            BrakingPoint(
                tire_friction=float(friction),
                entry_speed_mps=speed,
                distance_m=distance,
                duration_s=speed / decel,
            )
        )
    return SweepDataset(
        spec=BrakingSpec(reference_speed_kmh=speed_kmh),
        points=points,
        source="synthetic",
        environment={"note": "未连接模拟器，数据由饱和模型生成"},
    )


def run_demo(argv) -> int:
    parser = argparse.ArgumentParser(prog="main.py demo", description="生成运行效果图与指标")
    parser.add_argument("--out", default=DEFAULT_OUT_DIR, help="输出目录（默认 results）")
    parser.add_argument("--force-synthetic", action="store_true", help="忽略已有实测数据")
    args = parser.parse_args(argv)

    data_path = os.path.join(args.out, SIM_DATA)
    if os.path.exists(data_path) and not args.force_synthetic:
        with open(data_path, encoding="utf-8") as handle:
            stored = json.load(handle)
        print(f"读取实测数据：{data_path}")
        # 用存下来的点重算一遍摘要，保证 elasticity 等派生字段是最新的
        payload = summary(_dataset_from_payload(stored))
        payload["source"] = stored.get("source", "carla")
        payload["environment"] = stored.get("environment") or {}
    else:
        payload = summary(_synthetic_dataset())
        print("未找到实测数据，使用合成数据集（运行 main.py sim 可获得实测结果）")

    _print_summary(payload)

    dataset = _dataset_from_payload(payload)
    from .visualize import render

    os.makedirs(args.out, exist_ok=True)
    figure = os.path.join(args.out, FIGURE)
    render(dataset, payload, figure)
    metrics_path = os.path.join(args.out, METRICS)
    with open(metrics_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    print(f"\n运行效果图：{figure}")
    print(f"指标文件：{metrics_path}")
    return 0


def _dataset_from_payload(payload: dict) -> SweepDataset:
    spec = payload["spec"]
    points = [
        BrakingPoint(
            tire_friction=float(row["tire_friction"]),
            entry_speed_mps=float(row["entry_speed_kmh"]) / 3.6,
            distance_m=float(row["distance_m"]),
            duration_s=float(row.get("duration_s", 0.0)),
        )
        for row in payload["points"]
    ]
    return SweepDataset(
        spec=BrakingSpec(
            mass_kg=float(spec.get("mass_kg", 1500.0)),
            reference_speed_kmh=float(spec.get("reference_speed_kmh", 50.0)),
            brake_ratio=float(spec.get("brake_ratio", 1.0)),
        ),
        points=points,
        source=payload.get("source", "offline"),
        environment=payload.get("environment") or {},
    )


def run_sim(argv) -> int:
    parser = argparse.ArgumentParser(prog="main.py sim", description="在模拟器里跑真实扫掠")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--vehicle", default=DEFAULT_VEHICLE)
    parser.add_argument("--speed", type=float, default=50.0, help="制动初速（km/h）")
    parser.add_argument("--out", default=DEFAULT_OUT_DIR)
    parser.add_argument("--frictions", type=float, nargs="+", default=list(DEFAULT_FRICTIONS))
    args = parser.parse_args(argv)

    from .sim import connect

    client = connect(args.host, args.port)
    dataset = run_sweep(client, frictions=tuple(args.frictions),
                        vehicle_id=args.vehicle, target_kmh=args.speed)
    payload = summary(dataset)
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, SIM_DATA)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    _print_summary(payload)

    from .visualize import render

    figure = os.path.join(args.out, FIGURE)
    render(dataset, payload, figure)
    with open(os.path.join(args.out, METRICS), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    print(f"\n实测数据：{path}")
    print(f"运行效果图：{figure}")
    return 0


def run_fit(argv) -> int:
    parser = argparse.ArgumentParser(prog="main.py fit", description="对已有 JSON 重新拟合并打印")
    parser.add_argument("path", help="main.py sim 生成的 JSON")
    args = parser.parse_args(argv)
    with open(args.path, encoding="utf-8") as handle:
        payload = json.load(handle)
    _print_summary(summary(_dataset_from_payload(payload)))
    return 0


def main(argv=None) -> int:
    _tolerant_stdout()
    args = list(sys.argv[1:] if argv is None else argv)
    commands = {"demo": run_demo, "sim": run_sim, "fit": run_fit}
    if args and args[0] in commands:
        return commands[args[0]](args[1:])
    return run_demo(args)


if __name__ == "__main__":
    raise SystemExit(main())
