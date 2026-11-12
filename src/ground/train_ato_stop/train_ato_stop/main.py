"""离线入口：跑完四个场景，出图、出动图、出结果表。

不需要 ROS，也不需要 numpy/matplotlib，装了 Python 3 就能跑：

    python3 -m train_ato_stop.main --offline --output results
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

from . import render, scene, scenarios, sim
from .ato import AtoConfig
from .energy import EnergyBreakdown, energy_breakdown
from .train import TrainSpec


def _fmt(value: float, unit: str = '', digits: int = 2) -> str:
    return f'{value:.{digits}f}{unit}'


def run_offline(output: str) -> dict:
    os.makedirs(output, exist_ok=True)
    spec = TrainSpec()
    config = AtoConfig()
    results = [sim.run(sc, spec=spec, config=config) for sc in scenarios.all_scenarios()]
    breakdowns = {result.name: energy_breakdown(result, spec) for result in results}

    charts = {
        'flat': ('ato_flat.png', 'FLAT TRACK - SPEED PROFILE',
                 'ATO vs ATP vs STATIC LIMIT, 2.4 KM TO STATION'),
        'restriction': ('ato_restriction.png', 'SPEED RESTRICTION - SPEED PROFILE',
                        '80 KM/H TEMPORARY LIMIT FROM 800 M TO 1300 M'),
        'grade': ('ato_grade.png', 'GRADIENT - SPEED PROFILE',
                  '8 PERMILLE UPHILL THEN 12 PERMILLE DOWNHILL'),
    }
    for result in results:
        if result.name not in charts:
            continue
        name, title, subtitle = charts[result.name]
        canvas = scene.speed_profile_chart(result, title, subtitle)
        render.write_png(os.path.join(output, name), canvas)

    render.write_png(os.path.join(output, 'ato_stop_error.png'),
                     scene.stop_error_chart(results))

    flat = next(r for r in results if r.name == 'flat')
    frames = scene.render_animation(flat)
    render.write_gif(os.path.join(output, 'ato_demo.gif'), frames, delay=8)

    _write_report(os.path.join(output, 'report.md'), results)
    write_energy_report(
        os.path.join(output, 'energy_breakdown.csv'),
        os.path.join(output, 'energy_breakdown.md'),
        results,
        breakdowns,
    )
    for result in results:
        result.metrics['energy_breakdown'] = breakdowns[result.name].as_dict()
    return {r.name: r.metrics for r in results}


def _write_report(path: str, results: list[sim.SimResult]) -> None:
    lines = [
        '| 场景 | 运行时间 | 最高速度 | 停车误差 | 牵引能耗 | ATP 介入 | 超线路限速时长 |',
        '|---|---|---|---|---|---|---|',
    ]
    for r in results:
        m = r.metrics
        lines.append(
            f"| {r.title} | {_fmt(m['run_time'], ' s', 1)} | "
            f"{_fmt(m['max_speed_kph'], ' km/h', 1)} | "
            f"{m['stop_error']:+.2f} m | {_fmt(m['energy_kwh'], ' kWh', 1)} | "
            f"{m['atp_events']} | {_fmt(m['overspeed_time'], ' s')} |"
        )
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')


def write_energy_report(path_csv: str, path_md: str,
                        results: list[sim.SimResult],
                        breakdowns: dict[str, EnergyBreakdown]) -> None:
    headers = [
        'scenario',
        'reported_traction_energy_kwh',
        'traction_work_kwh',
        'traction_limit_shortfall_kwh',
        'retarding_work_kwh',
        'net_inertial_work_kwh',
        'acceleration_work_kwh',
        'deceleration_work_kwh',
        'resistance_work_kwh',
        'grade_work_kwh',
        'endpoint_kinetic_energy_kwh',
    ]
    rows = []
    for result in results:
        values = breakdowns[result.name].as_dict()
        rows.append({
            'scenario': result.name,
            'reported_traction_energy_kwh': round(
                result.metrics['energy_kwh'], 6),
            **values,
        })
    with open(path_csv, 'w', newline='', encoding='utf-8') as fh:
        writer = csv.DictWriter(fh, fieldnames=headers)
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        '| 场景 | 报告牵引 | 积分牵引 | 牵引缺口 | 阻滞 | 净惯性 | 加速 | 减速 | 基本阻力 | 坡道(上坡正) | 末端动能 |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for result in results:
        values = breakdowns[result.name]
        lines.append(
            f"| {result.title} | {result.metrics['energy_kwh']:.3f} | "
            f"{values.traction_work_kwh:.3f} | "
            f"{values.traction_limit_shortfall_kwh:.3f} | "
            f"{values.retarding_work_kwh:.3f} | "
            f"{values.net_inertial_work_kwh:.3f} | "
            f"{values.acceleration_work_kwh:.3f} | "
            f"{values.deceleration_work_kwh:.3f} | "
            f"{values.resistance_work_kwh:.3f} | "
            f"{values.grade_work_kwh:.3f} | "
            f"{values.endpoint_kinetic_energy_kwh:.3f} |"
        )
    with open(path_md, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines) + '\n')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='列车 ATO 精确停车仿真')
    parser.add_argument('--offline', action='store_true', help='不启动 ROS，只跑离线仿真并出图')
    parser.add_argument('--output', default='results', help='输出目录')
    args = parser.parse_args(argv)

    if not args.offline:
        # 没有 ROS 环境时给一句明确的提示，而不是抛 ImportError 栈
        try:
            import rclpy  # noqa: F401
        except ImportError:
            print('当前环境没有 rclpy。离线运行请加 --offline；'
                  '需要 ROS 话题请先 source 工作空间。')
            return 1
        from .ros_node import main as ros_main
        return ros_main()

    metrics = run_offline(args.output)
    print(f"{'场景':<12}{'停车误差':>10}{'运行时间':>10}{'能耗':>10}{'ATP':>6}")
    for name, m in metrics.items():
        print(f"{name:<12}{m['stop_error']:>+9.2f}m{m['run_time']:>9.1f}s"
              f"{m['energy_kwh']:>9.1f}kWh{m['atp_events']:>6d}")
    print(f'\n结果已写入 {os.path.abspath(args.output)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
