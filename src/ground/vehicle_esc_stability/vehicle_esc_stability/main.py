"""命令行入口：跑四个工况、打印指标表、生成图片。

用法：
    python3 -m vehicle_esc_stability.main --offline --output results
    python3 -m vehicle_esc_stability.main --offline --output /tmp/esc --scenario mu_split

不传 --offline 时也只做离线计算，ROS 2 可视化走 ros_node 与 launch 文件。
"""
from __future__ import annotations

import argparse
import os

from . import render, scene
from .scenarios import all_scenarios, by_name
from .sim import SimResult, run

METRIC_ROWS = (
    ('max_beta_deg', '最大侧偏角', 'deg', 2),
    ('max_yaw_rate_dps', '最大横摆角速度', 'deg/s', 1),
    ('yaw_error_rms_dps', '横摆跟踪误差 RMS', 'deg/s', 2),
    ('max_cross_track_m', '最大路径偏离', 'm', 2),
    ('stop_distance_m', '制动距离', 'm', 1),
    ('final_speed_kph', '终了车速', 'km/h', 1),
    ('spin', '是否甩尾', '', 0),
    ('esc_active_s', 'ESC 介入时长', 's', 2),
    ('esc_peak', 'ESC 峰值强度', '', 2),
)


def run_all() -> list[tuple[SimResult, SimResult]]:
    pairs = []
    for scenario in all_scenarios():
        off = run(scenario, esc_enabled=False)
        on = run(scenario, esc_enabled=True)
        pairs.append((off, on))
    return pairs


def print_table(pairs: list[tuple[SimResult, SimResult]]) -> None:
    header = f"{'工况':<16}{'ESC':<5}" + ''.join(
        f'{label:>16}' for _, label, _, _ in METRIC_ROWS)
    print(header)
    print('-' * len(header))
    for off, on in pairs:
        for result in (off, on):
            row = f'{result.scenario.name:<16}{"开" if result.esc_enabled else "关":<5}'
            for key, _, unit, digits in METRIC_ROWS:
                value = result.metrics[key]
                text = f'{value:.{digits}f}{unit}'
                row += f'{text:>16}'
            print(row)
        print()

    print('无 ESC 与有 ESC 的对比：')
    for off, on in pairs:
        beta = f'{off.metrics["max_beta_deg"]:6.1f} -> {on.metrics["max_beta_deg"]:5.1f} deg'
        cross = (f'{off.metrics["max_cross_track_m"]:6.2f} -> '
                 f'{on.metrics["max_cross_track_m"]:5.2f} m')
        spin = f'{int(off.metrics["spin"])} -> {int(on.metrics["spin"])}'
        print(f'  {off.scenario.name:<16} 侧偏角 {beta}   偏离 {cross}   甩尾 {spin}')


def _pick(pairs: list[tuple[SimResult, SimResult]], name: str):
    """按工况名取一组结果；只跑了单个工况时退化成第一组。"""
    for off, on in pairs:
        if off.scenario.name == name:
            return off, on
    return pairs[0]


def write_images(pairs: list[tuple[SimResult, SimResult]], output: str) -> list[str]:
    os.makedirs(output, exist_ok=True)
    written = []

    off, on = _pick(pairs, 'lane_change')
    path = os.path.join(output, 'esc_trajectory.png')
    render.write_png(path, scene.trajectory_figure(off, on))
    written.append(path)

    path = os.path.join(output, 'esc_yawrate.png')
    render.write_png(path, scene.state_figure(off, on))
    written.append(path)

    off_mu, on_mu = _pick(pairs, 'mu_split')
    path = os.path.join(output, 'esc_mu_split.png')
    render.write_png(path, scene.mu_split_figure(off_mu, on_mu))
    written.append(path)

    path = os.path.join(output, 'esc_summary.png')
    render.write_png(path, scene.summary_figure(pairs))
    written.append(path)

    path = os.path.join(output, 'esc_demo.gif')
    frames = scene.animation_frames(off, on)
    render.write_gif(path, frames, delay=10)
    written.append(path)
    return written


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='ESC 电子稳定控制仿真')
    parser.add_argument('--output', default='results', help='图片输出目录')
    parser.add_argument('--offline', action='store_true', help='只跑离线仿真（默认行为）')
    parser.add_argument('--scenario', default='', help='只跑单个工况，逗号分隔: mu_split')
    parser.add_argument('--no-images', action='store_true', help='只打印指标，不生成图片')
    args = parser.parse_args(argv)

    if args.scenario:
        names = [item.strip() for item in args.scenario.split(',') if item.strip()]
        scenarios = [by_name(name) for name in names]
    else:
        scenarios = all_scenarios()

    pairs = []
    for scenario in scenarios:
        print(f'>>> 运行工况 {scenario.name}：{scenario.title}')
        print(f'    路面 mu 左 {scenario.road.mu_left:.2f} / 右 {scenario.road.mu_right:.2f}，'
              f'初速 {scenario.initial_speed * 3.6:.0f} km/h')
        pairs.append((run(scenario, esc_enabled=False), run(scenario, esc_enabled=True)))

    print()
    print_table(pairs)

    if not args.no_images:
        written = write_images(pairs, args.output)
        print('图片已写入：')
        for path in written:
            size = os.path.getsize(path)
            print(f'  {path}  ({size / 1024:.1f} KB)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
