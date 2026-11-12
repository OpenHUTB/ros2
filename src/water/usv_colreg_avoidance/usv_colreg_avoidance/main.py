"""模块入口：离线跑全部场景出图出表，或转交给 ROS2 节点。"""
from __future__ import annotations

import argparse
import os
import sys

from . import render, scene, scenarios, sim
from .colreg import ColregConfig


def _scenario_report(rows: list) -> str:
    head = ('| 场景 | 不避让最小距离 | 避让后最小距离 | 碰撞 | 首次让路 | 峰值转向 (deg) |\n'
            '|---|---|---|---|---|---|\n')
    body = []
    for sc, res, base in rows:
        turns = ' / '.join(f'{t:.1f}' for t in res.metrics['peak_turn'])
        onset = [t for t in res.metrics['first_action'] if t is not None]
        onset_text = f'{min(onset):.0f} s' if onset else '—'
        body.append(
            f'| {sc.name} | {base.metrics["min_separation"]:.1f} m '
            f'| {res.metrics["min_separation"]:.1f} m '
            f'| {"是" if base.metrics["collision"] else "否"} → '
            f'{"是" if res.metrics["collision"] else "否"} '
            f'| {onset_text} | {turns} |')
    return head + '\n'.join(body) + '\n'


def run_offline(output_dir: str = 'results', config: ColregConfig | None = None,
                gif_scenario: str = 'crossing') -> list:
    """跑完四个场景，输出 PNG / GIF / Markdown 结果表。"""
    config = config or ColregConfig()
    os.makedirs(output_dir, exist_ok=True)

    rows = []
    by_name = {}
    for factory in scenarios.ALL:
        sc = factory()
        res = sim.run(sc, config=config)
        base = sim.run(sc, config=config, avoid=False)
        rows.append((sc, res, base))
        by_name[sc.name] = res

        canvas = scene.render_still(res, d_safe=config.d_safe)
        render.write_png(os.path.join(output_dir, f'usv_colreg_{sc.name}.png'), canvas)

    if gif_scenario in by_name:
        frames = scene.render_animation(by_name[gif_scenario], d_safe=config.d_safe)
        render.write_gif(os.path.join(output_dir, 'usv_colreg_demo.gif'), frames)

    table = _scenario_report(rows)
    with open(os.path.join(output_dir, 'report.md'), 'w', encoding='utf-8') as handle:
        handle.write('# 会遇避碰仿真结果\n\n')
        handle.write(f'安全会遇距离 D_safe = {config.d_safe:.0f} m，'
                     f'紧迫危险距离 = {config.d_emergency:.0f} m\n\n')
        handle.write(table)
    return rows


def main(argv: list[str] | None = None) -> int:
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

    parser = argparse.ArgumentParser(
        description='无人水面艇 COLREG 会遇避碰：离线仿真出图，或启动 ROS2 节点')
    parser.add_argument('--offline', action='store_true',
                        help='只跑离线仿真并输出图片与结果表（默认行为）')
    parser.add_argument('--ros', action='store_true', help='启动 ROS2 节点')
    parser.add_argument('--output', default='results', help='图片与结果表输出目录')
    parser.add_argument('--d-safe', type=float, default=400.0, help='安全会遇距离 (m)')
    args = parser.parse_args(argv)

    if args.ros:
        from .ros_node import main as ros_main
        ros_main()
        return 0

    config = ColregConfig(d_safe=args.d_safe)
    rows = run_offline(args.output, config)
    print(_scenario_report(rows))
    print(f'结果已写入 {os.path.abspath(args.output)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
