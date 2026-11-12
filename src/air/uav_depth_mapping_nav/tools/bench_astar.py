# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""A* 规划的离线基准：不需要 ROS、不需要仿真器，只要 numpy。

在固定的 200x200 栅格（0.25 m/格，即 50 m x 50 m）上，用随机起终点各跑
``--runs`` 次，统计耗时与路径长度；另外跑 4 个定性用例（空地直连、绕障、
无解、扩展上限）。

用法::

    python3 tools/bench_astar.py --runs 20

输出一张表，可直接贴进文档的「实验结果」一节。
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_depth_mapping_nav import astar as astar_lib  # noqa: E402

RESOLUTION = 0.25
INFLATION = 0.6
SIZE = 200


def empty_map():
    return np.zeros((SIZE, SIZE), dtype=bool)


def random_obstacles(seed=7, ratio=0.02):
    """随机散点障碍（2%）。

    比例不能大：0.6 m 的膨胀半径相当于 2.4 格，20% 的随机点膨胀后会糊死
    93% 的格子，不再是"可规划"的地图。
    """
    rng = np.random.default_rng(seed)
    blocked = rng.random((SIZE, SIZE)) < ratio
    blocked[:4, :] = blocked[-4:, :] = False
    blocked[:, :4] = blocked[:, -4:] = False
    return blocked


def corridor_map():
    """10 m 宽的走廊（两侧有墙），中间一道只开下半段的隔墙，必须绕行。"""
    blocked = np.zeros((SIZE, SIZE), dtype=bool)
    blocked[:, :20] = True            # 左墙 5 m
    blocked[:, 180:] = True           # 右墙 5 m
    blocked[:120, 100:104] = True     # 中间隔墙，只挡上半段
    return blocked


def run_case(blocked, runs, seed=0, allow_diagonal=True):
    rng = np.random.default_rng(seed)
    free = np.argwhere(~blocked)      # (N, 2) 的 (y, x)
    if len(free) < 2:
        raise SystemExit('地图上可通行格子太少')

    times, lengths, expansions = [], [], []
    successes = 0
    for _ in range(runs):
        pick = rng.choice(len(free), size=2, replace=False)
        start = (int(free[pick[0]][1]), int(free[pick[0]][0]))
        goal = (int(free[pick[1]][1]), int(free[pick[1]][0]))
        result = astar_lib.astar(blocked, start, goal,
                                 allow_diagonal=allow_diagonal,
                                 allow_unknown=False)
        if result['success']:
            successes += 1
            times.append(result['elapsed_ms'])
            lengths.append(result['length_cells'] * RESOLUTION)
            expansions.append(result['expansions'])
    return {
        'runs': runs,
        'success': successes,
        'success_rate': successes / runs * 100.0,
        'mean_ms': float(np.mean(times)) if times else float('nan'),
        'max_ms': float(np.max(times)) if times else float('nan'),
        'p95_ms': float(np.percentile(times, 95)) if times else float('nan'),
        'mean_m': float(np.mean(lengths)) if lengths else float('nan'),
        'mean_exp': float(np.mean(expansions)) if expansions else float('nan'),
    }


def qualitative_cases():
    """4 个定性用例，返回 (名称, 是否符预期, 说明)。"""
    cases = []

    blocked = empty_map()
    r = astar_lib.astar(blocked, (10, 10), (60, 10), allow_unknown=False)
    cases.append(('空地直连', r['success'] and len(r['path']) == 51,
                  f"success={r['success']} 格子数={len(r['path'])}（期望 51）"))

    blocked = np.zeros((SIZE, SIZE), dtype=bool)
    blocked[:, 30:34] = True             # 贯穿整幅地图的竖墙，没有缺口
    r = astar_lib.astar(blocked, (10, 50), (60, 50), allow_unknown=False)
    cases.append(('直线被墙挡（无缺口）', not r['success'],
                  f"success={r['success']} reason={r['reason']}（期望失败）"))

    blocked = np.zeros((SIZE, SIZE), dtype=bool)
    blocked[:90, 30:34] = True           # 同一堵墙，但 y>=90 是通的
    r = astar_lib.astar(blocked, (10, 50), (60, 50), allow_unknown=False)
    detour = r['length_cells'] * RESOLUTION if r['success'] else 0.0
    cases.append(('绕障（下方留缺口）', r['success'] and detour > 13.0,
                  f"success={r['success']} 长度={detour:.2f} m（直线 12.5 m，绕行应更长）"))

    blocked = empty_map()
    r = astar_lib.astar(blocked, (0, 0), (199, 199),
                        allow_unknown=False, max_expansions=50)
    cases.append(('扩展次数超限', r['reason'] == 'max_expansions',
                  f"reason={r['reason']}（期望 max_expansions）"))
    return cases


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=int, default=20, help='每个地图的随机用例数')
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    print(f'栅格 {SIZE}x{SIZE}，分辨率 {RESOLUTION} m/格，膨胀半径 {INFLATION} m，'
          f'每张图 {args.runs} 组随机起终点\n')

    for label, base in (('空旷地图', empty_map()),
                        ('随机散点障碍 2%', random_obstacles()),
                        ('10 m 走廊（错位隔墙）', corridor_map())):
        started = time.perf_counter()
        blocked = astar_lib.inflate(base, RESOLUTION, INFLATION)
        inflate_ms = (time.perf_counter() - started) * 1000.0
        stats = run_case(blocked, args.runs, seed=args.seed)
        print(f'{label}: 膨胀耗时 {inflate_ms:.1f} ms，膨胀后障碍占比 '
              f'{blocked.mean()*100:.1f}%')
        print(f'    成功率 {stats["success_rate"]:.0f}%  '
              f'平均 {stats["mean_ms"]:.2f} ms  最大 {stats["max_ms"]:.2f} ms  '
              f'P95 {stats["p95_ms"]:.2f} ms')
        print(f'    平均路径 {stats["mean_m"]:.2f} m，'
              f'平均扩展 {stats["mean_exp"]:.0f} 个节点')

    print('\n定性用例：')
    for name, ok, detail in qualitative_cases():
        print(f'    [{"PASS" if ok else "FAIL"}] {name}: {detail}')


if __name__ == '__main__':
    main()
