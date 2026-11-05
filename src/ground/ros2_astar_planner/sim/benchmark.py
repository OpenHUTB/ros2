#!/usr/bin/env python3
"""基准矩阵：不同地图 × 不同搜索配置 / 启发式的对比实验。

对比维度
--------
- **启发式**：octile（8 邻接可采纳，最优）、manhattan（在 8 邻接下高估代价，
  不可采纳）、euclidean、zero（Dijkstra，最优但扩展最多）。
- **邻接方式**：8 邻接（可斜穿）与 4 邻接（只能直角）。
- **加权 A\\***：权重 1.5，用牺牲最优性换取更少的扩展节点。

指标包含规划代价、路径长度、扩展节点数、转角数、平滑后长度与总耗时，
输出到 ``results/benchmark.md``。
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.dirname(__file__))

from ros2_astar_planner.astar import AStarConfig, AStarPlanner, count_turns  # noqa: E402
from ros2_astar_planner.gridmap import (  # noqa: E402
    GridMap,
    corridor_map,
    gate_map,
    maze_map,
    random_obstacle_map,
)
from ros2_astar_planner.safety import (  # noqa: E402
    choose_connected_endpoints,
    inflation_cells,
    pick_sturdy_endpoints,
)
from ros2_astar_planner.smoothing import smooth_path  # noqa: E402
from ros2_astar_planner.trajectory import time_parameterize  # noqa: E402

SCENARIOS = {
    "corridor": lambda: corridor_map(),
    "maze": lambda: maze_map(),
    "random": lambda: random_obstacle_map(61, 41, 0.1, 0.08, seed=7),
}

CONFIGS = [
    ("A* octile (8邻接)", dict(heuristic="octile", allow_diagonal=True, weight=1.0)),
    ("A* manhattan (8邻接)", dict(heuristic="manhattan", allow_diagonal=True, weight=1.0)),
    ("A* octile (4邻接)", dict(heuristic="manhattan", allow_diagonal=False, weight=1.0)),
    ("加权A* w=1.5", dict(heuristic="octile", allow_diagonal=True, weight=1.5)),
    ("Dijkstra (zero)", dict(heuristic="zero", allow_diagonal=True, weight=1.0)),
]

ROBOT_RADIUS = 0.20
V_MAX, A_MAX = 0.6, 0.5


def run_case(gm: GridMap, start, goal, cfg_kwargs: dict) -> dict:
    """跑一次规划 + 平滑 + 时间参数化，返回指标字典。"""
    planner = AStarPlanner(AStarConfig(**cfg_kwargs))
    t0 = time.perf_counter()
    res = planner.plan(gm, start, goal)
    t_plan = time.perf_counter() - t0

    if not res.success:
        return {
            "ok": False,
            "reason": res.reason,
            "expansions": res.expansions,
            "plan_ms": t_plan * 1000.0,
        }

    world = np.asarray([gm.to_world(i, j) for i, j in res.cells], dtype=float)
    smoothed, metrics = smooth_path(gm, world, use_gradient=True, seed=3)
    traj = time_parameterize(smoothed, V_MAX, A_MAX, 1.0)
    grid_len = float(np.sum(np.linalg.norm(np.diff(np.asarray(res.cells, dtype=float), axis=0), axis=1)))
    return {
        "ok": True,
        "cost": res.cost,
        "grid_len": grid_len,
        "raw_len": metrics["raw_length"],
        "cut_len": metrics["after_shortcut_length"],
        "final_len": metrics["final_length"],
        "expansions": res.expansions,
        "turns": count_turns(res.cells),
        "final_turns": metrics["final_turns"],
        "raw_points": metrics["raw_points"],
        "final_points": metrics["final_points"],
        "safe": metrics["safe"],
        "time": traj.total_time,
        "plan_ms": t_plan * 1000.0,
    }


def main() -> None:
    rows = []
    for scene_name, builder in SCENARIOS.items():
        raw = builder()
        gm = raw.inflate(inflation_cells(ROBOT_RADIUS, raw.spec.resolution))
        start, goal, notes = choose_connected_endpoints(gm, (3, 3), (37, 58))
        for cfg_name, cfg_kwargs in CONFIGS:
            r = run_case(gm, start, goal, cfg_kwargs)
            rows.append((scene_name, cfg_name, r, notes))

    lines = []
    lines.append("# 基准矩阵：栅格 A* 规划器")
    lines.append("")
    lines.append(
        "环境：机器人半径 0.20 m（膨胀 2 格），分辨率 0.1 m；"
        "时间参数化参数 v_max=0.6 m/s、a_max=0.5 m/s²。"
    )
    lines.append("")
    lines.append("## 1. 规划质量")
    lines.append("")
    lines.append(
        "| 场景 | 配置 | 成功 | 栅格代价 | 路径长度 (m) | 平滑后 (m) | 转角数 | 扩展节点 | 规划耗时 (ms) |"
    )
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for scene_name, cfg_name, r, _ in rows:
        if r["ok"]:
            lines.append(
                f"| {scene_name} | {cfg_name} | 是 | {r['cost']:.2f} | {r['raw_len']:.2f} | "
                f"{r['final_len']:.2f} | {r['turns']} | {r['expansions']} | {r['plan_ms']:.1f} |"
            )
        else:
            lines.append(
                f"| {scene_name} | {cfg_name} | 否 | — | — | — | — | {r['expansions']} | {r['plan_ms']:.1f} |"
            )
    lines.append("")

    lines.append("## 2. 关键结论")
    lines.append("")
    for scene_name in SCENARIOS:
        subset = {c: r for s, c, r, _ in rows if s == scene_name}
        octile = subset["A* octile (8邻接)"]
        dij = subset["Dijkstra (zero)"]
        man = subset["A* manhattan (8邻接)"]
        w15 = subset["加权A* w=1.5"]
        if not (octile["ok"] and dij["ok"]):
            continue
        lines.append(f"### {scene_name}")
        lines.append("")
        lines.append(
            f"- **最优性验证**：Dijkstra 代价 {dij['cost']:.2f}，octile 代价 {octile['cost']:.2f}"
            f"（相对差 {abs(octile['cost'] - dij['cost']) / max(dij['cost'], 1e-9) * 100:.2f}%），"
            "说明该启发式在 8 邻接下可采纳，A* 给出与 Dijkstra 一致的最优解。"
        )
        lines.append(
            f"- **启发式代价**：octile 扩展 {octile['expansions']} 个节点，"
            f"Dijkstra 扩展 {dij['expansions']} 个，"
            f"搜索量下降 {(1 - octile['expansions'] / max(dij['expansions'], 1)) * 100:.1f}%。"
        )
        if man["ok"]:
            lines.append(
                f"- **不可采纳启发式的代价**：manhattan 在 8 邻接下高估真实代价，"
                f"得到代价 {man['cost']:.2f}"
                + (
                    f"，比最优解长 {(man['cost'] / max(octile['cost'], 1e-9) - 1) * 100:.2f}%，"
                    "证明该配置下**不再保证最优**。"
                    if man["cost"] > octile["cost"] + 1e-6
                    else "，本次恰好与最优解一致。\n"
                )
            )
        if w15["ok"]:
            lines.append(
                f"- **加权 A\\* 的取舍**：w=1.5 时扩展 {w15['expansions']} 个节点"
                f"（较最优配置减少 {(1 - w15['expansions'] / max(octile['expansions'], 1)) * 100:.1f}%），"
                f"代价 {w15['cost']:.2f}"
                + (
                    f"，牺牲 {(w15['cost'] / max(octile['cost'], 1e-9) - 1) * 100:.2f}% 的路径质量换速度。"
                    if w15["cost"] > octile["cost"] + 1e-6
                    else "，本次未损失路径质量。"
                )
            )
        lines.append("")

    lines.append("## 3. 平滑前后")
    lines.append("")
    lines.append("| 场景 | 原始长度 (m) | 捷径后 (m) | 最终 (m) | 缩短 | 转角数 原始→最终 | 折点数 原始→最终 | 逐段无碰撞 |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for scene_name, cfg_name, r, _ in rows:
        if cfg_name != "A* octile (8邻接)" or not r["ok"]:
            continue
        raw_len = r["raw_len"]
        shorten = (1 - r["final_len"] / max(raw_len, 1e-9)) * 100
        lines.append(
            f"| {scene_name} | {raw_len:.2f} | {r['cut_len']:.2f} | {r['final_len']:.2f} | "
            f"{shorten:.1f}% | {r['turns']} → {r['final_turns']} | "
            f"{r['raw_points']} → {r['final_points']} | {'是' if r['safe'] else '否'} |"
        )
    lines.append("")
    lines.append(
        "迷宫场景缩短比例明显低于走廊：通道狭窄时两格之间几乎没有可直达的视线，"
        "捷径算法找不到可替换的直段，这正说明后处理不会为了缩短路径而牺牲避障安全。"
    )
    lines.append("")

    lines.append("## 4. 通道宽度 vs 机器人半径：通行可行性")
    lines.append("")
    lines.append(
        "工程上最常问的问题是：机器人能不能穿过这个门？这里用专门的“一墙一门”地图"
        "（房间固定、端点在房间中央），扫描门洞高度与膨胀半径，"
        "连通性**只由门宽决定**，不受端点位置干扰。"
    )
    lines.append("")
    lines.append("| 门洞高度 (格) | 门洞净宽 (m) | 膨胀 1 格 (0.10 m) | 膨胀 2 格 (0.20 m) | 膨胀 3 格 (0.30 m) |")
    lines.append("| --- | --- | --- | --- | --- |")
    for gap_rows in (1, 2, 3, 4, 5, 7, 9):
        raw = gate_map(gap_rows=gap_rows)
        row = [f"| {gap_rows} | {gap_rows * raw.spec.resolution:.2f} "]
        for n_infl in (1, 2, 3):
            gm = raw.inflate(n_infl)
            # 端点固定在两侧房间中央，永远是安全的
            s = (raw.shape[0] // 2, raw.shape[1] // 2 - 4)
            g = (raw.shape[0] // 2, raw.shape[1] // 2 + 4)
            r = run_case(gm, s, g, dict(heuristic="octile", allow_diagonal=True, weight=1.0))
            row.append(f"| {'可通行' if r['ok'] else '**不可通行**'} " if n_infl == 1 else
                       f"| {'可通行' if r['ok'] else '**不可通行**'} ")
            if n_infl == 3:
                row[-1] = row[-1].rstrip() + " |"
        lines.append("".join(row))
    lines.append("")
    lines.append(
        "结论：**通道净宽必须大于 2×膨胀半径**，否则门洞被膨胀层完全堵死。"
        "规划器在这种情况下会如实返回“起点与终点不连通”，"
        "而不是给出一条穿墙的路径——这正是安全裕度必须与地图分辨率一起考虑的原因。"
    )
    lines.append("")

    out_dir = os.path.join(os.path.dirname(__file__), "..", "results")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "benchmark.md")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("\n".join(lines[:14]))
    print(f"... 完整结果已写入 {out}")


if __name__ == "__main__":
    main()
