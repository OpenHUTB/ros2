#!/usr/bin/env python3
"""
任务 3 —— 神经网络规划器训练有效性验证
========================================
  1. Neural A* 加载训练得到的 planner_weights.npz；
  2. 与经典 A* 相比，在保持路径可行/近似最优的前提下显著减少扩展节点；
  3. 零权值消融使引导场退化为常数，节点削减优势消失；
  4. 局部避障策略加载 local_policy_weights.npz，左/右/前方障碍的转向方向正确；
  5. 零权值局部策略无法产生转向。

运行:
    ./main.sh --test-planning
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def build_map(size=200, res=0.1):
    """20 m x 20 m 地图，中间一堵带缺口的墙"""
    grid = np.zeros((size, size), dtype=np.int8)
    wx = int(size * 0.55)
    gap0, gap1 = int(size * 0.42), int(size * 0.58)
    grid[:, wx - 1:wx + 2] = 100
    grid[gap0:gap1, wx - 1:wx + 2] = 0
    origin = (-size * res / 2.0, -size * res / 2.0)
    return grid, origin, res


def path_min_clearance(path, grid, origin, res):
    ox, oy = origin
    best = 1e9
    for x, y in path:
        mx = int(round((x - ox) / res))
        my = int(round((y - oy) / res))
        if 0 <= my < grid.shape[0] and 0 <= mx < grid.shape[1]:
            if grid[my, mx] >= 50:
                return 0.0
            # 检查 3x3 邻域近似间隙
            patch = grid[max(0, my - 2):my + 3, max(0, mx - 2):mx + 3]
            if patch.max() >= 50:
                best = min(best, res * 1.0)
    return best


def main():
    from rov_mujoco.neural_path_planner import (
        BaselineAStarPlanner, NeuralAStarPlanner, NeuralLocalAvoidancePolicy,
        analytic_guidance,
    )

    print("=" * 68)
    print("  任务 3: 神经网络规划器与局部避障策略训练有效性验证")
    print("=" * 68)

    grid, origin, res = build_map()
    start = (origin[0] + 2.0, origin[1] + 2.0)
    goal = (origin[0] + 16.0, origin[1] + 16.0)

    # ── 1. 全局规划对比 ───────────────────────────────
    print("\n[1] Neural A* vs 经典 A* ...")
    baseline = BaselineAStarPlanner(resolution=res, clearance_m=0.3)
    neural = NeuralAStarPlanner(resolution=res, clearance_m=0.3)
    zero = NeuralAStarPlanner(resolution=res, clearance_m=0.3, zero_weights=True)

    print(f"    引导场网络: loaded={neural.guidance_loaded} source={neural.guidance_source}")
    assert neural.guidance_loaded, "planner_weights.npz 未加载"
    assert neural.guidance_source.endswith(".npz"), "应加载训练权值而非解析回退"

    p_base = baseline.plan(grid, origin, start, goal)
    n_base = baseline.expanded_nodes
    t_base = baseline.planning_time_ms

    p_nn = neural.plan(grid, origin, start, goal)
    n_nn = neural.expanded_nodes
    t_nn = neural.planning_time_ms

    p_zero = zero.plan(grid, origin, start, goal)
    n_zero = zero.expanded_nodes
    t_zero = zero.planning_time_ms

    def plen(p):
        return sum(math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1])
                   for i in range(len(p) - 1))

    print(f"    经典 A*    : 扩展={n_base:5d} | 路径长={plen(p_base):.2f} m | {t_base:.1f} ms")
    print(f"    Neural A*  : 扩展={n_nn:5d} | 路径长={plen(p_nn):.2f} m | {t_nn:.1f} ms")
    print(f"    零权值消融 : 扩展={n_zero:5d} | 路径长={plen(p_zero):.2f} m | {t_zero:.1f} ms")
    print(f"    节点削减   : Neural {(1 - n_nn / n_base) * 100:.1f}% | "
          f"零权值 {(1 - n_zero / n_base) * 100:.1f}%")

    assert plen(p_nn) < plen(p_base) * 1.25, "Neural A* 路径显著劣于经典 A*"
    assert math.hypot(p_nn[-1][0] - goal[0], p_nn[-1][1] - goal[1]) < 0.5, "未到达目标"
    assert n_nn < 0.85 * n_base, "Neural A* 未减少扩展节点"

    # 引导场本身必须由训练网络复现专家先验；零权值网络应输出常数 0.5。
    ox, oy = origin
    sx_i = int(round((start[0] - ox) / res))
    sy_i = int(round((start[1] - oy) / res))
    gx_i = int(round((goal[0] - ox) / res))
    gy_i = int(round((goal[1] - oy) / res))
    inflated = neural._inflate_obstacles(grid, neural.clearance_cells)
    phi_nn = neural._compute_neural_guidance_field(
        grid.shape, sx_i, sy_i, gx_i, gy_i, inflated)
    phi_zero = zero._compute_neural_guidance_field(
        grid.shape, sx_i, sy_i, gx_i, gy_i, inflated)
    phi_ref = analytic_guidance(grid.shape[0], grid.shape[1],
                                sx_i, sy_i, gx_i, gy_i, inflated)
    free = ~inflated
    corr = float(np.corrcoef(phi_nn[free], phi_ref[free])[0, 1])
    mse = float(np.mean((phi_nn[free] - phi_ref[free]) ** 2))
    print(f"    引导场复现: corr={corr:.5f}, MSE={mse:.6f}, "
          f"网络 std={phi_nn[free].std():.4f}, 零权值 std={phi_zero[free].std():.6f}")
    assert corr > 0.95 and mse < 1e-3, "训练网络未复现专家引导场"
    assert phi_nn[free].std() > 0.05, "训练引导场缺乏空间结构"
    assert phi_zero[free].std() < 1e-6, "零权值引导场应为常数"

    # ── 2. 局部避障策略 ───────────────────────────────
    print("\n[2] 局部避障策略网络 ...")
    policy = NeuralLocalAvoidancePolicy(safe_margin=0.65, max_speed=0.55)
    policy_zero = NeuralLocalAvoidancePolicy(safe_margin=0.65, max_speed=0.55,
                                             zero_weights=True)
    print(f"    策略网络: loaded={policy.weights_loaded} source={policy.weights_source}")
    assert policy.weights_loaded and policy.weights_source.endswith(".npz")

    def scan(beam_slice=None, dist=0.5):
        r = [10.0] * 72
        if beam_slice is not None:
            for b in range(*beam_slice):
                r[b] = dist
        return r

    pos, yaw, target = (0.0, 0.0), 0.0, (3.0, 0.0)

    free = policy.compute_cmd(scan(None), pos, yaw, target)
    front = policy.compute_cmd(scan((30, 42)), pos, yaw, target)
    left = policy.compute_cmd(scan((54, 66)), pos, yaw, target)
    right = policy.compute_cmd(scan((6, 18)), pos, yaw, target)

    for name, c in [("无遮挡", free), ("正前方", front), ("左前方", left), ("右前方", right)]:
        print(f"    {name:<6}: vx={c['vx']:+.3f} m/s, vy={c['vy']:+.3f}, "
              f"wz={c['wz']:+.3f} rad/s")

    assert free["vx"] > 0.35, "无遮挡时未保持巡航速度"
    assert abs(free["wz"]) < 0.10, "无遮挡时不应持续转向"
    assert front["vx"] < free["vx"], "正前方近障未减速"
    assert front["wz"] > 0.03, "正前方近障未转向"
    assert left["wz"] < 0.0 < right["wz"], "左右障碍转向方向错误"

    zc = policy_zero.compute_cmd(scan((30, 42)), pos, yaw, target)
    print(f"    零权值策略: vx={zc['vx']:+.3f} m/s, wz={zc['wz']:+.3f} rad/s")
    assert abs(zc["wz"]) < 1e-6, "零权值策略不应产生转向"
    assert abs(front["wz"]) > 0.03, "训练策略的转向能力未体现"

    print("\n✅ 规划与局部避障网络测试全部通过。")
    os._exit(0)


if __name__ == "__main__":
    main()
