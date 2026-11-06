#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_platoon.py —— 多车协同编队核心的无依赖测试
=====================================================
不依赖 CARLA / matplotlib，仅用 platoon_core（numpy）验证编队核心的确定性结论：
  1. 初始位姿互不重合（起步沿轨迹排开）；
  2. 稳态 leader-follower 间距有界（误差收敛到 spacing 附近，可用肉眼核对动图）；
  3. leader 有前视、能转向（最大横向偏移接近 S 形参考幅值）。

运行：python3 test_platoon.py   （任一断言失败则退出码非 0）
"""

import sys
import numpy as np

from platoon_core import Platoon, build_path_from_waypoints


def make_path():
    xs = np.linspace(0.0, 160.0, 400)
    mid, w = 80.0, 12.0
    ys = np.where(xs < mid - 20, 0.0,
                  np.where(xs > mid + 20, 0.0,
                           w * np.sin((xs - (mid - 20)) / 40.0 * np.pi)))
    return build_path_from_waypoints(np.column_stack([xs, ys])[::8],
                                     dense_per_m=2.0)


def test_initial_non_overlap():
    path, s = make_path()
    platoon = Platoon(path, s, n_vehicles=4, spacing=8.0, v_set=6.0,
                      lookahead=6.0, L=2.8, kp=1.2)
    st = platoon.initial_states()
    assert st.shape == (4, 3)
    min_gap = min(np.linalg.norm(st[i, :2] - st[j, :2])
                  for i in range(4) for j in range(i + 1, 4))
    assert min_gap >= 8.0 * 0.8, f"初始车距过小: {min_gap:.2f}m，起步重合"
    print(f"[PASS] 初始位姿互不重合（最小车距 {min_gap:.1f} m ≈ spacing 8m）")


def test_steady_spacing_bounded():
    path, s = make_path()
    platoon = Platoon(path, s, n_vehicles=4, spacing=8.0, v_set=6.0,
                      lookahead=6.0, L=2.8, kp=1.2)
    # 预热到稳态
    for _ in range(200):
        platoon.step(0.033)
    # 稳态采一段
    gaps = []
    for _ in range(100):
        st, _, _ = platoon.step(0.033)
        gaps.append(np.linalg.norm(st[0, :2] - st[1, :2]))
    gap = float(np.mean(gaps))
    assert abs(gap - 8.0) <= 1.5, f"稳态间距 {gap:.2f}m 超出误差带 (±1.5m)"
    print(f"[PASS] 稳态 L-F1 间距 {gap:.2f} m（目标 8 m，误差带 ±1.5 m）")


def test_leader_turns():
    path, s = make_path()
    platoon = Platoon(path, s, n_vehicles=4, spacing=8.0, v_set=6.0,
                      lookahead=6.0, L=2.8, kp=1.2)
    max_y = 0.0
    for _ in range(400):
        st, _, _ = platoon.step(0.033)
        max_y = max(max_y, float(st[0, 1]))
    assert max_y > 8.0, f"leader 最大横向偏移 {max_y:.1f}m < 8m，前视失效、未转向"
    print(f"[PASS] leader 有前视并能转向（最大横向偏移 {max_y:.1f} m，S 形幅值≈12m）")


if __name__ == "__main__":
    ok = True
    for fn in (test_initial_non_overlap, test_steady_spacing_bounded, test_leader_turns):
        try:
            fn()
        except AssertionError as e:
            ok = False
            print(f"[FAIL] {fn.__name__}: {e}")
    print("全部通过 ✓" if ok else "存在失败 ✗")
    sys.exit(0 if ok else 1)
