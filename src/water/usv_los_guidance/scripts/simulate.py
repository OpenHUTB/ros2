# -*- coding: utf-8 -*-
"""LOS 制导仿真入口：跟踪一段折线航点，画出船轨迹与横漂误差。"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from los_model import simulate  # noqa: E402
from simple_plot import plot_series  # noqa: E402

WAYPOINTS = [(0.0, 0.0), (12.0, 0.0), (12.0, 8.0)]
START = (0.0, 3.0)


def run():
    return simulate(WAYPOINTS, START)


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    traj, cte, ts = run()
    path_x = [p[0] for p in WAYPOINTS]
    path_y = [p[1] for p in WAYPOINTS]
    traj_x = [p[0] for p in traj]
    traj_y = [p[1] for p in traj]

    # 俯视轨迹图：路径（灰虚线用实线近似）与船轨迹（红）
    path_series = [
        ("path", path_x, path_y, (140, 140, 140)),
        ("USV", traj_x, traj_y, (200, 40, 40)),
    ]
    path1 = os.path.join(out_dir, "los_path.png")
    plot_series(path1, path_series, title="USV LOS path following",
                width=900, height=560, margin=(60, 30, 30, 150),
                ylabel="y (m)")

    # 横漂误差随时间衰减
    cte_series = [("cross-track error", ts, cte, (30, 90, 200))]
    path2 = os.path.join(out_dir, "los_crosstrack.png")
    plot_series(path2, cte_series, title="Cross-track error", width=900, height=560,
                margin=(60, 30, 30, 150), ylabel="y_e (m)")

    print("末位置: (%.2f, %.2f)" % traj[-1])
    print("最终横漂: %.4f m" % abs(cte[-1]))
    print("已生成: %s  %s" % (path1, path2))
    return path1, path2


if __name__ == "__main__":
    main(os.path.dirname(__file__))
