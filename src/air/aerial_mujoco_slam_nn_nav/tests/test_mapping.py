"""占据栅格与扫描匹配测试（合成走廊）。"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from aerial_mujoco_slam_nn_nav.mapping import OccupancyGrid, Slam  # noqa: E402


def raycast_room(pose, walls, angles, max_range=8.0):
    """在解析房间里做射线求交，返回测距。"""
    out = []
    for a in angles:
        ang = pose[2] + a
        dx, dy = math.cos(ang), math.sin(ang)
        best = max_range
        for (x1, y1, x2, y2) in walls:
            # 线段求交
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((x1 - pose[0]) * ey - (y1 - pose[1]) * ex) / den
            u = ((x1 - pose[0]) * dy - (y1 - pose[1]) * dx) / den
            if t > 0 and 0 <= u <= 1:
                best = min(best, t)
        out.append(min(best, max_range))
    return np.array(out)


def main():
    walls = [(-3, 1.5, 3, 1.5), (-3, -1.5, 3, -1.5), (3, -1.5, 3, 1.5), (-3, -1.5, -3, 1.5)]
    angles = np.linspace(-math.pi, math.pi, 180, endpoint=False)
    grid = OccupancyGrid(size_x=8, size_y=6, res=0.05)
    slam = Slam(grid)
    slam.reset((0.0, 0.0, 0.0))
    rng = np.random.default_rng(0)
    pose = np.array([0.0, 0.0, 0.0])
    for _ in range(40):
        pose[0] += 0.06
        ranges = raycast_room(pose, walls, angles)
        ranges = np.clip(ranges + rng.normal(0, 0.01, ranges.shape), 0.05, 8.0)
        slam.update((0.06, 0.0, 0.0), ranges, angles)
    err = math.hypot(slam.pose[0] - pose[0], slam.pose[1] - pose[1])
    print(f"pose error {err*100:.1f} cm, coverage {grid.coverage():.3f}")
    assert err < 0.10, err
    assert grid.coverage() > 0.10

    # 占据栅格应能检测到右侧墙
    occ = grid.occupied(0.6)
    ys, xs = np.where(occ)
    assert len(xs) > 50
    print("mapping ok")


if __name__ == "__main__":
    main()

