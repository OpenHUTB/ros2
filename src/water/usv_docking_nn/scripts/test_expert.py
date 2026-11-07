"""诊断：单独测试靠泊专家控制器，打印每个种子的过程与结果。"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from sim_core import DockSim, thrust_from_twist, BERTH_X
from oracle import DockExpert, build_path


def run(seed, steps=1100, verbose=False):
    sim = DockSim(seed=seed)
    obs = sim.reset(seed=seed)
    ex = DockExpert(build_path(sim.start))
    for k in range(steps):
        if sim.collided:
            return "collision", k, sim.pose(), obs
        if sim.docked():
            return "docked", k, sim.pose(), obs
        v, w = ex.act(sim.pose(), sim.speed(), sim.yaw_rate())
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        if verbose and k % 50 == 0:
            x, y, yaw = sim.pose()
            e = obs["berth"]
            print("    k=%3d x=%6.2f y=%+6.2f yaw=%+5.2f v=%.2f | ey=%+5.2f dyaw=%+5.2f d=%5.2f"
                  % (k, x, y, yaw, sim.speed(), e[0], e[1], e[2]))
    return "timeout", steps, sim.pose(), obs


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    verbose = len(sys.argv) > 2 and sys.argv[2] == "v"
    stats = {"docked": 0, "collision": 0, "timeout": 0}
    for s in range(n):
        res, k, pose, obs = run(s, verbose=(verbose and s == 0))
        stats[res] += 1
        x, y, yaw = pose
        print("seed %2d -> %-9s k=%3d  x=%6.2f y=%+6.2f yaw=%+5.2f" % (s, res, k, x, y, yaw))
    print("\n汇总:", stats, " 成功率=%.1f%%" % (100.0 * stats["docked"] / n))
