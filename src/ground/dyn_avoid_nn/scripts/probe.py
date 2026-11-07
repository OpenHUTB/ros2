"""探针：移动障碍避让专家，随机起点统计。"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from sim_core import DynSim, GOAL, thrust_from_twist
from oracle import build_path, DynExpert

sim0 = DynSim(seed=0)
path0 = build_path(sim0.obstacles, (-7.0,-5.0), GOAL)
print("参考路径 %d 点  长度 %.1f m" % (len(path0), float(np.sum(np.hypot(np.diff(path0[:,0]), np.diff(path0[:,1]))))))
res = {}
steps = []
for seed in range(10):
    rng = np.random.default_rng(seed)
    sim = DynSim(seed=seed)
    obs = sim.reset(None, rng)
    path = build_path(sim.obstacles, sim.pose()[:2], GOAL)
    ex = DynExpert(path)
    out = "timeout"
    for k in range(1500):
        v, w = ex.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["movers"], obs["scan"])
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        if sim.collided: out = "collision"; break
        if sim.reached(): out = "reached"; break
    res[out] = res.get(out, 0) + 1
    steps.append(k)
    print("seed %2d -> %-9s k=%4d d=%.2f" % (seed, out, k, sim.goal_distance()))
print("汇总:", res, " 平均步数 %.0f" % np.mean(steps))
