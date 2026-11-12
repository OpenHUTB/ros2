"""探针：狭窄通道通行专家，随机任务统计。"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from sim_core import GapSim, thrust_from_twist, MIN_PASSABLE
from oracle import build_path, GapExpert, passable_gaps

print("可通行开口:", [(round(g["center"],2), round(g["width"],2)) for g in passable_gaps()])
res = {}
steps = []
for seed in range(10):
    rng = np.random.default_rng(seed)
    sim = GapSim(seed=seed)
    obs = sim.reset(None, None, rng)
    path, gap = build_path(sim.start, sim.goal)
    ex = GapExpert(path)
    out = "timeout"
    for k in range(1500):
        v, w = ex.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["scan"])
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        if sim.collided: out = "collision"; break
        if sim.reached(): out = "reached"; break
    res[out] = res.get(out, 0) + 1
    steps.append(k)
    print("seed %2d goal=(%4.1f,%5.2f) 选开口 %.2f m(宽%.2f) -> %-9s k=%4d d=%.2f" %
          (seed, sim.goal[0], sim.goal[1], gap["center"] if gap else float('nan'),
           gap["width"] if gap else 0, out, k, sim.goal_distance()))
print("汇总:", res, " 平均步数 %.0f" % np.mean(steps))
