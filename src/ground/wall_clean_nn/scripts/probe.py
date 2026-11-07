"""探针：沿墙清扫。"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from sim_core import CoverSim, thrust_from_twist, START
from oracle import build_wall_path, WallExpert

sim = CoverSim(seed=0)
path, lap = build_wall_path(sim.obstacles, START[:2])
L = float(np.sum(np.hypot(np.diff(path[:,0]), np.diff(path[:,1]))))
print("路径点数 %d  长度 %.1f m  沿墙栅格 %d" % (len(path), L, int(sim.wall_mask.sum())))

obs = sim.reset(START)
ex = WallExpert(path)
errs = []
for k in range(12000):
    v, w = ex.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["scan"])
    tl, tr = thrust_from_twist(v, w)
    obs = sim.step(tl, tr)
    errs.append(abs(sim.wall_error() - 0.85))
    if k % 1000 == 0:
        print("k=%5d pose=(%5.2f,%5.2f) 沿墙覆盖=%.1f%% 贴墙误差=%.2f m v=%.2f" %
              (k, sim.pose()[0], sim.pose()[1], 100*sim.wall_coverage(), errs[-1], sim.speed()))
    if sim.collided:
        print("COLLIDED at", k, np.round(sim.pose(),2)); break
    if ex.k >= len(path)-1 and sim.speed() < 0.05:
        print("FINISHED at", k); break
print("最终: 沿墙覆盖率 %.1f%%  平均贴墙误差 %.3f m  步数 %d  碰撞 %s" %
      (100*sim.wall_coverage(), float(np.mean(errs)), k, sim.collided))
