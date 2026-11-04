"""开发期探针：步态行走测试。"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from humanoid_mujoco_slam_nn_nav import model as M  # noqa: E402
from humanoid_mujoco_slam_nn_nav.gait import Walker, GaitParams  # noqa: E402
from humanoid_mujoco_slam_nn_nav.sim import HumanoidSim  # noqa: E402


def run(v=0.22, w=0.0, seconds=10.0, scene="warehouse", seed=0, verbose=True):
    sim = HumanoidSim(scene=scene, seed=seed)
    sx, sy, syaw = sim.layout.start
    walker = Walker(GaitParams(), sx, sy, syaw)
    sim.reset()
    dt = 0.01
    walker.command(v, w)
    log = []
    t = 0.0
    while t < seconds:
        st = sim.state()
        q = walker.update(dt, st["roll"], st["pitch"])
        sim.set_targets(q)
        sim.step(int(round(dt / sim.model.opt.timestep)))
        t += dt
        if sim.is_fallen():
            break
        if verbose and abs(t % 1.0) < dt / 2:
            log.append((round(t, 1), round(st["pos"][0], 3), round(st["pos"][1], 3),
                        round(st["pos"][2], 3), round(st["yaw"], 3)))
    st = sim.state()
    out = {
        "t": round(t, 2),
        "dx": round(st["pos"][0] - sx, 3),
        "dy": round(st["pos"][1] - sy, 3),
        "z": round(st["pos"][2], 3),
        "yaw": round(st["yaw"], 3),
        "fallen": sim.is_fallen(),
        "vy": round(st["vel"][1], 3),
    }
    if verbose:
        for row in log:
            print("t=%4.1f x=%7.3f y=%7.3f z=%6.3f yaw=%6.3f" % row)
        print(out)
    return out


if __name__ == "__main__":
    run()
