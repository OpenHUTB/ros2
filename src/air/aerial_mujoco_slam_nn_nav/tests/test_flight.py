"""飞行控制测试：悬停、前飞、爬升、偏航。"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from aerial_mujoco_slam_nn_nav.sim import DroneSim


def main():
    sim = DroneSim(scene="warehouse")
    p0 = sim.base_pos()
    assert abs(p0[2] - 1.2) < 0.05, p0
    sim.set_command(vx=0.0)
    sim.step_dt(1.0)
    p1 = sim.base_pos()
    print(f"hover 1s: dz={p1[2]-p0[2]:+.3f} m")
    assert abs(p1[2] - 1.2) < 0.06

    sim.set_command(vx=0.6)
    sim.step_dt(2.0)
    print(f"forward 2s: dx={sim.base_pos()[0]-p1[0]:+.2f} m, vel={sim.velocity()[0]:.2f}")
    assert sim.base_pos()[0] - p1[0] > 0.5
    assert not sim.is_fallen()

    sim.set_command(vx=0.0, z_ref=1.8)
    sim.step_dt(1.5)
    print(f"climb: z={sim.base_pos()[2]:.2f} m")
    assert sim.base_pos()[2] > 1.6
    print("flight ok")


if __name__ == "__main__":
    main()
