"""ROV 物理与运动控制测试：定深悬停、前向、横移、偏航。"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rov_mujoco_slam_nn_nav.sim import ROVSim


def main():
    sim = ROVSim(scene="pool")
    p0 = sim.base_pos()
    assert abs(p0[2] - 1.0) < 0.06, p0
    sim.step_dt(1.0)
    print(f"定深 1s: dz={sim.base_pos()[2]-p0[2]:+.3f} m, roll={sim.rpy()[0]:+.3f}")
    assert abs(sim.base_pos()[2] - 1.0) < 0.08

    sim.set_command(surge=0.5)
    sim.step_dt(2.0)
    dx = sim.base_pos()[0] - p0[0]
    print(f"前向 2s: dx={dx:+.2f} m, vx={sim.velocity()[0]:.2f}")
    assert dx > 0.5 and not sim.is_fallen()

    sim.set_command(surge=0.0, sway=0.4)
    sim.step_dt(1.5)
    print(f"横移 1.5s: dy={sim.base_pos()[1]:+.2f} m")
    assert abs(sim.base_pos()[1]) > 0.2

    sim.set_command(sway=0.0, yaw_rate=0.6)
    sim.step_dt(1.0)
    print(f"偏航 1s: yaw={sim.rpy()[2]:.2f}")
    print("rov dynamics ok")


if __name__ == "__main__":
    main()
