#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""最小冒烟测试：加载 XML、悬停、释放物资。"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from uav_wind_aware_delivery.control import GeometricController  # noqa: E402
from uav_wind_aware_delivery.sim import DeliverySim, SimConfig  # noqa: E402


def main():
    sim = DeliverySim(SimConfig(wind=np.zeros(3)))
    ctrl = GeometricController()
    print("mass", sim.mass, "uav", sim.uav_mass, "payload", sim.payload_mass, "dt", sim.dt)
    p_d = np.array([0.0, 0.0, 1.8])
    for _ in range(400):
        ctrl.act(sim, p_d, np.zeros(3))
    st = sim.state()
    print("hover", st.pos.round(3), st.vel.round(3), "payload", st.payload_pos.round(3),
          "euler", st.euler.round(3))
    assert np.linalg.norm(st.pos - p_d) < 0.12, "hover error too large"
    sim.release()
    for _ in range(200):
        ctrl.act(sim, p_d, np.zeros(3))
    st = sim.state()
    print("after_release", st.payload_pos.round(3), st.payload_vel.round(3), "landed", st.landed)
    print("OK")


if __name__ == "__main__":
    main()
