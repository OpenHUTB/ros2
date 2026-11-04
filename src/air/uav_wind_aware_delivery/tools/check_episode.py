#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单集闭环调试：python check_episode.py wx wy [compensate]"""
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src"))
from uav_wind_aware_delivery.control import GeometricController  # noqa: E402
from uav_wind_aware_delivery.mission import DeliveryMission, MissionConfig  # noqa: E402
from uav_wind_aware_delivery.sim import DeliverySim, SimConfig  # noqa: E402


def main():
    wx = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    wy = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    comp = (sys.argv[3] != "0") if len(sys.argv) > 3 else True
    wind = np.array([wx, wy, 0.0])
    sim = DeliverySim(SimConfig(wind=wind, substeps=2))
    mission = DeliveryMission(
        MissionConfig(target=np.array([5.0, 2.0, 0.0]), compensate=comp,
                      release_alt=3.0, hover_time=0.8),
        model_path=os.path.join(ROOT, "models", "impact_mlp.npz"),
    )
    ctrl = GeometricController()
    released = False
    for i in range(5000):
        st = sim.state()
        out = mission.update(st)
        if out["release"] and not released:
            sim.release()
            released = True
            print(f"RELEASE t={st.t:.2f} pos={st.pos.round(3)} rel={mission.rel.round(3)} "
                  f"vel={st.vel.round(3)}")
        ctrl.act(sim, out["setpoint"], np.zeros(3))
        if i % 200 == 0:
            print(f"t={st.t:.2f} phase={out['phase']} p={st.pos.round(2)} "
                  f"v={st.vel.round(2)} payload={st.payload_pos.round(2)} "
                  f"pv={st.payload_vel.round(2)} landed={st.landed}")
        if mission.done:
            break
    st = sim.state()
    print("FINAL", mission.phase, "done", mission.done, "landed", st.landed,
          "released", st.released, "payload", st.payload_pos.round(3),
          "err", round(float(np.linalg.norm(st.payload_pos[:2] - np.array([5.0, 2.0]))), 3),
          "t", round(st.t, 2))


if __name__ == "__main__":
    main()
