"""开发期探针：IK 自检 + 站立稳定性。"""
import os
import sys
import math

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from humanoid_mujoco_slam_nn_nav import kinematics as K  # noqa: E402
from humanoid_mujoco_slam_nn_nav import model as M  # noqa: E402
from humanoid_mujoco_slam_nn_nav.sim import HumanoidSim  # noqa: E402


def ik_selftest(sim):
    pelvis, q = K.default_stand((0.0, 0.0), 0.0)
    print("stand pelvis", np.round(pelvis, 4))
    max_err = 0.0
    for s in ("l", "r"):
        qs = {k.split("_")[0] + "_" + k.split("_")[1]: v for k, v in q.items() if k.endswith("_" + s)}
        qs = {"hip_roll": q[f"hip_roll_{s}"], "hip_pitch": q[f"hip_pitch_{s}"],
              "knee_pitch": q[f"knee_pitch_{s}"],
              "ankle_pitch": q[f"ankle_pitch_{s}"], "ankle_roll": q[f"ankle_roll_{s}"]}
        ankle, r = K.leg_fk(s, qs, pelvis, 0.0)
        print(s, "fk ankle", np.round(ankle, 4), "R_z", np.round(r[:, 2], 3))
    # 随机位姿 IK 往返
    rng = np.random.default_rng(0)
    for _ in range(200):
        pelvis = np.array([rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(0.72, 0.88)])
        yaw = rng.uniform(-1, 1)
        s = "l" if rng.random() < 0.5 else "r"
        hip = K.hip_position(s, pelvis, yaw)
        target = hip + np.array([rng.uniform(-0.25, 0.25), rng.uniform(-0.12, 0.12),
                                 -rng.uniform(0.45, 0.66)])
        q = K.leg_ik(s, pelvis, yaw, target)
        ankle, _ = K.leg_fk(s, q, pelvis, yaw)
        max_err = max(max_err, float(np.linalg.norm(ankle - target)))
    print("IK roundtrip max err", round(max_err, 5), "m")


def stand_test(sim, seconds=4.0):
    pelvis, q = K.default_stand((sim.layout.start[0], sim.layout.start[1]), sim.layout.start[2])
    sim.reset()
    for _ in range(int(seconds / sim.model.opt.timestep)):
        sim.step(1)
    st = sim.state()
    print("after", seconds, "s pos", np.round(st["pos"], 3), "rpy",
          [round(v, 3) for v in (st["roll"], st["pitch"], st["yaw"])])
    print("fallen", st["fallen"], "contacts", sim.foot_contacts())
    for s in ("l", "r"):
        print(s, "ankle", np.round(sim.ankle_pos(s), 3))


def main():
    layout = M.make_layout("warehouse", 0)
    sim = HumanoidSim(layout=layout)
    ik_selftest(sim)
    stand_test(sim)


if __name__ == "__main__":
    main()
