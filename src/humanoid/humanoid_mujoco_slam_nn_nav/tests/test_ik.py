"""逆运动学 / 正运动学往返测试。"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from humanoid_mujoco_slam_nn_nav import kinematics as K  # noqa: E402


def main():
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(300):
        pelvis = np.array([rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(0.68, 0.80)])
        yaw = rng.uniform(-1.2, 1.2)
        side = "l" if rng.random() < 0.5 else "r"
        hip = K.hip_position(side, pelvis, yaw)
        target = hip + np.array([rng.uniform(-0.2, 0.2), rng.uniform(-0.1, 0.1),
                                 -rng.uniform(0.45, 0.62)])
        q = K.leg_ik(side, pelvis, yaw, target)
        ankle, rot = K.leg_fk(side, q, pelvis, yaw)
        worst = max(worst, float(np.linalg.norm(ankle - target)))
    print(f"IK roundtrip max error {worst*1000:.2f} mm")
    assert worst < 0.02, worst

    # 站立姿态脚底应贴地
    pelvis, q = K.default_stand((0.0, 0.0), 0.0)
    for side in ("l", "r"):
        qs = {k: q[f"{k}_{side}"] for k in ("hip_yaw", "hip_roll", "hip_pitch",
                                            "knee_pitch", "ankle_pitch", "ankle_roll")}
        ankle, rot = K.leg_fk(side, qs, pelvis, 0.0)
        assert abs(ankle[2] - K.stand_ankle_z()) < 1e-6
        up = rot @ np.array([0.0, 0.0, 1.0])
        assert up[2] > 0.999, up
    print("stand pose ok")


if __name__ == "__main__":
    main()
