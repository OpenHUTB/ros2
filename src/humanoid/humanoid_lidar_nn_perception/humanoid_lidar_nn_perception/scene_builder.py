"""Episode / scenario randomization.

Loads the bundled G1 scene (which already contains ground + fixed obstacles:
boxes, spheres, cylinders, capsules, ellipsoids). On each reset we randomize
the G1's starting (x, y, yaw) so the same static obstacles are scanned from
many different viewpoints -- this gives the CNN diverse training data without
needing a dynamic obstacle generator.
"""
from __future__ import annotations

import numpy as np
import mujoco


# bounds inside the 16x16 walled arena (walls at +/-8)
X_RANGE = (-6.0, 6.0)
Y_RANGE = (-6.0, 6.0)


def load_model(xml_path: str) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(xml_path)


def reset_episode(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    rng: np.random.Generator,
    start_xy: tuple[float, float] | None = None,
    start_yaw: float | None = None,
) -> None:
    """Reset data to the home keyframe, then perturb pelvis x/y/yaw."""
    mujoco.mj_resetDataKeyframe(model, data, 0)
    if start_xy is None:
        x = rng.uniform(*X_RANGE)
        y = rng.uniform(*Y_RANGE)
    else:
        x, y = start_xy
    if start_yaw is None:
        yaw = rng.uniform(-np.pi, np.pi)
    else:
        yaw = start_yaw
    # freejoint is qpos[0:7] = (x, y, z, qw, qx, qy, qz)
    data.qpos[0] = x
    data.qpos[1] = y
    # z stays at home keyframe (0.755)
    qw = np.cos(yaw / 2.0)
    qx = 0.0
    qy = 0.0
    qz = np.sin(yaw / 2.0)
    data.qpos[3:7] = [qw, qx, qy, qz]
    mujoco.mj_forward(model, data)
