"""2D horizontal LiDAR simulated with mujoco.mj_ray.

Casts `num_beams` rays in the XY plane from the pelvis height. Rays that hit
nothing return `max_range`. The robot's own geoms are excluded via
`bodyexclude` (passing the pelvis body id automatically excludes its whole
kinematic subtree).
"""
from __future__ import annotations

import numpy as np
import mujoco


class LiDAR2D:
    def __init__(
        self,
        model: mujoco.MjModel,
        num_beams: int = 360,
        max_range: float = 8.0,
        height_offset: float = 0.3,   # lift ray origin above pelvis to avoid self-hits
        robot_body_name: str = "pelvis",
    ):
        self.num_beams = num_beams
        self.max_range = max_range
        self.angles = np.linspace(-np.pi, np.pi, num_beams, endpoint=False)
        self.dir_x = np.cos(self.angles)
        self.dir_y = np.sin(self.angles)
        self.height_offset = height_offset
        self.body_id = model.body(robot_body_name).id
        # reusable output buffer
        self._geomid = np.zeros(1, dtype=np.int32)

    def scan(self, data: mujoco.MjData) -> np.ndarray:
        """Return ranges[num_beams] in meters. No hit -> max_range."""
        origin = data.xpos[self.body_id].copy()
        origin[2] += self.height_offset
        ranges = np.full(self.num_beams, self.max_range, dtype=np.float32)
        for i in range(self.num_beams):
            d = mujoco.mj_ray(
                data.model,
                data,
                origin,
                np.array([self.dir_x[i], self.dir_y[i], 0.0]),
                None,
                1,           # flg_static: hit static geoms too
                self.body_id,
                self._geomid,
            )
            if d >= 0.0:
                ranges[i] = float(d)
        return ranges
