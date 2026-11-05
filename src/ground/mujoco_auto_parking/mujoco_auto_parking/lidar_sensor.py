"""2D LiDAR simulated with MuJoCo ray casting."""

import math
from typing import Dict, Tuple

import numpy as np


class Lidar2D:
    """Ray-cast a horizontal fan of beams around the chassis."""

    def __init__(
        self,
        model,
        data,
        body_name: str = 'chassis',
        num_beams: int = 120,
        fov_deg: float = 360.0,
        min_range: float = 0.12,
        max_range: float = 8.0,
        sensor_offset: Tuple[float, float, float] = (0.0, 0.0, 0.15),
        noise_std: float = 0.0,
        seed: int = 0,
    ) -> None:
        import mujoco

        self._mujoco = mujoco
        self.model = model
        self.data = data
        self.body_id = int(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        )
        if self.body_id < 0:
            raise ValueError(f'body not found: {body_name}')
        self.num_beams = int(num_beams)
        self.fov_rad = math.radians(float(fov_deg))
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.sensor_offset = np.asarray(sensor_offset, dtype=np.float64)
        self.noise_std = float(noise_std)
        self._rng = np.random.default_rng(seed)

        self.beam_angles = np.linspace(
            -0.5 * self.fov_rad,
            0.5 * self.fov_rad,
            self.num_beams,
            endpoint=False,
        )
        if self.num_beams > 0:
            self.angle_increment = self.fov_rad / self.num_beams
        else:
            self.angle_increment = 0.0
        self.local_dirs = np.column_stack([
            np.cos(self.beam_angles),
            np.sin(self.beam_angles),
            np.zeros_like(self.beam_angles),
        ])
        self.ranges = np.full(self.num_beams, self.max_range, dtype=np.float32)

    @property
    def angle_min(self) -> float:
        return -0.5 * self.fov_rad

    @property
    def angle_max(self) -> float:
        return 0.5 * self.fov_rad - self.angle_increment

    def scan(self) -> Dict[str, np.ndarray]:
        """Return ranges (N,) and the world-frame sensor origin."""
        position = self.data.xpos[self.body_id]
        rotation = self.data.xmat[self.body_id].reshape(3, 3)
        origin = position + rotation @ self.sensor_offset
        world_dirs = (rotation @ self.local_dirs.T).T
        geomid = np.zeros(1, dtype=np.int32)
        ranges = np.full(self.num_beams, self.max_range, dtype=np.float32)
        for index, direction in enumerate(world_dirs):
            distance = self._mujoco.mj_ray(
                self.model,
                self.data,
                origin,
                direction,
                None,
                1,
                self.body_id,
                geomid,
            )
            if self.min_range <= distance <= self.max_range:
                ranges[index] = distance
        if self.noise_std > 0.0:
            ranges += self._rng.normal(
                0.0, self.noise_std, size=ranges.shape
            ).astype(np.float32)
            ranges = np.clip(ranges, self.min_range, self.max_range)
        self.ranges = ranges
        return {
            'ranges': ranges.copy(),
            'origin': origin.copy(),
            'angles': self.beam_angles.copy(),
        }

    def create_laserscan_msg(self, header):
        from sensor_msgs.msg import LaserScan

        message = LaserScan()
        message.header = header
        message.header.frame_id = 'laser_link'
        message.angle_min = float(self.angle_min)
        message.angle_max = float(self.angle_max)
        message.angle_increment = float(self.angle_increment)
        message.time_increment = 0.0
        message.scan_time = 0.0
        message.range_min = float(self.min_range)
        message.range_max = float(self.max_range)
        message.ranges = [float(value) for value in self.ranges]
        return message
