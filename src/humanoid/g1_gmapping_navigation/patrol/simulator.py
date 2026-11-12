import tempfile
from pathlib import Path

import mujoco
import numpy as np

from .locomotion import Walker
from .scene import build_scene


class IndoorSimulator(Walker):
    def __init__(self, assets, seed=0):
        # MuJoCo 3.2 parses an XML file; generated files remain outside the repo.
        with tempfile.TemporaryDirectory(prefix='humanoid_scene_') as folder:
            path = Path(folder)/'scene.xml'
            path.write_text(build_scene(assets, seed), encoding='utf-8')
            super().__init__(assets, path)
        self.renderer = None
        self.floor_id = self.model.geom('floor').id
        self.rng = np.random.default_rng(seed)
        self.odom = np.zeros(3)
        self._last_yaw = self.yaw

    def step(self):
        super().step()
        # Simulated velocity/gyro measurements, integrated independently of position.
        dt = self.model.opt.timestep*self.config['control_decimation']
        yaw = self.yaw
        rotation = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
        body_velocity = rotation @ self.data.qvel[:2]
        body_velocity += self.rng.normal(0, .003, 2)
        dyaw = (yaw-self._last_yaw+np.pi)%(2*np.pi)-np.pi
        self._last_yaw = yaw
        self.odom[2] += dyaw + self.rng.normal(0, .0002)
        c, s = np.cos(self.odom[2]), np.sin(self.odom[2])
        self.odom[:2] += np.array([[c, -s], [s, c]])@body_velocity*dt

    def camera_transform(self):
        """Optical pose relative to the level, ground-projected base frame."""
        camera = self.data.camera('front_camera')
        yaw = self.yaw
        c, s = np.cos(yaw), np.sin(yaw)
        inverse_heading = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        base = np.array([*self.data.qpos[:2], 0.0])
        translation = inverse_heading @ (camera.xpos - base)
        optical = camera.xmat.reshape(3, 3) @ np.diag([1, -1, -1])
        rotation = np.eye(4)
        rotation[:3, :3] = inverse_heading @ optical
        return translation.tolist(), rotation.tolist()

    def image(self, labels=False, depth=False):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=96, width=128)
        r = self.renderer
        r.disable_segmentation_rendering()
        r.disable_depth_rendering()
        r.update_scene(self.data, camera='front_camera')
        rgb = r.render().copy()
        if labels:
            r.enable_segmentation_rendering()
            segmentation = r.render().copy()
            r.disable_segmentation_rendering()
            return rgb, (segmentation[:, :, 0] == self.floor_id).astype(np.uint8)
        if depth:
            r.enable_depth_rendering()
            result = r.render().copy()
            r.disable_depth_rendering()
            return rgb, result
        return rgb

    def scan(self, count=180):
        origin = self.data.qpos[:3].copy()
        origin[2] = .65
        angles = np.linspace(-np.pi, np.pi, count, endpoint=False) + self.yaw
        directions = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(count)))
        groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        geom_ids = np.empty(count, dtype=np.int32)
        distances = np.empty(count)
        # Exclude the robot and limit the query to the published scan range.
        mujoco.mj_multiRay(self.model, self.data, origin, directions.ravel(), groups,
                           1, self.model.body('pelvis').id, geom_ids, distances, count, 8.0)
        distances[distances < 0] = 8.0
        return distances.clip(0.05, 8.0).tolist()

    def close(self):
        if self.renderer:
            self.renderer.close()
            self.renderer = None
