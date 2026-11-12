"""Generate indoor geometry without changing the upstream robot assets."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .locomotion import Walker


def build_scene(assets, seed=0):
    asset_dir = Path(assets).resolve() / 'resources/robots/g1_description'
    tree = ET.parse(asset_dir/'g1_12dof.xml')
    root = tree.getroot()
    root.find('compiler').set('meshdir', (asset_dir/'meshes').as_posix())
    world = root.find('worldbody')
    pelvis = world.find("body[@name='pelvis']")
    for geom in pelvis.iter('geom'):
        geom.set('group', '1')
    ET.SubElement(pelvis, 'camera', name='front', pos='0.16 0 0.35',
                  xyaxes='0 -1 0 0.35 0 0.94', fovy='75')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='640', offheight='480')
    ET.SubElement(visual, 'headlight', ambient='0.4 0.4 0.4', diffuse='0.6 0.6 0.6')
    rng = np.random.default_rng(seed)
    brightness = float(rng.uniform(.6, .95))
    ET.SubElement(world, 'light', pos='3 0 4', diffuse=f'{brightness} {brightness} {brightness}')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='15 15 .1', rgba='.65 .65 .60 1')
    boxes = [('left_wall', [4, 2, 1], [6, .12, 1]),
             ('right_wall', [4, -2, 1], [6, .12, 1]),
             ('back_wall', [-2, 0, 1], [.12, 2, 1]),
             ('end_wall', [10, 0, 1], [.12, 2, 1]),
             ('door_left', [5, 1.65, .9], [.12, .35, .9]),
             ('door_right', [5, -1.65, .9], [.12, .35, .9]),
             ('cabinet', [3, .95 + float(rng.uniform(-.15, .15)), .45], [.3, .3, .45])]
    for name, pos, size in boxes:
        rgba = [*rng.uniform(.25, .75, 3), 1]
        ET.SubElement(world, 'geom', name=name, type='box',
                      pos=' '.join(map(str, pos)), size=' '.join(map(str, size)),
                      rgba=' '.join(map(str, rgba)))
    ET.SubElement(world, 'geom', name='inspection_marker', type='box',
                  pos='9.84 0 1', size='.02 .35 .35', rgba='.1 .7 .8 1',
                  contype='0', conaffinity='0')
    return ET.tostring(root, encoding='unicode')


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

    def image(self, labels=False, depth=False):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=96, width=128)
        r = self.renderer
        r.disable_segmentation_rendering()
        r.disable_depth_rendering()
        r.update_scene(self.data, camera='front')
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
        ranges = []
        geom_id = np.zeros(1, dtype=np.int32)
        # Only static geometry: no robot self returns. Angle frame is level base_link.
        for angle in np.linspace(-np.pi, np.pi, count, endpoint=False):
            a = angle + self.yaw
            distance = mujoco.mj_ray(self.model, self.data, origin,
                np.array([np.cos(a), np.sin(a), 0.]), np.array([1,0,0,0,0,0], dtype=np.uint8), 1,
                self.model.body('pelvis').id, geom_id)
            ranges.append(float(np.clip(distance if distance >= 0 else 8., .05, 8.)))
        return ranges

    def close(self):
        if self.renderer:
            self.renderer.close()
            self.renderer = None
