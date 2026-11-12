"""G1 velocity-command adapter using Unitree's pretrained locomotion policy.

Observation ordering and PD gains follow unitree_rl_gym/deploy/deploy_mujoco
(BSD-3-Clause). This module does not train the underlying walking policy.
"""
from pathlib import Path
import math

import mujoco
import numpy as np
import torch
import yaml


class Walker:
    def __init__(self, assets, xml_path=None):
        assets = Path(assets).resolve()
        config_path = assets / 'deploy/deploy_mujoco/configs/g1.yaml'
        self.config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        def resolve(value):
            return value.replace('{LEGGED_GYM_ROOT_DIR}', assets.as_posix())
        self.model = mujoco.MjModel.from_xml_path(str(xml_path or resolve(self.config['xml_path'])))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.config['simulation_dt']
        torch.set_num_threads(2)
        self.policy = torch.jit.load(resolve(self.config['policy_path']), map_location='cpu').eval()
        self.default = np.asarray(self.config['default_angles'], dtype=np.float32)
        self.kp = np.asarray(self.config['kps'])
        self.kd = np.asarray(self.config['kds'])
        self.command = np.zeros(3, dtype=np.float32)
        self.action = np.zeros(self.config['num_actions'], dtype=np.float32)
        self.target = self.default.copy()
        self.counter = 0
        mujoco.mj_forward(self.model, self.data)

    def set_command(self, vx=0.0, vy=0.0, wz=0.0):
        value = np.array([vx, vy, wz], dtype=np.float32)
        if not np.isfinite(value).all():
            raise ValueError('Command must be finite')
        self.command[:] = np.clip(value, [-0.3, -0.2, -0.5], [0.5, 0.2, 0.5])

    @property
    def yaw(self):
        w, x, y, z = self.data.qpos[3:7]
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

    @property
    def fallen(self):
        return self.data.qpos[2] < 0.40 or not np.isfinite(self.data.qpos).all()

    def step(self):
        c, d = self.config, self.data
        for _ in range(c['control_decimation']):
            d.ctrl[:] = (self.target-d.qpos[7:])*self.kp-d.qvel[6:]*self.kd
            mujoco.mj_step(self.model, d)
            self.counter += 1
        w, x, y, z = d.qpos[3:7]
        gravity = np.array([2*(-z*x+w*y), -2*(z*y+w*x), 1-2*(w*w+z*z)])
        phase = 2*np.pi*(d.time % 0.8)/0.8
        observation = np.concatenate([
            d.qvel[3:6]*c['ang_vel_scale'], gravity,
            self.command*np.asarray(c['cmd_scale']),
            (d.qpos[7:]-self.default)*c['dof_pos_scale'],
            d.qvel[6:]*c['dof_vel_scale'], self.action,
            [np.sin(phase), np.cos(phase)],
        ]).astype(np.float32)
        with torch.inference_mode():
            self.action = self.policy(torch.from_numpy(observation)[None]).numpy().reshape(-1)
        self.target = self.action*c['action_scale']+self.default

    def smoke_test(self, seconds=12.0):
        start = self.data.qpos[:3].copy()
        min_height = float(start[2])
        while self.data.time < seconds:
            t = self.data.time
            self.set_command(0.0 if t < 2 or t >= 10 else 0.3,
                             wz=0.25 if 6 <= t < 10 else 0.0)
            self.step()
            min_height = min(min_height, float(self.data.qpos[2]))
            if self.fallen:
                break
        return dict(simulated_seconds=float(self.data.time), fallen=bool(self.fallen),
                    displacement_m=float(np.linalg.norm(self.data.qpos[:2]-start[:2])),
                    min_height_m=min_height, final_yaw_rad=float(self.yaw),
                    final_position=self.data.qpos[:3].tolist())
