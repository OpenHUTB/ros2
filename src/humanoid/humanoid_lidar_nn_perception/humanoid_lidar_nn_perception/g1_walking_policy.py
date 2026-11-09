"""G1 walking policy wrapper around the bundled Unitree ONNX policy.

The ONNX policy is a pre-trained Isaac Lab locomotion policy exported to ONNX
(provided by MuJoCo-LiDAR, upstream: https://github.com/discoververse/MuJoCo-LiDAR).
It accepts a velocity command [vx, vy, wz] and outputs 29 joint position targets.

This wrapper:
  * builds the 103-dim observation vector
  * integrates the gait phase counter
  * exposes set_command() so higher-level nodes (avoidance PPO / teleop) can
    override the desired velocity at any time
"""
from __future__ import annotations

import numpy as np
import onnxruntime as rt
import mujoco

JOINT_NUM = 29
CTRL_DT = 0.02
SIM_DT = 0.004
N_SUBSTEPS = int(round(CTRL_DT / SIM_DT))
ACTION_SCALE = 0.5
GAIT_FREQ = 1.5
PHASE_DT = 2 * np.pi * GAIT_FREQ * CTRL_DT


class G1WalkingPolicy:
    def __init__(self, model: mujoco.MjModel, policy_path: str):
        self.model = model
        self.policy = rt.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
        self.default_angles = np.array(model.keyframe("home").qpos[7 : 7 + JOINT_NUM])
        self.last_action = np.zeros(JOINT_NUM, dtype=np.float32)
        self.phase = np.array([0.0, np.pi])
        # external velocity command (vx, vy, wz) -- m/s, rad/s
        self.command = np.zeros(3, dtype=np.float32)
        self._output_names = ["continuous_actions"]
        self.pelvis_id = model.body("pelvis").id

    def set_command(self, vx: float, vy: float = 0.0, wz: float = 0.0) -> None:
        self.command[0] = float(vx)
        self.command[1] = float(vy)
        self.command[2] = float(wz)

    def _obs(self, data: mujoco.MjData) -> np.ndarray:
        linvel = data.sensor("local_linvel_pelvis").data
        gyro = data.sensor("gyro_pelvis").data
        imu_xmat = data.site_xmat[
            self.model.site("imu_in_pelvis").id
        ].reshape(3, 3)
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        qpos_j = data.qpos[7 : 7 + JOINT_NUM] - self.default_angles
        qvel_j = data.qvel[6 : 6 + JOINT_NUM]
        phase_obs = np.concatenate([np.cos(self.phase), np.sin(self.phase)])
        return np.hstack(
            [linvel, gyro, gravity, self.command, qpos_j, qvel_j, self.last_action, phase_obs]
        ).astype(np.float32)

    def step(self, data: mujoco.MjData) -> None:
        """Call once per control tick (every CTRL_DT). Assumes sim stepped N_SUBSTEPS."""
        obs = self._obs(data)
        action = self.policy.run(self._output_names, {"obs": obs.reshape(1, -1)})[0][0]
        self.last_action = action.copy()
        data.ctrl[:] = action * ACTION_SCALE + self.default_angles
        self.phase = np.fmod(self.phase + PHASE_DT + np.pi, 2 * np.pi) - np.pi

    def pelvis_height(self, data: mujoco.MjData) -> float:
        return float(data.xpos[self.pelvis_id, 2])

    def fell(self, data: mujoco.MjData) -> bool:
        return float(data.xpos[self.pelvis_id, 2]) < 0.4
