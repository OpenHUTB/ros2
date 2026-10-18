"""Train PPO obstacle-avoidance policy for G1.

Observation (6-dim):
  [nearest_dist/8, front_clear/8, left_clear/8, right_clear/8,
   cos(target_heading), sin(target_heading)]
Action (2-dim, in [-1,1]):
  a[0] -> vx  in [0, 0.4] m/s
  a[1] -> wz  in [-1.0, 1.0] rad/s
These commands are fed to the pre-trained ONNX walking policy, which outputs
joint targets. PPO only learns the high-level (vx, wz) avoidance skill.
"""
from __future__ import annotations
import sys, time, argparse
from pathlib import Path
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
import torch

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from humanoid_lidar_nn_perception.g1_walking_policy import G1WalkingPolicy, SIM_DT, N_SUBSTEPS
from humanoid_lidar_nn_perception.lidar_2d import LiDAR2D

MAX_RANGE = 8.0
FRONT_IDX = np.arange(165, 196)
LEFT_IDX = np.arange(60, 121)
RIGHT_IDX = np.arange(240, 301)


class AvoidanceEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, model_path: str, onnx_path: str, cnn_path: str, seed: int = 0):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.policy = G1WalkingPolicy(self.model, onnx_path)
        self.lidar = LiDAR2D(self.model, num_beams=360, max_range=MAX_RANGE)
        self.cnn = torch.jit.load(cnn_path)
        self.cnn.eval()
        self.rng = np.random.default_rng(seed)

        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.scan_period = 0.1
        self.next_scan_t = 0.0
        self.episode_steps = 0
        self.max_steps = 1000   # 20 s at 0.02 s ctrl
        self._obs = np.zeros(6, dtype=np.float32)
        self.target_heading = 0.0

    def _reset_data(self):
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        x = self.rng.uniform(-4, 4)
        y = self.rng.uniform(-4, 4)
        self.start_yaw = self.rng.uniform(-np.pi, np.pi)
        self.data.qpos[0], self.data.qpos[1] = x, y
        self.data.qpos[3:7] = [np.cos(self.start_yaw/2), 0, 0, np.sin(self.start_yaw/2)]
        # target heading is a fixed relative offset the robot must head toward
        self.target_heading = self.rng.uniform(-np.pi/3, np.pi/3)
        mujoco.mj_forward(self.model, self.data)
        self.next_scan_t = 0.0
        self.episode_steps = 0
        self._update_obs()

    def _current_yaw(self) -> float:
        # yaw from quaternion qw,qx,qy,qz
        qw, qx, qy, qz = self.data.qpos[3:7]
        return float(np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)))

    def _update_obs(self):
        scan = self.lidar.scan(self.data)
        with torch.no_grad():
            pred = self.cnn(torch.from_numpy(scan).unsqueeze(0))[0].numpy()
        self._obs[0] = pred[0]
        self._obs[1] = pred[1]
        self._obs[2] = pred[2]
        self._obs[3] = pred[3]
        # dynamic heading error: current yaw vs (start_yaw + target_heading)
        desired_yaw = self.start_yaw + self.target_heading
        err = desired_yaw - self._current_yaw()
        err = (err + np.pi) % (2*np.pi) - np.pi
        self._obs[4] = np.cos(err)
        self._obs[5] = np.sin(err)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._reset_data()
        return self._obs.copy(), {}

    def step(self, action):
        # action in [-1,1] -> vx in [-0.05, 0.4], wz in [-1.0, 1.0]
        vx = float(np.clip(action[0], -1, 1)) * 0.225 + 0.175
        wz = float(np.clip(action[1], -1, 1)) * 0.8
        self.policy.set_command(vx, 0.0, wz)

        for _ in range(N_SUBSTEPS):
            mujoco.mj_step(self.model, self.data)
        self.policy.step(self.data)
        self.episode_steps += 1

        if self.data.time >= self.next_scan_t:
            self.next_scan_t += self.scan_period
            self._update_obs()

        # reward shaping
        actual_vx = float(self.data.sensor("local_linvel_pelvis").data[0])
        front_clear = float(self._obs[1]) * MAX_RANGE
        left_clear = float(self._obs[2]) * MAX_RANGE
        right_clear = float(self._obs[3]) * MAX_RANGE
        cos_err = float(self._obs[4])

        # 1) forward progress (stronger than before)
        reward = 0.5 * actual_vx
        # 2) heading alignment (dynamic)
        reward += 0.2 * cos_err
        # 3) obstacle proximity penalty (soft, not cliff)
        if front_clear < 1.5:
            reward -= 1.0 * (1.5 - front_clear) / 1.5
        # 4) bonus for picking the clearer side when front is blocked
        if front_clear < 1.5:
            reward += 0.05 * (left_clear - right_clear) / MAX_RANGE * np.sign(wz)

        terminated = bool(self.policy.fell(self.data))
        if terminated:
            reward -= 2.0   # was -10, too harsh
        truncated = self.episode_steps >= self.max_steps

        return self._obs.copy(), reward, terminated, truncated, {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=r"D:\g1_run\scene_g1.xml")
    ap.add_argument("--onnx", default=r"D:\g1_run\g1_policy.onnx")
    ap.add_argument("--cnn", default=r"D:\g1_run\perception_cnn.pt")
    ap.add_argument("--out", default=r"D:\g1_run\avoid_ppo.zip")
    ap.add_argument("--timesteps", type=int, default=1000000)
    args = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv

    env = DummyVecEnv([lambda: AvoidanceEnv(args.model, args.onnx, args.cnn)])
    model = PPO(
        "MlpPolicy", env,
        verbose=1,
        n_steps=512,
        batch_size=128,
        learning_rate=3e-4,
        gamma=0.99,
        device="cpu",
    )
    t0 = time.time()
    model.learn(total_timesteps=args.timesteps)
    model.save(args.out)
    print(f"\nSaved PPO -> {args.out}")
    print(f"wall time: {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
