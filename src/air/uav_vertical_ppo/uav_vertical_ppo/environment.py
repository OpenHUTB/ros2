"""将 MuJoCo 垂直通道包装为 Gymnasium 连续控制环境."""
import gymnasium as gym
import mujoco
import numpy as np

from .paths import resource


class VerticalHoverEnv(gym.Env):
    """观测高度误差和垂直速度，输出四个旋翼的合成推力."""

    metadata = {'render_modes': ['rgb_array'], 'render_fps': 50}

    def __init__(self, scenario='calm', target=1.5):
        super().__init__()
        if scenario not in ('calm', 'gust', 'payload'):
            raise ValueError('场景必须是 calm、gust 或 payload')
        if not np.isfinite(target) or not 0.4 <= target <= 3.0:
            raise ValueError('目标高度必须位于 [0.4, 3.0] 米')
        self.model = mujoco.MjModel.from_xml_path(str(resource('models/vertical_quadrotor.xml')))
        self.data = mujoco.MjData(self.model)
        self.scenario = scenario
        self.target = float(target)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (1,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (2,), dtype=np.float32)
        self.renderer = None
        self.steps = 0

    def observation(self):
        """读取向下测距和速度传感器，不把外力或质量作为策略输入."""
        height = float(self.data.sensor('height_sensor').data[0]) + 0.05
        velocity = float(self.data.sensor('body_velocity').data[2])
        return np.array([height - self.target, velocity], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.steps = 0
        self.mass_scale = 1.15 if self.scenario == 'payload' else 1.0
        # 载荷直接进入质量和惯量，再重新计算 MuJoCo 的派生常量。
        self.model.body_mass[1] = self.mass_scale
        self.model.body_inertia[1] *= self.mass_scale / getattr(self, '_old_scale', 1.0)
        self._old_scale = self.mass_scale
        mujoco.mj_setConst(self.model, self.data)
        self.data.qpos[0] = self.np_random.uniform(0.5, 2.5)
        self.data.qvel[0] = self.np_random.uniform(-0.3, 0.3)
        mujoco.mj_forward(self.model, self.data)
        return self.observation(), {}

    def step(self, action):
        action = np.asarray(action, dtype=float)
        if action.shape != (1,) or not np.all(np.isfinite(action)):
            raise ValueError('动作必须是一个有限数值')
        command = float(np.clip(action[0], -1.0, 1.0))
        self.data.ctrl[0] = 9.81 + 6.0 * command
        # 第 2 至 3 秒施加 1.5 N 向下外力，其他时间清零。
        self.data.xfrc_applied[1, 2] = -1.5 if self.scenario == 'gust' and 2 <= self.data.time < 3 else 0.0
        for _ in range(4):
            mujoco.mj_step(self.model, self.data)
        self.steps += 1
        obs = self.observation()
        reward = 1.0 - 2.0 * obs[0] ** 2 - 0.15 * obs[1] ** 2 - 0.01 * command ** 2
        terminated = bool(self.data.qpos[0] < 0.15 or self.data.qpos[0] > 4.0)
        if terminated:
            reward -= 20.0
        info = {'time_s': float(self.data.time), 'height_m': float(self.data.qpos[0]),
                'target_m': self.target, 'velocity_m_s': float(self.data.qvel[0]),
                'thrust_n': float(self.data.ctrl[0])}
        return obs, float(reward), terminated, self.steps >= 400, info

    def render(self):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=360, width=480)
        self.renderer.update_scene(self.data, camera='side')
        return self.renderer.render()

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
