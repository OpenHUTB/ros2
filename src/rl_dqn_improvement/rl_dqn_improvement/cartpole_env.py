"""基于 MuJoCo 的 CartPole（倒立摆）环境，纯 numpy 接口。

观测（4 维）：小车位置 x、小车速度 xd、摆杆角度 th、摆杆角速度 thd
动作（2 个离散）：0=向左推(力 -10)，1=向右推(力 +10)
奖励：每存活一步 +1；当摆角超过 0.3 rad 或小车位置超界或达到最大步数时终止。
"""
import os

import numpy as np
import mujoco

XML_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'cartpole.xml')


class CartPoleEnv:
    """MuJoCo 倒立摆环境封装，提供与经典 CartPole 一致的接口。"""

    def __init__(self, xml_path=None, max_steps=200):
        self.xml = xml_path or XML_PATH
        self.model = mujoco.MjModel.from_xml_path(self.xml)
        self.data = mujoco.MjData(self.model)
        self.max_steps = max_steps
        self.steps = 0
        self.force = 20.0
        self.observation_dim = 4
        self.action_dim = 2

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        # 随机初始化小车位置与摆角，保证每次回合不同
        self.data.qpos[0] = np.random.uniform(-0.05, 0.05)
        self.data.qpos[1] = np.random.uniform(-0.05, 0.05)
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.steps = 0
        return self._obs()

    def _obs(self):
        return np.array([self.data.qpos[0], self.data.qvel[0],
                         self.data.qpos[1], self.data.qvel[1]],
                        dtype=np.float32)

    def step(self, a):
        # 离散动作 → 施加在滑块上的水平力
        self.data.ctrl[0] = -self.force if a == 0 else self.force
        mujoco.mj_step(self.model, self.data)
        self.steps += 1
        x, xd, th, thd = self._obs()
        done = bool(abs(th) > 0.3 or abs(x) > 2.4
                    or self.steps >= self.max_steps)
        return self._obs(), 1.0, done, {}

    def render_frame(self):
        """渲染一帧 RGB 图像（需有 OpenGL/Xvfb），返回 HxWx3 ndarray。"""
        r = mujoco.Renderer(self.model, 320, 240)
        mujoco.mj_forward(self.model, self.data)
        img = r.render()
        r.close()
        return img
