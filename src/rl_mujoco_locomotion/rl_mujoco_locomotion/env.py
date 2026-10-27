"""MuJoCo locomotion environment wrappers (HalfCheetah, Ant, Walker2d)."""

import os
import sys

# 渲染后端：仅在「Linux 且调用方未显式设置」时才默认使用无头 EGL；
# Windows / macOS 保留 MuJoCo 默认后端（glfw），避免 import 阶段直接报
# RuntimeError: invalid value for environment variable MUJOCO_GL: egl
if "MUJOCO_GL" not in os.environ and sys.platform.startswith("linux"):
    os.environ["MUJOCO_GL"] = "egl"

import shutil
import numpy as np
import mujoco

_HERE = os.path.dirname(os.path.abspath(__file__))
ASSET_DIR = os.path.normpath(os.path.join(_HERE, "..", "assets"))


def _asset_dirs():
    """assets 的候选目录：源码布局（包目录旁的 assets）与 colcon 安装布局（share/<pkg>/assets）。"""
    dirs = [ASSET_DIR]
    try:
        from ament_index_python.packages import get_package_share_directory

        share_assets = os.path.join(
            get_package_share_directory("rl_mujoco_locomotion"), "assets")
        if share_assets not in dirs:
            dirs.append(share_assets)
    except Exception:
        pass
    return dirs


def _find_asset(xml_name):
    """在候选目录中查找 MJCF 文件，找不到返回 None。"""
    for d in _asset_dirs():
        path = os.path.join(d, xml_name)
        if os.path.exists(path):
            return path
    return None

_ENV_CONFIG = {
    "halfcheetah": {
        "xml_name": "half_cheetah.xml",
        "ctrl_cost_weight": 0.1,
        "forward_reward_weight": 1.0,
        "reset_noise_scale": 0.1,
        "frame_skip": 5,
        "terminate_when_unhealthy": False,
    },
    "ant": {
        "xml_name": "ant.xml",
        "ctrl_cost_weight": 0.5,
        "forward_reward_weight": 1.0,
        "healthy_z_range": (0.2, 1.0),
        "contact_cost_weight": 5e-4,
        "contact_cost_range": (-1.0, 1.0),
        "reset_noise_scale": 0.1,
        "frame_skip": 5,
        "terminate_when_unhealthy": True,
        "exclude_current_positions_from_observation": True,
    },
    "walker2d": {
        "xml_name": "walker2d.xml",
        "ctrl_cost_weight": 0.001,
        "forward_reward_weight": 1.0,
        "healthy_z_range": (0.8, 2.0),
        "healthy_angle_range": (-1.0, 1.0),
        "reset_noise_scale": 0.005,
        "frame_skip": 4,
        "terminate_when_unhealthy": True,
    },
}


def _find_gymnasium_xml(xml_name):
    try:
        import gymnasium
        base = os.path.join(os.path.dirname(gymnasium.__file__), "envs", "mujoco", "assets")
        path = os.path.join(base, xml_name)
        if os.path.exists(path):
            return path
    except Exception:
        pass
    return None


def ensure_assets():
    """检查 MJCF 是否可用；只有缺失时才从 gymnasium 复制到源码布局的 assets 目录。"""
    missing = [cfg["xml_name"] for cfg in _ENV_CONFIG.values()
               if _find_asset(cfg["xml_name"]) is None]
    if not missing:
        return
    os.makedirs(ASSET_DIR, exist_ok=True)
    for xml_name in missing:
        src = _find_gymnasium_xml(xml_name)
        if src is not None:
            shutil.copy2(src, os.path.join(ASSET_DIR, xml_name))
            print(f"[env] Copied {xml_name} from gymnasium.")
        else:
            print(f"[env] WARNING: {xml_name} not found. Install gymnasium: pip install gymnasium[mujoco]")


class BaseLocomotionEnv:
    def __init__(self, name, render_mode=None):
        self.name = name
        self.cfg = _ENV_CONFIG[name]
        self.render_mode = render_mode
        xml_path = _find_asset(self.cfg["xml_name"])
        if xml_path is None:
            raise FileNotFoundError(
                f"Asset not found: {self.cfg['xml_name']}\n"
                f"searched: {_asset_dirs()}\n"
                "Please install gymnasium and run 'python -c \"import rl_mujoco_locomotion.env; rl_mujoco_locomotion.env.ensure_assets()\"'"
            )
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self._max_episode_steps = 1000
        self._elapsed_steps = 0
        self._init_qpos = self.data.qpos.copy()
        self._init_qvel = self.data.qvel.copy()
        self._renderer = None
        self._camera = None
        self._body_id = None

    def reset(self, seed=None):
        if seed is not None:
            np.random.seed(seed)
        noise = self.cfg["reset_noise_scale"]
        self.data.qpos[:] = self._init_qpos + np.random.uniform(-noise, noise, size=self.model.nq)
        self.data.qvel[:] = self._init_qvel + np.random.uniform(-noise, noise, size=self.model.nv)
        mujoco.mj_forward(self.model, self.data)
        self._elapsed_steps = 0
        return self._get_obs()

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)
        x_before = self.data.qpos[0]
        self.data.ctrl[:] = action
        for _ in range(self.cfg["frame_skip"]):
            mujoco.mj_step(self.model, self.data)
        x_after = self.data.qpos[0]
        obs = self._get_obs()
        reward, terminated = self._compute_reward(action, x_before, x_after)
        self._elapsed_steps += 1
        truncated = self._elapsed_steps >= self._max_episode_steps
        info = {}
        return obs, reward, terminated, truncated, info

    def _get_obs(self):
        qpos = self.data.qpos.copy()
        qvel = self.data.qvel.copy()
        if self.cfg.get("exclude_current_positions_from_observation") and self.name == "ant":
            qpos = qpos[2:]
        return np.concatenate([qpos, qvel]).astype(np.float64)

    def _compute_reward(self, action, x_before, x_after):
        dt = self.model.opt.timestep * self.cfg["frame_skip"]
        forward_reward = self.cfg["forward_reward_weight"] * (x_after - x_before) / dt
        ctrl_cost = self.cfg["ctrl_cost_weight"] * np.sum(np.square(action))
        contact_cost = 0.0
        if "contact_cost_weight" in self.cfg:
            contact_cost = self.cfg["contact_cost_weight"] * np.sum(
                np.square(np.clip(self.data.cfrc_ext, self.cfg["contact_cost_range"][0], self.cfg["contact_cost_range"][1]))
            )
        healthy = self._is_healthy()
        reward = forward_reward - ctrl_cost - contact_cost + float(healthy)
        terminated = self.cfg["terminate_when_unhealthy"] and not healthy
        return reward, terminated

    def _is_healthy(self):
        if not self.cfg["terminate_when_unhealthy"]:
            return True
        if self.name == "ant":
            z = self.data.qpos[2]
        elif self.name == "walker2d":
            z = self.data.qpos[1]
        else:
            return True
        min_z, max_z = self.cfg["healthy_z_range"]
        if not (min_z < z < max_z):
            return False
        if "healthy_angle_range" in self.cfg:
            angle = self.data.qpos[2]
            if not (self.cfg["healthy_angle_range"][0] < angle < self.cfg["healthy_angle_range"][1]):
                return False
        return True

    def render(self, width=640, height=480):
        if self.render_mode is None:
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(cam)
            cam.distance = self.cfg.get("cam_distance", 2.5)
            cam.elevation = -12.0
            cam.azimuth = 90.0
            self._camera = cam
            try:
                self._body_id = self.model.body('torso').id
            except Exception:
                self._body_id = 1
        if self._body_id is not None:
            self._camera.lookat[:] = self.data.xpos[self._body_id]
        self._renderer.update_scene(self.data, camera=self._camera)
        return self._renderer.render()

    @property
    def observation_space_dim(self):
        return self._get_obs().shape[0]

    @property
    def action_space_dim(self):
        return self.model.nu


def make_env(name, render_mode=None):
    return BaseLocomotionEnv(name, render_mode=render_mode)
