"""MuJoCo RGB / depth camera helper."""

import os
from typing import Optional

import numpy as np


def is_wsl() -> bool:
    """Detect WSL, where MuJoCo's default GLFW offscreen context can crash."""
    if os.environ.get('WSL_DISTRO_NAME'):
        return True
    try:
        with open('/proc/version', 'r', encoding='utf-8', errors='ignore') as f:
            return 'microsoft' in f.read().lower()
    except OSError:
        return False


class CameraSensor:
    """Render the car's front camera with MuJoCo's offscreen renderer."""

    def __init__(
        self,
        model,
        data,
        camera_name: str = 'front_camera',
        width: int = 320,
        height: int = 240,
        max_depth: float = 10.0,
        enable: bool = True,
    ) -> None:
        import mujoco

        self._mujoco = mujoco
        self.model = model
        self.data = data
        self.camera_name = camera_name
        self.width = int(width)
        self.height = int(height)
        self.max_depth = float(max_depth)
        self.camera_id = int(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        )
        if self.camera_id < 0:
            raise ValueError(f'camera not found: {camera_name}')
        self.renderer = None
        self.render_mode = 'placeholder'
        if enable:
            self._init_renderer()

    def _init_renderer(self) -> None:
        backend = os.environ.get('MUJOCO_GL', 'default').lower()
        if is_wsl() and backend not in ('osmesa', 'glfw'):
            print(
                '[CameraSensor] WSL 下未设置 osmesa/glfw，跳过离屏渲染 '
                '(设置 MUJOCO_GL=osmesa 可启用相机)'
            )
            return
        try:
            self.renderer = self._mujoco.Renderer(
                self.model, height=self.height, width=self.width
            )
            self.render_mode = 'real'
        except Exception as exc:  # noqa: BLE001 - GL backends raise various errors
            self.renderer = None
            print(f'[CameraSensor] 离屏渲染初始化失败: {exc}')

    def close(self) -> None:
        if self.renderer is not None:
            try:
                self.renderer.close()
            except Exception:
                pass
            self.renderer = None

    def render_rgb(self) -> np.ndarray:
        if self.renderer is None:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        try:
            self.renderer.update_scene(self.data, camera=self.camera_name)
            return np.asarray(self.renderer.render(), dtype=np.uint8)
        except Exception:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def render_depth(self) -> np.ndarray:
        if self.renderer is None:
            return np.full(
                (self.height, self.width), self.max_depth, dtype=np.float32
            )
        try:
            self.renderer.enable_depth_rendering()
            try:
                self.renderer.update_scene(self.data, camera=self.camera_name)
                depth = np.asarray(self.renderer.render(), dtype=np.float32)
            finally:
                self.renderer.disable_depth_rendering()
            if depth.ndim == 3:
                depth = depth[..., 0]
            return np.clip(depth, 0.0, self.max_depth)
        except Exception:
            return np.full(
                (self.height, self.width), self.max_depth, dtype=np.float32
            )

    def create_image_msg(self, header, rgb: Optional[np.ndarray] = None):
        from sensor_msgs.msg import Image

        image = self.render_rgb() if rgb is None else rgb
        message = Image()
        message.header = header
        message.header.frame_id = 'camera_optical_frame'
        message.height = int(image.shape[0])
        message.width = int(image.shape[1])
        message.encoding = 'rgb8'
        message.is_bigendian = 0
        message.step = int(image.shape[1] * 3)
        message.data = np.ascontiguousarray(image).tobytes()
        return message

    def create_depth_msg(self, header, depth: Optional[np.ndarray] = None):
        from sensor_msgs.msg import Image

        image = self.render_depth() if depth is None else depth
        message = Image()
        message.header = header
        message.header.frame_id = 'camera_optical_frame'
        message.height = int(image.shape[0])
        message.width = int(image.shape[1])
        message.encoding = '32FC1'
        message.is_bigendian = 0
        message.step = int(image.shape[1] * 4)
        message.data = np.ascontiguousarray(image, dtype=np.float32).tobytes()
        return message
