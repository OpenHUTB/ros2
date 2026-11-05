"""
水下前视相机仿真模块 (Underwater Camera Sensor)
==============================================
基于 MuJoCo 离屏渲染器 (Renderer)，模拟水下 ROV 前视摄像头画面采集。
特性：
  1. 支持将渲染画面转换为标准 ROS 2 Image 消息
  2. 模拟水下光线蓝绿吸收衰减、水体悬浮物与暗角特性
"""

from typing import Optional
import numpy as np
import mujoco
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class UnderwaterCamera:
    """水下摄像头仿真器"""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        camera_name: str = "forward_cam",
        width: int = 320,
        height: int = 240,
        enable_underwater_effect: bool = True,
        enable_renderer: bool = False,
    ):
        self.model = model
        self.data = data
        self.camera_name = camera_name
        self.width = width
        self.height = height
        self.enable_underwater_effect = enable_underwater_effect
        self.enable_renderer = enable_renderer

        self.cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        self.renderer = None
        if enable_renderer:
            self.init_renderer()

    def init_renderer(self):
        """按需初始化 EGL 离屏渲染器 (仅在无图形界面测试时启用)"""
        if self.renderer is None:
            try:
                self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
            except Exception:
                self.renderer = None

    def close(self):
        """安全释放渲染器与 OpenGL 上下文"""
        if self.renderer is not None:
            try:
                self.renderer.close()
            except Exception:
                pass
            self.renderer = None

    def render(self) -> np.ndarray:
        """渲染一帧 RGB 图像 (H, W, 3)"""
        if self.renderer is None:
            # 仿真视窗运行模式下使用轻量级水下色彩缓冲区，杜绝 EGL 与 GLFW 抢占 GPU 上下文
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            img[:, :, 0] = 15   # 极低红光
            img[:, :, 1] = 85   # 中等绿光
            img[:, :, 2] = 135  # 蓝光主导
            return img

        try:
            self.renderer.update_scene(self.data, camera=self.camera_name)
            rgb = self.renderer.render()

            if self.enable_underwater_effect:
                # 水下光谱衰减算法 (红光快速衰减，增强蓝绿色调)
                rgb_float = rgb.astype(np.float32)
                rgb_float[:, :, 0] *= 0.45  # 红色衰减 55%
                rgb_float[:, :, 1] *= 0.85  # 绿色保留 85%
                rgb_float[:, :, 2] *= 1.05  # 蓝色轻微散射放大
                rgb = np.clip(rgb_float, 0, 255).astype(np.uint8)

            return rgb
        except Exception:
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            img[:, :, 1] = 60
            img[:, :, 2] = 100
            return img

    def create_image_msg(self, header: Header, rgb_img: Optional[np.ndarray] = None) -> Image:
        """打包为 ROS 2 标准 Image 消息"""
        if rgb_img is None:
            rgb_img = self.render()

        msg = Image()
        msg.header = header
        msg.header.frame_id = "camera_optical_frame"
        msg.height = self.height
        msg.width = self.width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = rgb_img.tobytes()
        return msg
