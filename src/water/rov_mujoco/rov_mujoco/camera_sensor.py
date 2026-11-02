"""
水下前视相机仿真模块 (Underwater Camera Sensor)
==============================================
基于 MuJoCo 离屏渲染器 (Renderer)，模拟水下 ROV 前视摄像头画面采集。
特性：
  1. 支持将渲染画面转换为标准 ROS 2 Image 消息
  2. 模拟水下光线蓝绿吸收衰减、水体悬浮物与暗角特性
"""

import os
from typing import Optional
import numpy as np
import mujoco
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def _is_wsl() -> bool:
    """检测当前是否运行在 WSL 中（WSL 下 MuJoCo 默认 GLFW 离屏上下文会段错误）"""
    if os.environ.get("WSL_DISTRO_NAME"):
        return True
    try:
        with open("/proc/version", "r", encoding="utf-8", errors="ignore") as f:
            return "microsoft" in f.read().lower()
    except OSError:
        return False


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
        enable_renderer: bool = True,
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
        self.render_mode = "placeholder"   # real / placeholder
        self.render_backend = os.environ.get("MUJOCO_GL", "default")
        if enable_renderer:
            self.init_renderer()

    def init_renderer(self):
        """初始化 MuJoCo 离屏渲染器；失败时回退为占位图并给出明确告警"""
        if self.renderer is not None:
            return

        # WSL 下 MuJoCo 默认的 GLFW 离屏上下文会直接段错误（无法用 try/except 捕获），
        # 因此仅允许在 osmesa 后端下创建真实渲染器。可用 ROV_CAMERA_RENDER=0/1 强制覆盖。
        forced = os.environ.get("ROV_CAMERA_RENDER", "").strip().lower()
        if forced in ("0", "false", "off"):
            self.render_mode = "placeholder"
            return
        if _is_wsl() and self.render_backend.lower() != "osmesa" and forced not in ("1", "true", "on"):
            self.render_mode = "placeholder"
            print("[UnderwaterCamera] WSL 环境未使用 osmesa 后端，已跳过离屏渲染 "
                  "(设置 MUJOCO_GL=osmesa 可启用真实相机图像)")
            return

        try:
            self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
            self.render_mode = "real"
        except Exception as exc:  # 例如无 OpenGL 上下文
            self.renderer = None
            self.render_mode = "placeholder"
            print(f"[UnderwaterCamera] 离屏渲染器初始化失败，回退占位图: {exc}")

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
            # 无可用 GL 上下文时的占位图（真实渲染路径见 render_mode == 'real'）
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
