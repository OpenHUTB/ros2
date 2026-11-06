"""
common.py — 路径解析与声呐图像合成工具。
"""
import os
from pathlib import Path
import numpy as np


def get_package_root() -> Path:
    """返回包源码根目录。"""
    return Path(__file__).resolve().parent.parent


def find_model_path(explicit: str = "") -> str:
    if explicit and Path(explicit).exists():
        return explicit
    cand = get_package_root() / "models" / "rov_sonar_pool.xml"
    if cand.exists():
        return str(cand)
    raise FileNotFoundError("找不到 MJCF 模型 rov_sonar_pool.xml")


def find_weights_path(explicit: str = "") -> Path:
    if explicit:
        return Path(explicit)
    return get_package_root() / "weights" / "sonar_cnn.pt"


def set_mujoco_gl_backend() -> None:
    """import mujoco 前必须设置 OSMesa 后端。"""
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")


def generate_sonar_image(rov_xy: np.ndarray,
                        rock_pos: list,
                        pipe_pos: list,
                        wreck_pos: list,
                        size: int = 64,
                        rock_intensity: float = 0.6,
                        pipe_intensity: float = 0.9,
                        wreck_intensity: float = 0.35) -> np.ndarray:
    """
    合成侧扫声呐图像：池底三类目标物在 ROV 视角下的灰度强度图。
    为什么这么做：真实侧扫声呐回波强度随目标物材质/距离变化，
    这里用高斯亮斑近似每类目标物的声呐回波，作为 CNN 的训练输入。
    各类强度作为参数传入，让数据集可以按 label 控制哪类亮。

    返回 (size, size) uint8 灰度图。
    """
    img = np.zeros((size, size), dtype=np.float32)
    def to_px(x, y):
        cx = int((x + 5.0) / 10.0 * size)
        cy = int((y + 5.0) / 10.0 * size)
        return cx, cy

    def add_blobs(pos_list, intensity, sigma=3):
        for (px, py) in pos_list:
            cx, cy = to_px(px, py)
            for dy in range(-sigma*2, sigma*2+1):
                for dx in range(-sigma*2, sigma*2+1):
                    x, y = cx+dx, cy+dy
                    if 0 <= x < size and 0 <= y < size:
                        g = intensity * np.exp(-(dx*dx + dy*dy) / (2*sigma*sigma))
                        img[y, x] += g

    add_blobs(rock_pos, rock_intensity, sigma=4)
    add_blobs(pipe_pos, pipe_intensity, sigma=2)
    add_blobs(wreck_pos, wreck_intensity, sigma=3)

    img += np.random.randn(size, size) * 0.05
    img = np.clip(img, 0, 1)
    return (img * 255).astype(np.uint8)
