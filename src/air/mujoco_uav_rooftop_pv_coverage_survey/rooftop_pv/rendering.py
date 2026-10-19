"""MuJoCo rendering helpers / MuJoCo 渲染辅助函数。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from PIL import Image


def render_rgb(model: Any, data: Any, width: int = 320, height: int = 240) -> Image.Image:
    """Render the synchronized downward RGB camera / 渲染同步的下视 RGB 相机画面。"""
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive / 图像尺寸必须为正数")
    import mujoco

    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera="downward_rgb")
        pixels = renderer.render()
        return Image.fromarray(pixels, mode="RGB")
    finally:
        renderer.close()


def save_rgb(image: Image.Image, path: Path) -> None:
    """Save an RGB image and create only its parent path / 保存 RGB 图像并创建其父目录。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, format="PNG", optimize=True)
