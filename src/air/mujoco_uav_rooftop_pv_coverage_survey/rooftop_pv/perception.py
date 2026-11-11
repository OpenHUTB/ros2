"""Synchronized RGB and photovoltaic masks / 同步 RGB 与光伏掩码。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from PIL import Image


@dataclass(frozen=True)
class SensorFrame:
    """One aligned perception frame / 一帧对齐的感知数据。"""

    rgb: Image.Image
    mask: Image.Image
    pv_pixels: int
    total_pixels: int


class NadirSensor:
    """Persistent MuJoCo RGB and segmentation renderers / 持久化 MuJoCo RGB 与分割渲染器。"""

    def __init__(self, model: Any, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("image dimensions must be positive / 图像尺寸必须为正数")
        import mujoco

        self._mujoco = mujoco
        self._model = model
        self._rgb_renderer = mujoco.Renderer(model, height=height, width=width)
        self._seg_renderer = mujoco.Renderer(model, height=height, width=width)
        self._seg_renderer.enable_segmentation_rendering()
        self._pv_geom_ids = np.asarray(
            [
                geom_id
                for geom_id in range(model.ngeom)
                if (model.geom(geom_id).name or "").startswith("pv_panel_")
            ],
            dtype=np.int32,
        )
        if self._pv_geom_ids.size == 0:
            raise ValueError("scene contains no photovoltaic geometry / 场景没有光伏几何体")

    def capture(self, data: Any) -> SensorFrame:
        """Capture RGB and visible PV pixels at one pose / 在同一位姿采集 RGB 与可见光伏像素。"""
        self._rgb_renderer.update_scene(data, camera="downward_rgb")
        rgb_array = self._rgb_renderer.render().copy()
        self._seg_renderer.update_scene(data, camera="downward_rgb")
        segmentation = self._seg_renderer.render().copy()
        is_geom = segmentation[..., 1] == int(self._mujoco.mjtObj.mjOBJ_GEOM)
        is_pv = np.isin(segmentation[..., 0], self._pv_geom_ids)
        mask_array = np.where(is_geom & is_pv, 255, 0).astype(np.uint8)
        return SensorFrame(
            rgb=Image.fromarray(rgb_array, mode="RGB"),
            mask=Image.fromarray(mask_array, mode="L"),
            pv_pixels=int(np.count_nonzero(mask_array)),
            total_pixels=int(mask_array.size),
        )

    def close(self) -> None:
        self._rgb_renderer.close()
        self._seg_renderer.close()

    def __enter__(self) -> "NadirSensor":
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()
