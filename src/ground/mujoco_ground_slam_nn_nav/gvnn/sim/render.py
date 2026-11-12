"""Pillow-based top-down renderer used for GIF demos and documentation."""

from __future__ import annotations

import math
import os
from typing import Optional, Sequence

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .world import Scene, scene_rgb


def _font(size: int) -> ImageFont.ImageFont:
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    ):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


class TopDownRenderer:
    """960x600 canvas: 720x480 world map + camera/HUD sidebar."""

    def __init__(self, scene: Scene):
        self.scene = scene
        self.size = (960, 600)
        self.world_box = (10, 58, 720, 480)  # x, y, w, h
        self.world_w, self.world_h = self.world_box[2], self.world_box[3]
        self.font = _font(15)
        self.small = _font(13)
        self.big = _font(19)
        base = scene_rgb(scene)
        img = Image.fromarray((base * 255).astype(np.uint8), mode="RGB")
        img = img.resize((self.world_w, self.world_h), Image.NEAREST)
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
        self.background = img

    # ------------------------------------------------------------------ coords
    def px(self, x: float, y: float):
        return (int(self.world_box[0] + x / self.scene.width * self.world_w),
                int(self.world_box[1] + (1.0 - y / self.scene.height) * self.world_h))

    def draw_robot(self, draw: ImageDraw.ImageDraw, pose: Sequence[float],
                   color=(30, 150, 240), scale: float = 1.0) -> None:
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        cx, cy = self.px(x, y)
        length, width = 0.30 * scale * self.world_w / self.scene.width, \
            0.20 * scale * self.world_w / self.scene.width
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for lx, ly in ((length, width), (length, -width), (-length, -width), (-length, width)):
            pts.append((cx + int(c * lx - s * ly), cy - int(s * lx + c * ly)))
        draw.polygon(pts, fill=color, outline=(255, 255, 255))
        draw.line((cx, cy, int(cx + c * length * 1.8), int(cy - s * length * 1.8)),
                  fill=(255, 230, 40), width=3)

    # ------------------------------------------------------------------ render
    def render(self, truth: Optional[Sequence[float]] = None,
               slam_pose: Optional[Sequence[float]] = None,
               slam_data: Optional[np.ndarray] = None,
               path: Optional[Sequence[Sequence[float]]] = None,
               scan_points: Optional[np.ndarray] = None,
               bev: Optional[np.ndarray] = None,
               trail: Optional[Sequence[Sequence[float]]] = None,
               hud: Optional[Sequence[str]] = None,
               title: str = "GVNN") -> Image.Image:
        canvas = Image.new("RGB", self.size, (28, 30, 34))
        canvas.paste(self.background, (self.world_box[0], self.world_box[1]))

        if slam_data is not None:
            data = np.asarray(slam_data, dtype=np.int8)
            h, w = data.shape
            rgba = np.zeros((h, w, 4), dtype=np.uint8)
            rgba[data == 0] = (245, 245, 245, 70)
            rgba[data == 100] = (10, 10, 10, 190)
            layer = Image.fromarray(rgba, mode="RGBA")
            layer = layer.resize((self.world_w, self.world_h), Image.NEAREST)
            layer = layer.transpose(Image.FLIP_TOP_BOTTOM)
            canvas.paste(layer, (self.world_box[0], self.world_box[1]), layer)

        draw = ImageDraw.Draw(canvas)
        # world border
        draw.rectangle((self.world_box[0], self.world_box[1],
                        self.world_box[0] + self.world_w,
                        self.world_box[1] + self.world_h),
                       outline=(110, 115, 120), width=1)
        # goal
        gx, gy = self.px(self.scene.goal[0], self.scene.goal[1])
        draw.ellipse((gx - 10, gy - 10, gx + 10, gy + 10), outline=(60, 230, 90), width=3)
        draw.text((gx + 12, gy - 8), "GOAL", fill=(60, 230, 90), font=self.small)

        if trail:
            pts = [self.px(*p) for p in trail]
            if len(pts) > 1:
                draw.line(pts, fill=(60, 190, 100), width=2)

        if path:
            pts = [self.px(*p) for p in path]
            if len(pts) > 1:
                draw.line(pts, fill=(255, 195, 40), width=3)

        if scan_points is not None and len(scan_points):
            for x, y in scan_points[::3]:
                px, py = self.px(float(x), float(y))
                if 0 <= px < self.size[0] and 0 <= py < self.size[1]:
                    draw.ellipse((px - 1, py - 1, px + 1, py + 1), fill=(60, 200, 255))

        if truth is not None:
            self.draw_robot(draw, truth, (50, 190, 70), 1.15)
        if slam_pose is not None:
            self.draw_robot(draw, slam_pose, (30, 150, 240), 1.0)

        # sidebar
        panel_x = 748
        draw.text((10, 14), title, fill=(235, 235, 235), font=self.big)
        draw.text((10, 38), "MuJoCo 3.12.0  |  ROS Noetic  |  top-down map",
                  fill=(160, 180, 200), font=self.small)
        draw.rectangle((panel_x - 8, 58, 952, 286), outline=(90, 95, 105))
        if bev is not None:
            img = Image.fromarray(np.asarray(bev, dtype=np.uint8), mode="RGB")
            img = img.resize((220, 220), Image.NEAREST)
            canvas.paste(img, (panel_x, 68))
        draw.text((panel_x, 294), "BEV camera (64x64)", fill=(190, 200, 210), font=self.small)

        y = 330
        for line in (hud or []):
            draw.text((panel_x, y), line, fill=(225, 228, 232), font=self.small)
            y += 22
        draw.text((10, 565), "W/A/S/D teleop   SPACE stop   R reset   1/2/3 scene   Q quit",
                  fill=(145, 155, 165), font=self.small)
        return canvas
