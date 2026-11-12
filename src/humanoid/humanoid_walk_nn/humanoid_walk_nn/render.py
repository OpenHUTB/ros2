"""Top-down renderer for the humanoid walking project."""

from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from . import config as C
from .scenes import Scene

_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)


def _load_font(size: int) -> ImageFont.FreeTypeFont:
    for path in _FONT_CANDIDATES:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
    return ImageFont.load_default()


class TopDownRenderer:
    def __init__(self, scene: Scene, width_px: int = 860, hud_h: int = 96,
                 margin: int = 12):
        self.scene = scene
        self.width_px = width_px
        self.hud_h = hud_h
        self.margin = margin
        self.scale = (width_px - 2 * margin) / max(scene.width, scene.height)
        self.map_w = int(round(scene.width * self.scale))
        self.map_h = int(round(scene.height * self.scale))
        self.width = self.map_w + 2 * margin
        self.height = self.map_h + 2 * margin + hud_h
        self.palette = scene.palette
        self.font = _load_font(15)
        self.font_small = _load_font(13)
        self.font_big = _load_font(19)
        self._base = self._build_base()

    # ------------------------------------------------------------------ utils
    def to_px(self, x: float, y: float) -> Tuple[float, float]:
        return (self.margin + x * self.scale,
                self.margin + (self.scene.height - y) * self.scale)

    def _build_base(self) -> Image.Image:
        img = Image.new("RGB", (self.width, self.height), (16, 18, 22))
        d = ImageDraw.Draw(img)
        floor = self.palette["floor"]
        d.rectangle([self.margin, self.margin, self.margin + self.map_w,
                     self.margin + self.map_h], fill=floor)
        # 1 m grid
        for gx in range(0, int(self.scene.width) + 1, 1):
            p = self.to_px(gx, 0)
            colour = tuple(min(255, c + 8) for c in floor)
            d.line([(p[0], self.margin), (p[0], self.margin + self.map_h)],
                   fill=colour, width=1)
        for gy in range(0, int(self.scene.height) + 1, 1):
            p = self.to_px(0, gy)
            colour = tuple(min(255, c + 8) for c in floor)
            d.line([(self.margin, p[1]), (self.margin + self.map_w, p[1])],
                   fill=colour, width=1)
        # walls
        for cx, cy, hw, hh in self.scene.walls:
            x0, y0 = self.to_px(cx - hw, cy + hh)
            x1, y1 = self.to_px(cx + hw, cy - hh)
            d.rectangle([x0, y0, x1, y1], fill=self.palette["wall"],
                        outline=(60, 64, 70))
        # crates
        for i, (cx, cy, hw, hh) in enumerate(self.scene.obstacles):
            x0, y0 = self.to_px(cx - hw, cy + hh)
            x1, y1 = self.to_px(cx + hw, cy - hh)
            shade = (150 - (i % 4) * 12, 120 - (i % 4) * 8, 80)
            d.rectangle([x0, y0, x1, y1], fill=shade, outline=(70, 54, 34))
        # pillars
        for cx, cy, r in self.scene.pillars:
            x0, y0 = self.to_px(cx - r, cy + r)
            x1, y1 = self.to_px(cx + r, cy - r)
            d.ellipse([x0, y0, x1, y1], fill=(110, 112, 118), outline=(60, 62, 66))
        d.rectangle([self.margin, self.margin, self.margin + self.map_w,
                     self.margin + self.map_h], outline=(90, 96, 104), width=2)
        return img

    # ------------------------------------------------------------------- draw
    def render(self,
               pose: Sequence[float],
               scan: Optional[np.ndarray] = None,
               beam_angles: Optional[np.ndarray] = None,
               slam_pose: Optional[Sequence[float]] = None,
               occupancy: Optional[np.ndarray] = None,
               occupied_mask: Optional[np.ndarray] = None,
               path: Optional[Sequence[Sequence[float]]] = None,
               goal: Optional[Sequence[float]] = None,
               trail: Optional[Sequence[Sequence[float]]] = None,
               hud: Optional[Dict[str, str]] = None,
               mode: str = "TELEOP",
               key_label: str = "",
               message: str = "",
               coverage: float = 0.0,
               time: float = 0.0,
               phase: float = 0.0,
               balance: float = 0.0,
               falls: int = 0,
               carts: Optional[Sequence[Tuple[float, float, float, float]]] = None,
               ) -> Image.Image:
        img = self._base.copy()
        d = ImageDraw.Draw(img, "RGBA")
        mx0, my0 = self.margin, self.margin

        # ---------------------------------------------------------- SLAM map
        if occupancy is not None:
            prob = np.asarray(occupancy, dtype=np.float64)
            occ = np.asarray(occupied_mask, dtype=bool) if occupied_mask is not None else prob > 0.65
            free = prob < 0.35
            rows, cols = occ.shape
            sx = self.map_w / cols
            sy = self.map_h / rows
            layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
            ld = ImageDraw.Draw(layer)
            fy, fx = np.nonzero(free)
            for r, c in zip(fy[::12], fx[::12]):
                x0 = mx0 + c * sx
                y0 = my0 + self.map_h - (r + 1) * sy
                ld.rectangle([x0, y0, x0 + sx + 0.5, y0 + sy + 0.5],
                             fill=(70, 140, 120, 36))
            oy, ox = np.nonzero(occ)
            for r, c in zip(oy[::2], ox[::2]):
                x0 = mx0 + c * sx
                y0 = my0 + self.map_h - (r + 1) * sy
                ld.rectangle([x0, y0, x0 + sx + 0.5, y0 + sy + 0.5],
                             fill=(130, 236, 210, 180))
            img = Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")
            d = ImageDraw.Draw(img, "RGBA")

        # ------------------------------------------------------------- trail
        if trail and len(trail) > 1:
            pts = [self.to_px(p[0], p[1]) for p in trail]
            d.line(pts, fill=(250, 190, 90, 130), width=2)

        # -------------------------------------------------------------- path
        if path and len(path) > 1:
            pts = [self.to_px(p[0], p[1]) for p in path]
            d.line(pts, fill=(90, 235, 170, 210), width=3)
            for p in pts[:: max(1, len(pts) // 12)]:
                d.ellipse([p[0] - 2, p[1] - 2, p[0] + 2, p[1] + 2],
                          fill=(200, 255, 225, 220))

        # -------------------------------------------------------------- goal
        if goal is not None:
            gx, gy = self.to_px(goal[0], goal[1])
            r = max(6.0, 0.55 * self.scale)
            d.ellipse([gx - r, gy - r, gx + r, gy + r],
                      outline=(250, 210, 90, 255), width=3)
            d.line([gx - r * 1.3, gy, gx + r * 1.3, gy], fill=(250, 210, 90, 255), width=2)
            d.line([gx, gy - r * 1.3, gx, gy + r * 1.3], fill=(250, 210, 90, 255), width=2)
            d.text((gx + r + 4, gy - r), "GOAL", font=self.font_small,
                   fill=(250, 220, 120))

        # -------------------------------------------------------------- carts
        for cx, cy, hw, hh in (carts if carts is not None else []):
            x0, y0 = self.to_px(cx - hw, cy + hh)
            x1, y1 = self.to_px(cx + hw, cy - hh)
            d.rectangle([x0, y0, x1, y1], fill=(90, 150, 220),
                        outline=(40, 70, 110), width=2)

        # -------------------------------------------------------------- lidar
        px, py = self.to_px(pose[0], pose[1])
        if scan is not None and beam_angles is not None:
            for i in range(len(scan)):
                ang = pose[3] + float(beam_angles[i])
                rr = float(scan[i])
                if rr >= C.LIDAR_RANGE_MAX - 1e-6:
                    continue
                sx, sy = pose[0] + rr * math.cos(ang), pose[1] + rr * math.sin(ang)
                qx, qy = self.to_px(sx, sy)
                d.ellipse([qx - 1.3, qy - 1.3, qx + 1.3, qy + 1.3],
                          fill=(120, 240, 245, 200))

        # --------------------------------------------------------- humanoid
        scale = self.scale
        ca, sa = math.cos(pose[3]), math.sin(pose[3])

        def rot(dx, dy):
            return (px + (dx * ca - dy * sa) * scale * 1.8, py - (dx * sa + dy * ca) * scale * 1.8)

        p = 2.0 * math.pi * phase
        # legs (top-down: swing fore/aft around the body centre)
        for side, sign in (("L", 1.0), ("R", -1.0)):
            swing = 0.16 * math.sin(p + (0.0 if sign > 0 else math.pi))
            knee = 0.06 * max(0.0, math.sin(p + (0.6 if sign > 0 else math.pi + 0.6)))
            hip_pt = rot(0.0, 0.10 * sign)
            knee_pt = rot(swing, 0.16 * sign + 0.02 * sign)
            foot_pt = rot(swing * 1.6 + knee, 0.20 * sign)
            d.line([hip_pt, knee_pt, foot_pt], fill=(210, 210, 220), width=3)
            d.ellipse([foot_pt[0] - 2.5, foot_pt[1] - 2.5, foot_pt[0] + 2.5, foot_pt[1] + 2.5],
                      fill=(250, 250, 255))
        # torso + arms + head
        torso = [rot(0.16, 0.12), rot(0.16, -0.12), rot(-0.14, -0.12), rot(-0.14, 0.12)]
        d.polygon(torso, fill=self.palette["robot"], outline=(255, 245, 220))
        arm_swing = 0.14 * math.sin(p + math.pi)
        for sign in (1.0, -1.0):
            shoulder = rot(0.05, 0.14 * sign)
            hand = rot(0.05 + arm_swing * sign, 0.20 * sign)
            d.line([shoulder, hand], fill=(220, 220, 230), width=2)
        head = rot(0.10, 0.0)
        d.ellipse([head[0] - 4.5, head[1] - 4.5, head[0] + 4.5, head[1] + 4.5],
                  fill=(250, 235, 210), outline=(120, 110, 90))
        d.line([px, py, rot(0.45, 0.0)[0], rot(0.45, 0.0)[1]],
               fill=(255, 250, 220), width=2)

        # --------------------------------------------------------------- HUD
        y0 = self.margin + self.map_h + 8
        d.rectangle([0, self.margin + self.map_h, self.width, self.height], fill=(8, 12, 10))
        d.text((self.margin + 4, y0), f"MODE  {mode}", font=self.font_big,
               fill=(120, 235, 190))
        if message:
            d.text((self.margin + 4, y0 + 28), message[:56], font=self.font_small,
                   fill=(170, 194, 176))
        if hud:
            items = list(hud.items())[:6]
            col_w = (self.width - 2 * self.margin) // 3
            for i, (k, v) in enumerate(items):
                col = i % 3
                row = i // 3
                x = self.margin + 4 + col * col_w
                y = y0 + 50 + row * 22
                d.text((x, y), f"{k}: {v}", font=self.font_small, fill=(200, 216, 206))
        if key_label:
            d.text((self.width - 210, y0 + 2), f"KEY  {key_label}", font=self.font_big,
                   fill=(250, 210, 90))
        d.text((self.width - 210, y0 + 26), f"COVER {coverage * 100:5.1f}%",
               font=self.font_small, fill=(150, 220, 190))
        # balance gauge
        gx0, gy0 = self.width - 210, y0 + 48
        d.text((gx0, gy0), "BAL", font=self.font_small, fill=(190, 200, 210))
        d.rectangle([gx0 + 30, gy0 + 2, gx0 + 150, gy0 + 14], outline=(120, 130, 140))
        ratio = float(np.clip(0.5 + 0.5 * balance / C.BALANCE_LIMIT, 0.0, 1.0))
        colour = (90, 220, 120) if abs(balance) < 0.6 * C.BALANCE_LIMIT else (
            (245, 200, 70) if abs(balance) < C.BALANCE_LIMIT else (240, 80, 70))
        d.rectangle([gx0 + 32, gy0 + 4, gx0 + 32 + 116 * ratio, gy0 + 12], fill=colour)
        d.text((gx0 + 155, gy0), f"FALL {falls}", font=self.font_small, fill=(240, 160, 150))
        return img

    def render_simple(self, pose, scan=None, beam_angles=None, hud=None,
                      mode="TELEOP", key_label="", message="", trail=None):
        return self.render(pose, scan=scan, beam_angles=beam_angles, hud=hud,
                           mode=mode, key_label=key_label, message=message,
                           trail=trail)


def draw_map_only(scene: Scene, width_px: int = 740) -> Image.Image:
    r = TopDownRenderer(scene, width_px=width_px, hud_h=0)
    return r.render(scene.start, goal=scene.goal, mode=scene.name.upper())


def frames_to_gif(frames: List[Image.Image], path: str, fps: int = 8,
                  size: Optional[Tuple[int, int]] = None, colors: int = 64) -> int:
    if not frames:
        return 0
    if size is None:
        w = 800
        h = int(round(w * frames[0].height / max(1, frames[0].width)))
        size = (w, h)
    out = []
    for frame in frames:
        img = frame.convert("RGB").resize(size, Image.LANCZOS)
        out.append(img.quantize(colors=colors, method=Image.MEDIANCUT))
    out[0].save(path, save_all=True, append_images=out[1:],
                duration=int(1000 / fps), loop=0, optimize=True, disposal=2)
    return os.path.getsize(path)
