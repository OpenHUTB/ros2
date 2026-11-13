"""Top-down renderer for the aerial scene, SLAM map, plan and HUD."""

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
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
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
    """Renders the UAV, lidar, SLAM map, plan, no-fly zones and HUD."""

    def __init__(self, scene: Scene, width_px: int = 880, hud_h: int = 92,
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
        px = self.margin + x * self.scale
        py = self.margin + (self.scene.height - y) * self.scale
        return px, py

    def _build_base(self) -> Image.Image:
        img = Image.new("RGB", (self.width, self.height), (10, 22, 18))
        d = ImageDraw.Draw(img)
        ground = self.palette["ground"]
        d.rectangle([self.margin, self.margin,
                     self.margin + self.map_w, self.margin + self.map_h], fill=ground)
        # 10 m grid
        step = 10.0
        for gx in range(0, int(self.scene.width) + 1, int(step)):
            p = self.to_px(gx, 0)
            d.line([(p[0], self.margin), (p[0], self.margin + self.map_h)],
                   fill=tuple(min(255, c + 12) for c in ground), width=1)
        for gy in range(0, int(self.scene.height) + 1, int(step)):
            p = self.to_px(0, gy)
            d.line([(self.margin, p[1]), (self.margin + self.map_w, p[1])],
                   fill=tuple(min(255, c + 12) for c in ground), width=1)
        # buildings with height-dependent shade
        for cx, cy, hw, hh, hgt in self.scene.rects:
            x0, y0 = self.to_px(cx - hw, cy + hh)
            x1, y1 = self.to_px(cx + hw, cy - hh)
            f = min(1.0, hgt / 80.0)
            col = (int(66 + 70 * f), int(72 + 64 * f), int(78 + 60 * f))
            d.rectangle([x0, y0, x1, y1], fill=col,
                        outline=tuple(max(0, c - 26) for c in col), width=2)
            d.line([x0 + 2, y0 + 2, x1 - 2, y1 - 2], fill=(150, 156, 162), width=1)
        for cx, cy, r, hgt in self.scene.circles:
            x0, y0 = self.to_px(cx - r, cy + r)
            x1, y1 = self.to_px(cx + r, cy - r)
            f = min(1.0, hgt / 90.0)
            col = (int(70 + 70 * f), int(64 + 62 * f), int(54 + 52 * f))
            d.ellipse([x0, y0, x1, y1], fill=col,
                      outline=tuple(max(0, c - 24) for c in col), width=2)
        # no-fly zones (translucent red with hatch)
        nfz = self.palette["nfz"]
        layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
        ld = ImageDraw.Draw(layer)
        for cx, cy, r in self.scene.nfz:
            x0, y0 = self.to_px(cx - r, cy + r)
            x1, y1 = self.to_px(cx + r, cy - r)
            ld.ellipse([x0, y0, x1, y1], fill=(nfz[0], nfz[1], nfz[2], 70),
                       outline=(nfz[0], nfz[1], nfz[2], 220), width=3)
            step = max(8, int(3 * self.scale))
            for k in range(-int(r * self.scale), int(r * self.scale) + step, step):
                ld.line([(x0 + k, y0), (x0 + k + (y1 - y0), y1)],
                        fill=(nfz[0], nfz[1], nfz[2], 70), width=1)
        img = Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")
        d = ImageDraw.Draw(img)
        for cx, cy, r in self.scene.nfz:
            x0, y0 = self.to_px(cx - r, cy + r)
            d.text((x0 + 6, y0 + 4), "NFZ", font=self.font_small,
                   fill=(255, 220, 220))
        d.rectangle([self.margin, self.margin, self.margin + self.map_w,
                     self.margin + self.map_h], outline=(120, 170, 150), width=2)
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
               coverage: float = 0.0) -> Image.Image:
        img = self._base.copy()
        d = ImageDraw.Draw(img, "RGBA")
        mx0, my0 = self.margin, self.margin
        mx1, my1 = self.margin + self.map_w, self.margin + self.map_h

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
                ld.rectangle([x0, y0, x0 + sx + 0.6, y0 + sy + 0.6],
                             fill=(60, 120, 104, 45))
            oy, ox = np.nonzero(occ)
            for r, c in zip(oy[::2], ox[::2]):
                x0 = mx0 + c * sx
                y0 = my0 + self.map_h - (r + 1) * sy
                ld.rectangle([x0, y0, x0 + sx + 0.6, y0 + sy + 0.6],
                             fill=(120, 232, 210, 190))
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
            r = 2.2 * self.scale
            d.ellipse([gx - r, gy - r, gx + r, gy + r],
                      outline=(250, 210, 90, 255), width=3)
            d.line([gx - r * 1.3, gy, gx + r * 1.3, gy], fill=(250, 210, 90, 255), width=2)
            d.line([gx, gy - r * 1.3, gx, gy + r * 1.3], fill=(250, 210, 90, 255), width=2)
            d.text((gx + r + 4, gy - r), "PAD", font=self.font_small,
                   fill=(250, 220, 120))

        # ------------------------------------------------------------- lidar
        px, py = self.to_px(pose[0], pose[1])
        if scan is not None and beam_angles is not None:
            for i in range(len(scan)):
                ang = pose[3] + float(beam_angles[i])
                rr = float(scan[i])
                if rr >= C.LIDAR_RANGE_MAX - 1e-6:
                    continue
                sx, sy = pose[0] + rr * math.cos(ang), pose[1] + rr * math.sin(ang)
                qx, qy = self.to_px(sx, sy)
                d.ellipse([qx - 2.0, qy - 2.0, qx + 2.0, qy + 2.0],
                          fill=(120, 240, 245, 220))

        # -------------------------------------------------------- SLAM pose
        if slam_pose is not None:
            sx, sy = self.to_px(slam_pose[0], slam_pose[1])
            d.ellipse([sx - 5, sy - 5, sx + 5, sy + 5],
                      outline=(255, 255, 255, 200), width=2)

        # --------------------------------------------------------------- UAV
        hull = 1.5 * self.scale
        ang = pose[3]
        ca, sa = math.cos(ang), math.sin(ang)

        def rot(dx, dy):
            return (px + dx * ca - dy * sa, py - (dx * sa + dy * ca))

        # four rotor arms + rotors
        for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            ex, ey = rot(sx * hull * 0.85, sy * hull * 0.85)
            d.line([px, py, ex, ey], fill=(230, 236, 240), width=3)
            d.ellipse([ex - hull * 0.32, ey - hull * 0.32,
                       ex + hull * 0.32, ey + hull * 0.32],
                      outline=(250, 250, 250), width=2,
                      fill=(70, 90, 96, 160))
        d.ellipse([px - hull * 0.34, py - hull * 0.34,
                   px + hull * 0.34, py + hull * 0.34],
                  fill=(250, 176, 60), outline=(255, 236, 190), width=2)
        # heading line
        hx, hy = rot(hull * 1.35, 0)
        d.line([px, py, hx, hy], fill=(255, 245, 220), width=2)
        # altitude ring (colour encodes altitude)
        alt = float(pose[2])
        frac = max(0.0, min(1.0, alt / max(1.0, C.MAX_ALT)))
        ring = (int(90 + 130 * frac), int(230 - 120 * frac), 220, 220)
        d.ellipse([px - hull * 1.05, py - hull * 1.05,
                   px + hull * 1.05, py + hull * 1.05], outline=ring, width=2)

        # --------------------------------------------------------------- HUD
        y0 = self.margin + self.map_h + 8
        d.rectangle([0, self.margin + self.map_h, self.width, self.height],
                    fill=(8, 16, 12))
        d.text((self.margin + 4, y0), f"MODE  {mode}", font=self.font_big,
               fill=(120, 235, 200))
        if message:
            d.text((self.margin + 4, y0 + 30), message[:58], font=self.font_small,
                   fill=(170, 194, 176))
        if hud:
            items = list(hud.items())[:6]
            col_w = (self.width - 2 * self.margin) // 3
            for i, (k, v) in enumerate(items):
                col = i % 3
                row = i // 3
                x = self.margin + 4 + col * col_w
                y = y0 + 48 + row * 22
                d.text((x, y), f"{k}: {v}", font=self.font_small, fill=(200, 216, 206))
        if key_label:
            d.text((self.width - 210, y0 + 2), f"KEY  {key_label}", font=self.font_big,
                   fill=(250, 210, 90))
        d.text((self.width - 210, y0 + 34), f"COVER {coverage * 100:5.1f}%",
               font=self.font_small, fill=(150, 220, 190))
        return img

    def render_simple(self, pose, scan=None, beam_angles=None, hud=None,
                      mode="TELEOP", key_label="", message="",
                      trail=None) -> Image.Image:
        return self.render(pose, scan=scan, beam_angles=beam_angles, hud=hud,
                           mode=mode, key_label=key_label, message=message,
                           trail=trail)


def draw_map_only(scene: Scene, occupancy: Optional[np.ndarray] = None,
                  width_px: int = 720) -> Image.Image:
    """Static scene map for the documentation (buildings + NFZ + start pad)."""
    r = TopDownRenderer(scene, width_px=width_px, hud_h=0)
    if occupancy is None:
        return r.render(scene.start, goal=scene.goal, mode=scene.name.upper())
    return r.render(scene.start, occupancy=occupancy, goal=scene.goal,
                    mode=scene.name.upper())


def frames_to_gif(frames: List[Image.Image], path: str, fps: int = 8,
                  size: Tuple[int, int] = (800, 500), colors: int = 64) -> int:
    if not frames:
        return 0
    out = []
    for frame in frames:
        img = frame.convert("RGB").resize(size, Image.LANCZOS)
        out.append(img.quantize(colors=colors, method=Image.MEDIANCUT))
    out[0].save(path, save_all=True, append_images=out[1:],
                duration=int(1000 / fps), loop=0, optimize=True, disposal=2)
    return os.path.getsize(path)
