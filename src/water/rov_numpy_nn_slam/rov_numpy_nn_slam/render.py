"""Top-down OpenCV/PIL renderer for the ROV simulation, SLAM map and planning."""

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
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
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
    """Renders the ROV, sonar returns, SLAM map, plan and HUD into a PIL image."""

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
        img = Image.new("RGB", (self.width, self.height), (8, 18, 26))
        d = ImageDraw.Draw(img)
        # water body with a soft vertical gradient (deeper = darker)
        top = self.palette["water"]
        bottom = tuple(max(0, int(c * 0.55)) for c in top)
        for row in range(self.map_h):
            f = row / max(1, self.map_h - 1)
            col = tuple(int(a + (b - a) * f) for a, b in zip(top, bottom))
            d.line([(self.margin, self.margin + row),
                    (self.margin + self.map_w, self.margin + row)], fill=col)
        # seabed grid
        step = int(round(1.0 * self.scale))
        for gx in range(1, int(self.scene.width) + 1):
            p = self.to_px(gx, 0)
            d.line([(p[0], self.margin), (p[0], self.margin + self.map_h)],
                   fill=tuple(min(255, c + 10) for c in top), width=1)
        for gy in range(1, int(self.scene.height) + 1):
            p = self.to_px(0, gy)
            d.line([(self.margin, p[1]), (self.margin + self.map_w, p[1])],
                   fill=tuple(min(255, c + 10) for c in top), width=1)
        # obstacles
        obs = self.palette["obstacle"]
        for cx, cy, hw, hh in self.scene.rects:
            x0, y0 = self.to_px(cx - hw, cy + hh)
            x1, y1 = self.to_px(cx + hw, cy - hh)
            d.rectangle([x0, y0, x1, y1], fill=obs,
                        outline=tuple(max(0, c - 30) for c in obs), width=2)
        for cx, cy, r in self.scene.circles:
            x0, y0 = self.to_px(cx - r, cy + r)
            x1, y1 = self.to_px(cx + r, cy - r)
            d.ellipse([x0, y0, x1, y1], fill=obs,
                      outline=tuple(max(0, c - 30) for c in obs), width=2)
        # walls
        d.rectangle([self.margin, self.margin, self.margin + self.map_w,
                     self.margin + self.map_h], outline=(120, 170, 190), width=2)
        return img

    def _roof(self, draw: ImageDraw.ImageDraw, pts: Sequence[Tuple[float, float]],
              fill, outline) -> None:
        draw.polygon(list(pts), fill=fill, outline=outline)

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
        d.rectangle([mx0, my0, mx1, my1], outline=(90, 150, 170, 255), width=2)

        # ---------------------------------------------------------- SLAM map
        if occupancy is not None:
            prob = np.asarray(occupancy, dtype=np.float64)
            if occupied_mask is not None:
                occ = np.asarray(occupied_mask, dtype=bool)
            else:
                occ = prob > 0.65
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
                             fill=(44, 104, 118, 45))
            oy, ox = np.nonzero(occ)
            for r, c in zip(oy[::2], ox[::2]):
                x0 = mx0 + c * sx
                y0 = my0 + self.map_h - (r + 1) * sy
                ld.rectangle([x0, y0, x0 + sx + 0.6, y0 + sy + 0.6],
                             fill=(110, 232, 236, 190))
            img = Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")
            d = ImageDraw.Draw(img, "RGBA")

        # ------------------------------------------------------------- trail
        if trail and len(trail) > 1:
            pts = [self.to_px(p[0], p[1]) for p in trail]
            d.line(pts, fill=(250, 190, 90, 130), width=2)

        # -------------------------------------------------------------- path
        if path and len(path) > 1:
            pts = [self.to_px(p[0], p[1]) for p in path]
            d.line(pts, fill=(90, 235, 170, 200), width=3)
            for p in pts[:: max(1, len(pts) // 12)]:
                d.ellipse([p[0] - 2, p[1] - 2, p[0] + 2, p[1] + 2],
                          fill=(200, 255, 225, 220))

        # -------------------------------------------------------------- goal
        if goal is not None:
            gx, gy = self.to_px(goal[0], goal[1])
            r = 0.55 * self.scale
            d.ellipse([gx - r, gy - r, gx + r, gy + r],
                      outline=(250, 210, 90, 255), width=3)
            d.line([gx - r * 1.3, gy, gx + r * 1.3, gy], fill=(250, 210, 90, 255), width=2)
            d.line([gx, gy - r * 1.3, gx, gy + r * 1.3], fill=(250, 210, 90, 255), width=2)

        # ------------------------------------------------------------- sonar
        px, py = self.to_px(pose[0], pose[1])
        if scan is not None and beam_angles is not None:
            n = len(scan)
            pts = []
            for i in range(n):
                ang = pose[3] + float(beam_angles[i])
                rr = float(scan[i])
                if rr >= C.SONAR_RANGE_MAX - 1e-6:
                    continue
                sx, sy = pose[0] + rr * math.cos(ang), pose[1] + rr * math.sin(ang)
                qx, qy = self.to_px(sx, sy)
                pts.append((qx, qy))
                d.ellipse([qx - 2, qy - 2, qx + 2, qy + 2],
                          fill=(120, 240, 245, 230))
            # sonar fan boundary
            fov = C.SONAR_FOV
            fan = [self.to_px(pose[0], pose[1])]
            for a in (pose[3] - fov / 2, pose[3] + fov / 2):
                fan.append(self.to_px(pose[0] + C.SONAR_RANGE_MAX * math.cos(a),
                                      pose[1] + C.SONAR_RANGE_MAX * math.sin(a)))
            d.polygon(fan, outline=(70, 180, 190, 70))

        # -------------------------------------------------------- SLAM pose
        if slam_pose is not None:
            sx, sy = self.to_px(slam_pose[0], slam_pose[1])
            d.ellipse([sx - 5, sy - 5, sx + 5, sy + 5],
                      outline=(255, 255, 255, 200), width=2)

        # --------------------------------------------------------------- ROV
        hull = 0.52 * self.scale
        ang = pose[3]
        ca, sa = math.cos(ang), math.sin(ang)

        def rot(dx, dy):
            return (px + dx * ca - dy * sa, py - (dx * sa + dy * ca))

        tri = [rot(hull, 0), rot(-hull * 0.65, hull * 0.55),
               rot(-hull * 0.65, -hull * 0.55)]
        self._roof(d, tri, (250, 176, 60), (255, 236, 190))
        # thruster pods
        for tx, ty in ((0.2 * hull, 0.85 * hull), (0.2 * hull, -0.85 * hull),
                       (-0.8 * hull, 0.85 * hull), (-0.8 * hull, -0.85 * hull)):
            qx, qy = rot(tx, ty)
            d.ellipse([qx - 3, qy - 3, qx + 3, qy + 3], fill=(70, 80, 90))
        # heading line
        hx, hy = rot(hull * 1.7, 0)
        d.line([px, py, hx, hy], fill=(255, 245, 220), width=2)
        # depth ring: colour shows proximity to the surface
        depth = float(pose[2])
        frac = max(0.0, min(1.0, depth / max(0.1, self.scene.depth)))
        ring = (int(120 + 120 * frac), int(220 - 120 * frac), 220, 220)
        d.ellipse([px - hull * 1.25, py - hull * 1.25,
                   px + hull * 1.25, py + hull * 1.25], outline=ring, width=2)

        # --------------------------------------------------------------- HUD
        y0 = self.margin + self.map_h + 8
        d.rectangle([0, self.margin + self.map_h, self.width, self.height],
                    fill=(6, 14, 20))
        d.text((self.margin + 4, y0), f"MODE  {mode}", font=self.font_big,
               fill=(120, 235, 240))
        if message:
            d.text((self.margin + 4, y0 + 30), message[:58], font=self.font_small,
                   fill=(170, 194, 206))
        if hud:
            items = list(hud.items())[:6]
            col_w = (self.width - 2 * self.margin) // 3
            for i, (k, v) in enumerate(items):
                col = i % 3
                row = i // 3
                x = self.margin + 4 + col * col_w
                y = y0 + 48 + row * 22
                d.text((x, y), f"{k}: {v}", font=self.font_small, fill=(200, 216, 224))
        if key_label:
            d.text((self.width - 210, y0 + 2), f"KEY  {key_label}", font=self.font_big,
                   fill=(250, 210, 90))
        d.text((self.width - 210, y0 + 34), f"COVER {coverage * 100:5.1f}%",
               font=self.font_small, fill=(150, 220, 200))
        return img

    def render_simple(self, pose, scan=None, beam_angles=None, hud=None,
                      mode="TELEOP", key_label="", message="",
                      trail=None) -> Image.Image:
        return self.render(pose, scan=scan, beam_angles=beam_angles, hud=hud,
                           mode=mode, key_label=key_label, message=message,
                           trail=trail)


def draw_map_only(scene: Scene, occupancy: Optional[np.ndarray] = None,
                  width_px: int = 720) -> Image.Image:
    """Static map view (used by docs screenshots)."""
    r = TopDownRenderer(scene, width_px=width_px, hud_h=0)
    if occupancy is None:
        occ = scene.occupancy()
        mask = scene.occupancy()
        return r.render(scene.start, occupancy=occ.astype(np.float64),
                        occupied_mask=mask, mode=scene.name.upper())
    return r.render(scene.start, occupancy=occupancy, mode=scene.name.upper())


def frames_to_gif(frames: List[Image.Image], path: str, fps: int = 8,
                  size: Tuple[int, int] = (800, 500)) -> int:
    """Save a list of RGB frames as an optimised GIF; returns file size."""
    if not frames:
        return 0
    out = []
    for frame in frames:
        img = frame.convert("RGB").resize(size, Image.LANCZOS)
        out.append(img.quantize(colors=64, method=Image.MEDIANCUT))
    out[0].save(path, save_all=True, append_images=out[1:],
                duration=int(1000 / fps), loop=0, optimize=True, disposal=2)
    return os.path.getsize(path)
