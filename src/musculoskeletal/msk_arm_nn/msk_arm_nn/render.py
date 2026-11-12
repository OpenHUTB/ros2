"""Side-view renderer for the musculoskeletal arm (MSK-Arm)."""

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


class ArmRenderer:
    def __init__(self, scene: Scene, width_px: int = 760, hud_h: int = 108,
                 margin: int = 14):
        self.scene = scene
        self.hud_h = hud_h
        self.margin = margin
        self.x0, self.x1 = C.WORK_X
        self.y0, self.y1 = C.WORK_Y
        self.scale = (width_px - 2 * margin) / max(self.x1 - self.x0, self.y1 - self.y0)
        self.map_w = int(round((self.x1 - self.x0) * self.scale))
        self.map_h = int(round((self.y1 - self.y0) * self.scale))
        self.width = self.map_w + 2 * margin
        self.height = self.map_h + 2 * margin + hud_h
        self.palette = scene.palette
        self.font = _load_font(15)
        self.font_small = _load_font(13)
        self.font_big = _load_font(19)
        self._base = self._build_base()

    # ------------------------------------------------------------------ utils
    def to_px(self, x: float, y: float) -> Tuple[float, float]:
        return (self.margin + (x - self.x0) * self.scale,
                self.margin + (self.y1 - y) * self.scale)

    def _build_base(self) -> Image.Image:
        img = Image.new("RGB", (self.width, self.height), (14, 16, 20))
        d = ImageDraw.Draw(img)
        bg = self.palette["bg"]
        d.rectangle([self.margin, self.margin, self.margin + self.map_w,
                     self.margin + self.map_h], fill=bg)
        # 5 cm grid
        for gx in np.arange(-0.05, 0.66, 0.05):
            p = self.to_px(float(gx), 0)
            d.line([(p[0], self.margin), (p[0], self.margin + self.map_h)],
                   fill=tuple(min(255, c + 10) for c in bg))
        for gy in np.arange(-0.05, 0.66, 0.05):
            p = self.to_px(0, float(gy))
            d.line([(self.margin, p[1]), (self.margin + self.map_w, p[1])],
                   fill=tuple(min(255, c + 10) for c in bg))
        # table / floor line and base
        a = self.to_px(self.x0, 0.0)
        b = self.to_px(self.x1, 0.0)
        d.line([a, b], fill=self.palette["table"], width=5)
        bx, by = self.to_px(0.0, 0.0)
        d.ellipse([bx - 9, by - 9, bx + 9, by + 9], fill=(90, 96, 106),
                  outline=(200, 205, 215))
        # obstacles
        for cx, cy, hw, hh in self.scene.obstacles:
            x0p, y0p = self.to_px(cx - hw, cy + hh)
            x1p, y1p = self.to_px(cx + hw, cy - hh)
            d.rectangle([x0p, y0p, x1p, y1p], fill=(96, 100, 112),
                        outline=(150, 155, 170), width=2)
        d.rectangle([self.margin, self.margin, self.margin + self.map_w,
                     self.margin + self.map_h], outline=(90, 96, 104), width=2)
        return img

    # ------------------------------------------------------------------- draw
    def render(self,
               q: Sequence[float],
               activations: Optional[Sequence[float]] = None,
               path: Optional[Sequence[Sequence[float]]] = None,
               goal: Optional[Sequence[float]] = None,
               trail: Optional[Sequence[Sequence[float]]] = None,
               occupancy: Optional[np.ndarray] = None,
               hud: Optional[Dict[str, str]] = None,
               mode: str = "TELEOP",
               key_label: str = "",
               message: str = "",
               coverage: float = 0.0,
               time: float = 0.0,
               torque: float = 0.0,
               ee_error: float = 0.0,
               reached: int = 0,
               collisions: int = 0,
               ) -> Image.Image:
        img = self._base.copy()
        d = ImageDraw.Draw(img, "RGBA")
        mx0, my0 = self.margin, self.margin

        # ------------------------------------------------------ workspace map
        if occupancy is not None:
            prob = np.asarray(occupancy, dtype=np.float64)
            occ = prob > 0.65
            rows, cols = occ.shape
            sx = self.map_w / cols
            sy = self.map_h / rows
            layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
            ld = ImageDraw.Draw(layer)
            oy, ox = np.nonzero(occ)
            for r, c in zip(oy[::2], ox[::2]):
                x0 = mx0 + c * sx
                y0 = my0 + self.map_h - (r + 1) * sy
                ld.rectangle([x0, y0, x0 + sx + 0.5, y0 + sy + 0.5],
                             fill=(130, 236, 210, 170))
            img = Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")
            d = ImageDraw.Draw(img, "RGBA")

        # ------------------------------------------------------------- trail
        if trail and len(trail) > 1:
            pts = [self.to_px(p[0], p[1]) for p in trail]
            d.line(pts, fill=(250, 190, 90, 120), width=2)

        # -------------------------------------------------------------- path
        if path and len(path) > 1:
            pts = [self.to_px(p[0], p[1]) for p in path]
            d.line(pts, fill=(90, 235, 170, 210), width=3)

        # ------------------------------------------------------------ targets
        targets = self.scene.targets
        for i, (tx, ty) in enumerate(targets):
            px, py = self.to_px(tx, ty)
            r = C.TARGET_TOLERANCE * self.scale
            active = (goal is not None and i == int(round((goal[0] - tx) * 1e6)) % 1000)
            done = i < reached
            colour = ((120, 220, 140) if done else
                      (250, 200, 90) if (goal is not None and abs(goal[0] - tx) < 1e-6
                                         and abs(goal[1] - ty) < 1e-6) else (150, 160, 175))
            d.ellipse([px - r, py - r, px + r, py + r], outline=colour, width=3)
            d.text((px + r + 3, py - r - 4), "T%d" % (i + 1), font=self.font_small,
                   fill=colour)

        # --------------------------------------------------------------- arm
        q1, q2 = float(q[0]), float(q[1])
        base = self.to_px(0.0, 0.0)
        elbow = self.to_px(C.L1 * math.cos(q1), C.L1 * math.sin(q1))
        wrist = self.to_px(C.L1 * math.cos(q1) + C.L2 * math.cos(q1 + q2),
                           C.L1 * math.sin(q1) + C.L2 * math.sin(q1 + q2))
        acts = list(activations) if activations is not None else [0.0] * 4

        def muscle_colour(a: float):
            a = float(np.clip(a, 0.0, 1.0))
            return (int(170 + 70 * a), int(120 - 70 * a), int(110 - 70 * a))

        # muscle lines: shoulder pair along upper arm, elbow pair along forearm
        def offset(p0, p1, dist_px):
            dx, dy = p1[0] - p0[0], p1[1] - p0[1]
            n = math.hypot(dx, dy) or 1.0
            nx, ny = -dy / n, dx / n
            return (p0[0] + nx * dist_px, p0[1] + ny * dist_px), \
                   (p1[0] + nx * dist_px, p1[1] + ny * dist_px)

        for idx, (p0, p1, side) in enumerate((
                (base, elbow, +1), (base, elbow, -1), (elbow, wrist, +1), (elbow, wrist, -1))):
            a = acts[idx] if idx < len(acts) else 0.0
            width = int(3 + 5 * float(np.clip(a, 0, 1)))
            q0, q1p = offset(p0, p1, side * 7)
            d.line([q0, q1p], fill=muscle_colour(a), width=width)
        d.line([base, elbow, wrist], fill=self.palette["bone"], width=6)
        d.ellipse([elbow[0] - 5, elbow[1] - 5, elbow[0] + 5, elbow[1] + 5],
                  fill=(240, 240, 245))
        d.ellipse([wrist[0] - 7, wrist[1] - 7, wrist[0] + 7, wrist[1] + 7],
                  fill=(255, 235, 190), outline=(150, 130, 90), width=2)

        # --------------------------------------------------------------- HUD
        y0 = self.margin + self.map_h + 8
        d.rectangle([0, self.margin + self.map_h, self.width, self.height],
                    fill=(8, 12, 10))
        d.text((self.margin + 4, y0), f"MODE  {mode}", font=self.font_big,
               fill=(120, 235, 190))
        if message:
            d.text((self.margin + 4, y0 + 28), message[:54], font=self.font_small,
                   fill=(170, 194, 176))
        hud = hud or {}
        items = list(hud.items())[:6]
        col_w = (self.width - 2 * self.margin) // 3
        for i, (k, v) in enumerate(items):
            col = i % 3
            row = i // 3
            x = self.margin + 4 + col * col_w
            y = y0 + 50 + row * 22
            d.text((x, y), f"{k}: {v}", font=self.font_small, fill=(200, 216, 206))
        if key_label:
            d.text((self.width - 200, y0 + 2), f"KEY  {key_label}", font=self.font_big,
                   fill=(250, 210, 90))
        # activation bars
        for i, a in enumerate(acts[:4]):
            bx = self.width - 200 + (i % 2) * 95
            by = y0 + 26 + (i // 2) * 16
            d.rectangle([bx, by, bx + 70, by + 9], outline=(110, 120, 130))
            fill_w = int(70 * float(np.clip(a, 0, 1)))
            d.rectangle([bx, by, bx + fill_w, by + 9], fill=muscle_colour(a))
        d.text((self.width - 200, y0 + 60), f"ERR {ee_error * 100:4.1f}cm  TQ {torque:5.2f}",
               font=self.font_small, fill=(230, 210, 160))
        d.text((self.width - 200, y0 + 76), f"REACH {reached}  HIT {collisions}",
               font=self.font_small, fill=(200, 160, 160))
        d.text((self.margin + 4, y0 + 92), f"COVER {coverage * 100:5.1f}%  t={time:5.1f}s",
               font=self.font_small, fill=(150, 220, 190))
        return img


def draw_map_only(scene: Scene, width_px: int = 700) -> Image.Image:
    r = ArmRenderer(scene, width_px=width_px, hud_h=0)
    return r.render(scene.start_q, goal=scene.targets[0] if scene.targets else None,
                    mode=scene.name.upper(), time=0.0)


def frames_to_gif(frames: List[Image.Image], path: str, fps: int = 8,
                  size: Optional[Tuple[int, int]] = None, colors: int = 64) -> int:
    if not frames:
        return 0
    if size is None:
        w = 760
        h = int(round(w * frames[0].height / max(1, frames[0].width)))
        size = (w, h)
    out = []
    for frame in frames:
        img = frame.convert("RGB").resize(size, Image.LANCZOS)
        out.append(img.quantize(colors=colors, method=Image.MEDIANCUT))
    out[0].save(path, save_all=True, append_images=out[1:],
                duration=int(1000 / fps), loop=0, optimize=True, disposal=2)
    return os.path.getsize(path)
