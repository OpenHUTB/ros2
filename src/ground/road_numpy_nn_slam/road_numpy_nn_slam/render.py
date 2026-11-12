"""Top-down renderer for the road-vehicle scene, SLAM map, plan and HUD."""

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
    """Renders the car, lidar, SLAM map, plan, traffic lights and HUD."""

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
        return (self.margin + x * self.scale,
                self.margin + (self.scene.height - y) * self.scale)

    def _build_base(self) -> Image.Image:
        img = Image.new("RGB", (self.width, self.height), (18, 24, 18))
        d = ImageDraw.Draw(img)
        ground = self.palette["ground"]
        d.rectangle([self.margin, self.margin,
                     self.margin + self.map_w, self.margin + self.map_h], fill=ground)
        # 10 m grid
        for gx in range(0, int(self.scene.width) + 1, 10):
            p = self.to_px(gx, 0)
            d.line([(p[0], self.margin), (p[0], self.margin + self.map_h)],
                   fill=tuple(min(255, c + 10) for c in ground), width=1)
        for gy in range(0, int(self.scene.height) + 1, 10):
            p = self.to_px(0, gy)
            d.line([(self.margin, p[1]), (self.margin + self.map_w, p[1])],
                   fill=tuple(min(255, c + 10) for c in ground), width=1)
        road = self.palette["road"]
        curb = self.palette["curb"]
        lane = self.palette["lane"]
        # road surfaces with curb outlines
        for cx, cy, hw, hh in self.scene.roads:
            x0, y0 = self.to_px(cx - hw, cy + hh)
            x1, y1 = self.to_px(cx + hw, cy - hh)
            d.rectangle([x0, y0, x1, y1], fill=road, outline=curb, width=2)
        # lane centre dashes (only on non-intersection parts)
        for cx, cy, hw, hh in self.scene.roads:
            if hw > hh:      # east-west street
                y = cy
                x = cx - hw
                while x < cx + hw:
                    if not self._in_intersection(x, y):
                        a = self.to_px(x, y)
                        b = self.to_px(min(x + 3.0, cx + hw), y)
                        d.line([a, b], fill=lane, width=2)
                    x += 6.0
            else:            # north-south street
                x = cx
                y = cy - hh
                while y < cy + hh:
                    if not self._in_intersection(x, y):
                        a = self.to_px(x, y)
                        b = self.to_px(x, min(y + 3.0, cy + hh))
                        d.line([a, b], fill=lane, width=2)
                    y += 6.0
        # crosswalks near traffic lights
        for light in self.scene.lights:
            lx, ly = float(light["x"]), float(light["y"])
            axis = str(light.get("axis", "x"))
            if axis == "x":
                x0, y0 = self.to_px(lx - 4.5, ly + 4.0)
                x1, y1 = self.to_px(lx - 3.0, ly - 4.0)
            else:
                x0, y0 = self.to_px(lx - 4.0, ly + 4.5)
                x1, y1 = self.to_px(lx + 4.0, ly + 3.0)
            for k in range(6):
                f = k / 6.0
                g = (k + 0.4) / 6.0
                if axis == "x":
                    d.rectangle([x0 + (x1 - x0) * f, y0, x0 + (x1 - x0) * g, y1],
                                fill=(226, 222, 200))
                else:
                    d.rectangle([x0, y0 + (y1 - y0) * f, x1, y0 + (y1 - y0) * g],
                                fill=(226, 222, 200))
        # barriers with hazard stripes
        for cx, cy, hw, hh in self.scene.barriers:
            x0, y0 = self.to_px(cx - hw, cy + hh)
            x1, y1 = self.to_px(cx + hw, cy - hh)
            d.rectangle([x0, y0, x1, y1], fill=(200, 120, 40), outline=(120, 70, 20), width=2)
        # parked cars
        car_col = self.palette["car"]
        for i, (cx, cy, hw, hh) in enumerate(self.scene.parked):
            x0, y0 = self.to_px(cx - hw, cy + hh)
            x1, y1 = self.to_px(cx + hw, cy - hh)
            shade = tuple(max(0, c - (i % 3) * 18) for c in car_col)
            d.rounded_rectangle([x0, y0, x1, y1], radius=3, fill=shade,
                                outline=(30, 34, 40), width=1)
        # cones
        for cx, cy, r in self.scene.cones:
            x0, y0 = self.to_px(cx - r, cy + r)
            x1, y1 = self.to_px(cx + r, cy - r)
            d.ellipse([x0, y0, x1, y1], fill=(238, 120, 40), outline=(120, 60, 20))
        # traffic light poles (colour is drawn per frame)
        d.rectangle([self.margin, self.margin, self.margin + self.map_w,
                     self.margin + self.map_h], outline=(120, 140, 120), width=2)
        return img

    def _in_intersection(self, x: float, y: float) -> bool:
        verticals = [cx for cx, cy, hw, hh in self.scene.roads if hh > hw]
        horizontals = [cy for cx, cy, hw, hh in self.scene.roads if hw > hh]
        for vx in verticals:
            for hy in horizontals:
                if abs(x - vx) < 4.0 and abs(y - hy) < 4.0:
                    return True
        return False

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
               light: Optional[Dict[str, str]] = None,
               pedestrians: Optional[Sequence[Tuple[float, float]]] = None,
               steering: float = 0.0) -> Image.Image:
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
            for r, c in zip(fy[::14], fx[::14]):
                x0 = mx0 + c * sx
                y0 = my0 + self.map_h - (r + 1) * sy
                ld.rectangle([x0, y0, x0 + sx + 0.5, y0 + sy + 0.5],
                             fill=(70, 130, 100, 40))
            oy, ox = np.nonzero(occ)
            for r, c in zip(oy[::2], ox[::2]):
                x0 = mx0 + c * sx
                y0 = my0 + self.map_h - (r + 1) * sy
                ld.rectangle([x0, y0, x0 + sx + 0.5, y0 + sy + 0.5],
                             fill=(120, 232, 190, 190))
            img = Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")
            d = ImageDraw.Draw(img, "RGBA")

        # ------------------------------------------------------------- trail
        if trail and len(trail) > 1:
            pts = [self.to_px(p[0], p[1]) for p in trail]
            d.line(pts, fill=(250, 190, 90, 140), width=2)

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
            d.text((gx + r + 4, gy - r), "GOAL", font=self.font_small,
                   fill=(250, 220, 120))

        # -------------------------------------------------------------- lights
        for i, lt in enumerate(self.scene.lights):
            lx, ly = self.to_px(float(lt["x"]), float(lt["y"]))
            if light is not None and i == 0 and str(light.get("state", "")):
                state = str(light["state"])
            else:
                state = self._light_state(lt, time)
            col = {"green": (60, 220, 90), "yellow": (245, 215, 60),
                   "red": (240, 70, 60)}.get(state, (150, 150, 150))
            d.rectangle([lx - 3, ly - 18, lx + 3, ly - 10], fill=(40, 44, 48))
            d.ellipse([lx - 5, ly - 17, lx + 5, ly - 7], fill=col,
                      outline=(20, 22, 24), width=1)
            d.text((lx + 7, ly - 18), state[:1].upper(), font=self.font_small, fill=col)

        # --------------------------------------------------------- pedestrians
        peds = pedestrians if pedestrians is not None else []
        for px, py in peds:
            qx, qy = self.to_px(px, py)
            d.ellipse([qx - 3.5, qy - 5, qx + 3.5, qy + 2], fill=(240, 200, 160),
                      outline=(80, 60, 40))
            d.line([qx, qy + 2, qx, qy + 8], fill=(240, 200, 160), width=2)

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
                d.ellipse([qx - 1.4, qy - 1.4, qx + 1.4, qy + 1.4],
                          fill=(120, 240, 245, 210))

        # -------------------------------------------------------------- car
        hull = 1.3 * self.scale
        ang = pose[3]
        ca, sa = math.cos(ang), math.sin(ang)

        def rot(dx, dy):
            return (px + dx * ca - dy * sa, py - (dx * sa + dy * ca))

        body = [rot(hull * 0.9, hull * 0.34), rot(hull * 0.9, -hull * 0.34),
                rot(-hull * 0.9, -hull * 0.34), rot(-hull * 0.9, hull * 0.34)]
        d.polygon(body, fill=self.palette["car"], outline=(255, 240, 210))
        # windscreen
        ws = [rot(hull * 0.45, hull * 0.26), rot(hull * 0.45, -hull * 0.26),
              rot(hull * 0.10, -hull * 0.26), rot(hull * 0.10, hull * 0.26)]
        d.polygon(ws, fill=(40, 70, 96))
        # wheels: front wheels reflect the steering angle
        for wx, wy, front in ((hull * 0.55, hull * 0.36, True), (hull * 0.55, -hull * 0.36, True),
                              (-hull * 0.55, hull * 0.36, False), (-hull * 0.55, -hull * 0.36, False)):
            a = ang + (steering if front else 0.0)
            wx0, wy0 = hull * 0.34, hull * 0.10
            corners = [(-wx0, -wy0), (wx0, -wy0), (wx0, wy0), (-wx0, wy0)]
            pts = []
            for dx, dy in corners:
                rx = dx * math.cos(a) - dy * math.sin(a)
                ry = dx * math.sin(a) + dy * math.cos(a)
                pts.append(rot(wx + rx, wy + ry))
            d.polygon(pts, fill=(26, 26, 28), outline=(150, 150, 150))
        # heading + steering indicator
        hx, hy = rot(hull * 1.3, 0)
        d.line([px, py, hx, hy], fill=(255, 245, 220), width=2)
        d.ellipse([px - 2.2, py - 2.2, px + 2.2, py + 2.2], fill=(255, 240, 200))

        # --------------------------------------------------------------- HUD
        y0 = self.margin + self.map_h + 8
        d.rectangle([0, self.margin + self.map_h, self.width, self.height], fill=(8, 14, 10))
        d.text((self.margin + 4, y0), f"MODE  {mode}", font=self.font_big,
               fill=(120, 235, 190))
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
                y = y0 + 52 + row * 22
                d.text((x, y), f"{k}: {v}", font=self.font_small, fill=(200, 216, 206))
        if key_label:
            d.text((self.width - 210, y0 + 2), f"KEY  {key_label}", font=self.font_big,
                   fill=(250, 210, 90))
        d.text((self.width - 210, y0 + 26), f"COVER {coverage * 100:5.1f}%",
               font=self.font_small, fill=(150, 220, 190))
        return img

    def _light_state(self, light: Dict[str, float], t: float) -> str:
        phase = (t + float(light.get("offset", 0.0))) % C.LIGHT_CYCLE
        if phase < C.LIGHT_GREEN:
            return "green"
        if phase < C.LIGHT_GREEN + C.LIGHT_YELLOW:
            return "yellow"
        return "red"

    def render_simple(self, pose, scan=None, beam_angles=None, hud=None,
                      mode="TELEOP", key_label="", message="", trail=None):
        return self.render(pose, scan=scan, beam_angles=beam_angles, hud=hud,
                           mode=mode, key_label=key_label, message=message,
                           trail=trail)


def draw_map_only(scene: Scene, width_px: int = 760) -> Image.Image:
    r = TopDownRenderer(scene, width_px=width_px, hud_h=0)
    return r.render(scene.start, goal=scene.goal, mode=scene.name.upper(),
                    pedestrians=scene.pedestrian_positions(0.0))


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
