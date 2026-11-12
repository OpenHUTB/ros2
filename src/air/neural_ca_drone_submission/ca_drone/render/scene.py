"""Software renderer used for the demo videos and documentation screenshots.

Everything is drawn with Pillow: a painter's-algorithm 3-D chase view, a
ray-cast first-person RGB-D view, a bird's-eye occupancy map and a telemetry
strip.  The renderer is intentionally small (no OpenGL, no game engine) so the
same frames can be produced on a headless CI machine.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from ..sim.world import Box, Cylinder, Obstacle, World

Vec3 = Tuple[float, float, float]

SKY_TOP = (26, 40, 64)
SKY_HORIZON = (126, 152, 180)
GROUND = (40, 46, 55)
GRID = (74, 92, 112)
PATH_COLOR = (64, 224, 208)
GOAL_COLOR = (96, 230, 128)
WARN = (255, 176, 64)
DANGER = (255, 84, 84)
TEXT = (226, 234, 244)
MUTED = (140, 158, 178)

_FONT_CANDIDATES = [
    r"C:\Windows\Fonts\consola.ttf",
    r"C:\Windows\Fonts\segoeui.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
]


def _font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    name = r"C:\Windows\Fonts\consolab.ttf" if bold else _FONT_CANDIDATES[0]
    for path in ([name] + _FONT_CANDIDATES):
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                pass
    return ImageFont.load_default()


def _norm(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else v


def _mix(a: Sequence[int], b: Sequence[int], k: float) -> Tuple[int, int, int]:
    k = max(0.0, min(1.0, k))
    return tuple(int(round(a[i] * (1 - k) + b[i] * k)) for i in range(3))


def _shade(color: Sequence[int], k: float) -> Tuple[int, int, int]:
    return tuple(int(max(0, min(255, c * k))) for c in color)


class Camera:
    def __init__(self, pos: Vec3, target: Vec3, up: Vec3, fov_deg: float,
                 width: int, height: int):
        self.pos = np.asarray(pos, dtype=float)
        self.fwd = _norm(np.asarray(target, dtype=float) - self.pos)
        right = _norm(np.cross(self.fwd, np.asarray(up, dtype=float)))
        self.right = right
        self.up = np.cross(right, self.fwd)
        self.f = 0.5 * height / math.tan(math.radians(fov_deg) * 0.5)
        self.cx = width * 0.5
        self.cy = height * 0.5
        self.w = width
        self.h = height

    def to_cam(self, pts: np.ndarray) -> np.ndarray:
        d = np.asarray(pts, dtype=float) - self.pos
        return np.stack([d @ self.right, d @ self.up, d @ self.fwd], axis=-1)

    def project(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        cam = self.to_cam(pts)
        z = np.maximum(cam[..., 2], 1e-4)
        u = self.cx + self.f * cam[..., 0] / z
        v = self.cy - self.f * cam[..., 1] / z
        return np.stack([u, v], axis=-1), cam[..., 2]

    def project_point(self, p: Vec3) -> Optional[Tuple[float, float, float]]:
        cam = self.to_cam(np.asarray([p], dtype=float))[0]
        if cam[2] <= 0.12:
            return None
        u = self.cx + self.f * cam[0] / cam[2]
        v = self.cy - self.f * cam[1] / cam[2]
        return float(u), float(v), float(cam[2])


def _clip_near(cam_pts: List[np.ndarray], near: float = 0.15) -> List[np.ndarray]:
    """Sutherland-Hodgman clip of a camera-space polygon against z >= near."""
    out: List[np.ndarray] = []
    n = len(cam_pts)
    for i in range(n):
        a = cam_pts[i]
        b = cam_pts[(i + 1) % n]
        ain = a[2] >= near
        bin_ = b[2] >= near
        if ain:
            out.append(a)
        if ain != bin_:
            t = (near - a[2]) / (b[2] - a[2])
            out.append(a + t * (b - a))
    return out


class Scene3D:
    """Painter's-algorithm renderer for the chase view."""

    def __init__(self, image: Image.Image):
        self.img = image
        self.draw = ImageDraw.Draw(image, "RGBA")

    # ---------------------------------------------------------------- helpers
    def _poly(self, cam: Camera, pts: np.ndarray, fill, outline=None, width=1):
        cp = [np.asarray(p) for p in cam.to_cam(pts)]
        cp = _clip_near(cp)
        if len(cp) < 3:
            return
        arr = np.stack(cp)
        z = np.maximum(arr[:, 2], 1e-4)
        u = cam.cx + cam.f * arr[:, 0] / z
        v = cam.cy - cam.f * arr[:, 1] / z
        xy = list(zip(u.tolist(), v.tolist()))
        self.draw.polygon(xy, fill=fill)
        if outline is not None:
            self.draw.line(xy + [xy[0]], fill=outline, width=width)

    def _line3(self, cam: Camera, a: Vec3, b: Vec3, color, width=2):
        ca = cam.to_cam(np.asarray([a], dtype=float))[0]
        cb = cam.to_cam(np.asarray([b], dtype=float))[0]
        seg = [ca, cb]
        # clip against near plane
        if ca[2] < 0.15 and cb[2] < 0.15:
            return
        if ca[2] < 0.15:
            t = (0.15 - ca[2]) / (cb[2] - ca[2])
            ca = ca + t * (cb - ca)
        elif cb[2] < 0.15:
            t = (0.15 - cb[2]) / (ca[2] - cb[2])
            cb = cb + t * (ca - cb)
        pa = cam.project_point(cam.pos + ca[0] * cam.right + ca[1] * cam.up + ca[2] * cam.fwd)
        pb = cam.project_point(cam.pos + cb[0] * cam.right + cb[1] * cam.up + cb[2] * cam.fwd)
        if pa and pb:
            self.draw.line([(pa[0], pa[1]), (pb[0], pb[1])], fill=color, width=width)

    # ---------------------------------------------------------------- scenery
    def sky(self):
        w, h = self.img.size
        top = np.array(SKY_TOP, dtype=float)
        hor = np.array(SKY_HORIZON, dtype=float)
        arr = np.zeros((h, w, 3), dtype=np.uint8)
        for y in range(h):
            k = min(1.0, y / (h * 0.72))
            arr[y, :, :] = (top * (1 - k) + hor * k).astype(np.uint8)
        self.img.paste(Image.fromarray(arr), (0, 0))

    def ground(self, cam: Camera):
        x0, x1, y0, y1 = -30.0, 140.0, -90.0, 90.0
        quad = np.array([[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0]], dtype=float)
        self._poly(cam, quad, fill=GROUND + (255,))
        # grid lines with distance fade
        for x in np.arange(-20, 120, 4.0):
            a = np.array([x, -60.0, 0.0])
            b = np.array([x, 60.0, 0.0])
            self._line3(cam, a, b, GRID + (110,), 1)
        for y in np.arange(-60, 60, 4.0):
            a = np.array([-20.0, y, 0.0])
            b = np.array([120.0, y, 0.0])
            self._line3(cam, a, b, GRID + (110,), 1)

    def obstacle(self, cam: Camera, o: Obstacle, fog: float = 70.0):
        if isinstance(o, Box):
            self._box(cam, o, fog)
        else:
            self._cylinder(cam, o, fog)

    def _box(self, cam: Camera, o: Box, fog: float):
        x0, y0, z0 = o.x - o.sx / 2, o.y - o.sy / 2, o.z0
        x1, y1, z1 = o.x + o.sx / 2, o.y + o.sy / 2, o.z0 + o.sz
        v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
             (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        faces = [
            ((0, 3, 2, 1), (0, 0, -1)),
            ((4, 5, 6, 7), (0, 0, 1)),
            ((0, 1, 5, 4), (0, -1, 0)),
            ((1, 2, 6, 5), (1, 0, 0)),
            ((2, 3, 7, 6), (0, 1, 0)),
            ((3, 0, 4, 7), (-1, 0, 0)),
        ]
        light = _norm(np.array([-0.45, 0.55, 0.75]))
        for idx, n in faces:
            pts = np.array([v[i] for i in idx], dtype=float)
            cc = pts.mean(axis=0)
            if np.dot(n, cam.pos - cc) <= 0:
                continue
            depth = float(np.linalg.norm(cc - cam.pos))
            f = min(1.0, depth / fog)
            base = _shade(o.color, 0.55 + 0.45 * max(0.0, float(np.dot(n, light))))
            col = _mix(base, SKY_HORIZON, 0.55 * f) + (255,)
            self._poly(cam, pts, fill=col, outline=_shade(base, 0.55) + (180,), width=1)

    def _cylinder(self, cam: Camera, o: Cylinder, fog: float):
        seg = 16
        light = _norm(np.array([-0.45, 0.55, 0.75]))
        base = o.color
        ang = np.linspace(0, 2 * np.pi, seg, endpoint=False)
        ring = np.stack([o.x + o.radius * np.cos(ang),
                         o.y + o.radius * np.sin(ang)], axis=1)
        # side quads, back-face culled
        for i in range(seg):
            j = (i + 1) % seg
            n = np.array([math.cos((ang[i] + ang[j]) / 2),
                          math.sin((ang[i] + ang[j]) / 2), 0.0])
            cc = np.array([(ring[i, 0] + ring[j, 0]) / 2,
                           (ring[i, 1] + ring[j, 1]) / 2, o.z0 + o.height / 2])
            if np.dot(n, cam.pos - cc) <= 0:
                continue
            pts = np.array([[ring[i, 0], ring[i, 1], o.z0],
                            [ring[j, 0], ring[j, 1], o.z0],
                            [ring[j, 0], ring[j, 1], o.z0 + o.height],
                            [ring[i, 0], ring[i, 1], o.z0 + o.height]])
            depth = float(np.linalg.norm(cc - cam.pos))
            f = min(1.0, depth / fog)
            col = _shade(base, 0.55 + 0.45 * max(0.0, float(np.dot(n, light))))
            col = _mix(col, SKY_HORIZON, 0.55 * f) + (255,)
            self._poly(cam, pts, fill=col, outline=_shade(base, 0.5) + (170,))
        # top cap
        top = np.array([[ring[i, 0], ring[i, 1], o.z0 + o.height] for i in range(seg)])
        if cam.pos[2] > o.z0 + o.height:
            cc = np.array([o.x, o.y, o.z0 + o.height])
            depth = float(np.linalg.norm(cc - cam.pos))
            f = min(1.0, depth / fog)
            col = _mix(_shade(base, 1.15), SKY_HORIZON, 0.5 * f) + (255,)
            self._poly(cam, top, fill=col, outline=_shade(base, 0.6) + (180,))

    # ---------------------------------------------------------------- actors
    def drone(self, cam: Camera, p: np.ndarray, yaw: float, moving: bool = True):
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        def T(local: Sequence[float]) -> np.ndarray:
            return p + R @ np.asarray(local, dtype=float)

        body = [T((-0.28, -0.16, 0.0)), T((0.34, -0.16, 0.0)),
                T((0.34, 0.16, 0.0)), T((-0.28, 0.16, 0.0))]
        self._poly(cam, np.array(body) + np.array([0, 0, 0.03]), fill=(44, 50, 60, 255),
                   outline=(16, 18, 22, 255))
        top = [T((-0.24, -0.13, 0.10)), T((0.30, -0.13, 0.10)),
               T((0.30, 0.13, 0.10)), T((-0.24, 0.13, 0.10))]
        self._poly(cam, np.array(top), fill=(70, 78, 92, 255), outline=(20, 24, 30, 255))
        for sx, sy in ((0.42, 0.42), (0.42, -0.42), (-0.42, 0.42), (-0.42, -0.42)):
            hub = T((sx, sy, 0.16))
            arm_a = T((sx * 0.55, sy * 0.55, 0.06))
            self._line3(cam, tuple(arm_a), tuple(hub), (30, 34, 42, 255), 3)
            # rotor disc projected as a flat quad
            disc = []
            for k in range(12):
                a = 2 * math.pi * k / 12
                disc.append(T((sx + 0.20 * math.cos(a), sy + 0.20 * math.sin(a), 0.19)))
            col = (36, 40, 48, 150) if not moving else (90, 200, 220, 110)
            self._poly(cam, np.array(disc), fill=col, outline=(160, 220, 230, 90))
        # heading LED
        led = T((0.36, 0.0, 0.12))
        pp = cam.project_point(tuple(led))
        if pp:
            r = 3
            self.draw.ellipse([pp[0] - r, pp[1] - r, pp[0] + r, pp[1] + r],
                              fill=GOAL_COLOR + (255,))

    def path(self, cam: Camera, pts: np.ndarray):
        if pts is None or len(pts) < 2:
            return
        for a, b in zip(pts[:-1], pts[1:]):
            self._line3(cam, tuple(a), tuple(b), PATH_COLOR + (230,), 3)
            self._line3(cam, tuple(a), tuple(b), (220, 255, 255, 70), 1)
        for q in pts[:: max(1, len(pts) // 10)]:
            pp = cam.project_point(tuple(q))
            if pp:
                r = max(1.5, 90 / max(pp[2], 1))
                self.draw.ellipse([pp[0] - r, pp[1] - r, pp[0] + r, pp[1] + r],
                                  fill=PATH_COLOR + (255,))

    def goal(self, cam: Camera, g: np.ndarray):
        base = np.array([g[0], g[1], 0.0])
        top = np.array([g[0], g[1], g[2] + 1.2])
        self._line3(cam, tuple(base), tuple(top), GOAL_COLOR + (150,), 8)
        self._line3(cam, tuple(base), tuple(top), (230, 255, 235, 220), 2)
        pp = cam.project_point(tuple(top))
        if pp:
            self.draw.ellipse([pp[0] - 5, pp[1] - 5, pp[0] + 5, pp[1] + 5],
                              outline=GOAL_COLOR + (255,), width=2)


# --------------------------------------------------------------------------- sensors
def depth_image(world: World, pos: np.ndarray, yaw: float, width: int = 96,
                height: int = 72, fov_deg: float = 90.0,
                max_range: float = 20.0) -> Tuple[np.ndarray, np.ndarray]:
    """Render a pinhole depth image and a per-pixel object id buffer."""
    f = 0.5 * height / math.tan(math.radians(fov_deg) * 0.5)
    us = (np.arange(width) + 0.5 - width / 2) / f
    vs = (np.arange(height) + 0.5 - height / 2) / f
    uu, vv = np.meshgrid(us, vs)
    # camera frame: x right, y down, z forward (ROS REP-103 body frame)
    dir_cam = np.stack([uu, vv, np.ones_like(uu)], axis=-1).reshape(-1, 3)
    c, s = math.cos(yaw), math.sin(yaw)
    right = np.array([-s, c, 0.0])
    down = np.array([0.0, 0.0, -1.0])
    fwd = np.array([c, s, 0.0])
    dirs = (dir_cam[:, 0:1] * right + dir_cam[:, 1:2] * down +
            dir_cam[:, 2:3] * fwd)
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    depth, ids = world.raycast(pos, dirs, max_range=max_range)
    return depth.reshape(height, width), ids.reshape(height, width)


def colorize_depth(depth: np.ndarray, max_range: float = 20.0) -> Image.Image:
    d = np.clip(depth / max_range, 0, 1)
    # turbo-ish palette
    r = np.clip(1.6 - np.abs(4 * d - 3.0), 0, 1)
    g = np.clip(1.6 - np.abs(4 * d - 2.0), 0, 1)
    b = np.clip(1.6 - np.abs(4 * d - 1.0), 0, 1)
    arr = (np.stack([r, g, b], axis=-1) * 255).astype(np.uint8)
    return Image.fromarray(arr)


def ego_image(world: World, pos: np.ndarray, yaw: float, width: int = 240,
              height: int = 160, fov_deg: float = 90.0) -> Image.Image:
    w, h = width, height
    depth, ids = depth_image(world, pos, yaw, w, h, fov_deg, max_range=22.0)
    # object colours + distance fog
    obs = world.obstacles()
    arr = np.zeros((h, w, 3), dtype=np.float32)
    sky = np.array(SKY_HORIZON, dtype=np.float32)
    arr[:] = sky
    for i, o in enumerate(obs):
        mask = ids == i
        if not np.any(mask):
            continue
        base = np.array(o.color, dtype=np.float32) * 0.85
        d = depth[mask]
        fog = np.clip(d / 30.0, 0, 0.75)[:, None]
        arr[mask] = base * (1 - fog) + sky * fog
    ground_mask = (ids == -1) & (depth < 21.9)
    gd = depth[ground_mask]
    gcol = np.clip(0.55 - gd / 40.0, 0.12, 0.5)[:, None] * np.array([90, 96, 105])
    arr[ground_mask] = gcol
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(img, "RGBA")
    # scanlines + crosshair + frame
    for y in range(0, h, 3):
        d.line([(0, y), (w, y)], fill=(0, 0, 0, 22), width=1)
    d.line([(w // 2 - 8, h // 2), (w // 2 + 8, h // 2)], fill=(230, 240, 255, 190))
    d.line([(w // 2, h // 2 - 8), (w // 2, h // 2 + 8)], fill=(230, 240, 255, 190))
    d.rectangle([0, 0, w - 1, h - 1], outline=(90, 110, 130, 220))
    d.text((6, 4), "FWD RGB-D  640x480", font=_font(10), fill=(200, 220, 235, 230))
    d.text((w - 62, h - 14), "90.0 deg", font=_font(10), fill=(200, 220, 235, 200))
    return img


# --------------------------------------------------------------------------- dashboard
def _bev_panel(world: World, pos: np.ndarray, yaw: float, path: Optional[np.ndarray],
               size: Tuple[int, int] = (250, 170)) -> Image.Image:
    w, h = size
    img = Image.new("RGB", size, (18, 23, 30))
    d = ImageDraw.Draw(img, "RGBA")
    span_x = 20.0
    span_y = span_x * h / w
    x0, y0 = pos[0] - 3.5, pos[1] - span_y / 2

    def px(x, y):
        return ((x - x0) / span_x * w, h - (y - y0) / span_y * h)

    for gx in np.arange(math.floor(x0), x0 + span_x, 2.0):
        d.line([px(gx, y0), px(gx, y0 + span_y)], fill=(42, 52, 64, 255))
    for gy in np.arange(math.floor(y0), y0 + span_y, 2.0):
        d.line([px(x0, gy), px(x0 + span_x, gy)], fill=(42, 52, 64, 255))

    for o in world.obstacles():
        if isinstance(o, Box):
            ox0, ox1 = o.x - o.sx / 2, o.x + o.sx / 2
            oy0, oy1 = o.y - o.sy / 2, o.y + o.sy / 2
        else:
            ox0, ox1 = o.x - o.radius, o.x + o.radius
            oy0, oy1 = o.y - o.radius, o.y + o.radius
        if ox1 < x0 or ox0 > x0 + span_x or oy1 < y0 or oy0 > y0 + span_y:
            continue
        a = px(ox0, oy0)
        b = px(ox1, oy1)
        box = [min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])]
        if isinstance(o, Box):
            d.rectangle(box, fill=o.color + (215,), outline=(235, 240, 250, 255))
        else:
            d.ellipse(box, fill=o.color + (215,), outline=(235, 240, 250, 255))

    if path is not None and len(path) > 1:
        pts = [px(float(p[0]), float(p[1])) for p in path]
        d.line(pts, fill=PATH_COLOR + (230,), width=2)
    g = px(float(world.goal[0]), float(world.goal[1]))
    d.ellipse([g[0] - 4, g[1] - 4, g[0] + 4, g[1] + 4], outline=GOAL_COLOR + (255,), width=2)
    p = px(float(pos[0]), float(pos[1]))
    c, s = math.cos(yaw), math.sin(yaw)
    tri = [(p[0] + 7 * c, p[1] - 7 * s), (p[0] - 5 * c - 4 * s, p[1] + 5 * s - 4 * c),
           (p[0] - 5 * c + 4 * s, p[1] + 5 * s + 4 * c)]
    d.polygon(tri, fill=(245, 250, 255, 255))
    d.text((5, 4), "BEV / local map", font=_font(10), fill=(180, 200, 220, 220))
    d.rectangle([0, 0, w - 1, h - 1], outline=(80, 100, 120, 255))
    return img


def _text(d: ImageDraw.ImageDraw, xy, s, size=13, fill=TEXT, bold=False):
    d.text(xy, s, font=_font(size, bold), fill=fill)


def _rrect(d: ImageDraw.ImageDraw, box, radius: int = 6, **kwargs):
    """Rounded rectangle with a fallback for Pillow < 8.2 (Ubuntu 20.04)."""
    fn = getattr(d, "rounded_rectangle", None)
    if fn is not None:
        fn(box, radius=radius, **kwargs)
    else:  # pragma: no cover - exercised on Ubuntu 20.04 / Pillow 7
        d.rectangle(box, **kwargs)


def compose_frame(world: World, pos: np.ndarray, vel: np.ndarray, yaw: float,
                  path: Optional[np.ndarray], info: Optional[dict] = None,
                  size: Tuple[int, int] = (960, 600)) -> Image.Image:
    """Full demo frame: chase view + ego view + BEV + telemetry."""
    info = info or {}
    W, H = size
    view_h = 424
    img = Image.new("RGB", size, (12, 16, 22))
    view = Image.new("RGB", (W, view_h), SKY_TOP)
    scene = Scene3D(view)
    scene.sky()

    fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    cam_pos = pos - fwd * 8.0 + np.array([0.0, 0.0, 2.9])
    cam = Camera(tuple(cam_pos), tuple(pos + fwd * 5.0 + np.array([0, 0, 0.7])),
                 (0, 0, 1), 50.0, W, view_h)
    scene.ground(cam)

    obs = world.obstacles()
    order = sorted(obs, key=lambda o: -float(np.linalg.norm(o.center - cam.pos)))
    for o in order:
        scene.obstacle(cam, o)
    scene.goal(cam, world.goal)
    if path is not None and len(path) > 1:
        scene.path(cam, path)
    scene.drone(cam, pos, yaw)

    d = ImageDraw.Draw(view, "RGBA")
    # vignette + top/bottom fades
    for i in range(36):
        a = int(150 * (1 - i / 36))
        d.line([(0, i), (W, i)], fill=(8, 10, 14, a))
        d.line([(0, view_h - 1 - i), (W, view_h - 1 - i)], fill=(8, 10, 14, a))
    # HUD title
    _text(d, (18, 14), "NEURAL COLLISION AVOIDANCE  /  " + world.name.upper(), 16, TEXT, True)
    _text(d, (18, 38), info.get("subtitle", "perception -> neural A* -> neural control"),
          11, (170, 190, 210))
    # status pill
    mode = info.get("mode", "FULL")
    _rrect(d, [W - 190, 14, W - 16, 44], radius=6, fill=(12, 18, 26, 190),
            outline=(70, 210, 190, 220))
    _text(d, (W - 178, 21), f"MODE {mode}", 12, (120, 240, 220), True)
    _text(d, (W - 178, 48), f"scene {world.name}", 10, (150, 175, 195))
    events = info.get("events", [])[-3:]
    for i, ev in enumerate(events):
        yy = view_h - 58 + i * 16
        _rrect(d, [16, yy - 2, 16 + 8 * len(ev) + 18, yy + 14], radius=3,
               fill=(10, 14, 20, 160))
        _text(d, (24, yy), ev, 11, (222, 206, 150))
    img.paste(view, (0, 0))

    # bottom strip
    strip = Image.new("RGB", (W, H - view_h), (14, 18, 24))
    sd = ImageDraw.Draw(strip, "RGBA")
    sd.line([(0, 0), (W, 0)], fill=(48, 62, 78, 255), width=1)
    ego = ego_image(world, pos, yaw, 250, 162)
    bev = _bev_panel(world, pos, yaw, path, (250, 162))
    strip.paste(ego, (10, 9))
    strip.paste(bev, (270, 9))

    x = 536
    speed = float(np.linalg.norm(vel[:2]))
    clr = world.clearance(pos)
    _text(sd, (x, 10), "TELEMETRY", 11, MUTED, True)
    rows = [
        ("speed", f"{speed:5.2f} m/s"),
        ("clearance", f"{clr:5.2f} m"),
        ("position", f"({pos[0]:5.1f}, {pos[1]:5.1f})"),
        ("progress", f"{100.0 * info.get('progress', 0.0):5.1f} %"),
        ("plan", f"{info.get('plan_ms', 0):.1f} ms / {info.get('expanded', 0)} nodes"),
        ("infer", f"{info.get('infer_ms', 0):.1f} ms"),
        ("replans", str(info.get("replans", 0))),
    ]
    for i, (k, v) in enumerate(rows):
        yy = 30 + i * 18
        _text(sd, (x, yy), k.ljust(10), 12, MUTED)
        _text(sd, (x + 92, yy), v, 12, TEXT, True)
    # clearance bar
    bx, by, bw = x + 240, 34, 150
    sd.rectangle([bx, by, bx + bw, by + 10], outline=(80, 100, 120, 255))
    frac = max(0.0, min(1.0, clr / 4.0))
    col = DANGER if clr < 0.8 else (WARN if clr < 1.6 else (90, 210, 150))
    sd.rectangle([bx + 1, by + 1, bx + 1 + int((bw - 2) * frac), by + 9], fill=col + (255,))
    _text(sd, (bx, by + 14), "safety envelope", 10, MUTED)

    img.paste(strip, (0, view_h))
    return img
