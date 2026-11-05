# -*- coding: utf-8 -*-
"""OpenCV 可视化：俯视图 + 侧视图 + HUD。"""
from __future__ import annotations

import math
from collections import deque

import cv2
import numpy as np

W, H = 1180, 680
BG = (24, 28, 33)
PANEL = (32, 37, 44)
GRID = (52, 60, 68)
WHITE = (238, 242, 245)
MUTED = (150, 160, 170)
GREEN = (80, 210, 120)
ORANGE = (40, 155, 245)
BLUE = (235, 170, 70)
RED = (70, 70, 230)

# 世界范围
X_MIN, X_MAX = -1.0, 7.0
Y_MIN, Y_MAX = -1.0, 4.0


def _map(x, y, ox=20, oy=92, w=720, h=448):
    px = ox + (x - X_MIN) / (X_MAX - X_MIN) * w
    py = oy + (Y_MAX - y) / (Y_MAX - Y_MIN) * h
    return int(px), int(py)


def _side(x, z, ox=772, oy=92, w=380, h=448):
    px = ox + (x - X_MIN) / (X_MAX - X_MIN) * w
    py = oy + (4.2 - z) / 4.2 * h
    return int(px), int(py)


def _text(img, s, x, y, color=WHITE, scale=0.48, thick=1):
    cv2.putText(img, s, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thick, cv2.LINE_AA)


def _drone(img, cx, cy, yaw, color=WHITE, size=14):
    # 四个旋翼 + 机臂，简单表达 X 型
    rot = np.array([[math.cos(yaw), -math.sin(yaw)],
                    [math.sin(yaw), math.cos(yaw)]])
    pts = []
    for dx, dy in ((1, -1), (1, 1), (-1, 1), (-1, -1)):
        p = rot @ np.array([dx, dy]) * size * 0.75
        px, py = int(cx + p[0]), int(cy + p[1])
        cv2.circle(img, (px, py), 5, color, 1, cv2.LINE_AA)
        pts.append((px, py))
    cv2.line(img, pts[0], pts[2], color, 1, cv2.LINE_AA)
    cv2.line(img, pts[1], pts[3], color, 1, cv2.LINE_AA)
    cv2.circle(img, (int(cx), int(cy)), 6, ORANGE, -1, cv2.LINE_AA)
    cv2.line(img, (int(cx), int(cy)),
             (int(cx + math.cos(yaw) * 20), int(cy - math.sin(yaw) * 20)),
             (255, 255, 255), 2, cv2.LINE_AA)


def render(st, predicted=None, phase="", trail=None, target=(5.0, 2.0)):
    img = np.full((H, W, 3), BG, dtype=np.uint8)
    cv2.rectangle(img, (0, 0), (W, 52), (18, 21, 25), -1)
    _text(img, "WIND-AWARE PAYLOAD DELIVERY", 20, 33, WHITE, 0.72, 2)
    _text(img, "MuJoCo + ROS Noetic | NN impact prediction", 430, 32, MUTED, 0.46)

    # ---------------- 俯视图
    cv2.rectangle(img, (18, 64), (746, 552), PANEL, -1)
    cv2.rectangle(img, (18, 64), (746, 552), (55, 64, 72), 1)
    _text(img, "TOP-DOWN (x-y)", 30, 86, MUTED, 0.46)
    for gx in range(0, 8):
        cv2.line(img, _map(gx, Y_MIN), _map(gx, Y_MAX), GRID, 1)
        _text(img, str(gx), _map(gx, Y_MIN)[0] - 6, 570, MUTED, 0.38)
    for gy in range(-1, 5):
        cv2.line(img, _map(X_MIN, gy), _map(X_MAX, gy), GRID, 1)
        _text(img, str(gy), 4, _map(X_MIN, gy)[1] + 4, MUTED, 0.38)

    # 目标区
    tx, ty = float(target[0]), float(target[1])
    tp = _map(tx, ty)
    cv2.circle(img, tp, 26, GREEN, 2, cv2.LINE_AA)
    cv2.circle(img, tp, 4, GREEN, -1, cv2.LINE_AA)
    _text(img, "TARGET", tp[0] - 28, tp[1] - 34, GREEN, 0.42)

    # 轨迹
    if trail:
        pts = [_map(p[0], p[1]) for p in list(trail)[-220:]]
        for a, b in zip(pts[:-1], pts[1:]):
            cv2.line(img, a, b, (90, 110, 125), 1, cv2.LINE_AA)

    # 风矢量
    wx, wy = float(st.wind[0]), float(st.wind[1])
    base = np.array([_map(0.2, -0.6)[0], _map(0.2, -0.6)[1]])
    tip = base + np.array([wx * 18, -wy * 18])
    cv2.arrowedLine(img, tuple(base.astype(int)), tuple(tip.astype(int)), BLUE, 2,
                    cv2.LINE_AA, tipLength=0.25)
    _text(img, f"WIND {wx:+.1f},{wy:+.1f} m/s", 34, 522, BLUE, 0.42)

    up = _map(st.pos[0], st.pos[1])
    pp = _map(st.payload_pos[0], st.payload_pos[1])
    if predicted is not None:
        pr = _map(predicted[0], predicted[1])
        cv2.drawMarker(img, pr, RED, cv2.MARKER_CROSS, 16, 2)
        _text(img, "PRED", pr[0] + 8, pr[1] - 8, RED, 0.38)
    _drone(img, up[0], up[1], float(st.euler[2]))
    if not st.released:
        cv2.line(img, up, pp, (140, 150, 160), 1, cv2.LINE_AA)
    cv2.circle(img, pp, 7, ORANGE, -1, cv2.LINE_AA)
    cv2.circle(img, pp, 8, (255, 220, 170), 1, cv2.LINE_AA)

    # ---------------- 侧视图
    cv2.rectangle(img, (760, 64), (1160, 552), PANEL, -1)
    cv2.rectangle(img, (760, 64), (1160, 552), (55, 64, 72), 1)
    _text(img, "SIDE (x-z)", 772, 86, MUTED, 0.46)
    for gz in range(0, 5):
        cv2.line(img, _side(X_MIN, gz), _side(X_MAX, gz), GRID, 1)
        _text(img, f"{gz}m", 1128, _side(X_MIN, gz)[1] + 4, MUTED, 0.36)
    for gx in range(0, 8):
        cv2.line(img, _side(gx, 0), _side(gx, 4), (40, 46, 52), 1)
    # 目标
    cv2.line(img, _side(tx - 0.6, 0), _side(tx + 0.6, 0), GREEN, 4, cv2.LINE_AA)
    s_up = _side(st.pos[0], st.pos[2])
    s_pp = _side(st.payload_pos[0], st.payload_pos[2])
    _drone(img, s_up[0], s_up[1], float(st.euler[2]))
    if not st.released:
        cv2.line(img, s_up, s_pp, (160, 170, 180), 1, cv2.LINE_AA)
    cv2.circle(img, s_pp, 6, ORANGE, -1, cv2.LINE_AA)
    cv2.line(img, (760, 540), (1160, 540), (80, 90, 100), 1)

    # ---------------- HUD
    cv2.rectangle(img, (18, 566), (1160, 662), (20, 24, 29), -1)
    err = st.payload_pos[:2] - np.array([tx, ty])
    err_n = float(np.linalg.norm(err)) if st.released else float("nan")
    items = [
        ("PHASE", phase.upper(), ORANGE),
        ("ALT", f"{st.pos[2]:.2f} m", WHITE),
        ("WIND", f"{wx:+.1f}, {wy:+.1f} m/s", BLUE),
        ("PAYLOAD", "RELEASED" if st.released else "ATTACHED",
         GREEN if st.released else ORANGE),
        ("IMPACT ERR", "---" if not st.released else f"{err_n:.2f} m",
         GREEN if st.released and err_n < 0.5 else RED),
    ]
    x = 32
    for label, value, color in items:
        _text(img, label, x, 592, MUTED, 0.38)
        _text(img, value, x, 626, color, 0.58, 2)
        x += 220
    _text(img, "t = %.2fs" % st.t, 1010, 626, WHITE, 0.52)
    if st.landed:
        _text(img, "LANDED", 900, 592, GREEN, 0.42)
    return img


def save_poster(path, st, predicted, phase, trail, target=(5.0, 2.0)):
    img = render(st, predicted, phase, trail, target)
    cv2.imwrite(path, img)
