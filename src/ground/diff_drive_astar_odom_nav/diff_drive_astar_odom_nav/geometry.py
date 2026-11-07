#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""静态场景的唯一几何来源。

Gazebo 世界（worlds/static_obstacles.world）与占用栅格地图
（maps/static_map.pgm）都由这里的矩形列表生成，保证规划地图和仿真场景
严格对齐——地图和世界各写一份、靠人工对齐是这类实验最常见的错误来源。
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOM_HALF = 6.0          # 房间内部半宽（米）
WALL_THICKNESS = 0.2     # 墙体厚度
WALL_HEIGHT = 0.5
OBSTACLE_HEIGHT = 0.5
MAP_RESOLUTION = 0.05
MAP_ORIGIN = (-ROOM_HALF, -ROOM_HALF)


# (name, cx, cy, sx, sy)：矩形障碍，单位米，坐标原点在房间中心
OBSTACLES = [
    # 中间隔墙留 1.2 m 门洞：y in [-1.4, -0.2]
    ("wall_left", -0.25, -2.80, 0.30, 2.80),
    ("wall_right", -0.25, 2.00, 0.30, 4.40),
    # 门洞两侧的箱子，迫使规划路径真正绕行
    ("box_east", 2.20, 2.20, 1.60, 0.80),
    ("box_north", 1.80, -2.40, 1.00, 1.40),
    ("box_south", 3.60, 1.20, 0.80, 0.80),
]

START_POSE = (-4.5, -4.5, 0.0)
GOAL_POSE = (4.5, 4.5, math.pi / 4.0)


def all_rects():
    """返回场景中全部实心矩形：四面外墙 + 内部障碍。"""
    half = ROOM_HALF
    t = WALL_THICKNESS
    rects = [
        ("wall_bottom", 0.0, -half + t / 2.0, 2.0 * half, t),
        ("wall_top", 0.0, half - t / 2.0, 2.0 * half, t),
        ("wall_west", -half + t / 2.0, 0.0, t, 2.0 * half),
        ("wall_east", half - t / 2.0, 0.0, t, 2.0 * half),
    ]
    for name, cx, cy, sx, sy in OBSTACLES:
        rects.append((name, cx, cy, sx, sy))
    return rects


def point_in_rect(x, y, rect, margin=0.0):
    _, cx, cy, sx, sy = rect
    return (abs(x - cx) <= sx / 2.0 + margin) and (abs(y - cy) <= sy / 2.0 + margin)


def is_occupied(x, y):
    """世界坐标 (x, y) 是否被墙体/障碍覆盖。"""
    return any(point_in_rect(x, y, r) for r in all_rects())


def build_occupancy():
    """生成与世界对齐的占用栅格。

    返回 (grid, info)；grid[row, col] 为 bool，row=0 对应地图最下方
    （y = origin_y + resolution/2）。占用栅格坐标系以左下角为原点，
    PGM 图片以左上角为原点，写盘时再翻转。
    """
    import numpy as np

    size = int(round(2.0 * ROOM_HALF / MAP_RESOLUTION))
    xs = MAP_ORIGIN[0] + (np.arange(size) + 0.5) * MAP_RESOLUTION
    ys = MAP_ORIGIN[1] + (np.arange(size) + 0.5) * MAP_RESOLUTION
    xx, yy = np.meshgrid(xs, ys)
    occupied = np.zeros((size, size), dtype=bool)
    for rect in all_rects():
        _, cx, cy, sx, sy = rect
        occupied |= (np.abs(xx - cx) <= sx / 2.0) & (np.abs(yy - cy) <= sy / 2.0)
    info = {
        "width": size,
        "height": size,
        "resolution": MAP_RESOLUTION,
        "origin_x": MAP_ORIGIN[0],
        "origin_y": MAP_ORIGIN[1],
    }
    return occupied, info


def _write_pgm(path, grid):
    """写 P5 灰度 PGM：254 = 空闲（白），0 = 占用（黑），符合 map_server 约定；
    图片第一行对应地图最上方。"""
    height, width = grid.shape
    image = ((~grid).astype("uint8") * 254)[::-1]
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (width, height))
        image.tofile(f)


def _box_model(name, cx, cy, sx, sy, height):
    return (
        '    <model name="{name}">\n'
        '      <static>true</static>\n'
        '      <pose>{cx} {cy} {z} 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <collision name="collision">\n'
        '          <geometry><box><size>{sx} {sy} {height}</size></box></geometry>\n'
        '        </collision>\n'
        '        <visual name="visual">\n'
        '          <geometry><box><size>{sx} {sy} {height}</size></box></geometry>\n'
        '          <material><ambient>0.55 0.55 0.60 1</ambient><diffuse>0.65 0.65 0.70 1</diffuse></material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
    ).format(name=name, cx=cx, cy=cy, z=height / 2.0, sx=sx, sy=sy, height=height)


def write_world(path):
    blocks = "".join(
        _box_model(name, cx, cy, sx, sy,
                   WALL_HEIGHT if name.startswith("wall_") else OBSTACLE_HEIGHT)
        for name, cx, cy, sx, sy in all_rects()
    )
    content = (
        '<?xml version="1.0" ?>\n'
        '<sdf version="1.6">\n'
        '  <world name="default">\n'
        '    <include><uri>model://sun</uri></include>\n'
        '    <include><uri>model://ground_plane</uri></include>\n'
        '    <physics type="ode">\n'
        '      <max_step_size>0.001</max_step_size>\n'
        '      <real_time_factor>1.0</real_time_factor>\n'
        '      <real_time_update_rate>1000</real_time_update_rate>\n'
        '    </physics>\n'
        '    <scene><ambient>0.7 0.7 0.7 1</ambient><background>0.85 0.87 0.90 1</background></scene>\n'
        + blocks +
        '  </world>\n'
        '</sdf>\n'
    )
    Path(path).write_text(content, encoding="utf-8")


def write_map_assets(map_dir):
    grid, info = build_occupancy()
    _write_pgm(Path(map_dir) / "static_map.pgm", grid)
    (Path(map_dir) / "static_map.yaml").write_text(
        "image: static_map.pgm\n"
        "resolution: {res}\n"
        "origin: [{ox}, {oy}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n".format(
            res=info["resolution"], ox=info["origin_x"], oy=info["origin_y"]),
        encoding="utf-8",
    )
    return info


if __name__ == "__main__":
    # 允许直接执行：python3 -m diff_drive_astar_odom_nav.geometry <包目录>
    import sys
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
    info = write_map_assets(root / "maps")
    write_world(root / "worlds" / "static_obstacles.world")
    print("map/world written:", info)
