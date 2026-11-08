# -*- coding: utf-8 -*-
"""场景的唯一真实来源：墙体几何。

为什么要单一来源：
    栅格地图必须和 Gazebo 场景**严格一致**，否则粒子滤波会稳定收敛到错误位置，
    而这种错误极难排查（看起来像"滤波器不收敛"）。
    所以墙体只在这里定义一次，`tools/gen_scene.py` 同时生成：
        worlds/mcl_room.world   （Gazebo 场景）
        maps/mcl_room.pgm/.yaml （占据栅格地图）

坐标约定：世界坐标系米制，x 向右、y 向上（与 map_server / ROS 一致）。

每面墙是一个轴对齐矩形：
    dict(cx=, cy=, sx=, sz=, height=)
    cx, cy   中心（米）
    sx, sz   沿 x / y 方向的厚度与长度（米）
    height   墙高（米），默认 1.0
"""

# 地面中央的可行区域：10 m x 8 m
ROOM_SIZE = (10.0, 8.0)

WALL_THICKNESS = 0.20
WALL_HEIGHT = 1.00

# 四面外墙 + 内部隔断（形成走廊与两个房间，便于定位时有明显的几何特征）
WALLS = [
    # 外墙
    dict(cx=0.0, cy=+(ROOM_SIZE[1] / 2), sx=ROOM_SIZE[0] + WALL_THICKNESS, sz=WALL_THICKNESS),
    dict(cx=0.0, cy=-(ROOM_SIZE[1] / 2), sx=ROOM_SIZE[0] + WALL_THICKNESS, sz=WALL_THICKNESS),
    dict(cx=+(ROOM_SIZE[0] / 2), cy=0.0, sx=WALL_THICKNESS, sz=ROOM_SIZE[1] + WALL_THICKNESS),
    dict(cx=-(ROOM_SIZE[0] / 2), cy=0.0, sx=WALL_THICKNESS, sz=ROOM_SIZE[1] + WALL_THICKNESS),

    # 内部隔断：两条平行墙夹出一条走廊（y ∈ [-1, 1]），走廊两端留门
    dict(cx=-1.5, cy=+1.0, sx=5.0, sz=WALL_THICKNESS),
    dict(cx=+2.0, cy=+1.0, sx=2.0, sz=WALL_THICKNESS),
    dict(cx=-1.5, cy=-1.0, sx=5.0, sz=WALL_THICKNESS),
    dict(cx=+2.0, cy=-1.0, sx=2.0, sz=WALL_THICKNESS),

    # 右上角小隔间（提供另一处特征角点，避免场景左右对称）
    dict(cx=+2.5, cy=+3.0, sx=WALL_THICKNESS, sz=2.0),
    # 左下角短墙
    dict(cx=-3.0, cy=-2.5, sx=2.0, sz=WALL_THICKNESS),
]

# 机器人初始位姿（与地图一致）
SPAWN = dict(x=-3.5, y=0.0, yaw=0.0)

# 栅格地图参数
MAP_RESOLUTION = 0.05          # 米 / 像素
MAP_PADDING = 0.50             # 地图在房间外再留的边界（米）

# 激光雷达参数（与 urdf 里保持一致）
LIDAR_RANGE_MAX = 8.0
LIDAR_RANGE_MIN = 0.12
LIDAR_SAMPLES = 360
