# -*- coding: utf-8 -*-
"""可视化与仿真参数。

默认值写在下面的常量里；若找到 `config/params.yaml`，则用其中的键覆盖，
做到「配了确实生效」。查找顺序：

1. 环境变量 ``DUBINS_PARAMS`` 指向的文件
2. ROS 2 安装空间：``<share>/dubins_path/config/params.yaml``（``ros2 run`` 走的路径）
3. 源码树：``<包根>/config/params.yaml``（直接 ``python main.py`` 的路径）

yaml 为可选依赖：未安装 PyYAML 时静默退回内置默认值。
"""

import math
import os

# 画布
WIDTH = 900
HEIGHT = 600
FPS = 60

# 颜色
BG_COLOR = (24, 28, 38)
GRID_COLOR = (40, 46, 60)
START_COLOR = (100, 240, 130)        # 起点（绿色）
END_COLOR = (240, 100, 100)          # 终点（红色）
SHORTEST_COLOR = (80, 200, 240)      # 最短路径高亮（青色）
CANDIDATE_COLORS = [                 # 6 条候选路径（淡色）
    (90, 100, 120),
    (110, 120, 140),
    (130, 140, 160),
    (150, 160, 180),
    (170, 180, 200),
    (190, 200, 220),
]
CENTER_COLOR = (200, 120, 240)       # 圆心（紫色，调试用）
TEXT_COLOR = (220, 225, 235)
HUD_COLOR = (200, 210, 225)

# Dubins 默认位姿（起点 + 终点）
DEFAULT_START = (150.0, 480.0, 0.0)            # (x, y, yaw) yaw 单位弧度
DEFAULT_END = (750.0, 150.0, math.pi)          # 朝向 π（向左）
DEFAULT_RADIUS = 80.0                          # 最小转弯半径（像素）

# 采样
CURVE_SAMPLES = 200        # 路径采样点数
ANIM_STEPS = 120           # 绘制动画步数

# 录制
RECORD_EVERY = 2           # 每隔 N 步存一帧
RECORD_TAIL = 30           # 动画结束后补若干帧
RECORD_SCALE = 0.75        # 缩放比例控制 gif 体积


# ---------------------------------------------------------------------------
# params.yaml 加载
# ---------------------------------------------------------------------------
def _candidate_paths():
    """按优先级返回 params.yaml 的候选路径。"""
    paths = []
    env = os.environ.get("DUBINS_PARAMS")
    if env:
        paths.append(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        paths.append(os.path.join(get_package_share_directory("dubins_path"),
                                  "config", "params.yaml"))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    paths.append(os.path.join(os.path.dirname(here), "config", "params.yaml"))
    return paths


def find_params():
    """返回第一个存在的 params.yaml 路径，找不到返回 None。"""
    for p in _candidate_paths():
        if p and os.path.isfile(p):
            return p
    return None


def load_params(path=None):
    """读取 params.yaml 覆盖本模块的模块级参数，返回实际使用的路径（未加载返回 None）。"""
    global WIDTH, HEIGHT, FPS, DEFAULT_START, DEFAULT_END, DEFAULT_RADIUS
    global CURVE_SAMPLES, ANIM_STEPS

    path = path or find_params()
    if not path or not os.path.isfile(path):
        return None
    try:
        import yaml
    except ImportError:
        print("[Dubins] 未安装 PyYAML，跳过 %s（使用内置默认值）" % path)
        return None

    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    canvas = data.get("canvas") or {}
    if canvas.get("width") is not None:
        WIDTH = int(canvas["width"])
    if canvas.get("height") is not None:
        HEIGHT = int(canvas["height"])
    if canvas.get("fps") is not None:
        FPS = int(canvas["fps"])

    dubins = data.get("dubins") or {}
    if dubins.get("start"):
        DEFAULT_START = tuple(float(v) for v in dubins["start"])
    if dubins.get("end"):
        DEFAULT_END = tuple(float(v) for v in dubins["end"])
    if dubins.get("radius") is not None:
        DEFAULT_RADIUS = float(dubins["radius"])

    sampling = data.get("sampling") or {}
    if sampling.get("curve_samples") is not None:
        CURVE_SAMPLES = int(sampling["curve_samples"])
    if sampling.get("anim_steps") is not None:
        ANIM_STEPS = int(sampling["anim_steps"])
    return path