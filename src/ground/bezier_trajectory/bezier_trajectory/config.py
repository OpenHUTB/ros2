# -*- coding: utf-8 -*-
"""可视化与仿真参数。

默认值写在本文件里；若找到 `config/params.yaml`，则用其中的键覆盖默认值，
做到「配了确实生效」。查找顺序：

1. 环境变量 ``BEZIER_PARAMS`` 指向的文件
2. ROS 2 安装空间：``<share>/bezier_trajectory/config/params.yaml``（``ros2 run`` 走的路径）
3. 源码树：``<包根>/config/params.yaml``（直接 ``python main.py`` 的路径）

yaml 为可选依赖：未安装 PyYAML 时静默退回内置默认值。
"""

import os

# 画布
WIDTH = 900
HEIGHT = 600
FPS = 60

# 颜色
BG_COLOR = (24, 28, 38)
GRID_COLOR = (40, 46, 60)
CONTROL_POLY_COLOR = (90, 100, 120)
CURVE_COLOR = (80, 200, 240)        # Bezier 曲线（青色）
BSPLINE_COLOR = (240, 180, 80)      # B 样条对比（金色）
CONTROL_POINT_COLOR = (240, 100, 100)
ENDPOINT_COLOR = (100, 240, 130)
CURVATURE_COLOR = (200, 120, 240)
HUD_COLOR = (200, 210, 225)
TEXT_COLOR = (220, 225, 235)

# Bezier 曲线默认控制点（3 次）
DEFAULT_CONTROL_POINTS = [
    (120, 480),   # P0 起点
    (250, 150),   # P1
    (650, 150),   # P2
    (780, 480),   # P3 终点
]

# B 样条对比用的控制点（4 个，生成 3 次 B 样条）
BSPLINE_CONTROL_POINTS = [
    (120, 480),
    (300, 180),
    (600, 180),
    (780, 480),
]

# 采样
CURVE_SAMPLES = 200        # 曲线采样点数
CURVATURE_SAMPLES = 80     # 曲率可视化采样

# 录制
RECORD_FRAMES = True
RECORD_EVERY = 2           # 每隔 N 步存一帧
RECORD_TAIL = 30           # 动画结束后补若干帧
RECORD_SCALE = 0.75        # 缩放比例控制 gif 体积

# 动画
ANIM_STEPS = 120           # de Casteljau 动画步数


# ---------------------------------------------------------------------------
# params.yaml 加载
# ---------------------------------------------------------------------------
def _candidate_paths():
    """按优先级返回 params.yaml 的候选路径。"""
    paths = []
    env = os.environ.get("BEZIER_PARAMS")
    if env:
        paths.append(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        paths.append(os.path.join(get_package_share_directory("bezier_trajectory"),
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
    global DEFAULT_CONTROL_POINTS, BSPLINE_CONTROL_POINTS
    global CURVE_SAMPLES, CURVATURE_SAMPLES, ANIM_STEPS

    path = path or find_params()
    if not path or not os.path.isfile(path):
        return None
    try:
        import yaml
    except ImportError:
        print("[Bezier] 未安装 PyYAML，跳过 %s（使用内置默认值）" % path)
        return None

    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if data.get("control_points"):
        DEFAULT_CONTROL_POINTS = [tuple(float(v) for v in p)
                                  for p in data["control_points"]]
    if data.get("bspline_control_points"):
        BSPLINE_CONTROL_POINTS = [tuple(float(v) for v in p)
                                  for p in data["bspline_control_points"]]
    for key, name in (("curve_samples", "CURVE_SAMPLES"),
                      ("curvature_samples", "CURVATURE_SAMPLES"),
                      ("anim_steps", "ANIM_STEPS")):
        if data.get(key) is not None:
            globals()[name] = int(data[key])
    return path