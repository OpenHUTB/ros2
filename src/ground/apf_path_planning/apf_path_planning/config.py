# -*- coding: utf-8 -*-
"""APF 可视化参数与默认场景。

默认值写在下面的常量里；若找到 `config/params.yaml`，则用其中的键覆盖，
做到「配了确实生效」。查找顺序：

1. 环境变量 ``APF_PARAMS`` 指向的文件
2. ROS 2 安装空间：``<share>/apf_path_planning/config/params.yaml``（``ros2 run`` 走的路径）
3. 源码树：``<包根>/config/params.yaml``（直接 ``python main.py`` 的路径）

注意：本模块是 pygame 2D 可视化，没有 rclpy 节点，因此 params.yaml 是**模块配置**
而不是 ROS 参数文件（不含 ``ros__parameters``）。yaml 为可选依赖，未安装时自动退回默认值。
"""

import math
import os

# 画布
WIDTH = 800
HEIGHT = 600
FPS = 30

# 世界坐标范围（与画布 1:1，单位 = 像素）
WORLD_MIN = (0.0, 0.0)
WORLD_MAX = (float(WIDTH), float(HEIGHT))

# 颜色
BG = (18, 22, 28)
GRID = (28, 34, 42)
OBSTACLE = (220, 80, 70)
OBSTACLE_EDGE = (255, 120, 100)
GOAL = (90, 220, 120)
START = (90, 170, 240)
PATH = (90, 200, 240)
FORCE = (255, 220, 80)
HUD = (220, 226, 232)
POTENTIAL_LOW = (30, 40, 60)
POTENTIAL_HIGH = (180, 60, 50)

# 默认场景（世界坐标）
START_POSE = (60.0, 60.0)
GOAL_POSE = (720.0, 540.0)
OBSTACLES = [
    # (x, y, radius)
    (250.0, 200.0, 45.0),
    (400.0, 350.0, 55.0),
    (550.0, 200.0, 40.0),
    (300.0, 450.0, 50.0),
    (600.0, 420.0, 45.0),
]

# APF 算法参数
APF_K_ATT = 1.0
APF_K_REP = 5000.0
APF_D_THRESH_ATT = 150.0
APF_D_THRESH_REP = 80.0
APF_STEP_SIZE = 4.0
APF_MAX_ITER = 2000
APF_GOAL_TOL = 10.0
APF_ESCAPE_JITTER = 15.0
APF_ESCAPE_PATIENCE = 20

# 录制
RECORD_FPS = 30
RECORD_FRAMES = 120


# ---------------------------------------------------------------------------
# params.yaml 加载
# ---------------------------------------------------------------------------
def _candidate_paths():
    """按优先级返回 params.yaml 的候选路径。"""
    paths = []
    env = os.environ.get("APF_PARAMS")
    if env:
        paths.append(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        paths.append(os.path.join(get_package_share_directory("apf_path_planning"),
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
    global WIDTH, HEIGHT, FPS, WORLD_MAX
    global START_POSE, GOAL_POSE, OBSTACLES
    global APF_K_ATT, APF_K_REP, APF_D_THRESH_ATT, APF_D_THRESH_REP
    global APF_STEP_SIZE, APF_MAX_ITER, APF_GOAL_TOL

    path = path or find_params()
    if not path or not os.path.isfile(path):
        return None
    try:
        import yaml
    except ImportError:
        print("[APF] 未安装 PyYAML，跳过 %s（使用内置默认值）" % path)
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
    WORLD_MAX = (float(WIDTH), float(HEIGHT))

    scene = data.get("scene") or {}
    if scene.get("start"):
        START_POSE = tuple(float(v) for v in scene["start"])
    if scene.get("goal"):
        GOAL_POSE = tuple(float(v) for v in scene["goal"])
    if scene.get("obstacles"):
        OBSTACLES = [tuple(float(v) for v in o) for o in scene["obstacles"]]

    apf = data.get("apf") or {}
    for key, name in (("k_att", "APF_K_ATT"),
                      ("k_rep", "APF_K_REP"),
                      ("d_thresh_att", "APF_D_THRESH_ATT"),
                      ("d_thresh_rep", "APF_D_THRESH_REP"),
                      ("step_size", "APF_STEP_SIZE"),
                      ("goal_tol", "APF_GOAL_TOL")):
        if apf.get(key) is not None:
            globals()[name] = float(apf[key])
    if apf.get("max_iter") is not None:
        APF_MAX_ITER = int(apf["max_iter"])
    return path