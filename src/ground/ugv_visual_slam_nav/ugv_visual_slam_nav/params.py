"""全局参数：世界、车辆、相机、路标、控制周期。

所有模块只从这里取物理常量，避免同一数字散落在多处后不一致。
进程内可用 set_overrides() 覆盖少量参数（ROS 节点用它接收 launch 参数）。
"""

import math

# ---------------------------------------------------------------- 世界
WORLD_W = 20.0                 # 米
WORLD_H = 14.0
WALL_THICK = 0.15              # 墙厚（米），用于 AABB 求交，避免零厚度
WALL_H = 1.2                   # 墙高（米），单目测距需要已知高度

# ---------------------------------------------------------------- 栅格
GRID_RES = 0.1
GW = int(round(WORLD_W / GRID_RES))    # 200
GH = int(round(WORLD_H / GRID_RES))    # 140

# 占据栅格的判决阈值（对数几率）
LOGODDS_HIT = 0.85
LOGODDS_MISS = -0.40
LOGODDS_MIN = -4.0
LOGODDS_MAX = 4.0

# ---------------------------------------------------------------- 车辆
ROBOT_RADIUS = 0.22            # 车体外接圆半径
WHEELBASE = 0.32               # 轮距（米）
MAX_V = 0.60                   # 米/秒
MAX_W = 1.20                   # 弧度/秒
DT = 0.1                       # 控制周期 10 Hz
GOAL_TOL = 0.35                # 到点判定半径

# 里程计误差：轮速积分 + 打滑偏置
ODOM_SIGMA_V = 0.02            # 每步速度噪声（相对）
ODOM_SIGMA_W = 0.03            # 每步角速度噪声（相对）
ODOM_SIGMA_SLIP = 0.01         # 打滑导致的额外角度漂移

# ---------------------------------------------------------------- 相机
CAM_W = 160
CAM_H = 120
CAM_FOV = math.radians(90.0)
CAM_F = (CAM_W / 2.0) / math.tan(CAM_FOV / 2.0)   # 80.0 像素
CAM_H_MOUNT = 0.35             # 相机离地高度（米）
CAM_RANGE_MAX = 12.0           # 超过此距离视为无效

# ---------------------------------------------------------------- 路标
LM_RADIUS = 0.09
LM_HEIGHT = 0.55               # 已知高度，单目测距依赖它

# ---------------------------------------------------------------- 控制
REPLAN_DIST = 0.60             # 偏离当前路径超过此距离触发重规划
LOOKAHEAD = 0.45               # 纯追踪前视距离
ARRIVE_TIME_LIMIT = 150.0      # 单个 episode 最长仿真时间（秒）

# ---------------------------------------------------------------- 评测
ERR_OK = 0.35                  # 单步跟踪误差合格线（米）
LOST_FRAC_OK = 0.85            # 定位有效率合格线

# 场景键
SCENES = ('corridor', 'cluttered', 'dynamic', 'night')


def set_overrides(**kwargs):
    """把 launch 参数写回模块常量（只允许已有的大写常量）。"""
    g = globals()
    for k, v in kwargs.items():
        if k not in g or not k.isupper():
            raise KeyError('未知参数: %s' % k)
        g[k] = type(g[k])(v) if not isinstance(g[k], str) else v
