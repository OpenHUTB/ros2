"""全局参数：海底世界、ROV、前视相机、管道、控制周期。

所有模块只从这里取物理常量，避免同一数字散落在多处后不一致。
进程内可用 set_overrides() 覆盖少量参数（ROS 节点用它接收 launch 参数）。
"""

import math

# ---------------------------------------------------------------- 世界
WORLD_W = 24.0                 # 米（海底平面）
WORLD_H = 16.0

# ---------------------------------------------------------------- 栅格
GRID_RES = 0.1
GW = int(round(WORLD_W / GRID_RES))    # 240
GH = int(round(WORLD_H / GRID_RES))    # 160

# 占据栅格的判决阈值（对数几率）
LOGODDS_HIT = 0.85
LOGODDS_MISS = -0.40
LOGODDS_MIN = -4.0
LOGODDS_MAX = 4.0

# ---------------------------------------------------------------- ROV
ROBOT_RADIUS = 0.25            # ROV 外接圆半径
MAX_V = 0.55                   # 前进速度上限（米/秒）
MAX_W = 1.10                   # 偏航角速度上限（弧度/秒）
DT = 0.1                       # 控制周期 10 Hz
GOAL_TOL = 0.40                # 到点判定半径
SURGE_TAU = 0.60               # 推进器一阶滞后时间常数（秒）

# 里程计误差：DVL/惯性积分漂移。注意海流漂移不进入里程计，
# 这正是无 DVL 的 ROV "里程计越走越歪" 的系统性来源。
ODOM_SIGMA_V = 0.02
ODOM_SIGMA_W = 0.03

# ---------------------------------------------------------------- 相机
CAM_W = 128
CAM_H = 96
CAM_FOV = math.radians(140.0)   # 广角：横向偏置巡检时管道必须留在视野内
CAM_F = (CAM_W / 2.0) / math.tan(CAM_FOV / 2.0)   # ≈ 23 像素
CAM_ALT = 0.45                 # 相机离海床高度（米）
CAM_RANGE_MAX = 12.0

# ---------------------------------------------------------------- 海床特征
LM_RADIUS = 0.10               # 声应答器半径（已知尺寸，单目测距依赖它）
LM_HEIGHT = 0.90               # 应答器立杆高度（加高以保证远距可见性）

# ---------------------------------------------------------------- 管道
PIPE_RADIUS = 0.18             # 管道半径（米）
PIPE_HEIGHT = 0.32             # 管道顶高（渲染用）
FOLLOW_OFFSET = 0.90           # 巡检横向偏置：沿管道左侧保持的距离
LOOKAHEAD = 0.80               # 纯追踪前视距离
ARRIVE_TIME_LIMIT = 220.0      # 单个 episode 最长仿真时间（秒，往返全程）

# ---------------------------------------------------------------- 海流
CURRENT_MAX = 0.18             # current 场景横向流速幅值（米/秒）
CURRENT_OMEGA = 0.10           # 流向摆动角频率（弧度/秒）

# ---------------------------------------------------------------- 评测
ERR_OK = 0.35                  # 单步定位误差合格线（米）
LAT_OK = 0.45                  # 单步横向跟踪误差合格线（米）

# 场景键
SCENES = ('straight', 'curve', 'current', 'turbid')


def set_overrides(**kwargs):
    """把 launch 参数写回模块常量（只允许已有的大写常量）。"""
    g = globals()
    for k, v in kwargs.items():
        if k not in g or not k.isupper():
            raise KeyError('未知参数: %s' % k)
        g[k] = type(g[k])(v) if not isinstance(g[k], str) else v
