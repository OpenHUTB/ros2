"""参数：画布、障碍圆、目标点、势场增益与小车运动能力。"""

WORLD_W = 800
WORLD_H = 600
FPS = 60

START = (80.0, 520.0)
GOAL = (720.0, 110.0)

# 障碍圆：(圆心 x, 圆心 y, 半径)
OBSTACLES = [
    (300.0, 380.0, 70.0),
    (520.0, 250.0, 78.0),
    (620.0, 470.0, 62.0),
]

# 引力增益：方向为目标方向，幅值固定
K_ATT = 1.0

# 斥力增益与影响半径
ETA = 0.25
RHO0 = 55.0
REP_SCALE = 1000.0

# 小车运动能力
SPEED = 2.2
MAX_TURN = 0.16

# 局部极小点判定与逃逸
STALL_WINDOW = 55
STALL_PROGRESS = 3.0
ESCAPE_FRAMES = 70
ESCAPE_BIAS = 0.75

# 到达与碰撞判定
GOAL_TOL = 8.0
CLEARANCE_MARGIN = 4.0

# 运行上限：避免逃逸失败时无限循环、轨迹与历史无限增长
MAX_STEPS = 3000
TRAIL_MAX = 2000
HISTORY_MAX = STALL_WINDOW + 10
