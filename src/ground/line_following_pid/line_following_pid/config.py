"""参数：赛道、车辆、PID 增益与仿真步长。"""

WORLD_W = 900
WORLD_H = 600
FPS = 60

# 车辆
WHEELBASE = 18.0          # 前后轴距，像素
SPEED = 2.2               # 每帧前进距离
MAX_STEER = 0.65          # 最大前轮转角，弧度
ROBOT_R = 8.0

# 赛道：正弦曲线 y = amp*sin(k*x) + base
TRACK_AMP = 110.0
TRACK_K = 2.0 * 3.14159 / 500.0
TRACK_BASE = 300.0
TRACK_X0 = 60.0
TRACK_X1 = 860.0
TRACK_STEP = 6.0

# 横向误差期望值（沿赛道中心线为 0）
TARGET_OFFSET = 0.0

# PID
KP = 0.55
KI = 0.004
KD = 0.55
MAX_TOTAL = 20.0
