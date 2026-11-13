"""参数：画布、领航者轨迹、队形偏移与控制增益。"""

WORLD_W = 800
WORLD_H = 600
FPS = 60

# 领航者沿圆周运动
CENTER = (400.0, 300.0)
RADIUS = 160.0
OMEGA = 0.0125
PHASE0 = 0.0

# 队形偏移，以领航者朝向为参考：领航者在原点，x 向前，y 向右
LEADER_OFFSET = (0.0, 0.0)
FOLLOWER_OFFSETS = [(-95.0, -58.0), (-95.0, 58.0)]

# 初始位置，故意偏离队形，用于演示收敛
INIT_POSITIONS = [(120.0, 520.0), (700.0, 480.0)]

# 跟随控制增益与单帧最大位移
GAIN = 0.40
MAX_STEP = 3.4

# 队形误差判定
TOL = 6.0
SETTLE_STEPS = 260
