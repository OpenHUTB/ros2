"""vehicle_end_to_end_sim —— 全局常量与单位换算。

所有可调参数集中在 `config.py`，与 `config/sim_params.yaml` 保持语义一致。
物理常量忠实取自课程作业《基于端到端感知与物理动力学的车载仿真系统》。
"""

import math

# ---------- 窗口 ----------
WIDTH = 800
HEIGHT = 600
FPS = 60

# ---------- 单位换算 ----------
# 车速换算关系（与作业一致）：
#   像素速度 v_pixel  ↔  物理速度 v_ms
#   target_speed_pixel = (target_speed_ms / 60.0) * PIXELS_PER_METER
PIXELS_PER_METER = 1.0


def kmh_to_pixel(kmh):
    """目标车速（km/h）→ 底层仿真速度（像素/帧）。"""
    ms = kmh / 3.6
    return (ms / 60.0) * PIXELS_PER_METER


def pixel_to_kmh(pixel):
    """底层仿真速度（像素/帧）→ 实时车速（km/h）。"""
    return pixel * 60.0 / PIXELS_PER_METER * 3.6


# ---------- 车辆运动学（自行车模型） ----------
WHEELBASE_PIXEL = 7.0          # 轴距 L，仿真中设为 7.0 像素
MAX_STEER = 1.0                # steer 归一化到 [-1,1]
MAX_STEER_RAD = math.pi / 4.0  # 转向角上限 pi/4，update 中使用 tan(steer * pi/4)

# 车辆初始位姿
START_X = WIDTH // 2
START_Y = HEIGHT - 80
START_THETA = -math.pi / 2

# ---------- 感知（前方视野锥） ----------
VISION_VIEW_DEG = 40.0         # 前方视野半角 ±40°

# ---------- 决策阈值（像素距离） ----------
WARN_DIST_PIXEL = 300.0        # 预警线
SAFE_BRAKE_DIST_PIXEL = 180.0  # 安全红线，触发 AEB 紧急制动

# ---------- 控制 / 加减速 ----------
ACCEL_STEP = 0.02              # 正常加速：每帧速度增量
BRAKE_DECEL = 0.03             # AEB 恒减速度：每帧速度递减（平滑刹停）
STEER_SPEED = 0.02             # 转向角变化速率（松开自动回正）

# ---------- 设置界面 ----------
TARGET_SPEED_STEP_KMH = 10.0   # ↑ / ↓ 调节步长
MIN_SPEED_KMH = 10.0           # 最小目标车速
DEFAULT_TARGET_SPEED_KMH = 60.0

# ---------- 仿真世界障碍物（与作业一致） ----------
OBSTACLES = [(400, 200), (300, 100), (500, 150)]
