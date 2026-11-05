# -*- coding: utf-8 -*-
"""模块级常量：几何尺寸、台阶参数、步态参数、指标阈值。

所有长度单位 m，角度单位 rad，时间单位 s。
"""

PACKAGE_NAME = "humanoid_stair_climb"
MODEL_NAME = "stair_scene.xml"

# ---------------------------------------------------------------- 机器人几何
HIP_Z = 0.76          # 髋关节在基座坐标系下的高度
THIGH = 0.45          # 大腿长（髋 → 膝）
SHANK = 0.45          # 小腿长（膝 → 踝）
FOOT_DROP = 0.04      # 脚底相对踝关节的下沉量
LEG_MAX = THIGH + SHANK - 1e-3   # IK 可达半径上限（留一点余量避免奇异）
LEG_MIN = 0.25                   # IK 可达半径下限

# ---------------------------------------------------------------- 关节限位
# 必须与 models/stair_scene.xml 里各 joint 的 range 一致，
# test_standalone.py 会解析 XML 逐项比对，改一处必须同时改另一处。
# 取值依据：留出本场景实测行程的余量（hip 最大 1.4622、ankle 最大 0.9636），
# 不是实测机器人的机械限位。
JOINT_LIMITS = {
    "hip": (-1.6, 1.6),
    "knee": (-0.02, 2.1),
    "ankle": (-1.1, 1.1),
}

# 站立基座高度：膝盖保持约 0.78/0.90 的弯曲量，脚底正好落在 z=0
# （脚底 = STAND_Z + HIP_Z + dz - FOOT_DROP，dz=-0.78 时脚底为 0）
STAND_Z = 0.06

# ---------------------------------------------------------------- 台阶几何
STEP_HEIGHT = 0.12    # 每级台阶高度
STEP_DEPTH = 0.30     # 每级台阶进深
STEP_COUNT = 6        # 台阶级数
LANDING_X = STEP_DEPTH * STEP_COUNT   # 平台起点 x = 1.80
LANDING_TOP = STEP_HEIGHT * STEP_COUNT  # 平台顶面 z = 0.72

# ---------------------------------------------------------------- 步态参数
CYCLE_TIME = 1.4      # 一个完整步态周期
SWING_RATIO = 0.42    # 单腿摆动相占比
STRIDE = STEP_DEPTH   # 每只脚每周期上一级（0.30 m），两脚交替
LIFT_HEIGHT = 0.05    # 摆动相安全高度相对落点台阶面的余量（脚厚 0.04）
# 踝关节初始 x。脚几何前端比踝前伸 0.18 m，留 0.02 m 余量不碰第一级立面（x=0）
FOOT_START_L = -0.20
FOOT_START_R = FOOT_START_L - STRIDE   # 右脚初始 x = -0.50
BASE_X0 = (FOOT_START_L + FOOT_START_R) / 2   # 基座初始 x = -0.35
BASE_SPEED = STRIDE / CYCLE_TIME   # 0.214 m/s

# 基座高度跟随支撑脚的一阶低通系数（越大跟随越快）
BASE_Z_TAU = 0.10

# ---------------------------------------------------------------- 运行与控制
CONTROL_HZ = 50.0
TIME_LIMIT = 16.0     # 12 步约 8.4 s，留余量
HOLD_TIME = 2.0       # 到达平台后继续演示的时间

# ---------------------------------------------------------------- 指标阈值
REACH_X = LANDING_X - 0.05     # x ≥ 1.75 视为踏上平台
REACH_Z = LANDING_TOP - 0.02   # 基座高度 ≥ 0.70 视为已升到位
TOL_PENETRATION = 0.01         # 脚与台阶穿透超过 10 mm 记一次违规
TOL_FOOT_ERROR = 0.02          # 落脚点与目标位置偏差超过 20 mm 记一次违规
