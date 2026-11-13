# -*- coding: utf-8 -*-
"""模块级常量：几何尺寸、弹簧参数、控制增益、指标阈值。

所有长度单位 m，角度单位 rad，时间单位 s，力单位 N。
改动这里必须同步检查 test/test_standalone.py 里的阈值断言。
"""

PACKAGE_NAME = "mujoco_hopper_slip_control"
MODEL_NAME = "hopper_scene.xml"
VERSION = "1.0.0"

# ---------------------------------------------------------------- 质量与几何
# 与 models/hopper_scene.xml 里的 mass/pos 一一对应，测试会解析 XML 逐项比对。
MASS_TORSO = 4.6
MASS_THIGH = 0.3
MASS_SHANK = 0.2
MASS_FOOT = 0.9
TOTAL_MASS = MASS_TORSO + MASS_THIGH + MASS_SHANK + MASS_FOOT   # 6.0 kg

FOOT_RADIUS = 0.045
LEG_NOMINAL = 0.28          # 髋关节 -> 脚心 标称长度（0.07 + 0.21）
TORSO_TO_HIP = 0.07

# q = 0 时脚底恰好着地所对应的躯干高度
Z_REST = LEG_NOMINAL + FOOT_RADIUS     # 0.325

GRAVITY = 9.81

# ---------------------------------------------------------------- 弹簧（被动）
LEG_STIFFNESS = 2200.0      # N/m
LEG_DAMPING = 12.0          # N·s/m
LEG_Q_MIN = -0.03
LEG_Q_MAX = 0.20

# ---------------------------------------------------------------- 执行器
LEG_FORCE_LIMIT = 900.0
HIP_KP = 45.0
HIP_KV = 2.5
HIP_LIMIT = 1.0

# 飞行相腿长保持的 PD（把脚收回标称长度，准备着陆）
LEG_HOLD_KP = 900.0
LEG_HOLD_KD = 35.0
LEG_HOLD_FORCE_LIMIT = 300.0

# ---------------------------------------------------------------- 控制器
# 触地/离地去抖：连续 N 个物理步都满足条件才切换，避免接触抖动制造假相位
DEBOUNCE_STEPS = 3

# 能量伺服：按「上一跳的起飞竖直速度」增量修正，自动补偿触地冲击损耗
# 增益取 0.04：实测 0.08 以上会出现周期约 6 跳的极限环（顶点在 0.50~0.70 m 之间摆）
ENERGY_GAIN = 0.04
ENERGY_GAIN_MIN = 0.0
ENERGY_GAIN_MAX = 1.5
FORCE_INIT = 60.0           # 初始注能力，让第一跳能离地
FORCE_LIMIT = 800.0         # 注能力上限
FORCE_STOP_Q = 0.02         # 伸展到 q < 这个值就停止注能（接近离地）

# 离地后能量效率在线辨识：apex - z_lo ≈ η · vz²/(2g)，η 用一阶低通跟踪
EFFICIENCY_INIT = 1.0
EFFICIENCY_MIN = 0.55
EFFICIENCY_MAX = 1.05
EFFICIENCY_SMOOTH = 0.35

# 落点：Raibert 规则 offset = vx*T_ST/2 + k_v*(vx - vx_des)
RAIBERT_T_ST = 0.20
RAIBERT_KV = 0.10
RAIBERT_LEG_LEN = LEG_NOMINAL
RAIBERT_ANGLE_LIMIT = 0.60

# 支撑相髋关节是否自由。True = SLIP 模型（腿绕支撑脚被动翻转），
# 这是被动弹跳的前提；锁死髋角会迫使脚掌拖地、破坏支撑相动力学。
HIP_FREE_IN_STANCE = True

# ---------------------------------------------------------------- 仿真
TIMESTEP = 0.002
CONTROL_HZ = 500.0          # 与物理步同频，保证控制器看到的相位不丢

# ---------------------------------------------------------------- 实验默认值
TARGET_HEIGHT = 0.55        # 躯干顶点目标高度
TARGET_SPEED = 0.0          # 默认原地跳
START_Q = 0.11              # 初始压缩量
START_CLEARANCE = 0.02      # 初始离地间隙，让第一跳自然落体

# ---------------------------------------------------------------- 指标阈值
TOL_APEX = 0.03             # 顶点高度误差容差
SETTLE_HOPS = 8             # 取最后 N 跳算稳态
MIN_HOPS = 6                # 少于这个跳数视为没跑起来
FALL_Z = 0.12               # 躯干低于此高度判定为摔倒
DISTURB_DV = 1.0            # 抗扰实验横向冲量 m/s（1.5 会摔倒，见 README 能力边界）
DISTURB_T = 5.0             # 施加时刻
RECOVER_VX = 0.15           # |vx| 回到该阈值内视为速度恢复
