# -*- coding: utf-8 -*-
"""模块常量：CPG 参数、回合设置、学习器超参、评测口径。"""

PACKAGE_NAME = "humanoid_ars_phase_walk"
VERSION = "1.0.0"

# ---------------- CPG 基线（手工调参，已实测能走）
A_HIP = -0.35
A_KNEE = 0.45
FREQ = 1.2
KNEE_PHASE = 0.6
ANKLE_GAIN = 0.8

# ---------------- 回合
EPISODE_T = 4.0
CTRL_HZ = 50.0
FALL_PITCH = 1.2
FALL_DZ = -0.40

# ---------------- 奖励
W_VEL = 0.10
W_PITCH = 0.10
W_ALIVE = 0.01
W_EFFORT = 0.001

# ---------------- 学习器
ARS_DIRS = 6
ARS_SIGMA = 0.01
ARS_ALPHA = 0.002
ARS_SCALE = 0.05
BC_HIDDEN = 64
BC_EPOCHS = 500
DAGGER_ROUNDS = 4
SEED = 20261112

# ---------------- 评测口径
SUCCESS_DX = 1.20
SUCCESS_STEPS = 190