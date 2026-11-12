# -*- coding: utf-8 -*-
"""模块常量：前端参数、训练超参、评测档位。"""

PACKAGE_NAME = "humanoid_kws_command"
VERSION = "1.0.0"
MODEL_NAME = "kws_mlp.npz"
FEATURE_FILE = "features.npz"

# ---------------------------------------------------------------- 前端
SAMPLE_RATE = 16000
N_FFT = 512
WIN = 400            # 25 ms
HOP = 160            # 10 ms
N_MELS = 40
F_MIN = 80.0
F_MAX = 7600.0
PREEMPH = 0.97      # 预加重系数

# ---------------------------------------------------------------- 分类器
HIDDEN = 64
EPOCHS = 300
LR = 0.05
L2 = 1e-4
BATCH = 64
SEED = 20261112

# ---------------------------------------------------------------- 评测
TRAIN_SNRS = (None, 20, 10)     # 训练时见过：干净 + 20/10 dB
EVAL_SNRS = (None, 20, 10, 5, 0, -5)
# 5 / 0 / -5 dB 训练时**从未出现**，用来看鲁棒性是否真的泛化
UNSEEN_SNRS = (5, 0, -5)

# 拒识：未知词统一归入 unknown 类
UNKNOWN_LABEL = "unknown"