#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag/ 公共工具：路径、策略封装、数据集读取。

所有诊断脚本都从这里取路径，避免硬编码 /tmp，保证换机器后可直接跑。
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)          # 模块根目录（含 main.py）
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import main as _m  # noqa: E402  main.py 里的常量都是绝对路径

MODEL_PATH = _m.MODEL_PATH
DATASET = _m.DATASET
WEIGHTS = _m.WEIGHTS
DEFAULT_STAND_TARGET = _m.DEFAULT_STAND_TARGET
CTRL_DT = _m.CTRL_DT
run_episode = _m._run_episode


def add_path_args(p, dataset=False, weights=False, out=None):
    """给 argparse 加统一的路径参数。"""
    if dataset:
        p.add_argument("--dataset", default=DATASET,
                       help="数据集 npz，默认 data/e2e_dataset.npz")
    if weights:
        p.add_argument("--weights", default=WEIGHTS,
                       help="权重 npz，默认 models_trained/e2e_policy.npz")
    if out is not None:
        p.add_argument("--out", default=out, help="输出 npz 路径")
    return p


def nn_policy(predictor, cache=None):
    """端到端闭环策略：refresh 时重新推理，其余步复用上一次力矩。"""
    c = cache if cache is not None else {"u": np.zeros(15)}

    def policy(sim, refresh):
        if refresh:
            c["u"] = predictor.predict_from_image(sim.render_front(gray=True))
        return c["u"]

    return policy


def load_xy(path):
    """读数据集，返回 float64 的 (X, Y)。"""
    d = np.load(path)
    return d["X"].astype(np.float64), d["Y"].astype(np.float64)


def require(path, hint):
    if not os.path.exists(path):
        raise SystemExit(f"[错误] 找不到 {path}\n  {hint}")
    return path
