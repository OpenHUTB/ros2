#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""示范数据的纯数组操作：镜像增广、特征拼接、汇总、训练用 XY。

这里不 import 环境（MuJoCo），因此自检与单元测试可以在只装了 numpy 的
机器上跑；``dataset.py`` 负责真正跑仿真采集。
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from .ranges import (FEAT_DIM, FEAT_DIM_SEQ, HISTORY, MIRROR_IDX, N_AZ, N_EL,
                     build_features, build_sequence_features)

__all__ = ["KEYS", "DTYPES", "empty", "mirror_dataset", "slice_features",
           "dataset_summary", "build_xy"]

KEYS = ("hist", "vel", "goal_dir", "dist", "z", "u_nom", "u_tgt", "h",
        "q_tgt", "seed", "episode")

DTYPES = {"hist": np.float32, "vel": np.float32, "goal_dir": np.float32,
          "dist": np.float32, "z": np.float32, "u_nom": np.float32,
          "u_tgt": np.float32, "h": np.float32, "q_tgt": np.float32,
          "seed": np.int32, "episode": np.int32}

FLIP = np.array([1.0, -1.0, 1.0], np.float32)


def empty(n: int) -> Dict[str, np.ndarray]:
    return {"hist": np.zeros((n, HISTORY, N_EL, N_AZ), np.float32),
            "vel": np.zeros((n, 3), np.float32),
            "goal_dir": np.zeros((n, 3), np.float32),
            "dist": np.zeros(n, np.float32),
            "z": np.zeros(n, np.float32),
            "u_nom": np.zeros((n, 3), np.float32),
            "u_tgt": np.zeros((n, 3), np.float32),
            "h": np.zeros(n, np.float32),
            "q_tgt": np.zeros(n, np.float32),
            "seed": np.zeros(n, np.int32),
            "episode": np.zeros(n, np.int32)}


def mirror_dataset(d: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    """左右镜像增广：栅格方位角翻转、y 分量取反，episode 编号整体后移。"""
    m = {k: np.asarray(d[k], DTYPES[k]).copy() for k in KEYS}
    m["hist"] = np.asarray(d["hist"], np.float32)[..., MIRROR_IDX]
    for k in ("vel", "goal_dir", "u_nom", "u_tgt"):
        m[k] = (np.asarray(d[k], np.float32) * FLIP[None, :]).astype(np.float32)
    offset = int(np.max(np.asarray(d["episode"]))) + 1
    m["episode"] = (np.asarray(d["episode"], np.int32) + offset).astype(np.int32)
    out: Dict[str, np.ndarray] = {}
    for k in KEYS:
        out[k] = np.concatenate([np.asarray(d[k], DTYPES[k]), m[k]], axis=0)
    return out


def slice_features(d: Dict[str, np.ndarray], temporal: bool = True,
                   index: Optional[np.ndarray] = None) -> np.ndarray:
    """把原始记录拼成网络输入（不含标称指令）。"""
    hist = np.asarray(d["hist"], np.float32)
    n = len(hist)
    idx = np.arange(n) if index is None else np.asarray(index, int)
    out = np.zeros((len(idx), FEAT_DIM_SEQ if temporal else FEAT_DIM),
                   np.float32)
    for j, i in enumerate(idx):
        if temporal:
            out[j] = build_sequence_features(
                hist[i], d["vel"][i], d["goal_dir"][i], float(d["dist"][i]),
                float(d["z"][i]))
        else:
            out[j] = build_features(hist[i][0], d["vel"][i], d["goal_dir"][i],
                                    float(d["dist"][i]), float(d["z"][i]))
    return out


def dataset_summary(d: Dict[str, np.ndarray]) -> Dict[str, float]:
    n = len(d["seed"])
    if n == 0:
        return {"frames": 0.0, "episodes": 0.0, "seeds": 0.0}
    u_tgt = np.asarray(d["u_tgt"], float)
    u_nom = np.asarray(d["u_nom"], float)
    return {"frames": float(n),
            "episodes": float(len(np.unique(d["episode"]))),
            "seeds": float(len(np.unique(d["seed"]))),
            "h_mean": float(np.mean(d["h"])),
            "speed_mean": float(np.mean(np.linalg.norm(u_tgt[:, :2], axis=1))),
            "intervene": float(np.mean(
                np.linalg.norm(u_tgt - u_nom, axis=1) > 0.1))}


def build_xy(d: Dict[str, np.ndarray], temporal: bool = True,
             index: Optional[np.ndarray] = None):
    """原始记录 → (X, Y)；X 末尾三列是标称指令，Y 是教师修正指令。"""
    F = slice_features(d, temporal=temporal, index=index)
    sel = np.arange(len(F)) if index is None else np.asarray(index, int)
    X = np.concatenate([F, np.asarray(d["u_nom"], np.float32)[sel]], axis=1)
    Y = np.asarray(d["u_tgt"], np.float32)[sel]
    return X.astype(np.float64), Y.astype(np.float64)
