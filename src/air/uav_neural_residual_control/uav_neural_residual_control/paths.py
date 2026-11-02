#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""资源定位：同时兼容源码布局与 colcon 安装布局。"""
from __future__ import annotations

import os

__all__ = ["package_root", "artifacts_dir", "results_dir", "figures_dir",
           "default_net", "default_dataset", "sample_dir"]


def package_root() -> str:
    """包根目录（含 main.py / package.xml 的那一层）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    parent = os.path.dirname(here)
    if os.path.exists(os.path.join(parent, "package.xml")):
        return parent
    # 安装布局：<prefix>/lib/python3.x/site-packages/<pkg>/paths.py
    for up in (parent, os.path.dirname(parent)):
        if os.path.exists(os.path.join(up, "package.xml")):
            return up
    return parent


def _ensure(name: str) -> str:
    p = os.path.join(package_root(), name)
    os.makedirs(p, exist_ok=True)
    return p


def artifacts_dir() -> str:
    """训练产物（模型权重、数据集缓存）。"""
    return _ensure("artifacts")


def results_dir() -> str:
    """实验结果 JSON。"""
    return _ensure("artifacts/results")


def figures_dir() -> str:
    """文档配图。"""
    return _ensure("artifacts/figures")


def sample_dir() -> str:
    """随包提交的小样本数据。"""
    d = os.path.join(package_root(), "data", "sample")
    os.makedirs(d, exist_ok=True)
    return d


def default_net() -> str:
    """训练产物；没有时回退到随包提交的样例权重。

    样例权重只有 43 KB，让拿到仓库的人不跑训练也能直接 ``--eval`` / ``--node``。
    """
    p = os.path.join(artifacts_dir(), "residual_net.npz")
    if os.path.exists(p):
        return p
    s = os.path.join(sample_dir(), "residual_net.npz")
    return s if os.path.exists(s) else p


def default_dataset() -> str:
    return os.path.join(artifacts_dir(), "residual_dataset.npz")
