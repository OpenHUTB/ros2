#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""资源定位：统一查找 models / config / data 等资源目录。

colcon 安装后代码在 ``install/lib/python3.x/site-packages/``、资源在
``install/share/uav_mujoco_sim/``，两棵树分离，按层级上溯的写法会失效。
"""
from __future__ import annotations

import os
from typing import Optional

PKG_NAME = "uav_mujoco_sim"

#: 包根目录（源码布局下即仓库中的包目录）
_HERE = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(_HERE)


def _candidates(subdir: str):
    """按优先级给出候选目录。"""
    # 0) 环境变量覆盖（调试用）
    env = os.environ.get("UAV_MUJOCO_SIM_" + subdir.upper())
    if env:
        yield env
    # 1) 源码布局：<包目录>/../models 与 <包目录>/models
    yield os.path.join(os.path.dirname(PKG_DIR), subdir)
    yield os.path.join(PKG_DIR, subdir)
    # 2) 安装布局：通过 ament 索引找到 share/<包名>/
    try:
        from ament_index_python.packages import get_package_share_directory
        yield os.path.join(get_package_share_directory(PKG_NAME), subdir)
    except Exception:
        pass
    # 3) 兜底：向上一层层找（应对被拷贝到别处的场景），同时试安装布局的
    #    <prefix>/share/<包名>/——没 source ROS、ament 索引不可用时走这条
    d = PKG_DIR
    for _ in range(6):
        d = os.path.dirname(d)
        yield os.path.join(d, subdir)
        yield os.path.join(d, "share", PKG_NAME, subdir)


def find_dir(subdir: str, marker: Optional[str] = None) -> str:
    """返回第一个存在的候选目录；都不存在则抛异常并列出找过哪些地方。"""
    tried = []
    for c in _candidates(subdir):
        tried.append(c)
        if os.path.isdir(c) and (marker is None or
                                 os.path.exists(os.path.join(c, marker))):
            return c
    raise FileNotFoundError(
        "找不到 %s 目录（%s 需要它）。已尝试：\n  %s"
        % (subdir, PKG_NAME, "\n  ".join(tried)))


def find_file(subdir: str, name: str) -> str:
    """在候选目录里找单个文件。"""
    return os.path.join(find_dir(subdir), name)


def models_dir() -> str:
    return find_dir("models", marker="quadrotor.xml")


def quadrotor_xml() -> str:
    return os.path.join(models_dir(), "quadrotor.xml")


def config_dir() -> str:
    return find_dir("config")


def data_dir() -> str:
    return find_dir("data")
