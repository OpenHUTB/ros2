#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""加载 params.yaml 并构造模型/步态/调度对象。"""

import os
from typing import List, Tuple

import yaml

from .gait import GaitSpec
from .kinematics import BipedModel


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def build_model(cfg: dict) -> BipedModel:
    b = cfg["biped"]
    return BipedModel(
        thigh=float(b["thigh_length"]),
        shank=float(b["shank_length"]),
        foot_length=float(b["foot_length"]),
        ankle_height=float(b["ankle_height"]),
        torso_length=float(b["torso_length"]),
        masses=b["masses"],
    )


def build_gaits(cfg: dict) -> dict:
    gaits = {}
    for name, g in cfg["gaits"].items():
        gaits[name] = GaitSpec(
            name=name,
            cycle=float(g["cycle"]),
            stance=float(g["stance"]),
            step=float(g["step"]),
            clearance=float(g["clearance"]),
            hip_height=float(g["hip_height"]),
            bob=float(g["bob"]),
            lean=float(g["lean"]),
        )
    return gaits


def build_schedule(cfg: dict) -> List[Tuple[str, float]]:
    return [(item["gait"], float(item["duration"])) for item in cfg["schedule"]]


def default_params_path() -> str:
    """优先使用 ament share 目录（安装后 ros2 run 可直接启动），再回退源码目录。"""
    try:
        from ament_index_python.packages import (PackageNotFoundError,
                                                  get_package_share_directory)
        share = get_package_share_directory("biped_cubic_spline_gait_switch")
        candidate = os.path.join(share, "config", "params.yaml")
        if os.path.exists(candidate):
            return candidate
    except (ImportError, PackageNotFoundError, ValueError):
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "config", "params.yaml")
