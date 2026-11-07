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
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "config", "params.yaml")
