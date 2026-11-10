#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""加载 params.yaml 并构造动力学/控制器参数对象。"""

import os
from typing import Tuple

import numpy as np
import yaml

from .controller import ControllerGains, build_gains_from_dict
from .dynamics import QuadParams, WindParams


def load_config(yaml_path: str) -> dict:
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)


def build_params(cfg: dict) -> Tuple[QuadParams, WindParams, ControllerGains, dict]:
    q = cfg["quadcopter"]
    qp = QuadParams(
        mass=float(q["mass"]), arm=float(q["arm_length"]),
        ixx=float(q["ixx"]), iyy=float(q["iyy"]), izz=float(q["izz"]),
        k_f=float(q["k_f"]), k_tau=float(q["k_tau"]),
        min_omega=float(q["min_omega"]), max_omega=float(q["max_omega"]),
        motor_tau=float(q["motor_tau"]),
        gravity=float(cfg["simulation"]["gravity"]),
    )
    w = cfg["wind"]
    wp = WindParams(
        mean=np.array(w["mean"], float),
        gust_magnitude=np.array(w["gust_magnitude"], float),
        gust_start=float(w["gust_start"]),
        gust_duration=float(w["gust_duration"]),
        turbulence_sigma=float(w["turbulence_sigma"]),
        turbulence_bandwidth=float(w["turbulence_bandwidth"]),
        drag_coeff=float(w["drag_coeff"]),
        area=float(w["frontal_area"]),
        air_density=float(w["air_density"]),
    )
    gains = build_gains_from_dict(cfg)
    return qp, wp, gains, cfg


def default_params_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "config", "params.yaml")
