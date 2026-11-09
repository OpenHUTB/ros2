# -*- coding: utf-8 -*-
"""参数单一来源：离线脚本与 ROS 节点都从这里读 ``config/params.yaml``。

优先级：命令行/ROS 参数 > params.yaml > 本文件内的默认值。
这样改 params.yaml 时，两个离线脚本与节点会同步生效，
不会再出现“脚本里各写一份 DT / MAX_TORQUE”的不一致问题。
"""

import os

DEFAULTS = {
    "sim.dt": 0.005,
    "sim.duration": 30.0,
    "trajectory.type": "sine",
    "trajectory.amplitude_deg": 40.0,
    "trajectory.frequency_hz": 0.4,
    "trajectory.bias_deg": 30.0,
    "ctrl.kp": 30.0,
    "ctrl.kd": 3.0,
    # 模型: 单肌最大力 500 N × wrap 半径 0.035 m = 17.5 N·m
    "ctrl.max_torque": 17.5,
    "ctrl.limit_deg": 90.0,
    "ctrl.limit_low_deg": -15.0,
}

PARAMS_REL = os.path.join("config", "params.yaml")


def params_path(module_root=None):
    """定位 config/params.yaml（默认相对本包的上级模块目录）。"""
    if module_root is None:
        here = os.path.dirname(os.path.abspath(__file__))
        module_root = os.path.dirname(here)
    return os.path.join(module_root, PARAMS_REL)


def _flatten(yaml_text):
    """极简 YAML 解析：只处理本仓库这种两层 key: value 结构。"""
    flat = {}
    section = ""
    for raw in yaml_text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if not line.startswith(" ") and line.endswith(":"):
            section = line[:-1].strip()
            continue
        if ":" not in line:
            continue
        key, _, value = line.partition(":")
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        full = f"{section}.{key}" if section and line.startswith(" ") else key
        if value == "":
            section = key
            continue
        try:
            flat[full] = float(value)
        except ValueError:
            flat[full] = value
    return flat


def load(module_root=None):
    """返回参数字典：默认值 + params.yaml（若可解析）。"""
    result = dict(DEFAULTS)
    path = params_path(module_root)
    try:
        with open(path, "r", encoding="utf-8") as f:
            result.update(_flatten(f.read()))
    except OSError:
        pass
    return result


def get(params, key, cast=None):
    value = params.get(key, DEFAULTS.get(key))
    if cast is not None and value is not None:
        return cast(value)
    return value
