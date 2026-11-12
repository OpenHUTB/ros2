#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 模型封装：复位、步进、状态读取、施加扰动。

设计要点（都是踩过坑的地方）：

* **模型路径按文件位置解析**，不依赖当前工作目录。否则从 `~/ros2_ws` 里执行
  `python3 src/ground/.../main.py` 就会报"找不到 balance_car.xml"。
* **mujoco 延迟导入**：`--help`、纯逻辑单元测试、以及在没有 mujoco 的机器上
  跑 `main.py --list` 都不该因为缺依赖直接崩掉；只有真正要仿真时才抛
  `MujocoUnavailable`，并给出可执行的安装提示。
* **电机符号集中在 MOTOR_SIGN**：原实现里 `data.ctrl[0] = -torque` 的负号是
  "调出来的"，散落在循环里既解释不清也没法测试。这里把它提成常量，并在
  README 里说明它的物理含义（电机反作用力矩方向）。
"""

import os

import numpy as np

# 电机力矩作用到车轮的符号。改这个符号等价于把控制方向反过来，会让系统发散，
# 因此单独提出来并写进测试（test_motor_sign_matches_model）。
MOTOR_SIGN = -1.0

# qpos 索引（见 balance_car.xml 的自由度顺序）
IDX_X = 0       # 底盘沿 x 平移
IDX_ANGLE = 3   # 摆杆绕 y 转

DEFAULT_XML = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "balance_car.xml")


class MujocoUnavailable(RuntimeError):
    """本机没有可用的 mujoco（未安装或导入失败）。"""


def _import_mujoco():
    try:
        import mujoco  # noqa: F401
        return mujoco
    except Exception as exc:  # noqa: BLE001
        raise MujocoUnavailable(
            "未检测到 mujoco，无法进行物理仿真。\n"
            "  安装：pip install 'mujoco>=3.0.0'\n"
            "  本机已测版本：mujoco 3.17.0 / Python 3.12\n"
            "  只想看控制逻辑与指标计算可用：python3 main.py --selftest\n"
            f"  原始错误：{exc}"
        ) from exc


def mujoco_available():
    """不抛异常地探测 mujoco 是否可用（供 CLI 打印环境信息用）。"""
    try:
        _import_mujoco()
        return True
    except MujocoUnavailable:
        return False


def resolve_model_path(xml_path=None):
    """把模型路径解析成绝对路径：None 用默认模型，相对路径按本文件所在包解析。"""
    if xml_path is None:
        return os.path.abspath(DEFAULT_XML)
    if os.path.isabs(xml_path):
        return xml_path
    # 相对路径先按当前目录，找不到再按模块根目录（兼容从任意目录调用）
    if os.path.isfile(xml_path):
        return os.path.abspath(xml_path)
    alt = os.path.join(os.path.dirname(DEFAULT_XML), xml_path)
    return os.path.abspath(alt)


class BalanceEnv:
    """倒立摆两轮车的仿真环境（无渲染）。"""

    def __init__(self, xml_path=None, timestep=None):
        self.mujoco = _import_mujoco()
        self.xml_path = resolve_model_path(xml_path)
        if not os.path.isfile(self.xml_path):
            raise FileNotFoundError(f"找不到 MuJoCo 模型：{self.xml_path}")
        self.model = self.mujoco.MjModel.from_xml_path(self.xml_path)
        if timestep is not None:
            self.model.opt.timestep = float(timestep)
        self.data = self.mujoco.MjData(self.model)
        self.reset()

    # ---------------------------------------------------------------- 状态
    @property
    def timestep(self):
        return float(self.model.opt.timestep)

    @property
    def x(self):
        return float(self.data.qpos[IDX_X])

    @property
    def vx(self):
        return float(self.data.qvel[IDX_X])

    @property
    def angle(self):
        return float(self.data.qpos[IDX_ANGLE])

    @property
    def angle_rate(self):
        return float(self.data.qvel[IDX_ANGLE])

    # ---------------------------------------------------------------- 控制
    def reset(self, lean=0.05, x=0.0, velocity=0.0):
        """复位到给定初始倾角。lean 太大会直接倒，属于正常物理现象。"""
        self.mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[IDX_X] = float(x)
        self.data.qpos[IDX_ANGLE] = float(lean)
        self.data.qvel[IDX_X] = float(velocity)
        self.mujoco.mj_forward(self.model, self.data)

    def step(self, torque):
        """施加力矩并推进一步。torque 是"期望的平衡力矩"，符号由 MOTOR_SIGN 翻一次。"""
        ctrl = MOTOR_SIGN * float(torque)
        self.data.ctrl[0] = ctrl
        self.data.ctrl[1] = ctrl
        self.mujoco.mj_step(self.model, self.data)

    def disturb(self, velocity):
        """给底盘一个速度脉冲，模拟被推一把（键盘 ↑/↓ 在 GUI 里就是调这个）。"""
        self.data.qvel[IDX_X] += float(velocity)

    # ---------------------------------------------------------------- 信息
    def describe(self):
        return {
            "xml": self.xml_path,
            "timestep": self.timestep,
            "nq": int(self.model.nq),
            "nv": int(self.model.nv),
            "nu": int(self.model.nu),
        }


def describe_environment():
    """返回一行环境描述，供 CLI 启动时打印（便于复现问题）。"""
    try:
        mujoco = _import_mujoco()
        return "mujoco {} / numpy {} / Python {}".format(
            mujoco.__version__, np.__version__,
            ".".join(str(v) for v in __import__("sys").version_info[:3]))
    except MujocoUnavailable as exc:
        return str(exc).splitlines()[0]
