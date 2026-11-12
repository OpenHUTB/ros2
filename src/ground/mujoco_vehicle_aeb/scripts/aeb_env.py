#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 两车纵向场景的封装（无渲染）。

关键改动：**按关节名取下标，不再硬编码**。原实现用 ``data.qpos[11]`` /
``data.qvel[10]`` 读前车状态，而该模型只有 11 个 qpos、10 个 qvel（前车没有关节），
一进入循环就 ``IndexError``——脚本根本跑不起来。这里用
``mj_name2id(..., mjOBJ_JOINT, "lead_slide")`` 查地址，模型改了也不会错位。

mujoco 延迟导入：没有装 mujoco 的机器上，``--help``、控制器单元测试仍然可用。
"""
from __future__ import annotations

import os

import numpy as np

DEFAULT_XML = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models", "aeb_car.xml")

# 制动力上限（N）。1000 kg 自车 + 4×5 kg 车轮，6000 N ≈ 5.9 m/s²，
# 与常见乘用车 0.6 g 的制动强度同量级。
DEFAULT_MAX_FORCE = 6000.0


class MujocoUnavailable(RuntimeError):
    """本机没有可用的 mujoco。"""


def _import_mujoco():
    try:
        import mujoco  # noqa: F401
        return mujoco
    except Exception as exc:  # noqa: BLE001
        raise MujocoUnavailable(
            "未检测到 mujoco，无法进行物理仿真。\n"
            "  安装：pip install 'mujoco>=3.0.0'\n"
            "  本机实测版本：mujoco 3.2.7 / Python 3.12\n"
            "  只想验证控制律与指标可用：python3 main.py --selftest\n"
            f"  原始错误：{exc}"
        ) from exc


def mujoco_available():
    try:
        _import_mujoco()
        return True
    except MujocoUnavailable:
        return False


def resolve_model_path(xml_path=None):
    if xml_path is None:
        return os.path.abspath(DEFAULT_XML)
    if os.path.isabs(xml_path):
        return xml_path
    if os.path.isfile(xml_path):
        return os.path.abspath(xml_path)
    return os.path.abspath(os.path.join(os.path.dirname(DEFAULT_XML), xml_path))


def describe_environment():
    try:
        mujoco = _import_mujoco()
        import sys
        return "mujoco {} / numpy {} / Python {}".format(
            mujoco.__version__, np.__version__,
            ".".join(str(v) for v in sys.version_info[:3]))
    except MujocoUnavailable as exc:
        return str(exc).splitlines()[0]


class AEBEnv:
    """两车纵向运动环境。

    ``qpos`` 布局由关节名解析而来，对外只暴露语义化的属性：
    ``gap``（车头到前车尾的净间距）、``closing_speed`` 等。
    """

    # 车身长度（m）。XML 里 chassis/lead_body 的 size 是半长，故整长 2.4 m。
    BODY_LENGTH = 2.4

    def __init__(self, xml_path=None, timestep=None):
        self.mujoco = _import_mujoco()
        self.xml_path = resolve_model_path(xml_path)
        if not os.path.isfile(self.xml_path):
            raise FileNotFoundError(f"找不到 MuJoCo 模型：{self.xml_path}")
        self.model = self.mujoco.MjModel.from_xml_path(self.xml_path)
        if timestep is not None:
            self.model.opt.timestep = float(timestep)
        self.data = self.mujoco.MjData(self.model)

        self.ego_qpos = self._joint_qpos_adr("ego_free")
        self.ego_dof = self._joint_dof_adr("ego_free")
        self.lead_qpos = self._joint_qpos_adr("lead_slide")
        self.lead_dof = self._joint_dof_adr("lead_slide")
        self.reset()

    # ---------------------------------------------------------------- 内部
    def _joint_qpos_adr(self, name):
        jid = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f"模型里没有关节 {name!r}，请检查 {self.xml_path}")
        return int(self.model.jnt_qposadr[jid])

    def _joint_dof_adr(self, name):
        jid = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f"模型里没有关节 {name!r}，请检查 {self.xml_path}")
        return int(self.model.jnt_dofadr[jid])

    # ---------------------------------------------------------------- 状态
    @property
    def timestep(self):
        return float(self.model.opt.timestep)

    @property
    def time(self):
        return float(self.data.time)

    @property
    def ego_x(self):
        return float(self.data.qpos[self.ego_qpos])

    @property
    def ego_v(self):
        return float(self.data.qvel[self.ego_dof])

    @property
    def lead_x(self):
        return float(self.data.qpos[self.lead_qpos])

    @property
    def lead_v(self):
        return float(self.data.qvel[self.lead_dof])

    @property
    def gap(self):
        """车头到前车尾的净间距（m）：两车中心距 - 一个车身长。"""
        return float(self.lead_x - self.ego_x - self.BODY_LENGTH)

    @property
    def closing_speed(self):
        return float(self.ego_v - self.lead_v)

    # ---------------------------------------------------------------- 控制
    def reset(self, ego_x=0.0, ego_v=5.0, lead_x=35.0, lead_v=0.0):
        self.mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.ego_qpos] = float(ego_x)
        self.data.qpos[self.lead_qpos] = float(lead_x)
        self.data.qvel[self.ego_dof] = float(ego_v)
        self.data.qvel[self.lead_dof] = float(lead_v)
        self.mujoco.mj_forward(self.model, self.data)

    def apply_brake(self, brake, max_force=DEFAULT_MAX_FORCE):
        """把归一化制动力映射成**逆着运动方向**的纵向力（N）。

        为什么不能只写 ``-brake*F``：本模型没有滚动阻力，恒定的反向力会让车停下后
        **继续反向加速**（实测：恒定 0.35 制动 12 s 后 v = −19.5 m/s，等于倒着开）。
        真实制动器的力始终与运动方向相反，所以这里按速度符号取方向；
        速度极小时不施加力，避免在 0 附近抖动。
        """
        force = float(brake) * float(max_force)
        v = self.ego_v
        if abs(v) < 1e-9:
            signed = 0.0
        else:
            signed = -force if v > 0.0 else force
        self.data.qfrc_applied[:] = 0.0
        self.data.qfrc_applied[self.ego_dof] = signed
        return abs(signed)

    def hold_still(self):
        """驻车保持：把自车纵向速度清零（模拟真实车辆停住后保持制动）。

        本模型没有静摩擦模型，若不清零，自车会以低速一直滑下去，``stop_distance``
        也就永远算不出来。这一条是与物理近似的取舍，文档里已注明。
        """
        self.data.qvel[self.ego_dof] = 0.0

    def step(self):
        self.mujoco.mj_step(self.model, self.data)

    def describe(self):
        return {"xml": self.xml_path, "timestep": self.timestep,
                "nq": int(self.model.nq), "nv": int(self.model.nv),
                "ego_qpos": self.ego_qpos, "lead_qpos": self.lead_qpos}
