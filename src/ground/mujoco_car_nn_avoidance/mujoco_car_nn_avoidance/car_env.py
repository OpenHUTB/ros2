"""车辆物理仿真环境：基于 MuJoCo 的两轮差速小车。

采用实际调通的稳定驱动方案：
    - 轮速伺服交给 MuJoCo 原生 <velocity> 执行器（ctrl 直接写目标角速度），
      由 MuJoCo 内部按 kv/forcerange 求解，避免手写 P 伺服带来的震荡与掀翻；
    - 积分器用 implicitfast，速度相关项做隐式积分，2 ms 步长下更稳定；
    - 车身 4 kg、质心前移（让驱动轮承担 ~80% 重量），避免驱动轮法向力不足而空转打滑；
    - 自车几何体放在 geom group 2，射线检测用 geomgroup 屏蔽（见 sensors.py）。

职责：场景加载、步进、轮速伺服写入、车辆状态读取。
"""
import math

import numpy as np
import mujoco

from . import config
from .kinematics import differential_wheel_vels


class CarEnv:
    """封装一个 MuJoCo 场景与一辆两轮差速小车。"""

    def __init__(self, cfg, scene_name=None, xml_path=None):
        self.cfg = cfg
        self.scene_name = scene_name
        self.load(scene_name, xml_path)

    # ---------- 加载与重置 ----------
    def load(self, scene_name=None, xml_path=None):
        if xml_path is None:
            xml_path = self.cfg.scene_path(scene_name)
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.cfg.timestep
        # 固定 id 缓存
        self.drive_l_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "drive_l")
        self.drive_r_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "drive_r")
        self.chassis_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.free_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_root")
        jl = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_l_joint")
        jr = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_r_joint")
        self._free_qpos_adr = int(self.model.jnt_qposadr[self.free_joint_id])
        self._free_dof_adr = int(self.model.jnt_dofadr[self.free_joint_id])
        self.wl_vel_idx = self.model.jnt_dofadr[jl]
        self.wr_vel_idx = self.model.jnt_dofadr[jr]

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        # 起步静置：先零轮速推进，让轮胎接触稳定建立，避免空转与偏航
        n = max(1, int(round(self.cfg.settle_s / self.cfg.timestep)))
        self.data.ctrl[self.drive_l_id] = 0.0
        self.data.ctrl[self.drive_r_id] = 0.0
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    # ---------- 步进（速度执行器，ctrl=目标轮角速度） ----------
    def set_wheel_speeds(self, w_l, w_r):
        """写入左右轮目标角速度 (rad/s)，由 MuJoCo velocity 执行器伺服。"""
        self.data.ctrl[self.drive_l_id] = float(w_l)
        self.data.ctrl[self.drive_r_id] = float(w_r)

    def step(self, count=1):
        for _ in range(int(count)):
            mujoco.mj_step(self.model, self.data)

    def advance_control_period(self):
        """推进一个控制周期（控制频率与物理步长解耦）。"""
        self.step(self.cfg.steps_per_control)

    def step_vel(self, w_l, w_r):
        """兼容旧入口：设目标轮速并推进一个控制周期。"""
        self.set_wheel_speeds(w_l, w_r)
        self.advance_control_period()

    # ---------- 状态读取 ----------
    def pos(self):
        """车体世界位置 (x, y, z)。"""
        return np.array(self.data.xpos[self.chassis_id], dtype=np.float64).copy()

    def yaw(self):
        """车头航向角 yaw（弧度），车头朝 +x 时 yaw=0。"""
        q = self.data.qpos[self._free_qpos_adr + 3: self._free_qpos_adr + 7]
        qw, qx, qy, qz = q
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def wheel_vel(self):
        return (self.data.qvel[self.wl_vel_idx],
                self.data.qvel[self.wr_vel_idx])

    def forward_speed(self):
        """沿车头方向的分量线速度（近似）。"""
        v = self.data.qvel[self._free_dof_adr: self._free_dof_adr + 3]
        cosy = math.cos(self.yaw())
        siny = math.sin(self.yaw())
        return float(v[0] * cosy + v[1] * siny)

    # ---------- 目标轮速换算（差速运动学） ----------
    def desired_wheel_vel(self, steer, throttle):
        """(steer, throttle) -> 左右轮目标角速度；纯解算见 kinematics.py。"""
        return differential_wheel_vels(
            steer, throttle, self.cfg.v_max, self.cfg.omega_max,
            self.cfg.wheel_r, self.cfg.track, self.cfg.max_wheel_omega)
