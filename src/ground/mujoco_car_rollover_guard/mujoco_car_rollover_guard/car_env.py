# -*- coding: utf-8 -*-
"""车辆物理仿真环境：基于 MuJoCo 的两轮差速小车（带顶部载荷）。

稳定驱动方案沿用仓库已验证的差速底盘：
    - 轮速伺服交给 MuJoCo 原生 <velocity> 执行器（ctrl 直接写目标角速度）
    - 积分器用 implicitfast，2ms 步长下更稳定
    - 车身 4kg、质心前移，前支撑轮着地构成三点支撑
职责：场景加载、步进、轮速伺服写入、IMU 状态读取、侧翻判定。
"""
import math

import numpy as np
import mujoco

from .kinematics import differential_wheel_vels


class CarEnv:
    """封装一个 MuJoCo 场景与一辆两轮差速小车。"""

    def __init__(self, cfg, scene_name=None, xml_path=None):
        self.cfg = cfg
        self.scene_name = scene_name
        self.load(scene_name, xml_path)

    def load(self, scene_name=None, xml_path=None):
        if xml_path is None:
            xml_path = self.cfg.scene_path(scene_name)
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.cfg.timestep

        self.drive_l_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "drive_l")
        self.drive_r_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "drive_r")
        self.chassis_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.free_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_root")
        self._free_qpos_adr = int(self.model.jnt_qposadr[self.free_joint_id])

        self._sensors = {}
        for name in ("imu_gyro", "imu_acc", "imu_linvel"):
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                raise ValueError("场景缺少传感器 {}".format(name))
            adr = int(self.model.sensor_adr[sid])
            dim = int(self.model.sensor_dim[sid])
            self._sensors[name] = (adr, dim)

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        # 起步静置：轮速置零推进一小段时间，让轮胎接触稳定建立
        n = max(1, int(round(self.cfg.settle_s / self.cfg.timestep)))
        self.data.ctrl[self.drive_l_id] = 0.0
        self.data.ctrl[self.drive_r_id] = 0.0
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    # ---------- 步进 ----------
    def set_wheel_speeds(self, w_l, w_r):
        self.data.ctrl[self.drive_l_id] = float(w_l)
        self.data.ctrl[self.drive_r_id] = float(w_r)

    def step(self, count=1):
        for _ in range(int(count)):
            mujoco.mj_step(self.model, self.data)

    def advance_control_period(self):
        self.step(self.cfg.steps_per_control)

    def set_command(self, steer, throttle):
        """写入归一化指令并推进一个控制周期。"""
        wl, wr = differential_wheel_vels(
            steer, throttle, self.cfg.v_max, self.cfg.omega_max,
            self.cfg.wheel_r, self.cfg.track, self.cfg.max_wheel_omega)
        self.set_wheel_speeds(wl, wr)
        self.advance_control_period()

    # ---------- 状态读取 ----------
    def _sensor(self, name):
        adr, dim = self._sensors[name]
        return np.array(self.data.sensordata[adr: adr + dim], dtype=np.float64).copy()

    def imu(self):
        """返回机体系 IMU 数据：四元数 (w,x,y,z)、角速度、加速度、世界系线速度。"""
        return {
            "quat": np.array(self.data.xquat[self.chassis_id], dtype=np.float64),
            "gyro": self._sensor("imu_gyro"),
            "acc": self._sensor("imu_acc"),
            "linvel": self._sensor("imu_linvel"),
        }

    @staticmethod
    def roll_pitch(quat):
        """由机体系四元数解算横滚角与俯仰角（rad）。"""
        qw, qx, qy, qz = quat
        roll = math.atan2(2.0 * (qw * qx + qy * qz),
                          1.0 - 2.0 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
        return roll, pitch

    @staticmethod
    def body_x_axis(quat):
        qw, qx, qy, qz = quat
        return np.array([
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy + qw * qz),
            2.0 * (qx * qz - qw * qy),
        ])

    def forward_speed(self):
        """沿车头方向的前向速度（m/s）。"""
        v = self._sensor("imu_linvel")
        quat = np.array(self.data.xquat[self.chassis_id], dtype=np.float64)
        return float(v @ self.body_x_axis(quat))

    def up_z(self):
        """机体 z 轴在世界系 z 方向的分量，1 表示完全直立，<0 表示已翻。"""
        qw, qx, qy, qz = self.data.xquat[self.chassis_id]
        return float(1.0 - 2.0 * (qx * qx + qy * qy))

    def tilt_deg(self):
        """机体纵轴与竖直方向的夹角（度）。"""
        return float(math.degrees(math.acos(max(-1.0, min(1.0, self.up_z())))))

    def is_rollover(self):
        return self.tilt_deg() >= self.cfg.rollover_tilt_deg

    def pos(self):
        return np.array(self.data.xpos[self.chassis_id], dtype=np.float64).copy()
