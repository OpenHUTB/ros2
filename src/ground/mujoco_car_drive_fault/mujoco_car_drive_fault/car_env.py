# -*- coding: utf-8 -*-
"""车辆物理仿真环境：基于 MuJoCo 的两轮差速小车。

稳定驱动方案沿用仓库已验证的差速底盘：
    - 轮速伺服交给 MuJoCo 原生 <velocity> 执行器（ctrl 直接写目标角速度）
    - 积分器用 implicitfast，2ms 步长下更稳定
    - 车身 4kg、质心前移，前支撑轮着地构成三点支撑
在本题里额外提供两件事：
    - 一侧驱动轮关节阻尼注入（模拟轮毂轴承/制动拖滞），可在运行中切到另一侧
    - 轮速、横向偏差、航向角读取，供失效检测与补偿使用
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
        self.wheel_l_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_l_joint")
        self.wheel_r_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_r_joint")
        self._dof_l = int(self.model.jnt_dofadr[self.wheel_l_joint])
        self._dof_r = int(self.model.jnt_dofadr[self.wheel_r_joint])

        self._sensors = {}
        for name in ("imu_gyro", "imu_acc", "imu_linvel",
                     "wheel_l_vel", "wheel_r_vel"):
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                raise ValueError("场景缺少传感器 {}".format(name))
            self._sensors[name] = (int(self.model.sensor_adr[sid]),
                                   int(self.model.sensor_dim[sid]))

        self._nominal_damping = (float(self.model.dof_damping[self._dof_l]),
                                 float(self.model.dof_damping[self._dof_r]))
        # 场景 XML 里写的左轮阻尼就是该档的拖滞量，注入时不传值就直接沿用
        self.scene_damping = float(self.model.dof_damping[self._dof_l])

    def reset(self, fault_side="left", fault_damping=None):
        """重置并注入单侧拖滞；fault_side 取 left/right，None 表示健康。"""
        mujoco.mj_resetData(self.model, self.data)
        damping = self.scene_damping if fault_damping is None else float(fault_damping)
        healthy = float(self.cfg.healthy_damping)
        self.model.dof_damping[self._dof_l] = healthy
        self.model.dof_damping[self._dof_r] = healthy
        if fault_side == "left":
            self.model.dof_damping[self._dof_l] = damping
        elif fault_side == "right":
            self.model.dof_damping[self._dof_r] = damping
        mujoco.mj_forward(self.model, self.data)
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

    def wheel_speeds(self):
        """左右驱动轮实际角速度 (rad/s)。"""
        return float(self._sensor("wheel_l_vel")[0]), float(self._sensor("wheel_r_vel")[0])

    def imu(self):
        return {
            "quat": np.array(self.data.xquat[self.chassis_id], dtype=np.float64),
            "gyro": self._sensor("imu_gyro"),
            "acc": self._sensor("imu_acc"),
            "linvel": self._sensor("imu_linvel"),
        }

    @staticmethod
    def roll_pitch(quat):
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
        v = self._sensor("imu_linvel")
        quat = np.array(self.data.xquat[self.chassis_id], dtype=np.float64)
        return float(v @ self.body_x_axis(quat))

    def yaw(self):
        """车头相对 +x 的航向角（rad），逆时针为正。"""
        qw, qx, qy, qz = self.data.xquat[self.chassis_id]
        return float(math.atan2(2.0 * (qw * qz + qx * qy),
                                1.0 - 2.0 * (qy * qy + qz * qz)))

    def lateral_offset(self):
        """车身中心相对车道中线（y=0）的横向偏差（m）。"""
        return float(self.data.xpos[self.chassis_id][1])

    def distance(self):
        return float(self.data.xpos[self.chassis_id][0])

    def tilt_deg(self):
        qw, qx, qy, qz = self.data.xquat[self.chassis_id]
        up_z = 1.0 - 2.0 * (qx * qx + qy * qy)
        return float(math.degrees(math.acos(max(-1.0, min(1.0, up_z)))))
