# -*- coding: utf-8 -*-
"""MuJoCo 环境封装：差速驱动机器人 + 真实轮地接触。

与旧版（纯解析积分）的根本区别：
    这里的位姿由 MuJoCo 求解器积分出来 —— 有接触、有摩擦锥、有质量惯性，
    所以**打滑是物理结果，不是我塞进去的高斯噪声**。
    里程计仍然用编码器（关节角）按标称参数积分，漂移因此真实存在。

对外接口刻意与旧版保持一致（reset / set_twist / state），
这样标定与评估代码不用改，只是"被控对象"从模型换成了物理引擎。
"""

import math

import numpy as np

from . import config

config.ensure_render_backend()

import mujoco  # noqa: E402


class MujocoUnavailable(RuntimeError):
    pass


class RobotState(object):
    __slots__ = ("t", "x", "y", "z", "yaw", "pitch", "roll", "up_z",
                 "v", "w", "wheel", "ncon")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))

    def __repr__(self):
        return ("t=%.2f x=%+.3f y=%+.3f z=%.3f yaw=%+.3f up=%.3f v=%+.3f w=%+.3f"
                % (self.t, self.x, self.y, self.z, self.yaw, self.up_z, self.v, self.w))


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class DiffDriveEnv(object):
    """差速车 MuJoCo 环境。"""

    def __init__(self, scene_path, verbose=True):
        try:
            self.model = mujoco.MjModel.from_xml_path(scene_path)
        except Exception as exc:
            raise MujocoUnavailable("无法加载场景 %s：%s" % (scene_path, exc))
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self.steps_per_control = max(1, int(round(1.0 / (self.dt * config.CONTROL_HZ))))

        self._robot = self._id(mujoco.mjtObj.mjOBJ_BODY, "robot")
        self._wl_v = self._dofadr("wheel_l_joint")
        self._wr_v = self._dofadr("wheel_r_joint")
        self._wl_q = self._qposadr("wheel_l_joint")
        self._wr_q = self._qposadr("wheel_r_joint")

        self.reset((0.0, 0.0, 0.0))
        if verbose:
            print("[MuJoCo] %s（nq=%d, nu=%d, dt=%.4f）" % (scene_path, self.model.nq,
                                                           self.model.nu, self.dt))

    # ------------------------------------------------------------------
    def _id(self, objtype, name):
        i = mujoco.mj_name2id(self.model, objtype, name)
        if i < 0:
            raise MujocoUnavailable("场景里找不到 %s" % name)
        return i

    def _dofadr(self, joint):
        return int(self.model.jnt_dofadr[self._id(mujoco.mjtObj.mjOBJ_JOINT, joint)])

    def _qposadr(self, joint):
        return int(self.model.jnt_qposadr[self._id(mujoco.mjtObj.mjOBJ_JOINT, joint)])

    # ------------------------------------------------------------------
    def reset(self, pose=(0.0, 0.0, 0.0), settle=True):
        x, y, yaw = [float(v) for v in pose]
        d = self.data
        mujoco.mj_resetData(self.model, d)
        d.qpos[0] = x
        d.qpos[1] = y
        d.qpos[2] = config.SPAWN_Z
        d.qpos[3:7] = [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)]
        d.qpos[self._wl_q] = 0.0
        d.qpos[self._wr_q] = 0.0
        d.qvel[:] = 0.0
        d.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, d)
        if settle:
            for _ in range(int(round(config.SETTLE_SECONDS / self.dt))):
                mujoco.mj_step(self.model, d)
        self._phi_l0 = float(d.qpos[self._wl_q])
        self._phi_r0 = float(d.qpos[self._wr_q])
        return self.state()

    def set_wheels(self, wl, wr):
        lim = 30.0
        self.data.ctrl[0] = float(np.clip(wl, -lim, lim))
        self.data.ctrl[1] = float(np.clip(wr, -lim, lim))

    def set_twist(self, v, w):
        """车体目标速度 → 左右轮速（标称参数）。

            v = r(ωl + ωr)/2 ,  w = r(ωr - ωl)/b
        """
        v = float(np.clip(v, -config.V_MAX, config.V_MAX))
        w = float(np.clip(w, -config.W_MAX, config.W_MAX))
        r, b = config.WHEEL_RADIUS, config.TRACK
        self.set_wheels((v - w * b * 0.5) / r, (v + w * b * 0.5) / r)

    def step(self, n_control=1):
        for _ in range(int(n_control)):
            for _k in range(self.steps_per_control):
                mujoco.mj_step(self.model, self.data)
        return self.state()

    # ------------------------------------------------------------------
    def state(self):
        d = self.data
        R = d.xmat[self._robot].reshape(3, 3)
        x, y, z = d.xpos[self._robot]
        wl = float(d.qvel[self._wl_v])
        wr = float(d.qvel[self._wr_v])
        r, b = config.WHEEL_RADIUS, config.TRACK
        return RobotState(
            t=float(d.time), x=float(x), y=float(y), z=float(z),
            yaw=math.atan2(R[1, 0], R[0, 0]),
            pitch=math.asin(max(-1.0, min(1.0, R[2, 0]))),
            roll=math.atan2(R[2, 1], R[2, 2]),
            up_z=float(R[2, 2]),
            v=r * (wl + wr) * 0.5, w=r * (wr - wl) / b,
            wheel=(wl, wr), ncon=int(d.ncon))

    def wheel_angles(self):
        """累计轮子转角（编码器读数，相对 reset 时刻）。"""
        d = self.data
        return (float(d.qpos[self._wl_q]) - self._phi_l0,
                float(d.qpos[self._wr_q]) - self._phi_r0)

    def imu(self):
        """读 MuJoCo 的 <gyro>/<accelerometer> 传感器，返回真实刚体量。

        早期版本这里用的是 `cvel` 的**线速度**，还把它当成加速度发出去
        （评审指出：`linear_acceleration` 单位是 m/s²，EKF/robot_localization
        会按加速度用）。现在直接读 sensordata：
            gyro  → 机体角速度 (rad/s)
            accel → 比力 (m/s²)，含重力投影与向心加速度
        这两个量**独立于轮速**，所以神经网络拿到的输入不再是轮速的线性组合。
        """
        d = self.data
        g = self.model.sensor_adr[self._sg] if hasattr(self, "_sg") else None
        if not hasattr(self, "_sg"):
            self._sg = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "gyro")
            self._sa = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "accel")
            if self._sg < 0 or self._sa < 0:
                raise MujocoUnavailable("场景里缺少 <gyro>/<accelerometer> 传感器")
        g = int(self.model.sensor_adr[self._sg])
        a = int(self.model.sensor_adr[self._sa])
        return dict(gyro_x=float(d.sensordata[g]),
                    gyro_y=float(d.sensordata[g + 1]),
                    gyro_z=float(d.sensordata[g + 2]),
                    acc_x=float(d.sensordata[a]),
                    acc_y=float(d.sensordata[a + 1]),
                    acc_z=float(d.sensordata[a + 2]))

    def contact_count(self):
        return int(self.data.ncon)

    def measure(self, seconds=1.0):
        """跑一段时间并返回该段的平均滑移与角速度（用于自检）。"""
        st0 = self.state()
        dl0, dr0 = self.wheel_angles()
        for _ in range(int(round(seconds / self.dt))):
            mujoco.mj_step(self.model, self.data)
        st1 = self.state()
        dl1, dr1 = self.wheel_angles()
        r = config.WHEEL_RADIUS
        ds_wheel = r * 0.5 * ((dl1 - dl0) + (dr1 - dr0))
        ds_body = math.hypot(st1.x - st0.x, st1.y - st0.y)
        slip = 1.0 - ds_body / ds_wheel if abs(ds_wheel) > 1e-6 else 0.0
        dyaw = wrap_angle(st1.yaw - st0.yaw)
        return dict(slip=float(slip), ds_wheel=ds_wheel, ds_body=ds_body,
                    w_actual=dyaw / seconds, v_actual=ds_body / seconds)

    # ------------------------------------------------------------------
    def render(self, width=640, height=360, camera="chase"):
        """渲染一帧（无 GPU 时用 MUJOCO_GL=osmesa 软件渲染）。"""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height, width)
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    _renderer = None

    def close(self):
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None


def describe_env():
    import os
    import platform
    return " | ".join(["python %s" % platform.python_version(),
                       "numpy %s" % np.__version__,
                       "mujoco %s" % mujoco.__version__,
                       "MUJOCO_GL=%s" % (os.environ.get("MUJOCO_GL") or "default")])
