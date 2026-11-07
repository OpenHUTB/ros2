# -*- coding: utf-8 -*-
"""MuJoCo 环境封装：两轮自平衡机器人。

接口刻意保持极简（reset / set_torque / step / state），
因为上层只有一个控制器和一个神经网络，不需要复杂抽象。
"""

import math
import os

import numpy as np

from . import config

import mujoco  # noqa: E402  （不再在 import 阶段设置 MUJOCO_GL）


class MujocoUnavailable(RuntimeError):
    pass


class State(object):
    """一帧状态。角度单位：弧度。

    theta      倾角：0 = 竖直，**正值 = 车身向后仰**，负值 = 前倾
    theta_dot  倾角角速度
    v          车体沿 x 的速度（前进为正）
    wheel      左右轮角速度 [左, 右]
    """

    __slots__ = ("t", "x", "y", "z", "theta", "theta_dot", "yaw", "v", "w",
                 "wheel", "ncon")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))

    def __repr__(self):
        return ("t=%.2f x=%+.3f θ=%+.3f rad θ̇=%+.2f v=%+.3f yaw=%+.3f"
                % (self.t, self.x, self.theta, self.theta_dot, self.v, self.yaw))


class BalanceBotEnv(object):

    def __init__(self, scene_path, verbose=True):
        try:
            self.model = mujoco.MjModel.from_xml_path(scene_path)
        except Exception as exc:
            raise MujocoUnavailable("无法加载场景 %s：%s" % (scene_path, exc))
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self.steps_per_control = max(1, int(round(1.0 / (self.dt * config.CONTROL_HZ))))

        self._base = self._id(mujoco.mjtObj.mjOBJ_BODY, "base")
        self._wl = self._dofadr("wheel_l_joint")
        self._wr = self._dofadr("wheel_r_joint")
        self._wl_q = self._qposadr("wheel_l_joint")
        self._wr_q = self._qposadr("wheel_r_joint")
        self._renderer = None

        self.reset()
        if verbose:
            print("[MuJoCo] %s（nq=%d, nu=%d, dt=%.4f, 整备质量 %.2f kg）"
                  % (scene_path, self.model.nq, self.model.nu, self.dt,
                     float(self.model.body_mass.sum())))

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
    def reset(self, x=0.0, y=0.0, yaw=0.0, tilt=None):
        """复位。默认给一个小倾角，否则完全竖直时数值上永远不动。"""
        d = self.data
        tilt = config.SPAWN_TILT if tilt is None else float(tilt)
        mujoco.mj_resetData(self.model, d)
        d.qpos[0] = float(x)
        d.qpos[1] = float(y)
        d.qpos[2] = config.SPAWN_Z
        # 绕 y 轴倾斜 = 四元数 (cos(θ/2), 0, sin(θ/2), 0) 与 yaw 组合；
        # 这里只处理小倾角 + 任意 yaw
        cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
        cp, sp = math.cos(0.5 * tilt), math.sin(0.5 * tilt)
        d.qpos[3] = cy * cp
        d.qpos[4] = sy * sp
        d.qpos[5] = -cy * sp
        d.qpos[6] = sy * cp
        d.qpos[self._wl_q] = 0.0
        d.qpos[self._wr_q] = 0.0
        d.qvel[:] = 0.0
        d.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, d)
        return self.state()

    def set_torque(self, tau_l, tau_r):
        """给左右轮力矩（N·m），已被 TORQUE_MAX 限幅。"""
        lim = config.TORQUE_MAX
        self.data.ctrl[0] = float(np.clip(tau_l, -lim, lim))
        self.data.ctrl[1] = float(np.clip(tau_r, -lim, lim))

    def step(self, n_control=1):
        for _ in range(int(n_control)):
            for _k in range(self.steps_per_control):
                mujoco.mj_step(self.model, self.data)
        return self.state()

    # ------------------------------------------------------------------
    def state(self):
        d = self.data
        R = d.xmat[self._base].reshape(3, 3)
        x, y, z = d.xpos[self._base]
        # 车身 x 轴在世界系的 z 分量 = sin(倾角)，**向前倾时该值为负**
        theta = math.asin(max(-1.0, min(1.0, R[2, 0])))
        vel = d.cvel[self._base]          # 角速度(0:3) + 线速度(3:6)，世界系
        vx = float(vel[3] * R[0, 0] + vel[4] * R[1, 0] + vel[5] * R[2, 0])
        wl, wr = float(d.qvel[self._wl]), float(d.qvel[self._wr])
        return State(t=float(d.time), x=float(x), y=float(y), z=float(z),
                     # 注意：cvel[1] 与 dθ/dt **符号相反**（实测 dθ/dt = -cvel[1]）。
                     # 早期直接取 vel[1] 导致阻尼项在"加能量"，kd 越大倒得越快。
                     theta=theta, theta_dot=float(-vel[1]),
                     yaw=math.atan2(R[1, 0], R[0, 0]),
                     v=vx, w=float(vel[2]),
                     wheel=(wl, wr), ncon=int(d.ncon))

    def wheel_angles(self):
        d = self.data
        return float(d.qpos[self._wl_q]), float(d.qpos[self._wr_q])

    def is_upright(self):
        return abs(self.state().theta) < config.FALL_ANGLE

    # ------------------------------------------------------------------
    def render(self, width=480, height=270, camera="chase"):
        """渲染一帧。后端在这里按需选择，失败自动回退。

        MuJoCo 的渲染后端由 `MUJOCO_GL` 决定，且**只能在第一次创建 Renderer 之前**
        生效。所以不能放在 import 阶段（那会让不渲染的脚本也被拖挂），
        而是在这里逐个候选去试：default → osmesa → egl。
        """
        if self._renderer is not None:
            self._renderer.update_scene(self.data, camera=camera)
            return self._renderer.render()

        last_exc = None
        for backend in config.render_backend_candidates():
            os.environ["MUJOCO_GL"] = backend
            try:
                self._renderer = mujoco.Renderer(self.model, height, width)
                print("[渲染] MUJOCO_GL=%s 可用" % backend)
                break
            except Exception as exc:          # noqa: BLE001
                last_exc = exc
                self._renderer = None
        if self._renderer is None:
            raise RuntimeError(
                "没有可用的 MuJoCo 渲染后端（试过 %s）。"
                "无桌面环境请安装 libosmesa6，或设置 MUJOCO_GL=egl。原始错误：%s"
                % (config.render_backend_candidates(), last_exc))
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None


def describe_env():
    import platform
    return " | ".join(["python %s" % platform.python_version(),
                       "numpy %s" % np.__version__,
                       "mujoco %s" % mujoco.__version__,
                       "MUJOCO_GL=%s" % (__import__("os").environ.get("MUJOCO_GL") or "default")])
