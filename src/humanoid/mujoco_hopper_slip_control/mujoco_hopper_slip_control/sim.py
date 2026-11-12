# -*- coding: utf-8 -*-
"""MuJoCo 仿真封装：模型加载、单步推进、遥测采集、离屏渲染。"""

import os
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

import mujoco_hopper_slip_control.config as config
from mujoco_hopper_slip_control.controller import HopperController

# 模型路径：包目录的上一级 models/hopper_scene.xml（源码树与 install 后都能找到）
_PKG_DIR = Path(__file__).resolve().parent
_CANDIDATES = [
    _PKG_DIR.parent / "models" / config.MODEL_NAME,           # 源码树：<pkg>/models/
    _PKG_DIR / "models" / config.MODEL_NAME,                  # install 后 share/
]


def find_model() -> Path:
    """定位 hopper_scene.xml，找不到就抛出带路径提示的错误。"""
    env = os.environ.get("HOPPER_MODEL")
    if env:
        p = Path(env).expanduser()
        if p.is_file():
            return p
        raise FileNotFoundError(f"HOPPER_MODEL 指向的文件不存在：{p}")
    for p in _CANDIDATES:
        if p.is_file():
            return p
    tried = "\n".join(f"  - {p}" for p in _CANDIDATES)
    raise FileNotFoundError(
        f"找不到 MuJoCo 模型 {config.MODEL_NAME}，已尝试：\n{tried}\n"
        "可用环境变量 HOPPER_MODEL 指定绝对路径。")


class HopperSim:
    """单足弹跳机器人仿真。

    ``qpos`` 布局与 models/hopper_scene.xml 一致：
    ``[root_x, root_z, hip, leg]``。
    """

    def __init__(self, model_path: Optional[Path] = None, target_height: float = config.TARGET_HEIGHT,
                 target_speed: float = config.TARGET_SPEED, start_q: float = config.START_Q):
        import mujoco    # 延迟导入：controller/metrics 在无 mujoco 环境也能测

        self._mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path or find_model()))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = config.TIMESTEP

        m = self.model
        self._j = {}
        for name in ("root_x", "root_z", "hip", "leg"):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"模型缺少关节 {name}")
            self._j[name] = (int(m.jnt_qposadr[jid]), int(m.jnt_dofadr[jid]))
        self._foot_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "foot_geom")
        self._floor_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self._act_leg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "leg_motor")
        self._act_hip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip_servo")
        self.mass = float(np.sum(m.body_mass))
        self.start_q = float(start_q)

        self.controller = HopperController(target_height=target_height,
                                           target_speed=target_speed, mass=self.mass)

        self.renderer = None
        self._cam = None
        self.reset()

    # ------------------------------------------------------------------ 状态
    def reset(self) -> None:
        """恢复到初始压缩位姿并清空控制器状态。"""
        mujoco = self._mujoco
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self._j["leg"][0]] = self.start_q
        self.data.qpos[self._j["root_z"][0]] = config.Z_REST - self.start_q + config.START_CLEARANCE
        self.data.qpos[self._j["hip"][0]] = 0.0
        self.data.qpos[self._j["root_x"][0]] = 0.0
        self.controller.reset()
        mujoco.mj_forward(self.model, self.data)

    def _contact(self) -> bool:
        d = self.data
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = int(c.geom1), int(c.geom2)
            if (a == self._foot_geom and b == self._floor_geom) or \
               (a == self._floor_geom and b == self._foot_geom):
                return True
        return False

    def state(self) -> Dict[str, float]:
        """当前状态量，供控制器与日志使用。"""
        d, j = self.data, self._j
        return {
            "z": float(d.qpos[j["root_z"][0]]),
            "vz": float(d.qvel[j["root_z"][1]]),
            "x": float(d.qpos[j["root_x"][0]]),
            "vx": float(d.qvel[j["root_x"][1]]),
            "q": float(d.qpos[j["leg"][0]]),
            "qd": float(d.qvel[j["leg"][1]]),
            "hip_q": float(d.qpos[j["hip"][0]]),
        }

    # ------------------------------------------------------------------ 积分
    def step(self) -> Dict[str, float]:
        """推进一个控制步（= 一个物理步），返回本步遥测。"""
        s = self.state()
        cmd = self.controller.update(s["z"], s["vz"], s["vx"], s["q"], s["qd"],
                                     s["hip_q"], self._contact())
        self.data.ctrl[self._act_leg] = cmd.leg_force
        self.data.ctrl[self._act_hip] = cmd.hip_target
        force = cmd.leg_force
        self._mujoco.mj_step(self.model, self.data)
        s = self.state()
        s.update(phase=cmd.phase.value, f_ex=self.controller.f_ex,
                 leg_cmd=force, contact=self._contact())
        return s

    def run(self, duration: float, disturb_at: Optional[float] = None,
            disturb_dv: float = config.DISTURB_DV,
            stop_on_fall: bool = True) -> Dict[str, np.ndarray]:
        """跑一段时间，返回等长遥测数组。

        :param duration: 仿真时长 (s)
        :param disturb_at: 在该时刻施加一次水平速度冲量 (m/s)，None 表示不加
        :param disturb_dv: 冲量大小
        :param stop_on_fall: 躯干低于 ``FALL_Z`` 时提前结束
        """
        n = int(round(duration / config.TIMESTEP))
        keys = ("t", "z", "vz", "x", "vx", "q", "qd", "hip_q",
                "phase", "f_ex", "leg_cmd", "contact")
        log: Dict[str, List[float]] = {k: [] for k in keys}
        hit = False
        for _ in range(n):
            s = self.step()
            t = float(self.data.time)
            if disturb_at is not None and not hit and t >= disturb_at:
                self.data.qvel[self._j["root_x"][1]] += disturb_dv
                hit = True
            log["t"].append(t)
            for k in keys[1:]:
                v = s[k]
                if k == "phase":
                    log[k].append(1.0 if v == "stance" else 0.0)
                elif k == "contact":
                    log[k].append(1.0 if v else 0.0)
                else:
                    log[k].append(float(v))
            if stop_on_fall and s["z"] < config.FALL_Z:
                break
        return {k: np.asarray(v, dtype=float) for k, v in log.items()}

    # ------------------------------------------------------------------ 渲染
    def render(self, width: int = 640, height: int = 360, camera: str = "track") -> np.ndarray:
        """离屏渲染一帧 RGB。"""
        mujoco = self._mujoco
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height, width)
        cam = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera) \
            if camera else -1
        self.renderer.update_scene(self.data, camera=cam if cam >= 0 else None)
        return self.renderer.render()

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
