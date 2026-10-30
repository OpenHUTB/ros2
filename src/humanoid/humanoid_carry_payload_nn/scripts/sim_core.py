#!/usr/bin/env python3
"""sim_core.py —— 人形机器人双臂搬运负载的 MuJoCo 仿真核心（无 ROS 依赖）。

职责：
  1. 加载自建 MJCF 模型（models/humanoid_carry.xml），并按场景改写负载质量；
  2. 把步态规划（gait.py）的关节目标角与平衡控制器（model.py）的残差补偿
     叠加后写入位置伺服执行器；
  3. 输出躯干姿态、里程计、评价指标与前视/侧视渲染图像。

与 ROS 的关系：本文件不 import rospy，standalone 入口（main.py）、离线评测
（evaluate.py）与 ROS 节点共用同一套仿真逻辑，因此没有 ROS 环境也能跑通。
"""

import math
import os

import mujoco
import numpy as np

from gait import GaitGenerator

_MODEL_NAME = "humanoid_carry.xml"

#: 双臂"抱持"姿态（肩 pitch / 肘 pitch），与模型 weld 约束的初始位形一致
HOLD_POSE = {
    "shoulder_pitch_l": -0.55, "elbow_l": -1.05,
    "shoulder_pitch_r": -0.55, "elbow_r": -1.05,
}
#: 初始根高度（脚掌刚好贴地，对应 MJCF 中 root body 的 pos z）
ROOT_Z = 0.83
#: 负载箱在初始位形下的世界坐标（与 MJCF 中 body 的 pos 一致）
PAYLOAD_POS = (0.34, 0.0, 1.00)
#: 模型里负载箱的标称质量（MJCF 中 geom mass="2"）
NOMINAL_PAYLOAD_MASS = 2.0
#: "空载"场景下负载箱的实际质量（保留极小值避免零惯量奇异）
ZERO_PAYLOAD_MASS = 0.05


def package_dir():
    """包根目录：源码运行时为包目录，catkin 安装后由 roslib 解析到 share/<pkg>。"""
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        import roslib.packages
        return roslib.packages.get_pkg_dir("humanoid_carry_payload_nn")
    except Exception:                                             # noqa: BLE001
        return os.path.dirname(here)


def package_path(rel):
    """把包内相对路径（如 models/balance_mlp.npz）解析为绝对路径。"""
    return os.path.join(package_dir(), rel)


def default_model_path():
    """定位 MJCF 模型：源码运行时位于包的 ../models/，安装后位于 share/<pkg>/models/。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(os.path.dirname(here), "models", _MODEL_NAME),
        os.path.join(here, "..", "models", _MODEL_NAME),
        package_path(os.path.join("models", _MODEL_NAME)),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    return os.path.abspath(candidates[0])


def open_viewer(sim, log=None):
    """打开 MuJoCo 实时可视化窗口；无图形环境时返回 None（仿真仍继续）。"""
    def say(msg, warn=False):
        if log is not None:
            (log.warn if warn else log.info)(msg)
        else:
            print(("[WARN] " if warn else "[INFO] ") + msg)

    try:
        import mujoco.viewer
    except Exception as exc:                                      # noqa: BLE001
        say("无法加载 MuJoCo 可视化模块：%s" % exc, warn=True)
        return None
    try:
        view = mujoco.viewer.launch_passive(sim.model, sim.data)
    except Exception as exc:                                      # noqa: BLE001
        say("打开可视化窗口失败：%s" % exc, warn=True)
        say("需要图形界面：虚拟机桌面直接运行，SSH 远程请用 ssh -X；"
            "无图形环境请加 --no-viewer。", warn=True)
        return None
    try:
        torso = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        if torso >= 0:
            view.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            view.cam.trackbodyid = torso
            view.cam.distance = 3.4
            view.cam.azimuth = 135.0
            view.cam.elevation = -12.0
    except Exception:                                             # noqa: BLE001
        pass
    say("已打开 MuJoCo 可视化窗口（关闭窗口或按 ESC 退出）")
    return view


class CarryPayloadSim:
    """双臂搬运负载的人形机器人仿真封装。"""

    def __init__(self, model_path=None, payload_mass=2.0, gait=None):
        path = model_path or default_model_path()
        # 中文/非 ASCII 路径：先读字符串再编译
        with open(path, encoding="utf-8") as f:
            self.model = mujoco.MjModel.from_xml_string(f.read())
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self.model_path = path

        self.root_x_q = int(self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")])
        self.root_z_q = int(self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root_z")])
        self.root_pitch_q = int(self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root_pitch")])
        self.root_pitch_v = int(self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root_pitch")])
        self.root_yaw_q = int(self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root_yaw")])
        self.payload_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "payload_free")
        self.payload_q = int(self.model.jnt_qposadr[self.payload_joint])
        self.payload_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "payload")
        self._base_inertia = self.model.body_inertia[self.payload_body].copy()

        self.actuator = {}
        self.joint_qadr = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            self.actuator[name] = i
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            self.joint_qadr[name] = int(self.model.jnt_qposadr[i])

        self.gait = gait or GaitGenerator()
        self.payload_mass = float(payload_mass)
        self._renderer = None
        # 跑步机皮带（速度由指令给定；机器人原地行走）
        self.belt_dof = int(self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "belt_x")])
        self.root_yaw_v = int(self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root_yaw")])
        # 等效搬运里程（皮带走过的距离 = 速度指令积分）
        self.distance = 0.0
        self.reset(self.payload_mass)

    # ------------------------------------------------------------------
    def reset(self, payload_mass=None, phase=0.0):
        """重置到"双手抱持负载、自然站立"的初始状态。"""
        if payload_mass is not None:
            self.payload_mass = float(payload_mass)
        mujoco.mj_resetData(self.model, self.data)
        q = self.data.qpos
        q[self.root_x_q] = 0.0
        q[self.root_z_q] = 0.0
        q[self.root_pitch_q] = 0.0
        q[self.root_yaw_q] = 0.0
        for name, value in HOLD_POSE.items():
            q[self.joint_qadr[name]] = value
        q[self.payload_q:self.payload_q + 3] = PAYLOAD_POS
        q[self.payload_q + 3:self.payload_q + 7] = [1.0, 0.0, 0.0, 0.0]

        effective_mass = max(self.payload_mass, ZERO_PAYLOAD_MASS)
        self.model.body_mass[self.payload_body] = effective_mass
        self.model.body_inertia[self.payload_body] = \
            self._base_inertia * (effective_mass / NOMINAL_PAYLOAD_MASS)

        self.data.ctrl[:] = 0.0
        for name, value in HOLD_POSE.items():
            self.data.ctrl[self.actuator[name]] = value
        self.gait.reset(phase)
        self.distance = 0.0
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------
    def posture(self):
        """躯干姿态：(roll, pitch, roll_rate, pitch_rate)。

        本模型的根关节由助行架导轨约束：侧移与横滚被锁定，因此 roll 恒为 0，
        俯仰角直接取 root_pitch 关节角（等价于矢状面内的躯干倾角）。
        """
        pitch = float(self.data.qpos[self.root_pitch_q])
        pitch_rate = float(self.data.qvel[self.root_pitch_v])
        return 0.0, pitch, 0.0, pitch_rate

    def features(self, roll, pitch, roll_rate, pitch_rate, v_cmd, w_cmd):
        return {
            "roll": roll, "pitch": pitch,
            "roll_rate": roll_rate, "pitch_rate": pitch_rate,
            "payload_norm": self.payload_mass / 4.0,
            "v_cmd": float(v_cmd), "w_cmd": float(w_cmd),
            "phase_sin": math.sin(2 * math.pi * self.gait.phase),
            "phase_cos": math.cos(2 * math.pi * self.gait.phase),
        }

    # ------------------------------------------------------------------
    def step(self, v_cmd=0.0, w_cmd=0.0, controller=None):
        """推进一个物理步。

        参数:
            v_cmd: 前进速度指令（m/s）
            w_cmd: 偏航角速度指令（rad/s，正值为左转）
            controller: 平衡控制器（提供 act(features) -> 补偿角字典）；
                        None 表示只跑开环步态（对照基线）
        返回: 本步的特征字典（供训练/记录使用）
        """
        self.gait.advance(v_cmd, w_cmd, self.dt)
        targets = self.gait.targets(v_cmd)

        # 跑步机：皮带以 -v_cmd 的速度向后跑，机器人原地行走；转向由助行架转盘给定
        self.data.qvel[self.belt_dof] = -float(v_cmd)
        self.data.qvel[self.root_yaw_v] = float(w_cmd)
        self.distance += float(v_cmd) * self.dt

        roll, pitch, roll_rate, pitch_rate = self.posture()
        feats = self.features(roll, pitch, roll_rate, pitch_rate, v_cmd, w_cmd)
        corr = controller.act(feats) if controller is not None else {}

        ankle_pitch = corr.get("ankle_pitch", 0.0)
        ankle_roll = corr.get("ankle_roll", 0.0)
        hip_pitch = corr.get("hip_pitch", 0.0)
        for side in ("l", "r"):
            self.data.ctrl[self.actuator["hip_pitch_" + side]] = \
                targets["hip_pitch_" + side] + hip_pitch
            self.data.ctrl[self.actuator["knee_" + side]] = targets["knee_" + side]
            self.data.ctrl[self.actuator["ankle_pitch_" + side]] = \
                targets["ankle_pitch_" + side] + ankle_pitch
            self.data.ctrl[self.actuator["hip_roll_" + side]] = \
                targets["hip_roll_" + side] - ankle_roll * 0.5
            self.data.ctrl[self.actuator["ankle_roll_" + side]] = \
                targets["ankle_roll_" + side] + ankle_roll
            self.data.ctrl[self.actuator["hip_yaw_" + side]] = targets["hip_yaw_" + side]
        for name, value in HOLD_POSE.items():
            self.data.ctrl[self.actuator[name]] = value

        mujoco.mj_step(self.model, self.data)
        feats["expert"] = corr
        return feats

    # ------------------------------------------------------------------
    def odom(self):
        x = float(self.data.qpos[self.root_x_q])
        z = ROOT_Z + float(self.data.qpos[self.root_z_q])
        yaw = float(self.data.qpos[self.root_yaw_q])
        return {"x": x, "y": 0.0, "z": z, "yaw": yaw}

    def standing(self, min_tilt_deg=35.0):
        """未摔倒判定：俯仰角不超限且根高度没有跌落到蹲姿以下。"""
        _, pitch, _, _ = self.posture()
        return bool(abs(math.degrees(pitch)) < min_tilt_deg
                    and self.odom()["z"] > ROOT_Z - 0.30)

    def render(self, camera="overview", height=480, width=640):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:                                     # noqa: BLE001
                pass
            self._renderer = None
