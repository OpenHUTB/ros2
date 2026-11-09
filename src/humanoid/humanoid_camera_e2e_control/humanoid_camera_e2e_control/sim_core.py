# -*- coding: utf-8 -*-
"""
sim_core.py —— 人形机器人 MuJoCo 仿真核心（无 ROS 依赖）

职责：
  1. 加载避障场景人形 MJCF（models/humanoid_obstacle.xml）；
  2. 站姿 PD 伺服维持直立（髋/踝 roll 单独加强增益）；
  3. 运动控制：前进为速度级改写底盘世界系线速度；转向为姿态级旋转四元数；
  4. 感知输出：跟随视角相机 RGB 图像、第三方视角图像、里程计、关节状态、站立状态、
     人形与障碍物的接触检测。

说明：本模块为演示级运动学控制——直接改写 freejoint 速度与根部四元数，不包含
双足步态生成；图像端到端策略只学习"从画面到速度指令"的映射，不代表真实步行
动力学准确性（详见 docs/humanoid/humanoid_camera_e2e_control.md）。

本文件不依赖 ROS，standalone 入口（main.py / scripts/*.py）与 ROS 节点
（mujoco_sim_node.py / e2e_policy_node.py）共用同一套仿真逻辑。
"""

import math
import os

import numpy as np
import mujoco

_MODEL_NAME = "humanoid_obstacle.xml"

# 跟随视角 / 第三方视角图像尺寸（H, W），网络输入尺寸与之保持一致
IMG_H = 96
IMG_W = 128


def _default_model_path():
    """返回包内 MJCF 模型路径（源码目录 / ament share 多级回退）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    here_real = os.path.dirname(os.path.realpath(__file__))
    candidates = [
        os.path.join(here_real, "..", "models", _MODEL_NAME),
        os.path.join(here, "..", "models", _MODEL_NAME),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory("humanoid_camera_e2e_control"), "models", _MODEL_NAME))
    except Exception:                                             # noqa: BLE001
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0]


def open_viewer(sim, log=None):
    """打开 MuJoCo 实时可视化窗口；无图形环境返回 None。"""
    def say(msg, warn=False):
        if log is not None:
            (log.warning if warn else log.info)(msg)
        else:
            print(("[WARN] " if warn else "[INFO] ") + msg)

    try:
        import mujoco.viewer
    except Exception as exc:                                      # noqa: BLE001
        say(f"无法加载 MuJoCo 可视化模块：{exc}", warn=True)
        return None
    try:
        view = mujoco.viewer.launch_passive(sim.model, sim.data)
    except Exception as exc:                                      # noqa: BLE001
        say(f"打开可视化窗口失败：{exc}", warn=True)
        say("可视化窗口需要图形界面（DISPLAY）；无图形环境请用 --no-viewer / visualize:=false",
            warn=True)
        return None
    try:
        torso = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        if torso >= 0:
            view.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            view.cam.trackbodyid = torso
            view.cam.distance = 3.2
            view.cam.azimuth = 140.0
            view.cam.elevation = -12.0
    except Exception:                                             # noqa: BLE001
        pass
    say("已打开 MuJoCo 可视化窗口（关闭窗口或按 ESC 退出）")
    return view


class HumanoidSim:
    """人形机器人 MuJoCo 仿真封装。"""

    #: 站姿 PD 增益（关节力矩 = kp*(目标-当前) - kv*角速度）
    KP = 120.0
    KV = 6.0
    KP_ROLL = 400.0
    #: 运动指令限幅（网络输出反归一化也使用同一限幅）
    V_MAX = 0.30      # m/s
    W_MAX = 0.60      # rad/s

    def __init__(self, model_path=None, realtime=False):
        path = model_path or _default_model_path()
        with open(path, encoding="utf-8") as f:
            xml = f.read()
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)

        # 定位 root freejoint 与人形关节
        self.root_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        self.root_qadr = self.model.jnt_qposadr[self.root_id]
        self.root_dadr = self.model.jnt_dofadr[self.root_id]
        self.human_qadr = [self.model.jnt_qposadr[i]
                           for i in range(self.model.njnt)
                           if self.model.jnt_qposadr[i] > self.root_qadr]
        self.human_dadr = [self.model.jnt_dofadr[i]
                           for i in range(self.model.njnt)
                           if self.model.jnt_qposadr[i] > self.root_qadr]
        self.roll_idx = []
        for nm in ("hip_roll_l", "hip_roll_r", "ankle_roll_l", "ankle_roll_r"):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, nm)
            self.roll_idx.append(self.human_qadr.index(self.model.jnt_qposadr[jid]))

        # 人形 / 障碍物 geom id（供接触检测）
        self._human_geom_ids = set()
        for i in range(self.model.ngeom):
            body_id = self.model.geom_bodyid[i]
            if body_id == self.root_id:
                self._human_geom_ids.add(i)
        self._obstacle_geom_ids = set()
        for nm in ("pl1", "pl2", "pl3", "pl4", "pr1", "pr2", "pr3", "pr4",
                   "ba", "bb", "bc"):
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, nm)
            if gid >= 0:
                self._obstacle_geom_ids.add(gid)

        self.realtime = realtime
        self.dt = self.model.opt.timestep

        # 复用 Renderer，避免每帧重建上下文带来的开销
        self._renderers = {}
        self.reset()

    # ------------------------------------------------------------------
    # 基础操作
    # ------------------------------------------------------------------
    def reset(self, x=0.0, y=0.0, yaw=0.0):
        """重置仿真；可指定初始位置与朝向（rad）。"""
        mujoco.mj_resetData(self.model, self.data)
        r = self.root_qadr
        self.data.qpos[r] = x
        self.data.qpos[r + 1] = y
        self.data.qpos[r + 2] = 0.95
        # 初始四元数：绕 z 轴 yaw
        self.data.qpos[r + 3: r + 7] = self._yaw_to_quat(yaw)
        mujoco.mj_forward(self.model, self.data)

    @staticmethod
    def _yaw_to_quat(yaw):
        h = yaw / 2.0
        return np.array([math.cos(h), 0.0, 0.0, math.sin(h)])

    def step(self, v_lin=0.0, w_yaw=0.0):
        """执行一步仿真：站姿 PD + 速度级前进 + 姿态级转向。"""
        v_lin = float(np.clip(v_lin, -self.V_MAX, self.V_MAX))
        w_yaw = float(np.clip(w_yaw, -self.W_MAX, self.W_MAX))

        # 1) 站姿 PD 力矩（目标姿态：全部关节直立 0）
        ctrl = self.KP * (-self.data.qpos[self.human_qadr]) \
            - self.KV * self.data.qvel[self.human_dadr]
        for qi in self.roll_idx:
            ctrl[qi] = self.KP_ROLL * (-self.data.qpos[self.human_qadr[qi]]) \
                - self.KV * self.data.qvel[self.human_dadr[qi]]
        self.data.ctrl[:] = np.clip(ctrl, -250.0, 250.0)

        # 2) 前进/后退：速度级（按当前偏航角分解到世界系 vx/vy）
        yaw = self._yaw()
        self.data.qvel[self.root_dadr] = v_lin * math.cos(yaw)
        self.data.qvel[self.root_dadr + 1] = v_lin * math.sin(yaw)
        # 3) 转向：姿态级（清除角速度，直接旋转四元数，避免脚底摩擦滑移失稳）
        self.data.qvel[self.root_dadr + 3: self.root_dadr + 6] = 0.0
        if w_yaw != 0.0:
            d = w_yaw * self.dt
            qw, qx, qy, qz = self.data.qpos[self.root_qadr + 3: self.root_qadr + 7]
            dw, dx, dy, dz = math.cos(d / 2), 0.0, 0.0, math.sin(d / 2)
            nw = qw * dw - qx * dx - qy * dy - qz * dz
            nx = qw * dx + qx * dw + qy * dz - qz * dy
            ny = qw * dy - qx * dz + qy * dw + qz * dx
            nz = qw * dz + qx * dy - qy * dx + qz * dw
            self.data.qpos[self.root_qadr + 3: self.root_qadr + 7] = np.array([nw, nx, ny, nz])

        mujoco.mj_step(self.model, self.data)

    # ------------------------------------------------------------------
    # 感知输出
    # ------------------------------------------------------------------
    def _yaw(self):
        """底盘当前偏航角（绕世界系 z，rad）。"""
        r = self.root_qadr
        qw, qx, qy, qz = self.data.qpos[r + 3: r + 7]
        return math.atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz))

    def read_odom(self):
        """里程计：位置、偏航角、速度。"""
        q = self.data.qpos
        r = self.root_qadr
        return {
            "x": float(q[r]), "y": float(q[r + 1]), "z": float(q[r + 2]),
            "yaw": self._yaw(),
            "vx": float(self.data.qvel[self.root_dadr]),
            "vy": float(self.data.qvel[self.root_dadr + 1]),
            "wz": float(self.data.qvel[self.root_dadr + 5]),
        }

    def read_joint_positions(self):
        """关节位置（rad），键为关节名。"""
        out = {}
        for i in range(self.model.njnt):
            if self.model.jnt_qposadr[i] <= self.root_qadr:
                continue
            nm = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            out[nm] = float(self.data.qpos[self.model.jnt_qposadr[i]])
        return out

    def _get_renderer(self, camera, height, width):
        key = (camera, height, width)
        if key not in self._renderers:
            self._renderers[key] = mujoco.Renderer(self.model, height=height, width=width)
        return self._renderers[key]

    def render_front(self):
        """感知图像（uint8, H×W×3），端到端 CNN 的输入。

        使用运行时 ``MjvCamera``（free 模式）跟随机器人本体：相机位于机器人后方，
        以固定俯视角看向机器人躯干，前方障碍相对机器人出现在画面左右两侧，
        使"障碍相对方位—转向指令"在视觉上可辨识。演示路段控制在该相机视野内，
        视角方案与能力边界见模块文档。
        """
        ren = self._get_renderer("free", IMG_H, IMG_W)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.trackbodyid = -1
        cam.distance = 2.6
        cam.azimuth = 180.0      # 相机位于机器人身后，看向机器人前方
        cam.elevation = 32.0     # 俯视角度，同时保留近处地面与前方立柱
        od = self.read_odom()
        cam.lookat[:] = [od["x"], od["y"], 0.6]
        ren.update_scene(self.data, camera=cam)
        return ren.render()

    def render_overview(self, height=360, width=480):
        """第三方视角图像（演示 / GIF 录制用）。"""
        r = self._get_renderer("overview", height, width)
        r.update_scene(self.data, camera="overview")
        return r.render()

    def standing(self):
        """是否仍保持站立（根高 > 0.9 m）。"""
        return self.data.qpos[self.root_qadr + 2] > 0.9

    def contact_obstacle(self):
        """人形是否与障碍物发生接触（True 表示碰到立柱 / 散块）。"""
        for k in range(self.data.ncon):
            c = self.data.contact[k]
            g1, g2 = c.geom1, c.geom2
            if (g1 in self._human_geom_ids and g2 in self._obstacle_geom_ids) or \
               (g2 in self._human_geom_ids and g1 in self._obstacle_geom_ids):
                return True
        return False


def calibrate_command(v: float, w: float, v_max: float = HumanoidSim.V_MAX,
                      w_max: float = HumanoidSim.W_MAX, v_gain: float = 1.4):
    """部署出口指令标定：线速度做温和增益补偿，偏航角速度保持网络原值并限幅。

    训练教师线速度为 0.10 m/s，归一化上限取 V_MAX=0.30，网络在演示画面与训练
    分布略有偏移时 v 容易被压小；此处对 v 施加固定增益并限幅，w 不放大以免
    过度转向。standalone 演示与 ROS2 策略节点共用同一标定口径。
    """
    v = float(np.clip(v * v_gain, 0.0, v_max))
    w = float(np.clip(w, -w_max, w_max))
    return v, w
