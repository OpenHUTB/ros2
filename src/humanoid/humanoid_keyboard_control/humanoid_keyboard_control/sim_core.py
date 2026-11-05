# -*- coding: utf-8 -*-
"""
sim_core.py —— 人形机器人 MuJoCo 仿真核心（无 ROS 依赖）

职责：
  1. 加载自建简化人形 MJCF 模型（models/humanoid_simple.xml）；
  2. 站姿控制：所有关节 PD 伺服回直立位（髋/踝 roll 单独加强增益）；
  3. 运动控制（键盘指令 v/w）：
       - 前进/后退：速度级控制（按当前偏航角将指令速度分解到世界系 vx/vy，
         使"前进"始终沿机器人当前朝向）；
       - 转向：姿态级控制（直接对底盘四元数做微小旋转，避免双足原地
         旋转时脚底摩擦滑移导致的侧倾失稳）；
  4. 输出：里程计（x, y, yaw）、关节状态、前视相机渲染。

说明：本模块为演示级运动学控制——直接改写 freejoint 速度与根部四元数，
不包含双足步态生成；位移/转角指标仅验证指令执行准确性，不代表真实
步行控制或动力学准确性（详见 docs/humanoid/humanoid_keyboard_control.md）。

本文件不依赖 ROS，standalone 入口（main.py）与 ROS 节点（mujoco_sim_node.py）
共用同一套仿真逻辑。
"""

import math
import os

import numpy as np
import mujoco

# 模型文件位于包内 models/ 目录。定位方式见 _default_model_path()：
# 源码运行 / 符号链接安装时在包的 ../models/；colcon 正式安装后模型被放到
# share/<pkg>/models/，与 site-packages 下的 .py 不同级，需通过 ament 索引查找。
_MODEL_NAME = "humanoid_simple.xml"


def _default_model_path():
    """返回包内 MJCF 模型的可用路径（多级回退）。

    依次尝试：
      1. ``realpath`` 后的 ../models/ —— colcon --symlink-install 时包目录是指向
         源码的符号链接，realpath 能回到源码树，那里确实有 models/；
      2. ``abspath`` 后的 ../models/ —— 直接以源码目录运行时有效；
      3. ament 索引的 ``share/<pkg>/models/`` —— 正式安装（拷贝而非软链）时模型
         被装到 share 下，与 Python 包目录不同级，只能靠索引定位。

    三者都不存在时返回第 1 个候选，便于报错信息直接指向预期位置。
    本函数不强制依赖 ROS：无 ament_index_python 时自动跳过第 3 种。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    here_real = os.path.dirname(os.path.realpath(__file__))
    candidates = [
        os.path.join(here_real, "..", "models", _MODEL_NAME),
        os.path.join(here, "..", "models", _MODEL_NAME),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory("humanoid_keyboard_control"), "models", _MODEL_NAME))
    except Exception:                                             # noqa: BLE001
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0]


_MODEL_REL = _default_model_path()


def open_viewer(sim, log=None):
    """打开 MuJoCo 实时可视化窗口（可直接看到人形机器人界面）。

    需要图形环境：本机 / 虚拟机桌面直接可用；SSH 远程需 `ssh -X` 转发 X11，
    或设置 ``export DISPLAY=:0``。无图形环境时返回 ``None`` 并给出提示，
    调用方可平滑退化为无窗口运行（不影响物理仿真与话题发布）。

    参数:
        sim: ``HumanoidSim`` 实例（窗口直接绑定其 model/data）
        log: 可选 ``rclpy`` logger；提供时用 logger 输出提示，否则用 print
    """
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
        say("可视化窗口需要图形界面（DISPLAY），可尝试：", warn=True)
        say("  · 本机 / 虚拟机桌面：直接运行即可看到人形机器人界面；", warn=True)
        say("  · SSH 远程：用 `ssh -X` 开启 X11 转发，或 `export DISPLAY=:0`；", warn=True)
        say("  · 无图形环境：standalone 用 `--no-viewer`；ROS2 用 `visualize:=false`。", warn=True)
        return None
    # 相机跟踪躯干，便于观察站立与行走位姿
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

    #: 站姿 PD 增益（关节力矩 = kp * (目标-当前) - kv * 角速度）
    KP = 120.0
    KV = 6.0
    #: 髋/踝 roll 关节单独加强，抗侧倾
    KP_ROLL = 400.0
    #: 运动指令限幅
    V_MAX = 0.3      # m/s
    W_MAX = 0.6      # rad/s

    def __init__(self, model_path=None, realtime=False):
        # 中文/非 ASCII 路径兼容：先读文件内容再从字符串加载
        path = model_path or _MODEL_REL
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
        # roll 关节（抗侧倾加强）
        self.roll_idx = []
        for nm in ("hip_roll_l", "hip_roll_r", "ankle_roll_l", "ankle_roll_r"):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, nm)
            self.roll_idx.append(self.human_qadr.index(self.model.jnt_qposadr[jid]))

        self.realtime = realtime
        self.dt = self.model.opt.timestep
        self.reset()

    # ------------------------------------------------------------------
    # 基础操作
    # ------------------------------------------------------------------
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def step(self, v_lin=0.0, w_yaw=0.0):
        """执行一步仿真。

        参数:
            v_lin: 线速度指令（m/s，正值沿机器人当前朝向"前进"）
            w_yaw: 偏航角速度指令（rad/s），姿态级控制

        说明：v_lin 会按当前偏航角分解为世界系 vx/vy，因此转向后仍沿
        机器人朝向前进；w_yaw 为 0 时机器人按当前朝向匀速直线移动。
        """
        v_lin = float(np.clip(v_lin, -self.V_MAX, self.V_MAX))
        w_yaw = float(np.clip(w_yaw, -self.W_MAX, self.W_MAX))

        # 1) 站姿 PD 力矩（目标姿态：全部关节直立 0）
        ctrl = self.KP * (-self.data.qpos[self.human_qadr]) - self.KV * self.data.qvel[self.human_dadr]
        for qi in self.roll_idx:
            ctrl[qi] = self.KP_ROLL * (-self.data.qpos[self.human_qadr[qi]]) \
                       - self.KV * self.data.qvel[self.human_dadr[qi]]
        self.data.ctrl[:] = np.clip(ctrl, -250.0, 250.0)

        # 2) 前进/后退：速度级控制（按偏航角将指令速度分解到世界系 vx/vy）
        yaw = self._yaw()
        self.data.qvel[self.root_dadr] = v_lin * math.cos(yaw)
        self.data.qvel[self.root_dadr + 1] = v_lin * math.sin(yaw)
        # 3) 转向：姿态级控制（清除角速度，直接旋转四元数）
        self.data.qvel[self.root_dadr + 3: self.root_dadr + 6] = 0.0
        if w_yaw != 0.0:
            d = w_yaw * self.dt
            qw, qx, qy, qz = self.data.qpos[self.root_qadr + 3: self.root_qadr + 7]
            dw, dx, dy, dz = math.cos(d / 2), 0.0, 0.0, math.sin(d / 2)
            nw = qw*dw - qx*dx - qy*dy - qz*dz
            nx = qw*dx + qx*dw + qy*dz - qz*dy
            ny = qw*dy - qx*dz + qy*dw + qz*dx
            nz = qw*dz + qx*dy - qy*dx + qz*dw
            self.data.qpos[self.root_qadr + 3: self.root_qadr + 7] = np.array([nw, nx, ny, nz])

        mujoco.mj_step(self.model, self.data)

    # ------------------------------------------------------------------
    # 感知输出
    # ------------------------------------------------------------------
    def _yaw(self):
        """底盘当前偏航角（绕世界系 z，rad）。"""
        r = self.root_qadr
        qw, qx, qy, qz = self.data.qpos[r + 3: r + 7]
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def read_odom(self):
        """里程计：位置 (x, y)、偏航角、速度。"""
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

    def render(self, camera="front_camera", height=480, width=640):
        """渲染相机图像（RGB ndarray）。"""
        renderer = mujoco.Renderer(self.model, height=height, width=width)
        renderer.update_scene(self.data, camera=camera)
        img = renderer.render()
        del renderer
        return img

    def standing(self):
        """是否仍保持站立（根高 > 0.9 m）。"""
        return self.data.qpos[self.root_qadr + 2] > 0.9
