# -*- coding: utf-8 -*-
"""
sim_core.py —— 人形机器人平衡与抗扰恢复 MuJoCo 仿真核心（无 ROS 依赖）

职责：
  1. 加载自建双足人形 MJCF 模型（models/humanoid_balance.xml），底盘 freejoint 自由；
  2. 站姿 PD 伺服（目标为直立位），可选叠加平衡控制器的关节目标修正；
  3. 外力扰动注入：对 torso 施加 6 维力/力矩（data.xfrc_applied），
     支持「冲量式（持续 dt 秒）」与「持续力式」两种扰动；
  4. 状态读取：质心位置/速度/加速度、ZMP、支撑多边形与稳定裕度、躯干倾角、跌倒判定。

说明：本模型的底盘**不做速度锁定**（这点与 humanoid_keyboard_control 的
humanoid_simple.xml 不同），机器人完全依靠双脚与地面的接触以及关节力矩站立，
因此可以直接施加外力扰动并观察「被推 → 恢复 / 跌倒」的真实物理过程。

实现注记：质心速度/加速度统一由 ``subtree_com[0]``（全身质心）的有限差分求得，
每步仿真后更新一次；不依赖 ``subtree_linvel`` / ``cvel`` 等内部缓存字段
（实测在 MuJoCo 3.16 中 ``subtree_linvel[0]`` 恒为 0，不可直接用）。

本文件不依赖 ROS，standalone 入口（main.py）与 ROS 节点（mujoco_sim_node.py）
共用同一套仿真逻辑。
"""

import os

import numpy as np
import mujoco

# 模型文件位于包内 models/ 目录
_MODEL_REL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "models", "humanoid_balance.xml")

#: 15 个受控关节（顺序与 XML 中 actuator 声明一致）
JOINT_NAMES = (
    "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_pitch_l", "ankle_roll_l",
    "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_pitch_r", "ankle_roll_r",
    "shoulder_pitch_l", "elbow_l", "shoulder_pitch_r", "elbow_r", "neck",
)

#: 侧向（roll）关节，单独加强增益以抗侧倾
ROLL_JOINTS = ("hip_roll_l", "hip_roll_r", "ankle_roll_l", "ankle_roll_r")

#: 跌倒判据
FALL_HEIGHT = 0.85      # 根高低于此值判为跌倒 (m)
FALL_TILT = 0.55        # 躯干倾角超过此值判为跌倒 (rad, ≈31°)

#: 质心加速度差分的一阶低通系数
ACC_FILTER = 0.3

#: 躯干倾角速率差分的一阶低通系数（倾角速率为旋转矩阵的原始差分，噪声大，
#: 不平滑时微分项会把控制器推向高频震荡，见 balance/ankle_strategy.py 的说明）
LEAN_RATE_FILTER = 0.7


class BalanceSim:
    """人形机器人平衡仿真封装。"""

    #: 站姿 PD 增益
    KP = 120.0
    KV = 6.0
    #: 髋/踝 roll 加强增益（抗侧倾）
    KP_ROLL = 400.0
    #: 关节力矩限幅
    TORQUE_LIMIT = 300.0

    def __init__(self, model_path=None, params=None):
        self.params = params or {}
        # 中文/非 ASCII 路径兼容：先读文件内容再从字符串加载
        path = model_path or self._default_model_path()
        with open(path, encoding="utf-8") as handle:
            xml = handle.read()
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)

        # 底盘 freejoint
        self.root_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        self.root_qadr = self.model.jnt_qposadr[self.root_id]
        self.root_dadr = self.model.jnt_dofadr[self.root_id]
        self.torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")

        # 关节 -> qpos/qvel 地址（按 JOINT_NAMES 顺序）
        qadr, dadr = [], []
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qadr.append(self.model.jnt_qposadr[jid])
            dadr.append(self.model.jnt_dofadr[jid])
        self.joint_qadr = np.array(qadr)
        self.joint_dadr = np.array(dadr)
        self.roll_idx = [JOINT_NAMES.index(n) for n in ROLL_JOINTS]

        # 脚掌 geom（用于支撑多边形）
        self.foot_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "foot_geom_l"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "foot_geom_r"),
        ]

        # 参数覆盖
        self.KP = float(self.params.get("kp", self.KP))
        self.KV = float(self.params.get("kv", self.KV))
        self.KP_ROLL = float(self.params.get("kp_roll", self.KP_ROLL))
        self.TORQUE_LIMIT = float(self.params.get("torque_limit", self.TORQUE_LIMIT))
        self.fall_height = float(self.params.get("fall_height", FALL_HEIGHT))
        self.fall_tilt = float(self.params.get("fall_tilt", FALL_TILT))

        self.dt = self.model.opt.timestep
        #: 平衡控制器（由外部注入，见 balance.py）
        self.controller = None
        #: 待施加的扰动
        self._push_fx = np.zeros(3)
        self._push_tx = np.zeros(3)
        self._push_left = 0.0
        #: 质心运动学差分状态
        self._last_com = None
        self._com_vel = np.zeros(3)
        self._com_acc = np.zeros(3)
        #: 躯干倾角差分状态（lean = 躯干轴相对竖直方向的倾斜分量）
        self._last_lean = None
        self._lean_rate = np.zeros(2)
        #: 扰动记录（供指标统计）
        self.push_history = []
        self.reset()

    # ------------------------------------------------------------------
    # 基础操作
    # ------------------------------------------------------------------
    @staticmethod
    def _default_model_path():
        """包内默认模型路径。

        优先用源码树内的 ``models/``（standalone 运行、以及 ``colcon build
        --symlink-install`` 安装的场景）；若以**非符号链接**方式安装，源码树路径
        失效，则回退到 ``share/<package>/models/``（由 setup.py 的 data_files 安装）。
        """
        if os.path.exists(_MODEL_REL):
            return _MODEL_REL
        try:
            from ament_index_python.packages import get_package_share_directory
            share = os.path.join(get_package_share_directory("humanoid_push_recovery"),
                                 "models", "humanoid_balance.xml")
            if os.path.exists(share):
                return share
        except Exception:                                         # noqa: BLE001
            pass
        return _MODEL_REL

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._push_fx = np.zeros(3)
        self._push_tx = np.zeros(3)
        self._push_left = 0.0
        self._last_com = None
        self._com_vel = np.zeros(3)
        self._com_acc = np.zeros(3)
        self._last_lean = None
        self._lean_rate = np.zeros(2)
        self.push_history = []
        self._update_com()
        self._update_lean()
        if self.controller is not None:
            self.controller.reset(self)

    def apply_push(self, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0), duration=0.08):
        """在 torso 上施加外力/力矩。

        参数:
            force:    世界系力矢量 (N)
            torque:   世界系力矩矢量 (N·m)
            duration: 持续时间 (s)；0.08 s 量级即接近冲量式推扰
        """
        self._push_fx = np.asarray(force, dtype=float)
        self._push_tx = np.asarray(torque, dtype=float)
        self._push_left = float(duration)
        self.push_history.append({
            "force": self._push_fx.copy(),
            "torque": self._push_tx.copy(),
            "duration": float(duration),
            "time": float(self.data.time),
            # 冲量大小（N·s），用于临界冲量统计
            "impulse": float(np.linalg.norm(self._push_fx)) * float(duration),
        })

    def step(self, balance=True):
        """推进一步仿真：站姿 PD + 平衡力矩 → 施加扰动 → mj_step。"""
        q = self.data.qpos[self.joint_qadr]
        qd = self.data.qvel[self.joint_dadr]

        # 1) 站姿 PD：目标为直立位（全 0）
        ctrl = self.KP * (0.0 - q) - self.KV * qd
        for qi in self.roll_idx:
            ctrl[qi] = self.KP_ROLL * (0.0 - q[qi]) - self.KV * qd[qi]

        # 2) 叠加平衡控制器输出的**关节力矩**修正量 (N·m)
        if balance and self.controller is not None:
            ctrl = ctrl + self.controller.compute(self)
        self.data.ctrl[:] = np.clip(ctrl, -self.TORQUE_LIMIT, self.TORQUE_LIMIT)

        # 3) 外力扰动（施加在 torso 上）
        self.data.xfrc_applied[self.torso_id, :] = 0.0
        if self._push_left > 0.0:
            self.data.xfrc_applied[self.torso_id, 0:3] = self._push_fx
            self.data.xfrc_applied[self.torso_id, 3:6] = self._push_tx
            self._push_left -= self.dt

        mujoco.mj_step(self.model, self.data)
        self._update_com()
        self._update_lean()

    # ------------------------------------------------------------------
    # 质心运动学（每步更新一次，见 step / reset）
    # ------------------------------------------------------------------
    def _update_com(self):
        com = np.array(self.data.subtree_com[0], dtype=float)
        if self._last_com is None:
            self._last_com = com.copy()
            self._com_vel = np.zeros(3)
            self._com_acc = np.zeros(3)
            return
        vel = (com - self._last_com) / self.dt
        raw_acc = (vel - self._com_vel) / self.dt
        self._last_com = com.copy()
        self._com_vel = vel
        self._com_acc = ACC_FILTER * raw_acc + (1.0 - ACC_FILTER) * self._com_acc

    def _update_lean(self):
        """躯干倾角（lean_x 前倾 / lean_y 左倾）及其速率的有限差分。

        ``lean`` 取躯干 z 轴（身体竖直轴）在世界系中的水平分量：
        前倾（向 +x 倒）时 ``lean_x > 0``，左倾（向 +y 倒）时 ``lean_y > 0``。
        该量即为踝策略 PD 律的输入（见 ``balance/ankle_strategy.py``）。
        """
        lean = self.torso_lean()
        if self._last_lean is None:
            self._last_lean = lean.copy()
            self._lean_rate = np.zeros(2)
            return
        raw = (lean - self._last_lean) / self.dt
        self._lean_rate = (LEAN_RATE_FILTER * self._lean_rate
                           + (1.0 - LEAN_RATE_FILTER) * raw)
        self._last_lean = lean.copy()

    # ------------------------------------------------------------------
    # 状态读取
    # ------------------------------------------------------------------
    def com_position(self):
        """全身质心位置 (m)。"""
        return np.array(self.data.subtree_com[0], dtype=float)

    def com_velocity(self):
        """全身质心线速度 (m/s)。"""
        return self._com_vel.copy()

    def com_acceleration(self):
        """全身质心线加速度 (m/s²)。"""
        return self._com_acc.copy()

    def root_height(self):
        """底盘高度 (m)。"""
        return float(self.data.qpos[self.root_qadr + 2])

    def torso_tilt(self):
        """躯干相对竖直方向的倾角 (rad)。"""
        z_axis = self.data.xmat[self.torso_id].reshape(3, 3)[:, 2]
        return float(np.arccos(np.clip(z_axis[2], -1.0, 1.0)))

    def torso_lean(self):
        """躯干倾斜的水平分量 ``(lean_x, lean_y)``（无量纲，≈ 小角度下的倾角）。

        取躯干 z 轴（身体竖直轴）在世界系中的 x / y 分量：前倾时 ``lean_x > 0``，
        左倾时 ``lean_y > 0``，即「躯干向哪一侧倒」。踝策略 PD 律以此为输入。
        """
        z_axis = self.data.xmat[self.torso_id].reshape(3, 3)[:, 2]
        return np.array([float(z_axis[0]), float(z_axis[1])], dtype=float)

    def zmp(self, accel=None, com=None):
        """零矩点（ZMP）水平位置 (m)。

        采用线性倒立摆（LIPM）近似：``ZMP = CoM_xy − (z_com/g)·CoM_acc_xy``。
        该式在质心高度近似恒定时成立，是本模块的稳定性判据来源。
        """
        com = self.com_position() if com is None else np.asarray(com, dtype=float)
        accel = self.com_acceleration() if accel is None else np.asarray(accel, dtype=float)
        g = -float(self.model.opt.gravity[2])
        return com[:2] - (com[2] / g) * accel[:2]

    def foot_corners(self):
        """两只脚底面的 8 个角点（世界系），用于构建支撑多边形。"""
        pts = []
        for gid in self.foot_geom_ids:
            size = self.model.geom_size[gid]
            center = self.data.geom_xpos[gid]
            rot = self.data.geom_xmat[gid].reshape(3, 3)
            for sx in (-1.0, 1.0):
                for sy in (-1.0, 1.0):
                    pts.append(center + rot @ np.array([sx * size[0], sy * size[1], -size[2]]))
        return np.array(pts)

    def foot_polygon(self):
        """支撑多边形（脚底 8 角点在 xy 平面的凸包，逆时针）。"""
        from .balance import convex_hull
        return convex_hull(self.foot_corners()[:, :2])

    def read_state(self, with_margin=False):
        """一次性读取控制与评价所需的全部状态。

        ``with_margin=True`` 时额外计算支撑多边形与稳定裕度（供评价与话题发布使用）；
        控制回路内默认关闭，以免每步都做一次几何计算影响吞吐。
        """
        com = self.com_position()
        acc = self.com_acceleration()
        lean = self.torso_lean()
        state = {
            "time": float(self.data.time),
            "com": com,
            "com_vel": self.com_velocity(),
            "com_acc": acc,
            "zmp": self.zmp(acc, com),
            "root_height": self.root_height(),
            "tilt": self.torso_tilt(),
            "lean_x": float(lean[0]),
            "lean_y": float(lean[1]),
            "lean_x_rate": float(self._lean_rate[0]),
            "lean_y_rate": float(self._lean_rate[1]),
            "fallen": self.fallen(),
        }
        if with_margin:
            from .balance import stability_margin
            polygon = self.foot_polygon()
            state["support_polygon"] = polygon
            state["stability_margin"] = stability_margin(
                com, state["com_vel"], com[2], polygon)
            state["static_margin"] = stability_margin(
                com, state["com_vel"], com[2], polygon, use_capture_point=False)
        return state

    def fallen(self):
        """是否跌倒：根高过低或躯干倾角过大。"""
        return bool(self.root_height() < self.fall_height or self.torso_tilt() > self.fall_tilt)

    def standing(self):
        """是否仍保持站立。"""
        return not self.fallen()

    def render(self, camera="overview", height=300, width=400):
        """渲染指定相机的图像（RGB ndarray）。"""
        renderer = mujoco.Renderer(self.model, height=height, width=width)
        renderer.update_scene(self.data, camera=camera)
        img = renderer.render()
        del renderer
        return img

    def set_com_reference_marker(self, x=None):
        """把地面参考线 ``marker_ref`` 移到指定 x（默认当前质心投影 x）的正下方。

        参考线是一段无质量、无碰撞的细条，只用于在渲染图像中读出质心的水平位移
        （演示动图用固定侧视相机，位移在画面中可见）。不传入 ``x`` 时应复位之后
        立即调用，把参考线放到初始质心正下方。
        """
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "marker_ref")
        if bid < 0:
            return False
        if x is None:
            x = float(self.com_position()[0])
        self.model.body_pos[bid, 0] = float(x)
        mujoco.mj_kinematics(self.model, self.data)
        return True

    def close(self):
        """释放资源（本实现无外部句柄，保留统一接口）。"""
        pass


def load_params(path=None):
    """读取 config/params.yaml，返回扁平参数字典（standalone 与 ROS2 共用同一文件）。

    兼容三种结构，优先级从高到低：
      1. 按节点分节 ``/balance_sim_node: {ros__parameters: {...}}``（本模块采用）；
      2. 通配节点 ``/**`` 下的 ``ros__parameters``；
      3. 扁平结构（直接就是参数字典）。
    """
    import yaml
    path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "config", "params.yaml")
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, dict):
        return {}

    def section_of(key):
        node = raw.get(key)
        if isinstance(node, dict) and isinstance(node.get("ros__parameters"), dict):
            return node["ros__parameters"]
        return None

    for key in ("/balance_sim_node", "balance_sim_node", "/**"):
        section = section_of(key)
        if section is not None:
            return dict(section)
    return {k: v for k, v in raw.items() if not isinstance(v, dict)}


def open_viewer(sim, log=None):
    """打开 MuJoCo 实时可视化窗口（可直接看到机器人被推与恢复的过程）。

    需要图形环境：本机 / 虚拟机桌面直接可用；SSH 远程需 `ssh -X` 转发 X11，
    或设置 ``export DISPLAY=:0``。无图形环境时返回 ``None`` 并给出提示，
    调用方可平滑退化为无窗口运行（不影响物理仿真与话题发布）。

    参数:
        sim: ``BalanceSim`` 实例（窗口直接绑定其 model/data）
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
        say("  · 本机 / 虚拟机桌面：直接运行即可看到机器人被推与恢复的过程；", warn=True)
        say("  · SSH 远程：用 `ssh -X` 开启 X11 转发，或 `export DISPLAY=:0`；", warn=True)
        say("  · 无图形环境：standalone 用 `--no-viewer`；ROS2 用 `visualize:=false`。", warn=True)
        return None
    # 相机跟踪躯干，便于观察被推后的位移与姿态变化
    try:
        torso = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        if torso >= 0:
            view.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            view.cam.trackbodyid = torso
            view.cam.distance = 3.4
            view.cam.azimuth = 145.0
            view.cam.elevation = -12.0
    except Exception:                                             # noqa: BLE001
        pass
    say("已打开 MuJoCo 可视化窗口（关闭窗口或按 Ctrl-C 退出）")
    return view
