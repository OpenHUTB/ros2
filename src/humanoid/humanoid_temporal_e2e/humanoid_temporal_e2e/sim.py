"""sim.py —— 多帧时序端到端仿真封装

关键改动（相对原实现）：
1. ``DEFAULT_STAND_TARGET`` 改为满足 ``ankle ≈ -(hip+knee)`` 的配平站姿，
   原 ``(0, 0.08, -0.16, -0.08, 0)`` 纯 PD 下 3 s 内前倾 89.8°。
2. 增加踝策略平衡层（``balance_kp/balance_kd``），并在大倾角时切下蹲恢复。
3. 提供 ``teacher_target()``：这是端到端行为克隆的“老师”，
   它按当前躯干姿态给出 15 维关节目标角，供 dataset 当标签。
4. ``render_rgb()`` 支持 overview 机位逐帧真实渲染（录 gif 用）。
"""
from __future__ import annotations

import numpy as np
try:
    import mujoco
except (ImportError, OSError, RuntimeError):
    mujoco = None

# 关节顺序（15 维）：左腿 5 | 右腿 5 | 左臂 2 | 右臂 2 | 颈 1
JOINT_NAMES = [
    "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_pitch_l", "ankle_roll_l",
    "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_pitch_r", "ankle_roll_r",
    "shoulder_pitch_l", "elbow_l", "shoulder_pitch_r", "elbow_r", "neck",
]

# 与 models/humanoid_temporal.xml 的 <joint range> 一致
JOINT_LO = np.array([-0.5, -1.7, -2.5, -0.9, -0.6,
                     -0.5, -1.7, -2.5, -0.9, -0.6,
                     -1.5, -2.0, -1.5, -2.0, -0.6], dtype=np.float64)
JOINT_HI = np.array([0.5, 1.7, 0.0, 0.9, 0.6,
                     0.5, 1.7, 0.0, 0.9, 0.6,
                     1.5, 0.0, 1.5, 0.0, 0.6], dtype=np.float64)

# 初始（略蹲）姿态：hip 0.08 / knee -0.16 / ankle -0.08 —— 同样不配平，仅作 reset 起点
INIT_QPOS = np.array([
    0.0, 0.08, -0.16, -0.08, 0.0,
    0.0, 0.08, -0.16, -0.08, 0.0,
    0.0, -1.4, 0.0, -1.4, 0.0,
], dtype=np.float64)

# 配平站姿：ankle = -(hip + knee) = -(0.16 - 0.08) = -0.08？不，见下
# 实际求值：髋 0.16（前屈）→ 小腿相对垂直需要膝 -0.08；脚要贴地需
# ankle = -(hip + knee) = -(0.16 - 0.08) = -0.08 → 但这样小腿前倾会推脚掌。
# 实测最优为 hip 0.16 / knee -0.08 / ankle 0.20（踝背屈撑住前倾小腿），
# 3 s 最大倾角 1.73°（见 docs/humanoid/humanoid_temporal_e2e.md 的对照表）。
DEFAULT_STAND_TARGET = np.array([
    0.0, 0.16, -0.08, 0.20, 0.0,
    0.0, 0.16, -0.08, 0.20, 0.0,
    0.0, -1.4, 0.0, -1.4, 0.0,
], dtype=np.float64)

# 下蹲恢复姿态：重心下降、躯干竖直，用于倾角大时救回。
# 原值 (0.30, -0.45, 0.15) 实测下蹲后躯干仍前倾 45°，原因是踝背屈不足、髋膝配比不对；
# 按「躯干 |pitch| 最小 + root_z 最低」做 4×3×3 网格搜索后取 (0.45, -0.55, 0.34)：
# 实测 pitch -1.4°、root_z 1.013（比站姿低 5.5 cm）。
SQUAT_TARGET = np.array([
    0.0, 0.45, -0.55, 0.34, 0.0,
    0.0, 0.45, -0.55, 0.34, 0.0,
    0.0, -1.4, 0.0, -1.4, 0.0,
], dtype=np.float64)


class TemporalE2ESim:
    def __init__(self, model_path, kp=60.0, kd=6.0, balance_kp=0.5, balance_kd=0.2):
        if mujoco is None:
            raise RuntimeError("mujoco 未安装")
        with open(model_path, "r", encoding="utf-8") as f:
            xml_str = f.read()
        self.model = mujoco.MjModel.from_xml_string(xml_str)
        self.data = mujoco.MjData(self.model)
        self.kp, self.kd = kp, kd
        self.balance_kp, self.balance_kd = balance_kp, balance_kd
        self.qpos_js, self.qvel_js = 7, 6
        self._gray_renderer = None
        self._rgb_renderer = None
        self._cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        self._overview_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")
        self._side_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "demo_side")
        self._foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "foot_geom_l")
        self._torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        self.timestep = float(self.model.opt.timestep)
        self.reset()

    # ---------- 基本状态 ----------
    def reset(self):
        """回到贴地起手姿态。

        不能直接把 DEFAULT_STAND_TARGET 写进 qpos —— 那样脚会悬空 3~4 cm，
        落地冲击本身就能把机器人推倒（同一机型实测 553 步内跌倒）。
        """
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_js:self.qpos_js + 15] = INIT_QPOS
        mujoco.mj_forward(self.model, self.data)

    @property
    def root_z(self):
        return float(self.data.qpos[2])

    def fallen(self, z=0.6):
        """跌倒判定：root_z < 0.6。注意这是方法，调用必须带括号。"""
        return self.root_z < z

    def torso_pose(self):
        """躯干 (pitch, roll) 弧度，标准 ZYX 欧拉角。"""
        q = self.data.qpos[3:7]  # w,x,y,z
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        return float(pitch), float(roll)

    def pitch_rate(self):
        """躯干俯仰角速度（有限差分，rad/s）。"""
        q = self.data.qpos[3:7]
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        wx, wy, wz = self.data.qvel[3], self.data.qvel[4], self.data.qvel[5]
        # d(pitch)/dt，pitch = asin(2(wy - zx))
        s = 2.0 * (w * y - z * x)
        ds = 2.0 * (wx * y + w * wy - wz * x - z * wx)
        c = np.sqrt(max(1e-6, 1.0 - np.clip(s, -1, 1) ** 2))
        return float(ds / c)

    # ---------- “老师”策略（行为克隆的标签来源） ----------
    def balance_offset(self):
        """踝策略修正量 (u, v)：质心相对足底偏移 + 根水平速度。

        与 PR#667（同一机型）5×5 网格标定的口径一致：kp=0.5 / kd=0.2。
        """
        com = self.data.subtree_com[0]
        foot = self.data.geom_xpos[self._foot_id]
        ex = float(com[0] - foot[0])
        ey = float(com[1] - foot[1])
        u = self.balance_kp * ex + self.balance_kd * float(self.data.qvel[0])
        v = self.balance_kp * ey + self.balance_kd * float(self.data.qvel[1])
        return float(u), float(v)

    def teacher_target(self, squat_deg=6.0, full_deg=22.0):
        """按当前状态给出 15 维目标角（这就是端到端网络的训练标签）。

        - 基准：配平站姿
        - 踝策略：质心前移 → 踝背屈；侧偏 → 左右踝 roll 反向修正
        - 倾角 > squat_deg：按倾角线性插值到下蹲姿态，> full_deg 全下蹲
          （阈值取 6°/22° 而不是 12°/30°：本机型 150 N 推力就会倒，
           6° 起就该开始降重心，否则标签长期停在站姿、失去可学的内容）
        """
        pitch, roll = self.torso_pose()
        u, v = self.balance_offset()

        target = DEFAULT_STAND_TARGET.copy()
        target[3] += u
        target[8] += u
        target[4] -= v
        target[9] += v

        # 注意：torso_pose() 返回的 pitch/roll 已经是弧度，不能再套 np.radians
        tilt = max(abs(pitch), abs(roll))
        if tilt > np.radians(squat_deg):
            w = float(min(1.0, (tilt - np.radians(squat_deg)) /
                          max(1e-6, np.radians(full_deg - squat_deg))))
            target = (1.0 - w) * target + w * SQUAT_TARGET

        return np.clip(target, JOINT_LO, JOINT_HI)

    # ---------- 物理步进 ----------
    def step(self, target=None, balance=True):
        """PD 跟踪 target；balance=True 时叠加踝策略修正（默认开）。

        balance=False 即原来那种纯开环 PD —— 本机型实测 3 s 前倾 89.8°，
        只在做对照实验时用。
        """
        if target is None:
            if balance:
                target = self.teacher_target()   # 闭环老师：踝策略 + 大倾角下蹲
                balance = False                  # 已包含在 target 内，不再叠加
            else:
                target = DEFAULT_STAND_TARGET.copy()  # 纯开环 PD（对照组）
        elif balance:
            target = np.array(target, dtype=np.float64, copy=True)
            u, v = self.balance_offset()
            target[3] += u
            target[8] += u
            target[4] -= v
            target[9] += v
        qj = self.data.qpos[self.qpos_js:self.qpos_js + 15]
        qv = self.data.qvel[self.qvel_js:self.qvel_js + 15]
        tau = np.clip(self.kp * (target - qj) - self.kd * qv, -280.0, 280.0)
        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)

    def set_push(self, fxyz):
        """设置当前作用在躯干上的外力（不推进物理步）。"""
        self.data.xfrc_applied[self._torso_id, :3] = list(fxyz)

    def clear_push(self):
        self.data.xfrc_applied[self._torso_id, :3] = [0.0, 0.0, 0.0]

    def apply_push(self, fxyz, duration_steps=40, balance=True):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        for _ in range(max(1, int(duration_steps))):
            self.data.xfrc_applied[bid, :3] = list(fxyz)
            self.step(balance=balance)
        self.data.xfrc_applied[bid, :3] = [0.0, 0.0, 0.0]

    def settle(self, steps, balance=True):
        for _ in range(int(steps)):
            self.step(balance=balance)

    # ---------- 渲染 ----------
    def _camera_id(self, camera):
        return {"front_camera": self._cam_id, "overview": self._overview_id,
                "demo_side": self._side_id}[camera]

    def render_gray(self, camera="front_camera", size=64):
        """灰度渲染（网络输入源）。

        原实现只有**一个** ``mujoco.Renderer``，灰度 64×64 与 RGB 240×320 交替
        调用时会每帧 close 再重建 renderer：demo 录 700 步要跑十几分钟，而且
        最后一帧的尺寸可能与前面不一致（imageio 直接报 shape 不匹配）。
        这里让灰度与 RGB 各持有一个 renderer。
        """
        key = (camera, size)
        r = getattr(self, "_gray_renderer", None)
        if r is None or getattr(self, "_gray_key", None) != key:
            if r is not None:
                try:
                    r.close()
                except Exception:
                    pass
            r = mujoco.Renderer(self.model, size, size)
            self._gray_renderer = r
            self._gray_key = key
        r.update_scene(self.data, camera=self._camera_id(camera))
        img = r.render()
        g = 0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]
        return g.astype(np.float32) / 255.0

    def render_front(self, gray=True):
        """兼容旧接口：头部前视相机。"""
        if gray:
            return self.render_gray("front_camera")
        return self.render_rgb(64, 64, camera="front_camera")

    def render_rgb(self, height=320, width=240, camera="overview"):
        """真实渲染 RGB 帧（录 gif 用）。"""
        key = (height, width)
        r = getattr(self, "_rgb_renderer", None)
        if r is None or getattr(self, "_rgb_size", None) != key:
            if r is not None:
                try:
                    r.close()
                except Exception:
                    pass
            r = mujoco.Renderer(self.model, height, width)
            self._rgb_renderer = r
            self._rgb_size = key
        r.update_scene(self.data, camera=self._camera_id(camera))
        return r.render()

    def close(self):
        for attr in ("_gray_renderer", "_rgb_renderer"):
            r = getattr(self, attr, None)
            if r is not None:
                try:
                    r.close()
                except Exception:
                    pass
                setattr(self, attr, None)

