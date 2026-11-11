#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pickplace_planner.py —— 纯决策模块：解析 IK + 三次多项式轨迹 + 5 状态机

只依赖 numpy，**不导入 mujoco、不加载任何模型文件**。
因此 ROS 状态机节点（main_pick_fsm.py）可以只拿它做运动学/轨迹决策，
不必为了算 IK 去加载整份 XML 模型（避免与 bridge 的物理状态不同步，也省内存）。

职责边界：
  - 负责：笛卡尔路点 -> 关节角（解析 IK）、状态迁移、轨迹点生成、到位判据
  - 不负责：模型加载、mj_step、抓取锁定、方块位姿计算（这些在 pickplace_core.py）

调用方：
  - main_pick_fsm.py  -> 只用 PickPlacePlanner（轻量，无 mujoco）
  - pickplace_core.py -> 组合 PickPlacePlanner + MuJoCo 物理
"""
import numpy as np

from cubic_interpolator import CubicInterpolator

# ---------------- 状态常量 ----------------
STATE_RESET = 0        # 复位
STATE_APPROACH = 1     # 靠近（方块上方约 10cm）
STATE_DESCEND = 2      # 下降（到抓取位姿）
STATE_GRASP = 3        # 抓取
STATE_LIFT_PLACE = 4   # 抬升 + 搬运 + 放置 + 释放

STATE_NAMES = ["RESET", "APPROACH", "DESCEND", "GRASP", "LIFT_PLACE"]

# ---------------- 机械臂几何常量（必须与 models/ur5e.xml 一致） ----------------
SHOULDER_H = 0.40      # 肩关节高度
L1 = 0.40              # 上臂长度
L2 = 0.35              # 前臂长度
TOOL = 0.08            # 腕关节到末端执行器参考点(ee_site)距离
L2P = L2 + TOOL        # 前臂+末端等效长度

# 关节顺序：协议顺序，bridge 发布 /joint_states 与 FSM 读取都按这个顺序
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "gripper_joint",
]

# 关节限位（rad），与 XML 一致
JOINT_RANGE = [
    (-3.14, 3.14),   # shoulder_pan
    (-1.60, 1.60),   # shoulder_lift
    (-2.80, 2.80),   # elbow
    (-1.20, 1.20),   # wrist_1
    (-3.14, 3.14),   # wrist_2
    (0.00, 1.20),    # gripper
]

# ---------------- 笛卡尔路点（末端 ee_site 的世界坐标） ----------------
WPT_HOME      = (0.15, 0.0, 0.55)   # 复位
WPT_APPROACH  = (0.40, 0.0, 0.20)   # 方块正上方约 10cm
WPT_GRASP     = (0.40, 0.0, 0.12)   # 抓取位（方块顶面上方 3cm）
WPT_LIFT      = (0.30, 0.0, 0.45)   # 抬升
WPT_PLACE_A   = (-0.30, 0.0, 0.45)  # 搬运到放置点上方
WPT_PLACE     = (-0.40, 0.0, 0.12)  # 放置位

GRASP_CUBE_OFFSET = 0.06            # 方块中心沿末端局部 z 偏移（抓取姿态）
GRASP_CUBE_INIT_POS = (0.4, 0.0, 0.05)  # 方块初始/复位位置（与 XML body pos 一致）
GRASP_DIST_MAX = 0.10               # 允许抓取的 ee_site-方块中心 最大距离(m)

CTRL_HZ = 50.0
MOVE_DUR = {
    STATE_RESET: 3.0,
    STATE_APPROACH: 2.0,
    STATE_DESCEND: 2.0,
}
MOVE_DUR_LIFT_PLACE = [2.0, 3.0, 2.0]   # 抬升 / 搬运 / 放置下降

SETTLE_TIMEOUT = 4.0
THRESH_EMPTY = 0.06
THRESH_LOAD = 0.10
GRASP_HOLD = 0.8
RELEASE_HOLD = 0.6


def clamp_joints(q):
    """按 JOINT_RANGE 逐关节裁剪（含夹爪通道）"""
    q = np.asarray(q, dtype=float).copy()
    for i, (lo, hi) in enumerate(JOINT_RANGE):
        q[i] = np.clip(q[i], lo, hi)
    return q


def fk(xyz_free_q):
    """解析前向运动学（不加载模型即可算末端 ee_site 位置）。

    本模型连杆方向为 (sinθ, cosθ)，末端：
        r = L1·sin(θ1) + L2P·sin(θ1+θ2)
        z = SHOULDER_H + L1·cos(θ1) + L2P·cos(θ1+θ2)
        x = r·cos(pan),  y = r·sin(pan)
    与 models/ur5e.xml 的几何一致，用于 IK 分支择优（无需 mj_forward）。
    """
    q = np.asarray(xyz_free_q, dtype=float)
    pan, t1, t2 = float(q[0]), float(q[1]), float(q[2])
    r = L1 * np.sin(t1) + L2P * np.sin(t1 + t2)
    z = SHOULDER_H + L1 * np.cos(t1) + L2P * np.cos(t1 + t2)
    return np.array([r * np.cos(pan), r * np.sin(pan), z])


def ik(xyz, tol_note=None):
    """
    解析逆运动学（闭式两连杆解，双分支择优）：
        pan = atan2(y, x)
        (lift=θ1, elbow=θ2) 使 ee_site 到达目标平面坐标 (r, z)
    本函数是纯计算，不读写任何仿真状态（无 qpos 污染问题）。
    """
    px, py, pz = float(xyz[0]), float(xyz[1]), float(xyz[2])
    pan = np.arctan2(py, px)
    r = np.hypot(px, py)
    if r < 1e-9:
        pan = 0.0
        r = 1e-6

    rx, rz = r, pz - SHOULDER_H
    d = np.hypot(rx, rz)
    dmax = L1 + L2P
    dmin = abs(L1 - L2P)
    if d > dmax:                      # 超工作空间：缩到可达边界
        rx, rz = rx * dmax / d, rz * dmax / d
        d = dmax
    elif d < dmin:
        rx, rz = rx * dmin / d, rz * dmin / d
        d = dmin

    c2 = (d * d - L1 * L1 - L2P * L2P) / (2.0 * L1 * L2P)
    c2 = float(np.clip(c2, -1.0, 1.0))
    base = np.arctan2(rz, rx)         # 目标相对肩部的方向（标准角）
    target = np.array([xyz[0], xyz[1], xyz[2]], dtype=float)
    best = None
    for s in (+1.0, -1.0):
        b2 = s * np.arccos(c2)
        b1 = base - np.arctan2(L2P * np.sin(b2), L1 + L2P * np.cos(b2))
        t1 = np.pi / 2.0 - b1         # 本模型 lift
        t2 = -b2                      # 本模型 elbow（标准角 β2 = -θ2）
        if not (JOINT_RANGE[1][0] <= t1 <= JOINT_RANGE[1][1]):
            continue
        if not (JOINT_RANGE[2][0] <= t2 <= JOINT_RANGE[2][1]):
            continue
        q = np.array([pan, t1, t2, 0.0, 0.0, 0.0])
        err = np.linalg.norm(fk(q) - target)
        # 优先误差小，其次折角温和
        score = (err, max(abs(t1), abs(t2)))
        if best is None or score < best[0]:
            best = (score, q, err)
    if best is None:
        # 双分支都不可行：退化为几何最近点 + 限位截断
        t1 = np.clip(np.arctan2(rx, rz) * 1.2, *JOINT_RANGE[1])
        t2 = np.clip(0.4, *JOINT_RANGE[2])
        return clamp_joints(np.array([pan, t1, t2, 0.0, 0.0, 0.0]))
    return clamp_joints(best[1])


class PickPlacePlanner:
    """
    5 状态决策器（纯 numpy）：

      RESET      : 回到 HOME + 通知方块复位（('reset', True)）
      APPROACH   : 三次多项式轨迹到方块正上方
      DESCEND    : 下降到抓取位姿
      GRASP      : 夹爪闭合 + 保持（('grasp', True)）
      LIFT_PLACE : 抬升 -> 搬运 -> 下降放置 -> 释放（('grasp', False)）-> 回 RESET

    decide(q_now, has_cube, dt) 返回 (next_state, next_has_cube, cmd)，
    cmd 为 None / 单个 ('target'|'grasp'|'reset', 参数) / 其列表。
    本类不执行任何指令，执行方（bridge / 无界面 run_fsm）按 cmd 自行落地。
    """

    CTRL_HZ = CTRL_HZ
    JOINT_NAMES = JOINT_NAMES

    def __init__(self, verbose=False):
        self.verbose = bool(verbose)
        self.nj = len(JOINT_NAMES)

        self.state = STATE_RESET
        self.has_cube = False
        self.phase = 0
        self.sub = 0
        self.phase_t = 0.0
        self.traj = None
        self.traj_i = 0
        self.move_target = None
        self.gripper_cmd = 0.0
        self.transitions = []

    # ---------------- 内部工具 ----------------
    def _reset_phase(self):
        self.phase = 0
        self.phase_t = 0.0
        self.traj = None
        self.traj_i = 0
        self.move_target = None

    def _enter_state(self, s):
        self.state = s
        self._reset_phase()
        self.sub = 0
        self.transitions.append(s)
        if self.verbose:
            print(f"[FSM] -> {STATE_NAMES[s]}")

    @staticmethod
    def _reached(q_now, target, th):
        return np.max(np.abs(np.asarray(q_now) - np.asarray(target))) < th

    def _move(self, q_now, target_xyz, dur, load, dt):
        """状态内一段移动：一次生成轨迹，随后逐点下发 + 到位等待。"""
        if self.phase == 0:
            target_q = ik(target_xyz)
            target_q[5] = self.gripper_cmd   # 到达判据与轨迹的夹爪通道一致
            self.move_target = target_q
            self.traj = np.array(list(CubicInterpolator(q_now, target_q, dur, 1.0 / CTRL_HZ)))
            self.traj[:, 5] = self.gripper_cmd
            self.traj_i = 0
            self.phase = 1
        if self.phase == 1:
            if self.traj_i < len(self.traj):
                q = self.traj[self.traj_i]
                self.traj_i += 1
                return False, ('target', q)
            self.phase = 2
            self.phase_t = 0.0
        if self.phase == 2:
            th = THRESH_LOAD if load else THRESH_EMPTY
            if self._reached(q_now, self.move_target, th) or self.phase_t > SETTLE_TIMEOUT:
                if self.phase_t > SETTLE_TIMEOUT and self.verbose:
                    print("[FSM] settle timeout -> force advance")
                self.phase = 3
                return True, None
            return False, ('target', self.move_target)
        return True, None

    # ---------------- 决策 ----------------
    def decide(self, q_now, has_cube, dt):
        """FSM 单步决策，返回 (next_state, next_has_cube, cmd)"""
        self.phase_t += dt
        s = self.state
        cmd = None
        ns, nhc = s, has_cube

        if s == STATE_RESET:
            # 必须在 _move 之前判定：_move 会把 phase 推进到 1，
            # 之后再判 phase==0 永远不成立，方块复位请求会丢（第 2 周期抓取失败）
            was_phase0 = (self.phase == 0)
            done, cmd = self._move(q_now, WPT_HOME, MOVE_DUR[STATE_RESET], False, dt)
            cmds = []
            if was_phase0:
                cmds.append(('reset', True))   # 通知执行方把方块放回抓取点
            if cmd is not None:
                cmds.append(cmd)
            if done:
                ns = STATE_APPROACH
                self._enter_state(ns)
                return ns, False, None
            return ns, nhc, cmds if cmds else None

        elif s == STATE_APPROACH:
            done, cmd = self._move(q_now, WPT_APPROACH, MOVE_DUR[STATE_APPROACH], False, dt)
            if done:
                ns = STATE_DESCEND
                self._enter_state(ns)
                return ns, False, None

        elif s == STATE_DESCEND:
            done, cmd = self._move(q_now, WPT_GRASP, MOVE_DUR[STATE_DESCEND], False, dt)
            if done:
                ns = STATE_GRASP
                self._enter_state(ns)
                return ns, False, None

        elif s == STATE_GRASP:
            if self.phase == 0:
                self.phase = 1
                self.phase_t = 0.0
                self.gripper_cmd = 1.0
                return s, True, ('grasp', True)
            if self.phase == 1:
                if self.phase_t > GRASP_HOLD:
                    self.phase = 2
                    self.phase_t = 0.0
                return s, True, None
            if self.phase == 2:
                if self.phase_t > 0.3:
                    ns = STATE_LIFT_PLACE
                    self._enter_state(ns)
                    return ns, True, None
                return s, True, None

        elif s == STATE_LIFT_PLACE:
            if self.sub == 0:
                done, cmd = self._move(q_now, WPT_LIFT, MOVE_DUR_LIFT_PLACE[0], True, dt)
                if done:
                    self.sub = 1
                    self._reset_phase()
            elif self.sub == 1:
                done, cmd = self._move(q_now, WPT_PLACE_A, MOVE_DUR_LIFT_PLACE[1], True, dt)
                if done:
                    self.sub = 2
                    self._reset_phase()
            elif self.sub == 2:
                done, cmd = self._move(q_now, WPT_PLACE, MOVE_DUR_LIFT_PLACE[2], True, dt)
                if done:
                    self.sub = 3
                    self._reset_phase()
            elif self.sub == 3:   # 释放
                if self.phase == 0:
                    self.phase = 1
                    self.phase_t = 0.0
                    self.gripper_cmd = 0.0
                    return s, True, ('grasp', False)
                if self.phase == 1:
                    if self.phase_t > RELEASE_HOLD:
                        ns = STATE_RESET
                        self._enter_state(ns)
                        return ns, False, None
                    return s, True, None

        return ns, nhc, cmd

    def start(self):
        """把状态机置为初始 RESET（记录首个 transition）"""
        self.state = STATE_RESET
        self._reset_phase()
        self.sub = 0
        self.has_cube = False
        self.gripper_cmd = 0.0
        self.transitions = []
        self._enter_state(STATE_RESET)


if __name__ == "__main__":
    # 纯运动学自检：六个路点 IK 往返误差（不需要 mujoco）
    planner = PickPlacePlanner(verbose=True)
    for name, wpt in [("HOME", WPT_HOME), ("APPROACH", WPT_APPROACH),
                      ("GRASP", WPT_GRASP), ("LIFT", WPT_LIFT),
                      ("PLACE_A", WPT_PLACE_A), ("PLACE", WPT_PLACE)]:
        q = ik(wpt)
        err = np.linalg.norm(fk(q) - np.array(wpt))
        print(f"[IK] {name:9s} err={err*1000:.3f}mm  q={np.round(q, 3)}")
