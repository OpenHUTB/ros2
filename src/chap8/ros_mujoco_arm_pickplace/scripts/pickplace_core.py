#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pickplace_core.py —— 抓放核心（与 ROS 解耦，可独立无界面测试）

职责：
  1. 加载 MuJoCo 模型，按关节名字维护 qpos/qvel 地址（不受 freejoint 顺序影响）
  2. 解析逆运动学：给定末端 xyz，解出 [pan, lift, elbow, wrist1, wrist2, gripper]
  3. 5 状态有限状态机：复位 -> 靠近 -> 下降 -> 抓取 -> 抬升放置 -> 循环
  4. 抓取：运动学硬连接（每周期把方块位姿锁定到末端执行器，跨版本稳定；
     XML 保留 weld 约束作参考，但运行时不再依赖约束求解）

ROS 桥与 ROS FSM 都复用本模块的逻辑，保证本地测试与实机行为一致。
"""
import os
import numpy as np
import mujoco
from cubic_interpolator import CubicInterpolator

# ---------------- 状态常量 ----------------
STATE_RESET = 0        # 复位
STATE_APPROACH = 1     # 靠近（方块上方约 10cm）
STATE_DESCEND = 2      # 下降（到抓取位姿）
STATE_GRASP = 3        # 抓取（weld 绑定方块）
STATE_LIFT_PLACE = 4   # 抬升 + 搬运 + 放置 + 释放

STATE_NAMES = ["RESET", "APPROACH", "DESCEND", "GRASP", "LIFT_PLACE"]

# ---------------- 机械臂几何常量（必须与 XML 一致） ----------------
SHOULDER_H = 0.40      # 肩关节高度
L1 = 0.40              # 上臂长度
L2 = 0.35              # 前臂长度
TOOL = 0.08            # 腕关节到末端执行器参考点(ee_site)距离
L2P = L2 + TOOL        # 前臂+末端等效长度

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
    q = np.asarray(q, dtype=float).copy()
    for i, (lo, hi) in enumerate(JOINT_RANGE):
        q[i] = np.clip(q[i], lo, hi)
    return q


def find_model_path():
    """按候选顺序定位 ur5e.xml，兼容 源码树/ROS devel 安装 两种运行方式。
    优先级：环境变量 UR5E_XML > 脚本相对路径 > catkin share 路径。"""
    env = os.environ.get("UR5E_XML")
    if env and os.path.exists(env):
        return env
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(script_dir, "..", "models", "ur5e.xml"),
        os.path.join(script_dir, "models", "ur5e.xml"),
        os.path.join(script_dir, "..", "share", "ros_mujoco_arm_pickplace",
                     "models", "ur5e.xml"),
    ]
    for c in candidates:
        if os.path.exists(c):
            return os.path.abspath(c)
    return os.path.abspath(candidates[0])


class PickPlaceCore:
    def __init__(self, xml_path=None, verbose=True):
        if xml_path is None:
            xml_path = find_model_path()
        self.xml_path = os.path.abspath(xml_path)
        self.verbose = verbose
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)

        # ---- 关节索引（按名字，防 qpos 顺序变化） ----
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "gripper_joint",
        ]
        self.nj = len(self.joint_names)
        self.jnt_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                        for n in self.joint_names]
        self.qpos_adr = [self.model.jnt_qposadr[j] for j in self.jnt_ids]
        self.dof_adr = [self.model.jnt_dofadr[j] for j in self.jnt_ids]

        # ---- 方块 freejoint ----
        self.cube_free_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "cube_free")
        self.cube_qpos_adr = self.model.jnt_qposadr[self.cube_free_id]   # 7: pos(3)+quat(4)
        self.cube_dof_adr = self.model.jnt_dofadr[self.cube_free_id]     # 6

        # ---- 物体/约束 id ----
        self.weld_eq = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "grasp_weld")
        self.ee_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ee_link")
        self.cube_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self.ee_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        if self.weld_eq < 0 or self.ee_site < 0:
            raise RuntimeError("XML 缺少 grasp_weld / ee_site")

        # ---- 运行状态 ----
        self.target_q = np.zeros(self.nj)
        self._pending_weld = None        # None=无请求, True/False=请求
        self.gripper_cmd = 0.0
        self.attached = False            # 方块是否被“抓取”（运动学硬连接）

        # ---- FSM 状态 ----
        self.state = STATE_RESET
        self.has_cube = False
        self.phase = 0          # 每个状态内部相位
        self.sub = 0            # LIFT_PLACE 子步
        self.phase_t = 0.0
        self.traj = None
        self.traj_i = 0
        self.move_target = None
        self.state_entered_at = None
        self.transitions = []   # 记录状态迁移历史（供测试/日志）

        # 保证 weld 初始关闭（XML 已 active="false"，这里再防御一次）
        self.data.eq_active[self.weld_eq] = 0

        # ============ 抓取实现说明（重要） ============
        # 实测 mujoco 3.17.0 中 <equality><weld> 约束的雅可比行为异常
        # （efc_J 稀疏行全空、激活瞬间产生错误冲击，方块被莫名推开），
        # 因此本项目抓取采用“运动学硬连接”：
        #   抓取后每个控制周期直接把方块位姿锁定到末端执行器参考系，
        #   并清零方块速度 —— 100% 确定、跨 mujoco 版本均可用、无 NaN 风险。
        # XML 中仍保留 grasp_weld（active=false）供旧版本/教学参考，
        # 但本代码永远不会激活它。

    # ================= 模型查询 =================
    def read_joints(self):
        """按名字读当前关节角 (6,)"""
        return np.array([self.data.qpos[a] for a in self.qpos_adr])

    def read_joint_vel(self):
        return np.array([self.data.qvel[a] for a in self.dof_adr])

    def read_cube_pose(self):
        a = self.cube_qpos_adr
        return self.data.qpos[a:a + 3].copy(), self.data.qpos[a + 3:a + 7].copy()

    def ee_pos(self):
        """ee_site 世界坐标（site 位置要用 site_xpos，不能直接 xpos）"""
        return self.data.site_xpos[self.ee_site].copy()

    def cube_center(self):
        return self.data.xpos[self.cube_body].copy()

    def reset_cube(self, pos=None):
        """把方块复位到抓取点（循环演示用），速度清零。"""
        a = self.cube_qpos_adr
        if pos is None:
            pos = GRASP_CUBE_INIT_POS
        self.data.qpos[a:a + 3] = pos
        self.data.qpos[a + 3:a + 7] = np.array([1.0, 0.0, 0.0, 0.0])
        self.data.qvel[self.cube_dof_adr:self.cube_dof_adr + 6] = 0.0

    def set_target(self, q):
        q = clamp_joints(q)
        q[5] = self.gripper_cmd
        self.target_q = q.copy()

    def apply_target(self):
        self.data.ctrl[:] = self.target_q

    # ================= 运动学 =================
    def fk(self, q):
        """前向运动学：给定关节角，返回 ee_site 世界坐标"""
        for a, v in zip(self.qpos_adr, q):
            self.data.qpos[a] = v
        mujoco.mj_forward(self.model, self.data)
        return self.ee_pos()

    def ik(self, xyz):
        """
        解析逆运动学（闭式两连杆解，带双分支选择）：
          pan = atan2(y, x)
          (lift=θ1, elbow=θ2) 使 ee_site 到达目标平面坐标 (rx, rz)
        末端局部坐标用标准两连杆公式（角度从 +x 轴计量）再映射回本模型约定
        （本模型连杆方向为 (sinθ, cosθ)，即标准角 β = π/2 - θ）。
        注意：本函数不修改仿真状态（内部 FK 校验会暂存/恢复 qpos）。
        """
        saved_qpos = self.data.qpos.copy()   # fk() 会写 qpos，这里先保存
        try:
            return self._ik_impl(xyz)
        finally:
            self.data.qpos[:] = saved_qpos

    def _ik_impl(self, xyz):
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
        if d > dmax:
            rx, rz = rx * dmax / d, rz * dmax / d
            d = dmax
        elif d < dmin:
            rx, rz = rx * dmin / d, rz * dmin / d
            d = dmin

        c2 = (d * d - L1 * L1 - L2P * L2P) / (2.0 * L1 * L2P)
        c2 = float(np.clip(c2, -1.0, 1.0))
        base = np.arctan2(rz, rx)   # 目标相对肩部的方向（标准角）
        best = None
        for s in (+1.0, -1.0):
            b2 = s * np.arccos(c2)
            b1 = base - np.arctan2(L2P * np.sin(b2), L1 + L2P * np.cos(b2))
            t1 = np.pi / 2.0 - b1    # 本模型 lift
            t2 = -b2                 # 本模型 elbow（标准角 β2 = -θ2）
            if not (JOINT_RANGE[1][0] <= t1 <= JOINT_RANGE[1][1]):
                continue
            if not (JOINT_RANGE[2][0] <= t2 <= JOINT_RANGE[2][1]):
                continue
            q = np.array([pan, t1, t2, 0.0, 0.0, 0.0])
            err = np.linalg.norm(self.fk(q) - np.array(xyz, dtype=float))
            # 优先选误差小、其次折角温和的解
            score = (err, max(abs(t1), abs(t2)))
            if best is None or score < best[0]:
                best = (score, q, err)
        if best is None:
            # 双分支都不可行：退化为几何最近点 + 限位截断
            t1 = np.clip(np.arctan2(rx, rz) * 1.2, *JOINT_RANGE[1])
            t2 = np.clip(0.4, *JOINT_RANGE[2])
            q = np.array([pan, t1, t2, 0.0, 0.0, 0.0])
            return clamp_joints(q)
        return clamp_joints(best[1])

    def verify_ik(self, xyz, tol=0.02):
        """检查 IK 解是否满足末端位置误差（供测试与日志）"""
        q = self.ik(xyz)
        p = self.fk(q)
        return np.linalg.norm(p - np.array(xyz)), q

    # ================= 抓取 =================
    def grasp(self, on, grab_dist=0.09):
        """
        请求开启/关闭 weld。开启前做两件事：
          1. 距离校验：ee_site 与方块中心距离 < grab_dist 才允许
          2. 预对齐：把方块瞬移到夹爪位姿(relpose 偏移处)，速度清零，
             保证 weld 激活瞬间零约束违约、不产生冲击
        """
        self._pending_weld = bool(on)

    def _attach_cube(self):
        """把方块锁定到末端执行器：先预对齐（瞬移+清速），再标记 attached。
        之后每个控制周期 step() 会自动把方块位姿同步到末端（见 _sync_cube）。"""
        ee = self.data.xpos[self.ee_body]
        quat = self.data.xquat[self.ee_body].copy()
        off = np.array([0.0, 0.0, GRASP_CUBE_OFFSET])
        target_pos = ee + quat_rot(quat, off)
        a = self.cube_qpos_adr
        self.data.qpos[a:a + 3] = target_pos
        self.data.qpos[a + 3:a + 7] = quat
        self.data.qvel[self.cube_dof_adr:self.cube_dof_adr + 6] = 0.0
        self.attached = True
        if self.verbose:
            print("[GRASP] cube attached (kinematic lock)")

    def _detach_cube(self):
        """释放方块：解除锁定并清零速度，方块随即受重力落回地面。"""
        self.attached = False
        self.data.qvel[self.cube_dof_adr:self.cube_dof_adr + 6] = 0.0
        if self.verbose:
            print("[RELEASE] cube released")

    def _sync_cube(self):
        """运动学硬连接：把方块位姿同步到末端执行器参考系，速度清零。"""
        a = self.cube_qpos_adr
        ee = self.data.xpos[self.ee_body]
        quat = self.data.xquat[self.ee_body].copy()
        off = np.array([0.0, 0.0, GRASP_CUBE_OFFSET])
        self.data.qpos[a:a + 3] = ee + quat_rot(quat, off)
        self.data.qpos[a + 3:a + 7] = quat
        self.data.qvel[self.cube_dof_adr:self.cube_dof_adr + 6] = 0.0

    def apply_weld_request(self):
        if self._pending_weld is None:
            return
        want = self._pending_weld
        self._pending_weld = None
        if want:
            ee = self.data.site_xpos[self.ee_site]  # 用 IK 参考点判定距离
            cube = self.data.xpos[self.cube_body]
            dist = np.linalg.norm(ee - cube)
            if dist < GRASP_DIST_MAX:
                self._attach_cube()
            else:
                if self.verbose:
                    print(f"[GRASP] rejected! ee-cube dist={dist:.3f} > {GRASP_DIST_MAX}")
        else:
            self._detach_cube()

    # ================= 仿真步进 =================
    def step(self, n_substeps=5):
        self.apply_weld_request()
        self.apply_target()
        for _ in range(n_substeps):
            if self.attached:
                self._sync_cube()
            mujoco.mj_step(self.model, self.data)

    # ================= FSM =================
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

    def _reached(self, q_now, target, th):
        return np.max(np.abs(q_now - target)) < th

    def _move(self, q_now, target_xyz, dur, load, dt):
        """
        当前状态内执行一段移动（cubic 轨迹 + 到位等待）。
        返回 (done, cmd)，cmd 为 ('target', q) 或 None。
        """
        if self.phase == 0:
            target_q = self.ik(target_xyz)
            target_q[5] = self.gripper_cmd   # 到达判据与轨迹的夹爪通道一致
            self.move_target = target_q
            self.traj = np.array(list(CubicInterpolator(q_now, target_q, dur, 1.0 / CTRL_HZ)))
            self.traj[:, 5] = self.gripper_cmd   # 夹爪跟随 gripper_cmd
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
                    print(f"[FSM] settle timeout -> force advance")
                self.phase = 3
                return True, None
            return False, ('target', self.move_target)
        return True, None

    def decide(self, q_now, has_cube, dt):
        """FSM 决策：返回 (next_state, next_has_cube, cmd)"""
        self.phase_t += dt
        s = self.state
        cmd = None
        ns, nhc = s, has_cube

        if s == STATE_RESET:
            done, cmd = self._move(q_now, WPT_HOME, MOVE_DUR[STATE_RESET], False, dt)
            cmds = []
            if self.phase == 0:
                # 每个循环开始：方块复位回抓取点（headless 就地执行，
                # ROS 模式下通过 ('reset', True) 指令通知桥端执行）
                self.reset_cube()
                cmds.append(('reset', True))
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

    # ================= 完整循环（无界面测试 / 独立运行用） =================
    def run_fsm(self, max_cycles=2, max_steps=200000, log_each=True):
        """
        无 ROS 直接跑完整 FSM + 物理仿真（无 viewer，纯计算）。
        返回 (cycles, nan_count, ee_errors)
        """
        mujoco.mj_forward(self.model, self.data)
        cycles = 0
        nan_count = 0
        ee_errors = []
        cube_x_history = []
        # 先验证 IK
        for name, wpt in [("HOME", WPT_HOME), ("APPROACH", WPT_APPROACH),
                          ("GRASP", WPT_GRASP), ("LIFT", WPT_LIFT),
                          ("PLACE_A", WPT_PLACE_A), ("PLACE", WPT_PLACE)]:
            err, q = self.verify_ik(wpt)
            ee_errors.append((name, err, q))
            if self.verbose:
                print(f"[IK] {name:9s} err={err*100:.1f}cm  q={np.round(q,3)}")

        self._enter_state(STATE_RESET)
        prev_state = self.state
        cube_start_x = None
        for step_i in range(max_steps):
            q_now = self.read_joints()
            ns, nhc, cmd = self.decide(q_now, self.has_cube, 1.0 / CTRL_HZ)
            if ns != self.state:
                prev_state = self.state
            self.state, self.has_cube = ns, nhc
            if cmd is not None:
                for c in (cmd if isinstance(cmd, list) else [cmd]):
                    if c[0] == 'target':
                        self.set_target(c[1])
                    elif c[0] == 'grasp':
                        self.grasp(c[1])
                    # 'reset' 已在 decide 内就地执行，headless 无需再处理
            self.step(10)   # 10 子步 = 20ms 仿真时间

            if not np.all(np.isfinite(self.data.qacc)):
                nan_count += 1

            if self.state == STATE_GRASP and self.has_cube and cube_start_x is None:
                cube_start_x = self.cube_center()[0]
            cube_x_history.append(self.cube_center()[0])

            if self.state == STATE_RESET and prev_state == STATE_LIFT_PLACE:
                cycles += 1
                if self.verbose:
                    print(f"[FSM] cycle #{cycles} complete")
                if cycles >= max_cycles:
                    break
            prev_state = self.state

        # 释放后稳定检查
        for _ in range(500):
            self.step(10)
            if not np.all(np.isfinite(self.data.qacc)):
                nan_count += 1
        cube_final_z = self.cube_center()[2]

        result = {
            "cycles": cycles,
            "nan_count": nan_count,
            "ee_errors": ee_errors,
            "cube_start_x": cube_start_x,
            "cube_final_z": cube_final_z,
            "cube_x_range": (min(cube_x_history), max(cube_x_history)),
            "transitions": self.transitions,
        }
        if self.verbose:
            print("[TEST RESULT]")
            for k, v in result.items():
                print(f"  {k}: {v}")
        return result


def quat_rot(q, v):
    """四元数 q=(w,x,y,z) 旋转向量 v"""
    q = np.asarray(q, dtype=float)
    v = np.asarray(v, dtype=float)
    qv = np.array([q[0], q[1], q[2], q[3]])
    # 使用标准公式 v' = v + 2*w*cross(qv_xyz, v) + 2*cross(qv_xyz, cross(qv_xyz, v))
    qxyz = qv[1:]
    w = qv[0]
    t = 2.0 * np.cross(qxyz, v)
    return v + w * t + np.cross(qxyz, t)


if __name__ == "__main__":
    core = PickPlaceCore()
    res = core.run_fsm(max_cycles=1)
