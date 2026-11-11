#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pickplace_core.py —— MuJoCo 物理核心（与 ROS 解耦，可独立无界面测试）

职责（只做物理，不再承担决策）：
  1. 加载 MuJoCo 模型，按关节名字维护 qpos/qvel 地址（不受 freejoint 顺序影响）
  2. 位置伺服目标写入、mj_step 步进、按名字读关节角/方块位姿
  3. 抓取：运动学硬连接（每周期把方块位姿锁定到末端执行器，跨版本稳定；
     XML 保留 weld 约束作参考，但运行时不再依赖约束求解）
  4. 无界面整周期跑测（run_fsm），用于 CI/回归验证

决策（解析 IK / 三次插值 / 5 状态迁移）已抽到 pickplace_planner.py，
本模块只是把它组合进来，保持旧的 PickPlaceCore API 不变：
ROS 桥用本模块（它本来就要模型），ROS 状态机只用 planner（不加载模型）。
"""
import os
import numpy as np
import mujoco

# 决策层（纯 numpy，无 mujoco 依赖）—— 常量与函数在此转发，保持旧接口
from pickplace_planner import (  # noqa: F401  (对外接口的一部分)
    STATE_RESET, STATE_APPROACH, STATE_DESCEND, STATE_GRASP, STATE_LIFT_PLACE,
    STATE_NAMES, JOINT_NAMES, JOINT_RANGE,
    SHOULDER_H, L1, L2, TOOL, L2P,
    WPT_HOME, WPT_APPROACH, WPT_GRASP, WPT_LIFT, WPT_PLACE_A, WPT_PLACE,
    GRASP_CUBE_OFFSET, GRASP_CUBE_INIT_POS, GRASP_DIST_MAX,
    CTRL_HZ, MOVE_DUR, MOVE_DUR_LIFT_PLACE,
    SETTLE_TIMEOUT, THRESH_EMPTY, THRESH_LOAD, GRASP_HOLD, RELEASE_HOLD,
    clamp_joints, ik, PickPlacePlanner,
)


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


def quat_rot(q, v):
    """四元数 q=(w,x,y,z) 旋转向量 v（MuJoCo 约定 w 在前）"""
    q = np.asarray(q, dtype=float)
    v = np.asarray(v, dtype=float)
    qxyz = q[1:]
    w = q[0]
    t = 2.0 * np.cross(qxyz, v)
    return v + w * t + np.cross(qxyz, t)


class PickPlaceCore:
    def __init__(self, xml_path=None, verbose=True):
        if xml_path is None:
            xml_path = find_model_path()
        self.xml_path = os.path.abspath(xml_path)
        self.verbose = verbose

        # ---------- 决策器（纯 numpy，不读写 data） ----------
        self.planner = PickPlacePlanner(verbose=verbose)

        # ---------- 模型 ----------
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)

        # ---- 关节索引（按名字，防 qpos 顺序变化） ----
        self.joint_names = list(JOINT_NAMES)
        self.nj = len(self.joint_names)
        self.jnt_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                        for n in self.joint_names]
        self.qpos_adr = [self.model.jnt_qposadr[j] for j in self.jnt_ids]
        self.dof_adr = [self.model.jnt_dofadr[j] for j in self.jnt_ids]

        # ---- 方块 freejoint ----
        self.cube_free_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "cube_free")
        if self.cube_free_id < 0:
            raise RuntimeError("XML 缺少 cube_free（方块必须是 freejoint）")
        self.cube_qpos_adr = self.model.jnt_qposadr[self.cube_free_id]   # 7: pos(3)+quat(4)
        self.cube_dof_adr = self.model.jnt_dofadr[self.cube_free_id]     # 6

        # ---- 物体/约束 id ----
        self.weld_eq = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "grasp_weld")
        self.ee_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ee_link")
        self.cube_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self.ee_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        if self.ee_site < 0:
            raise RuntimeError("XML 缺少 ee_site")

        # ---- 物理侧运行状态 ----
        self.target_q = np.zeros(self.nj)
        self._pending_weld = None        # None=无请求, True/False=请求
        self.attached = False            # 方块是否被“抓取”（运动学硬连接）

        # 保证 weld 初始关闭（XML 已 active="false"，这里再防御一次）
        if self.weld_eq >= 0:
            self.data.eq_active[self.weld_eq] = 0

        # ============ 抓取实现说明（重要） ============
        # 实测 mujoco 3.17.0 中 <equality><weld> 约束的雅可比行为异常
        # （efc_J 稀疏行全空、激活瞬间产生错误冲击，方块被莫名推开），
        # 因此本项目抓取采用“运动学硬连接”：
        #   抓取后每个控制周期直接把方块位姿锁定到末端执行器参考系，
        #   并清零方块速度 —— 100% 确定、跨 mujoco 版本均可用、无 NaN 风险。
        # XML 中仍保留 grasp_weld（active=false）供旧版本/教学参考，
        # 但本代码永远不会激活它。

    # ================= 决策层代理（保持旧 API） =================
    @property
    def state(self):
        return self.planner.state

    @state.setter
    def state(self, v):
        self.planner.state = v

    @property
    def has_cube(self):
        return self.planner.has_cube

    @has_cube.setter
    def has_cube(self, v):
        self.planner.has_cube = bool(v)

    @property
    def transitions(self):
        return self.planner.transitions

    @property
    def phase(self):
        return self.planner.phase

    @property
    def gripper_cmd(self):
        return self.planner.gripper_cmd

    @gripper_cmd.setter
    def gripper_cmd(self, v):
        self.planner.gripper_cmd = float(v)

    def _enter_state(self, s):
        self.planner._enter_state(s)

    def decide(self, q_now, has_cube, dt):
        """FSM 决策（转交 planner；本函数不修改仿真数据）"""
        return self.planner.decide(q_now, has_cube, dt)

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
        q[5] = self.planner.gripper_cmd
        self.target_q = q.copy()

    def apply_target(self):
        self.data.ctrl[:] = self.target_q

    # ================= 运动学（模型侧校验用） =================
    def fk(self, q):
        """前向运动学：给定关节角，返回 ee_site 世界坐标（会写 data.qpos）"""
        for a, v in zip(self.qpos_adr, q):
            self.data.qpos[a] = v
        mujoco.mj_forward(self.model, self.data)
        return self.ee_pos()

    def ik(self, xyz):
        """解析逆运动学：转交 planner（纯计算，不碰 data.qpos）"""
        return ik(xyz)

    def verify_ik(self, xyz, tol=0.02):
        """用 MuJoCo 自身 FK 独立校验 IK 解（供测试与日志），结束后恢复 qpos"""
        q = self.ik(xyz)
        saved = self.data.qpos.copy()
        try:
            p = self.fk(q)
        finally:
            self.data.qpos[:] = saved
        return np.linalg.norm(p - np.array(xyz, dtype=float)), q

    # ================= 抓取 =================
    def grasp(self, on, grab_dist=0.09):
        """请求开启/关闭抓取（实际处理在 apply_weld_request，主循环内执行）"""
        self._pending_weld = bool(on)

    def _attach_cube(self):
        """把方块锁定到末端执行器：先预对齐（瞬移+清速），再标记 attached。"""
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
            ee = self.ee_pos()                    # 用 IK 参考点判定距离
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

    # ================= 完整循环（无界面测试 / 独立运行用） =================
    def run_fsm(self, max_cycles=2, max_steps=200000, log_each=True):
        """
        无 ROS 直接跑完整 FSM + 物理仿真（无 viewer，纯计算）。
        返回 dict：cycles / nan_count / ee_errors / cube_x_range / transitions ...
        """
        # 关节归零，方块归位，保证每次跑测起点一致
        for a in self.qpos_adr:
            self.data.qpos[a] = 0.0
        self.data.qvel[:] = 0.0
        self.reset_cube()
        mujoco.mj_forward(self.model, self.data)

        cycles = 0
        nan_count = 0
        ee_errors = []
        cube_x_history = []

        # 先验证 IK（用模型 FK 独立核对解析解）
        for name, wpt in [("HOME", WPT_HOME), ("APPROACH", WPT_APPROACH),
                          ("GRASP", WPT_GRASP), ("LIFT", WPT_LIFT),
                          ("PLACE_A", WPT_PLACE_A), ("PLACE", WPT_PLACE)]:
            err, q = self.verify_ik(wpt)
            ee_errors.append((name, err, q))
            if self.verbose:
                print(f"[IK] {name:9s} err={err*1000:.3f}mm  q={np.round(q, 3)}")

        self.planner.start()
        prev_state = self.state
        cube_start_x = None
        for _ in range(max_steps):
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
                    elif c[0] == 'reset':
                        self.reset_cube()
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


if __name__ == "__main__":
    core = PickPlaceCore()
    core.run_fsm(max_cycles=2)
