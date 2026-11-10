#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""具身人形机器人 —— 物理仿真 + 多传感器感知 + 平衡行走控制。

对应课程任务（1）（2）（3）中"人形机器人 / 肌肉骨骼人"这一对象：
对具备平衡能力的双足人形做物理仿真，用键盘控制其运动，并用传感器数据
（激光测距、深度相机）实现感知，进而对给定轨迹做闭环运动控制。

三种运行模式
------------
``sim``      MuJoCo 物理仿真 + 感知 + 平衡步态，跑完整"走过去"闭环（默认）
``offline``  不依赖 MuJoCo：用纯运动学模型把同一套感知/规划/步态/状态机
             跑一遍并打印指标，用于无图形环境（CI、无 GPU 虚拟机）自证
``teleop``   仅物理仿真 + 键盘控制（W/A/S/D 前后左右、Q/E 转向、空格站立）

命令行::

    python3 scripts/main.py --mode sim --seconds 40
    python3 scripts/main.py --mode offline
    python3 scripts/main.py --mode teleop

入口约定（仓库要求）：``main.py`` 位于模块根目录，``main.sh`` / ``main.bat``
直接调用它；本文件是真正的实现，被根目录入口 import 执行。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)          # .../humanoid_embodied_control
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from humanoid_embodied_control import gait, metrics, perception  # noqa: E402
from humanoid_embodied_control.balance import BalanceController, is_fallen, tilt_angle  # noqa: E402
from humanoid_embodied_control.geometry import clamp, wrap_pi  # noqa: E402
from humanoid_embodied_control.state_machine import State, TaskStateMachine  # noqa: E402

MODEL_PATH = os.path.join(_PKG_ROOT, "models", "humanoid.xml")

DEFAULTS = {
    # --- 任务 ---
    "goal_x": 4.5, "goal_y": 0.0,
    "seconds": 40.0,
    "mode": "sim",
    # --- 步态 ---
    # 可行参数由 MuJoCo 实测扫描确定（25 s、躯干不落地）：
    #   stride=0.20 lift=0.10 period=1.3 fz=1.0 → 前进 2.60 m，躯干 0.944~0.957
    #   stride=0.20 lift=0.15 → 2.3 s 后 tilt 发散摔倒
    #   stride=0.30 lift=0.25 → 1.9 s 后摔倒
    # 踝策略（akp）在本模型上开任何非零值都会发散，默认关
    "stride": 0.20, "lift": 0.10, "period": 1.3, "arm_swing": 0.25,
    "speed_max": 0.20, "lookahead": 0.85,
    # --- 平衡 ---
    "kp_balance": 3200.0, "kd_balance": 640.0,
    "kp_pos": 0.0, "kd_pos": 0.0,
    "z_hold": 0.946, "fz_scale": 1.0,
    # --- 感知（仿真里由 MuJoCo 传感器的射线扫描合成）---
    "n_rays": 96, "angle_span": 2.4, "range_max": 3.0,
    "obstacle_slow": 1.2, "obstacle_stop": 0.65, "avoid_clear": 1.0,
    # --- 输出 ---
    "log_every": 1.0, "json_out": "", "seed": 7,
    "render": False, "telemetry": True,
}


# ======================================================================
# 感知：MuJoCo 里的激光测距与深度相机
# ======================================================================
class SimPerception:
    """用 MuJoCo 的 ray-cast 合成"激光雷达 + 深度相机"两路数据。

    为什么用 ray-cast 而不是给模型加 ``rangefinder`` 传感器：
    一帧需要几十条射线，写成传感器会让 XML 变成几十行重复声明，且朝向固定
    无法跟着躯干转；``mujoco.mj_ray`` 每次调用返回最近命中点的距离，
    语义与 ``sensor_msgs/LaserScan`` 完全一致，也便于在离线单测里构造输入。

    深度相机用同样几条中心射线派生出"前方可通行比例"（:func:`perception.depth_free_ratio`
    的输入等价物），保证 sim 与 offline 两种模式共用同一套感知函数。
    """

    def __init__(self, model, data, n_rays=96, angle_span=2.4, range_max=3.0,
                 robot_geomexclude=None):
        import mujoco  # 延迟导入：offline 模式不需要 MuJoCo

        self._mujoco = mujoco
        self.model = model
        self.data = data
        self.n_rays = int(n_rays)
        self.angle_span = float(angle_span)
        self.range_max = float(range_max)
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        self._geomid = np.zeros(1, dtype=np.int32)

    def scan(self):
        """返回一帧测距列表（长度 ``n_rays``），``range_max`` 表示无回波。"""
        mujoco = self._mujoco
        pos = self.data.xpos[self.body_id]
        # 躯干朝向：取本体系 x 轴在世界系的方向
        rot = self.data.xmat[self.body_id].reshape(3, 3)
        heading = math.atan2(rot[1, 0], rot[0, 0])
        origin = np.array([float(pos[0]), float(pos[1]), float(pos[2]) - 0.20])

        ranges = []
        for i in range(self.n_rays):
            a = heading - 0.5 * self.angle_span + self.angle_span * i / (self.n_rays - 1)
            vec = np.array([math.cos(a), math.sin(a), -0.03])
            dist = mujoco.mj_ray(self.model, self.data, origin, vec, None, 1,
                                 self.body_id, self._geomid)
            geom = int(self._geomid[0])
            # 命中自身躯干（self.body_id 之外的本体几何）不算障碍：取不到
            # "自身几何"的可靠标识时，凡是距离异常小且命中本体 body 的射线一律忽略。
            if dist < 0.0 or geom <= 0:
                dist = self.range_max
            elif self.model.geom_bodyid[geom] == self.body_id:
                dist = self.range_max
            ranges.append(float(min(dist, self.range_max)))
        return ranges

    def lidar_sectors(self):
        return perception.lidar_sectors(self.scan(), self.angle_span, self.range_max, 16)

    def depth_free(self):
        """用中心 9 条射线派生"前方可通行比例"，口径与深度图 ROI 一致。"""
        ranges = self.scan()
        centre = len(ranges) // 2
        rows = [[ranges[centre + k] for k in range(-4, 5)]]
        return perception.depth_free_ratio(rows, self.range_max)


# ======================================================================
# 仿真后端
# ======================================================================
class HumanoidSim:
    """MuJoCo 人形仿真包装：状态读写、传感器读取、控制量下发。"""

    #: 模型里 12 个关节的控制通道起点（前 6 个是平衡广义力）
    CTRL_JOINT_OFFSET = 6

    def __init__(self, model_path=MODEL_PATH):
        import mujoco

        self._mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        self.joint_names = list(gait.JOINT_ORDER)
        self._qadr = [self._joint_qpos_adr(n) for n in self.joint_names]
        self._dadr = [self._joint_dof_adr(n) for n in self.joint_names]

    # ---------- 索引 ----------
    def _joint_qpos_adr(self, name):
        jid = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise KeyError("模型缺少关节 %s" % name)
        return int(self.model.jnt_qposadr[jid])

    def _joint_dof_adr(self, name):
        jid = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_dofadr[jid])

    # ---------- 状态 ----------
    def base_position(self):
        return tuple(float(v) for v in self.data.qpos[0:3])

    def base_velocity(self):
        return tuple(float(v) for v in self.data.qvel[0:3])

    def orientation_quat(self):
        q = self.data.sensor("torso_quat").data
        return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    def angular_velocity(self):
        w = self.data.sensor("torso_gyro").data
        return (float(w[0]), float(w[1]), float(w[2]))

    def yaw(self):
        rot = self.data.xmat[self.torso_id].reshape(3, 3)
        return math.atan2(rot[1, 0], rot[0, 0])

    def joint_angles(self):
        return [float(self.data.qpos[a]) for a in self._qadr]

    def joint_velocities(self):
        return [float(self.data.qvel[a]) for a in self._dadr]

    def foot_contacts(self):
        """左右脚的接触标志（用于报告，不影响控制）。"""
        return (float(abs(self.data.sensor("foot_l_touch").data[0])),
                float(abs(self.data.sensor("foot_r_touch").data[0])))

    # ---------- 控制 ----------
    def apply(self, balance_forces, joint_torques):
        """下发 6 个平衡广义量 + 12 个关节力矩。"""
        for k in range(6):
            self.data.ctrl[k] = float(balance_forces[k])
        off = self.CTRL_JOINT_OFFSET
        for k, tau in enumerate(joint_torques):
            self.data.ctrl[off + k] = float(tau)

    def step(self):
        self._mujoco.mj_step(self.model, self.data)


# ======================================================================
# 闭环控制器（sim / offline 共用）
# ======================================================================
class WalkingController:
    """把感知、规划、步态、平衡粘合在一起的控制回路。

    设计约束：**sim 与 offline 必须走同一份代码**，否则离线自检就失去意义。
    与后端相关的只有三个调用：速度指令、关节目标角、平衡量下发，由后端适配。
    """

    def __init__(self, goal=(4.5, 0.0), stride=0.45, lift=0.55, period=0.9,
                 speed_max=0.45, lookahead=0.85, obstacle_slow=1.2,
                 obstacle_stop=0.65, avoid_clear=1.0,
                 balance=None, start_pose=None, ankle_kp=0.0):
        self.goal = (float(goal[0]), float(goal[1]))
        self.stride = float(stride)
        self.lift = float(lift)
        self.period = float(period)
        self.speed_max = float(speed_max)
        self.lookahead = float(lookahead)
        # 踝策略默认关闭：实测在本模型上开任何非零 kp（即使只作用在支撑腿）
        # 都会在摆动相造成发散，kb=3200 的根力矩环才是主力稳定手段。
        # ankle_kp>0 仅供对比实验使用。
        self.ankle_kp = float(ankle_kp)
        self.balance = balance or BalanceController()
        self.sm = TaskStateMachine(obstacle_slow=obstacle_slow,
                                   obstacle_stop=obstacle_stop,
                                   avoid_clear=avoid_clear)
        self.phase = 0.0
        self.pose = gait.stand_pose() if start_pose is None else dict(start_pose)
        self.last_cmd = 0.0
        self.last_free = 1.0

    # ------------------------------------------------------------------
    def command(self, t, x, y, yaw, front_clear, goal_distance=None):
        """根据本体状态与感知结果算出速度标量指令（m/s，沿当前朝向）。

        返回 ``(speed, state, turn_bias)``：
        speed    前进速度（可为 0）
        state    当前任务状态
        turn_bias 期望转向（非 0 时由调用方叠加到左侧/右侧步幅上）
        """
        if goal_distance is None:
            goal_distance = math.hypot(self.goal[0] - x, self.goal[1] - y)

        fallen = False
        state = self.sm.update(t, goal_distance, front_clear, fallen=fallen)
        self.sm.tick_avoid(0.0)

        if state in (State.REACHED, State.BLOCKED, State.FALLEN):
            self.last_cmd = 0.0
            return 0.0, state, 0.0

        heading_err = wrap_pi(math.atan2(self.goal[1] - y, self.goal[0] - x) - yaw)
        scale = self.sm.speed_scale(front_clear)
        # 航向误差大连带着降低前进速度（先转正再走），避免"边走边偏"越走越远
        align = max(0.15, math.cos(clamp(abs(heading_err), 0.0, math.pi / 2)))
        speed = self.speed_max * scale * align
        # AVOID 期间不再额外减速：步态相位对速度标量敏感，0.2 s 内
        # 从 0.2 降到 0.06 会让相位推进骤降，单腿支撑时间变长 → 摔倒。
        # 绕行动作由 turn 偏置完成，速度保持不变。
        self.last_cmd = speed
        return speed, state, heading_err

    # ------------------------------------------------------------------
    def gait_target(self, dt, speed, turn_bias=0.0, quat=None, gyro=None):
        """推进步态相位，返回 12 维**关节目标角**（弧度）。

        ``quat``/``gyro`` 给定时会叠加踝/髋平衡补偿——仿真与 offline 两条链路
        必须走同一份补偿，否则离线自检就没有意义。
        """
        rate = gait.phase_rate(max(speed, 1e-6), self.stride, self.period)
        if speed <= 1e-3:
            # 速度接近零时相位停在"双脚并拢"附近，避免原地踏步
            target = gait.blend_pose(self.pose, gait.stand_pose(), min(1.0, 6.0 * dt))
        else:
            self.phase += rate * dt
            target = gait.walk_pose(self.phase, self.stride, self.lift,
                                    arm_swing=DEFAULT_ARM_SWING, lean=0.0)
        if turn_bias:
            # turn_bias 缩放得很小：本模型没有偏航舵，转向只能靠"左右腿不同
            # 摆幅"实现，增益过大立刻造成侧向滑移摔倒（实测 0.25 即发散）。
            target["hip_l_pitch"] = clamp(target["hip_l_pitch"] + 0.05 * turn_bias, -1.0, 0.5)
            target["hip_r_pitch"] = clamp(target["hip_r_pitch"] - 0.05 * turn_bias, -1.0, 0.5)
        if quat is not None and gyro is not None:
            self.balance.apply_ankle_strategy(target, quat, gyro,
                                              kp=self.ankle_kp, kd=self.ankle_kp * 0.18)
        self.pose = target
        return gait.to_vector(target)

    # ------------------------------------------------------------------
    def balance_forces(self, quat, gyro, position, velocity, yaw=None):
        """根节点 6 个广义力。给定 ``yaw`` 时开启偏航保持（指向目标）。"""
        if yaw is not None:
            want_yaw = math.atan2(self.goal[1] - position[1], self.goal[0] - position[0])
        else:
            want_yaw = None
        return self.balance.compute_balance(quat, gyro, position, velocity,
                                            yaw_target=want_yaw, yaw=yaw)

    def joint_command(self, target_vec):
        """12 维关节目标角 → 平滑后的下发指令（弧度，直接写 ctrl[6:18]）。"""
        return self.balance.joint_command(target_vec)

    def joint_torques(self, target_vec, angles, velocities):
        """仅用于指标/日志统计的等效伺服力矩（不参与下发）。"""
        return self.balance.compute_joint_torques(target_vec, angles, velocities)


DEFAULT_ARM_SWING = 0.25


# ======================================================================
# 主循环：MuJoCo sim
# ======================================================================
def run_sim(args):
    sim = HumanoidSim()
    perc = SimPerception(sim.model, sim.data, n_rays=args.n_rays,
                         angle_span=args.angle_span, range_max=args.range_max)
    balance = BalanceController(kp_balance=args.kp_balance, kd_balance=args.kd_balance,
                                kp_pos=args.kp_pos, kd_pos=args.kd_pos)
    ctrl = WalkingController(goal=(args.goal_x, args.goal_y), stride=args.stride,
                             lift=args.lift, period=args.period, speed_max=args.speed_max,
                             lookahead=args.lookahead, obstacle_slow=args.obstacle_slow,
                             obstacle_stop=args.obstacle_stop, avoid_clear=args.avoid_clear,
                             balance=balance)

    dt = float(sim.model.opt.timestep)
    # 步态（关节目标角）必须以高频率更新：相位按周期推进，20 Hz 的步态更新
    # 会让"摆动相开始"出现半个周期的跳变，机器人被自己的目标阶跃踢翻。
    # 实测本模型需要 ≥100 Hz 的步态更新，因此 gait_every 与感知/决策解耦。
    gait_every = max(1, int(round(0.01 / dt)))      # 100 Hz 步态更新
    decision_every = max(gait_every, int(round(0.05 / dt)))  # 20 Hz 感知/决策

    samples = []
    last_log = -1e9
    target_vec = gait.to_vector(ctrl.pose)
    speed = 0.0

    total_steps = int(args.seconds / dt)
    for step in range(total_steps):
        t = step * dt
        x, y, z = sim.base_position()
        quat = sim.orientation_quat()
        gyro = sim.angular_velocity()

        if step % gait_every == 0:
            sectors = perc.lidar_sectors() if step % decision_every == 0 else None
            if sectors is not None:
                depth_free = perc.depth_free()
                front = perception.front_clearance(sectors, args.angle_span, args.range_max)
                free = perception.sensor_fusion(perception.sectors_mean(sectors), depth_free)
                goal_dist = math.hypot(ctrl.goal[0] - x, ctrl.goal[1] - y)
                speed, state, turn = ctrl.command(t, x, y, sim.yaw(), front, goal_dist)
            target_vec = ctrl.gait_target(dt * gait_every, speed, turn, quat, gyro)

            if args.telemetry and t - last_log >= args.log_every:
                last_log = t
                print("[t=%5.2f] pos=(%+.2f,%+.2f) z=%.3f %-7s speed=%.2f front=%.2f "
                      "free=%.2f tilt=%5.1f° goal=%.2f" % (
                          t, x, y, z, state.value, speed, front, free,
                          math.degrees(tilt_angle(quat)), goal_dist), flush=True)

            if args.telemetry and t - last_log >= args.log_every and step % decision_every == 0:
                last_log = t
                print("[t=%5.2f] pos=(%+.2f,%+.2f) z=%.3f %-7s speed=%.2f front=%.2f "
                      "free=%.2f tilt=%5.1f° goal=%.2f" % (
                          t, x, y, z, state.value, speed, front, free,
                          math.degrees(tilt_angle(quat)), goal_dist), flush=True)

            if step % decision_every == 0:
                fallen = is_fallen(quat, z)
                if fallen:
                    ctrl.sm.update(t, goal_dist, front, fallen=True)
                samples.append(_sample(t, x, y, sim.yaw(), speed, ctrl, target_vec))
                if state in (State.REACHED, State.BLOCKED, State.FALLEN) or fallen:
                    break

        # 高频平衡回路：根节点 6 个力矩 + 12 个角度指令
        bf = balance.compute_balance(quat, gyro, sim.base_position(), sim.base_velocity(),
                                     yaw_target=math.atan2(ctrl.goal[1] - y, ctrl.goal[0] - x),
                                     yaw=sim.yaw())
        cmd = balance.joint_command(target_vec)
        sim.apply(bf, cmd)
        sim.step()

    report = metrics.summarize(samples)
    report.update({
        "mode": "sim",
        "final_state": ctrl.sm.state.value,
        "transitions": ctrl.sm.transitions,
        "success": ctrl.sm.state is State.REACHED,
        "goal": ctrl.goal,
        "torso_height": round(sim.base_position()[2], 4),
    })
    return report


def _sample(t, x, y, yaw, speed, ctrl, target_vec):
    return {
        "t": t, "x": x, "y": y, "yaw": yaw, "speed": speed,
        "targets": list(target_vec),
        "path_error": abs(math.hypot(ctrl.goal[0] - x, ctrl.goal[1] - y)),
    }


# ======================================================================
# 离线模式：无 MuJoCo 的运动学闭环
# ======================================================================
def run_offline(args):
    """纯运动学回放：同一套感知/规划/步态/状态机，用积分模型代替物理引擎。

    用途是在没有图形环境与 MuJoCo 的机器上自证"控制链路是通的"：
    步态相位 → 名义步速 → 平面运动学积分 → 感知（解析障碍模型）→ 状态机。
    这里**不声称**能验证平衡，平衡只在 ``--mode sim`` 下由 MuJoCo 验证。
    """
    balance = BalanceController(kp_balance=args.kp_balance, kd_balance=args.kd_balance)
    ctrl = WalkingController(goal=(args.goal_x, args.goal_y), stride=args.stride,
                             lift=args.lift, period=args.period, speed_max=args.speed_max,
                             lookahead=args.lookahead, obstacle_slow=args.obstacle_slow,
                             obstacle_stop=args.obstacle_stop, avoid_clear=args.avoid_clear,
                             balance=balance)

    dt = 0.05
    x, y, yaw = 0.0, 0.0, 0.0
    samples = []
    goal_dist = math.hypot(ctrl.goal[0] - x, ctrl.goal[1] - y)
    # 离线没有真实 IMU：用一个理想化的"零倾角"四元数配零角速度，
    # 这样 gait_target 的踝补偿输入与实际倾角口径一致（不会误加补偿）。
    quat = (1.0, 0.0, 0.0, 0.0)
    gyro = (0.0, 0.0, 0.0)

    for step in range(int(args.seconds / dt)):
        t = step * dt
        sectors = _synthetic_sectors(x, y, yaw, args)
        front = perception.front_clearance(sectors, args.angle_span, args.range_max)
        # 深度通道：离线没有真实深度图，用前方距离的等价 ROI 代替，
        # 保证 sim 与 offline 走的是同一个 sensor_fusion 口径。
        depth_free = perception.depth_free_ratio([[max(front, 0.05)] * 5], args.range_max)
        free = perception.sensor_fusion(perception.sectors_mean(sectors), depth_free)
        speed, state, turn = ctrl.command(t, x, y, yaw, front, goal_dist)
        target_vec = ctrl.gait_target(dt, speed, turn, quat, gyro)

        yaw = wrap_pi(yaw + 0.6 * turn * dt)
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
        goal_dist = math.hypot(ctrl.goal[0] - x, ctrl.goal[1] - y)

        if args.telemetry:
            print("[t=%5.2f] pos=(%+.2f,%+.2f) %-7s speed=%.2f front=%.2f free=%.2f goal=%.2f" % (
                t, x, y, state.value, speed, front, free, goal_dist), flush=True)

        samples.append({
            "t": t, "x": x, "y": y, "yaw": yaw, "speed": speed,
            "targets": target_vec,
            "path_error": goal_dist,
            "heading_error": wrap_pi(math.atan2(ctrl.goal[1] - y, ctrl.goal[0] - x) - yaw),
        })

        if state in (State.REACHED, State.BLOCKED, State.FALLEN):
            break

    report = metrics.summarize(samples)
    report.update({
        "mode": "offline",
        "final_state": ctrl.sm.state.value,
        "transitions": ctrl.sm.transitions,
        "success": ctrl.sm.state is State.REACHED,
        "goal": ctrl.goal,
        "note": "离线运动学回放，仅验证控制链路；平衡需 --mode sim",
    })
    return report


def _synthetic_sectors(x, y, yaw, args):
    """解析障碍模型：前方 2.2 m 处有一根半径 0.3 m 的柱子。"""
    obstacles = [(2.2, 0.05, 0.30)]
    ranges = []
    for i in range(args.n_rays):
        a = yaw - 0.5 * args.angle_span + args.angle_span * i / (args.n_rays - 1)
        best = args.range_max
        for ox, oy, rad in obstacles:
            # 射线-圆求交（二维），取最近正根
            dx, dy = math.cos(a), math.sin(a)
            fx, fy = x - ox, y - oy
            b = 2.0 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - rad * rad
            disc = b * b - 4.0 * c
            if disc < 0:
                continue
            sq = math.sqrt(disc)
            for root in ((-b - sq) / 2.0, (-b + sq) / 2.0):
                if root > 0.05 and root < best:
                    best = root
        ranges.append(best)
    return perception.lidar_sectors(ranges, args.angle_span, args.range_max, 16)


# ======================================================================
# 键盘遥操作
# ======================================================================
def run_teleop(args, seconds=None):
    """物理仿真 + 键盘控制（W/S 前后、A/D 左右、Q/E 转向、空格站立、ESC 退出）。"""
    import select
    import termios
    import tty

    sim = HumanoidSim()
    balance = BalanceController(kp_balance=args.kp_balance, kd_balance=args.kd_balance)
    dt = float(sim.model.opt.timestep)
    phase = 0.0
    speed = 0.0
    turn = 0.0

    def read_key(timeout=0.0):
        dr, _, _ = select.select([sys.stdin], [], [], timeout)
        if not dr:
            return ""
        return sys.stdin.read(1)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("W/S 前后、A/D 左右、Q/E 转向、空格站立、ESC 退出", flush=True)
    try:
        tty.setcbreak(fd)
        elapsed = 0.0
        limit = args.seconds if seconds is None else seconds
        while elapsed < limit:
            key = read_key(0.0).lower()
            if key == "\x1b":
                break
            if key == "w":
                speed = min(speed + 0.1, args.speed_max)
            elif key == "s":
                speed = max(speed - 0.1, -args.speed_max * 0.5)
            elif key == " ":
                speed, turn = 0.0, 0.0
            elif key == "q":
                turn = 0.4
            elif key == "e":
                turn = -0.4
            elif key == "a":
                turn = 0.15
            elif key == "d":
                turn = -0.15

            if abs(speed) > 1e-3:
                phase = (phase + gait.phase_rate(abs(speed), args.stride, args.period) * dt) % 1.0
                pose = gait.walk_pose(phase, args.stride, args.lift)
            else:
                pose = gait.stand_pose()
            if turn:
                pose["hip_l_pitch"] = clamp(pose["hip_l_pitch"] + 0.25 * turn, -1.0, 0.5)
                pose["hip_r_pitch"] = clamp(pose["hip_r_pitch"] - 0.25 * turn, -1.0, 0.5)

            quat = sim.orientation_quat()
            gyro = sim.angular_velocity()
            balance.apply_ankle_strategy(pose, quat, gyro, kp=0.9, kd=0.16)

            bf = balance.compute_balance(quat, gyro,
                                         sim.base_position(), sim.base_velocity())
            cmd = balance.joint_command(gait.to_vector(pose))
            sim.apply(bf, cmd)
            sim.step()
            elapsed += dt
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

    x, y, z = sim.base_position()
    return {"mode": "teleop", "final_xy": (round(x, 3), round(y, 3)),
            "z": round(z, 3), "fallen": is_fallen(sim.orientation_quat(), z)}


# ======================================================================
# CLI
# ======================================================================
def build_parser():
    p = argparse.ArgumentParser(description="具身人形：物理仿真 + 感知 + 平衡行走控制")
    p.add_argument("--mode", choices=("sim", "offline", "teleop"), default=DEFAULTS["mode"])
    for key, value in DEFAULTS.items():
        if key == "mode":
            continue
        flag = "--" + key
        if isinstance(value, bool):
            p.add_argument(flag, action="store_true", default=value)
        elif isinstance(value, int) and not isinstance(value, bool):
            p.add_argument(flag, type=int, default=value)
        elif isinstance(value, float):
            p.add_argument(flag, type=float, default=value)
        else:
            p.add_argument(flag, default=value)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.mode == "sim":
        report = run_sim(args)
    elif args.mode == "offline":
        report = run_offline(args)
    else:
        report = run_teleop(args)

    text = json.dumps(report, ensure_ascii=False, indent=2)
    print("\n==== 运行结果 ====")
    print(text)
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            fh.write(text)
        print("已写入 %s" % args.json_out)
    return 0 if report.get("success", report.get("mode") == "teleop") else 1


if __name__ == "__main__":
    sys.exit(main())
