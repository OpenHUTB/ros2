# -*- coding: utf-8 -*-
"""sim_core.py —— 跌倒保护仿真的 MuJoCo 薄封装（无 ROS 依赖）

这一层只做四件事：
  1. 加载 models/humanoid_fall.xml（15 关节双足，底盘 freejoint 完全自由）；
  2. 把机器人摆到「站姿 + 双脚恰好贴地」的初始状态；
  3. 施加水平推扰（写 data.xfrc_applied，持续若干秒）；
  4. 读状态：躯干倾角/角速度、质心、头高、各部件的接触力。

控制律不在这里（见 protection.py），指标统计也不在这里（见 metrics.py）。
这样 standalone 与 ROS2 节点共用同一套物理与状态定义，不会出现两边口径不一致。

坐标系：+x 机器人正前方，+z 向上。
姿态一律用 mju_quat2Mat 把 body 四元数转成旋转矩阵再算，
不直接用 data.xmat —— 实测 xmat 与 mju_quat2Mat 的结果相差一个转置
（xmat.reshape(3,3).T == mju_quat2Mat 的输出），直接按行主序索引会把
前倾/后仰的符号弄反，这个坑在调试时浪费了很久。
"""

import os

import numpy as np
import mujoco

from . import config

#: 模型文件相对本包的路径
_MODEL_REL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "models", "humanoid_fall.xml")

#: 会与地面发生接触、且需要在指标里区分的部件
CONTACT_PARTS = {
    "head": ("head_geom",),
    "hand": ("hand_geom_l", "hand_geom_r"),
    "forearm": ("fore_geom_l", "fore_geom_r"),
    "knee": ("shin_geom_l", "shin_geom_r"),
    "torso": ("torso_geom",),
    "foot": ("foot_geom_l", "foot_geom_r"),
}

#: 质心速度的差分低通系数
COM_VEL_FILTER = 0.3
#: 倾角速率的差分低通系数（原始差分噪声大，不平滑会把保护动作推向高频抖动）
TILT_RATE_FILTER = 0.6


def default_model_path(model_path=None):
    """模型路径解析：优先显式传入，其次包内相对路径，最后安装目录。

    colcon --symlink-install 时 ``..`` 相对路径仍然指回源码树；
    但如果包被完整 copy 到 install/ 下，models/ 不会跟着走，
    所以再回退到 share/<pkg>/models/。
    """
    if model_path:
        return model_path
    if os.path.exists(_MODEL_REL):
        return os.path.abspath(_MODEL_REL)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("humanoid_fall_protection")
        candidate = os.path.join(share, "models", "humanoid_fall.xml")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    return os.path.abspath(_MODEL_REL)


class FallSim:
    """人形机器人跌倒仿真封装。"""

    def __init__(self, model_path=None):
        path = default_model_path(model_path)
        with open(path, encoding="utf-8") as handle:
            xml = handle.read()
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)

        self.joint_qadr = {}
        self.joint_dadr = {}
        for name in config.JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError("模型缺少关节 {}".format(name))
            self.joint_qadr[name] = int(self.model.jnt_qposadr[jid])
            self.joint_dadr[name] = int(self.model.jnt_dofadr[jid])

        self.actuator_ids = {}
        for name in config.JOINT_NAMES:
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            self.actuator_ids[name] = int(aid)

        self.torso = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        self.head_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "head_geom")
        self.foot_geoms = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, g)
                           for g in ("foot_geom_l", "foot_geom_r")]

        # 关节名 → geom id 列表，供接触力查询
        self.part_geoms = {}
        for part, geoms in CONTACT_PARTS.items():
            self.part_geoms[part] = {int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, g))
                                     for g in geoms}

        self._com_prev = None
        self._com_vel = np.zeros(3)
        self._tilt_prev = 0.0
        self._tilt_rate = 0.0

    # ---------------------------------------------------------------- 状态
    def reset(self):
        """摆到站姿并让脚底恰好贴地。

        模型的初始根高是按直腿（膝关节 0）设计的，站姿微屈膝后脚底会离地
        约 8 mm，直接放开会让机器人先「掉」一下，产生 5° 左右的初始倾角抖动，
        掩盖后面真正关心的响应。这里按实际脚底高度修正一次根高。
        """
        mujoco.mj_resetData(self.model, self.data)
        for name, value in config.STAND_POSE.items():
            self.data.qpos[self.joint_qadr[name]] = value
        mujoco.mj_forward(self.model, self.data)

        lowest = min(float(self.data.geom_xpos[g][2]) for g in self.foot_geoms)
        # 脚掌 box 半高 0.025：把脚底抬到 z=0.025 处，即脚掌正好落在地面上
        self.data.qpos[2] -= (lowest - 0.025)
        mujoco.mj_forward(self.model, self.data)

        self._com_prev = self.com().copy()
        self._com_vel = np.zeros(3)
        self._tilt_prev = self.tilt()
        self._tilt_rate = 0.0

    def com(self):
        """全身质心位置（世界系）。"""
        return self.data.subtree_com[0].copy()

    def com_vel(self):
        """质心速度（差分 + 一阶低通）。

        不用 subtree_linvel / cvel：在 MuJoCo 3.16 上 subtree_linvel[0] 恒为 0，
        cvel 的坐标系也与直觉不符，实测都不可靠。
        """
        return self._com_vel.copy()

    def tilt(self):
        """躯干倾角（rad），前倾为正、后仰为负。

        取 torso 局部 z 轴在世界系的方向：直立时 (0,0,1)，前倾时 x 分量为正。
        """
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, self.data.xquat[self.torso])
        rot = mat.reshape(3, 3)
        up = rot @ np.array([0.0, 0.0, 1.0])
        return float(np.arctan2(up[0], up[2]))

    def tilt_rate(self):
        """躯干绕 y 轴的角速度（rad/s），前倾为正。

        用 MuJoCo 给出的刚体角速度，不用倾角差分：差分在推力施加的那一帧
        会产生 2 rad/s 量级的尖峰（50 kg 的躯干被 300 N 蹬一下，单帧位置变化
        被 dt=2 ms 放大），实测会让判据在倾角还只有 5° 时就误触发。
        改用刚体角速度后，触发点回到「真的开始翻」的时刻。
        """
        return float(self._tilt_rate)

    def head_height(self):
        """头部球心高度（m）。球半径 0.09，球心低于 0.09 即几何穿透地面。"""
        return float(self.data.geom_xpos[self.head_geom][2])

    def head_height_min_gap(self):
        """头部球面到地面的最小间隙（m），负值表示穿透。"""
        return self.head_height() - 0.09

    def base_height(self):
        return float(self.data.qpos[2])

    def joint_pos(self, name):
        return float(self.data.qpos[self.joint_qadr[name]])

    def joint_vel(self, name):
        return float(self.data.qvel[self.joint_dadr[name]])

    def foot_positions(self):
        return [self.data.geom_xpos[g].copy() for g in self.foot_geoms]

    # ---------------------------------------------------------------- 驱动
    def set_torque(self, name, torque, limit=None):
        limit = config.DEFAULTS["torque_limit"] if limit is None else limit
        self.data.ctrl[self.actuator_ids[name]] = float(np.clip(torque, -limit, limit))

    def zero_torque(self):
        self.data.ctrl[:] = 0.0

    def apply_push(self, force, active):
        """水平推扰：force 沿 +x（正值为向前推，人向前扑），active=False 时清零。

        直接写 xfrc_applied（世界系 6 维力/力矩，作用在 torso 的 frame origin），
        与 humanoid_push_recovery 的扰动注入方式一致，便于两个模块的结论互相引用。
        """
        self.data.xfrc_applied[self.torso, 0] = float(force) if active else 0.0

    # ---------------------------------------------------------------- 接触
    def part_contact(self, part):
        """返回某部件当前与地面/其它物体的接触法向力之和（N）。

        mj_contactForce 的输出在**接触坐标系**里，该坐标系的 x 轴沿法线，
        y/z 是切向，所以法向力是 f[0]，不是 f[2]。
        取错分量时，头部整个「压」在地面上的读数会变成只有十几牛的切向力，
        看起来像「头没砸到地」，会把结论带偏。这个坑踩过一次，写在这里备忘。
        """
        geoms = self.part_geoms.get(part)
        if not geoms:
            return 0.0
        total = 0.0
        force = np.zeros(6)
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            if contact.geom1 in geoms or contact.geom2 in geoms:
                mujoco.mj_contactForce(self.model, self.data, i, force)
                total += abs(float(force[0]))
        return total

    def part_touching(self, part, threshold=1.0):
        return self.part_contact(part) > threshold

    # ---------------------------------------------------------------- 步进
    def step(self):
        mujoco.mj_step(self.model, self.data)
        # 每步更新差分状态，供控制律与指标读取
        com = self.com()
        if self._com_prev is not None:
            raw = (com - self._com_prev) / self.dt
            self._com_vel = (1.0 - COM_VEL_FILTER) * self._com_vel + COM_VEL_FILTER * raw
        self._com_prev = com

        # 躯干角速度：torso 相对 root 没有关节，两者角速度相同；freejoint 的
        # qvel[3:6] 就是世界系角速度。用它而不是局部系速度，是因为身体翻过
        # 90° 之后局部轴会跟着翻转，符号会跳变。
        # 实测标定：qvel[4] 与 d(tilt)/dt 同号（前倾为正），直接取用即可，
        # 不要再取负号——取了会变成「倾角在涨、角速度却是负的」，判据永远打不中。
        raw_rate = float(self.data.qvel[4])
        self._tilt_rate = (1.0 - TILT_RATE_FILTER) * self._tilt_rate + TILT_RATE_FILTER * raw_rate
        self._tilt_prev = self.tilt()

    def forward(self):
        mujoco.mj_forward(self.model, self.data)
