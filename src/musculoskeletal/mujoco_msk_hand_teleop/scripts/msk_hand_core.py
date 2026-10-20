#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""肌肉骨骼灵巧手核心仿真库（无 ROS 依赖，可独立测试）。

职责：
  1. 加载 MuJoCo 模型（3 个场景 XML，均 include 手本体 hand_model.xml）；
  2. 接收 11 维肌肉激活指令（0~1），步进物理（内部含激活一阶动力学）；
  3. 提供指尖位姿、关节状态、按键深度、球体高度、接触力等查询接口。

节点关系（ROS1 Noetic）：
  msk_hand_keyboard_node / msk_hand_nn_node  --发布-->  /msk_hand/muscle_cmd
  msk_hand_sim_node --调用--> MskHandSim（本文件）
"""
import os
import numpy as np
import mujoco


def _share_dir():
    """返回包根目录：优先 rospkg（ROS 环境），否则回退源码目录。"""
    try:
        import rospkg
        return rospkg.RosPack().get_path("mujoco_msk_hand_teleop")
    except Exception:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


PKG_DIR = _share_dir()
MODEL_DIR = os.path.join(PKG_DIR, "models")

# 场景名 -> XML 文件
SCENES = {
    "free":   os.path.join(MODEL_DIR, "scene_free.xml"),
    "button": os.path.join(MODEL_DIR, "scene_button.xml"),
    "ball":   os.path.join(MODEL_DIR, "scene_ball.xml"),
}

# 11 条肌肉的顺序（与 XML <actuator> 一致，也是消息向量顺序）
MUSCLE_NAMES = [
    "m_index_flex", "m_index_ext",
    "m_middle_flex", "m_middle_ext",
    "m_thumb_flex", "m_thumb_ext", "m_thumb_add",
    "m_wrist_flex", "m_wrist_ext",
    "m_wrist_yaw_l", "m_wrist_yaw_r",
]
N_MUSCLE = len(MUSCLE_NAMES)

JOINT_NAMES = [
    "j_wrist_pitch", "j_wrist_yaw",
    "j_index_mcp", "j_index_pip",
    "j_middle_mcp", "j_middle_pip",
    "j_thumb_abd", "j_thumb_mcp",
]
N_JOINT = len(JOINT_NAMES)

TIP_SITES = ["tip_index", "tip_middle", "tip_thumb"]


class MskHandSim:
    """MuJoCo 灵巧手仿真封装。"""

    def __init__(self, scene="free", model_path=None):
        xml = model_path or SCENES[scene]
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.scene = scene
        # 缓存地址，避免每步查名字
        self._tip_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, s)
                         for s in TIP_SITES]
        self._jnt_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
                         for j in JOINT_NAMES]
        self._target_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE,
                                              "target_marker")
        self._ctrl_view = np.zeros(self.model.nu)

    # ------------------------------------------------------------------ #
    def reset(self, seed=0):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def step(self, activation, n_sub=5):
        """给定激活指令（0~1 向量），步进 n_sub 个物理子步（共 10 ms）。"""
        a = np.clip(np.asarray(activation, dtype=float).ravel(), 0.0, 1.0)
        self._ctrl_view[:] = a
        self.data.ctrl[:] = self._ctrl_view
        for _ in range(n_sub):
            mujoco.mj_step(self.model, self.data)

    def set_target_marker(self, pos):
        """free 场景中把红色目标标记移动到 pos。"""
        if self._target_site >= 0:
            self.model.site_pos[self._target_site] = pos

    # ---------------------------- 查询接口 ----------------------------- #
    @property
    def time(self):
        return self.data.time

    def tip_positions(self):
        return np.array([self.data.site_xpos[i].copy() for i in self._tip_ids])

    def index_tip(self):
        return self.data.site_xpos[self._tip_ids[0]].copy()

    def joint_positions(self):
        return np.array([self.data.qpos[self.model.jnt_qposadr[i]]
                         for i in self._jnt_ids])

    def button_depth(self):
        """按键被压下的深度（米）。未按压时 ~0。"""
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "j_button")
        if jid < 0:
            return 0.0
        adr = self.model.jnt_qposadr[jid]
        return float(max(0.0, self.data.qpos[adr]))

    def ball_height(self):
        """球心相对球托顶面的高度（米）。"""
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ball")
        if bid < 0:
            return 0.0
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "g_pedestal")
        top = 0.0
        if gid >= 0:
            top = float(self.model.geom_pos[gid][2] + self.model.geom_size[gid][1])
        return float(self.data.xpos[bid][2] - top)

    def ball_pos(self):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ball")
        return self.data.xpos[bid].copy() if bid >= 0 else np.zeros(3)

    def fingertip_contact_force(self):
        """所有指尖几何体上的法向接触力之和（N）。"""
        tip_geoms = set()
        for g in ["g_index_tip", "g_middle_tip", "g_thumb_tip"]:
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, g)
            if gid >= 0:
                tip_geoms.add(gid)
        total = np.zeros(6)
        force = np.zeros(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.geom1 in tip_geoms or c.geom2 in tip_geoms:
                mujoco.mj_contactForce(self.model, self.data, i, force)
                total += np.abs(force)
        return float(np.linalg.norm(total[:3]))

    def muscle_activations(self):
        """肌肉当前实际激活度（含一阶动力学延迟）。"""
        return self.data.act.copy() if self.model.na > 0 else np.zeros(N_MUSCLE)


# ------------------------------ 自检 ---------------------------------- #
if __name__ == "__main__":
    sim = MskHandSim("free")
    print("nq=%d nu=%d na=%d nsensor=%d" % (sim.model.nq, sim.model.nu,
                                            sim.model.na, sim.model.nsensor))
    sim.reset()
    print("rest index tip:", np.round(sim.index_tip(), 4))
    # 全屈曲测试：食指+中指+腕屈
    cmd = np.zeros(N_MUSCLE)
    cmd[MUSCLE_NAMES.index("m_index_flex")] = 1.0
    cmd[MUSCLE_NAMES.index("m_middle_flex")] = 1.0
    for _ in range(150):  # 1.5 s
        sim.step(cmd)
    print("full-flex index tip:", np.round(sim.index_tip(), 4))
    print("joint angles:", np.round(sim.joint_positions(), 3))
    # 腕部扫描
    cmd = np.zeros(N_MUSCLE)
    cmd[MUSCLE_NAMES.index("m_wrist_flex")] = 1.0
    for _ in range(150):
        sim.step(cmd)
    print("wrist-flex index tip:", np.round(sim.index_tip(), 4))
    cmd = np.zeros(N_MUSCLE)
    cmd[MUSCLE_NAMES.index("m_wrist_ext")] = 1.0
    for _ in range(150):
        sim.step(cmd)
    print("wrist-ext  index tip:", np.round(sim.index_tip(), 4))
