"""
humanoid_human_following 仿真核心
=================================
自建简化双足人形在 MuJoCo 场景中实时检测并跟随一个移动目标（红色球体），
通过「距离保持 + 转向」控制律持续刷新跟随目标，供 ROS2 节点与 standalone 复用。
"""

import numpy as np
import mujoco

# ========== 控制参数 ==========
CONTROL_HZ = 20.0
D_KEEP = 0.8      # 期望跟随距离 (m)
K_V = 0.6         # 距离 → 速度增益
K_YAW = 1.2       # 角度 → 转向增益
V_MAX = 0.35      # 前进限幅 (m/s)
W_MAX = 0.5       # 转向限幅 (rad/s)

MODEL = """
<mujoco model="humanoid_following">
  <option gravity="0 0 -10"/>
  <visual><global offwidth="1280" offheight="1024"/></visual>
  <worldbody>
    <light pos="0 0 6"/>
    <geom name="ground" type="plane" size="10 10 0.1" rgba=".85 .85 .85 1"/>
    <geom name="cyl_ref" type="cylinder" pos="4 -2 0.5" size="0.2 0.5" rgba="1 0 0 .8"/>
    <geom name="box_ref" type="box" pos="-3 3 0.4" size="0.3 0.3 0.4" rgba="0 0 1 .8"/>
    <body name="target" pos="0 0 0.3">
      <freejoint/>
      <geom name="target_sphere" type="sphere" size="0.10" rgba="1 0 0 1" group="1" contype="1"/>
    </body>
    <body name="root" pos="0 0 1.08">
      <freejoint/>
      <geom name="pelvis" type="box" pos="0 0 0" size="0.18 0.12 0.10" rgba=".4 .4 .8 1" contype="0"/>
      <geom name="torso" type="box" pos="0 0 0.22" size="0.14 0.10 0.20" rgba=".5 .5 .9 1" contype="0"/>
      <geom name="head" type="sphere" pos="0 0 0.50" size="0.09" rgba=".9 .9 .9 1" contype="0"/>
      <body name="leg_L" pos="-0.08 0 0">
        <geom name="thigh_L" type="capsule" fromto="0 0 -0.05 0 0 -0.45" size="0.05" contype="0"/>
        <geom name="shin_L" type="capsule" fromto="0 0 -0.45 0 0 -0.95" size="0.04" contype="0"/>
      </body>
      <body name="leg_R" pos="0.08 0 0">
        <geom name="thigh_R" type="capsule" fromto="0 0 -0.05 0 0 -0.45" size="0.05" contype="0"/>
        <geom name="shin_R" type="capsule" fromto="0 0 -0.45 0 0 -0.95" size="0.04" contype="0"/>
      </body>
      <body name="arm_L" pos="-0.20 0 0.22">
        <geom name="upper_L" type="capsule" fromto="0 0 0 0 0 -0.40" size="0.04" contype="0"/>
      </body>
      <body name="arm_R" pos="0.20 0 0.22">
        <geom name="upper_R" type="capsule" fromto="0 0 0 0 0 -0.40" size="0.04" contype="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class FollowController:
    """MuJoCo 人体目标跟随控制器（standalone 与 ROS 节点共用）。"""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_string(MODEL)
        self.data = mujoco.MjData(self.model)
        root_body = self.model.body('root')
        self.root_qadr = self.model.jnt_qposadr[root_body.jntadr[0]]
        self.root_vadr = self.model.jnt_dofadr[root_body.jntadr[0]]
        self.target_body = self.model.body('target')
        self.segments = [((0.0, 0.0), (3.0, 0.0)),
                         ((3.0, 0.0), (3.0, 2.0))]
        self.seg_t = 5.0
        self.pause_t = 2.0
        self.t = 0.0
        self.seg = 0
        self.phase = 'move'
        self.freeze = False   # True 时目标固定在当前位置，不推进轨迹

    def target_pos(self):
        return self.data.xpos[self.target_body.id]

    def update_target(self, dt):
        if self.freeze:
            return
        self.t += dt
        if self.phase == 'pause':
            if self.t >= self.pause_t:
                self.t = 0.0
                self.phase = 'move'
                self.seg = (self.seg + 1) % len(self.segments)
            return
        (x0, y0), (x1, y1) = self.segments[self.seg]
        frac = min(self.t / self.seg_t, 1.0)
        x, y = x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac
        jid = self.model.body_jntadr[self.target_body.id]
        j0 = self.model.jnt_qposadr[jid]
        self.data.qpos[j0:j0+3] = [x, y, 0.3]
        # 归零目标球速度，避免自由关节受重力掉落
        v0 = self.model.jnt_dofadr[jid]
        self.data.qvel[v0:v0+6] = 0.0
        if frac >= 1.0:
            self.t = 0.0
            self.phase = 'pause'

    def follow_step(self, dt):
        self.update_target(dt)
        rx = self.data.qpos[self.root_qadr]
        ry = self.data.qpos[self.root_qadr + 1]
        dx = self.target_pos()[0] - rx
        dy = self.target_pos()[1] - ry
        d = np.hypot(dx, dy) + 1e-6
        psi = np.arctan2(dy, dx)
        q = self.data.qpos[self.root_qadr+3:self.root_qadr+7]
        yaw = np.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
        e_yaw = (psi - yaw + np.pi) % (2*np.pi) - np.pi
        e = d - D_KEEP
        v = np.clip(K_V * e, -V_MAX, V_MAX)
        w = np.clip(K_YAW * e_yaw, -W_MAX, W_MAX)
        # 运动学驱动：直接覆盖根位置与朝向（靠每周期覆盖抵消重力，稳定跟随）
        new_yaw = yaw + w * dt
        self.data.qpos[self.root_qadr] = rx + v * np.cos(new_yaw) * dt
        self.data.qpos[self.root_qadr + 1] = ry + v * np.sin(new_yaw) * dt
        c, s = np.cos(new_yaw / 2), np.sin(new_yaw / 2)
        self.data.qpos[self.root_qadr+3:self.root_qadr+7] = np.array([c, 0.0, 0.0, s])
        # 归零速度，避免自由关节受重力持续加速
        self.data.qvel[self.root_vadr:self.root_vadr+6] = 0.0
        return d, v, w, e_yaw
