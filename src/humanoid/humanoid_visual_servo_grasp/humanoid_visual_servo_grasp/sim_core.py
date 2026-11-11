"""
humanoid_visual_servo_grasp 仿真核心
====================================
自建带单臂的简化人形，用前视相机感知目标物（红色立方体）位置，
通过逆运动学（IK）反解肩/肘关节角，驱动手臂伸向目标并合拢两指夹具完成抓取。

说明：
- 人形底座固定（不参与物理），仅手臂 3 个铰链关节 + 2 个手指关节活动；
- 目标感知为理想化视觉伺服（直接读目标物世界坐标）；
- 运动学驱动（直接覆盖关节 qpos，靠每周期覆盖抵消重力），不含真实动力学控制。
"""

import numpy as np
import mujoco

# ========== 控制参数 ==========
CONTROL_HZ = 50.0
K_IK = 0.15        # IK 关节速度增益（低增益使手臂渐进伸向目标，便于演示与观察）
REACH_TOL = 0.06   # 末端到达判定距离 (m)
GRASP_ANG = 0.05   # 夹具合拢角度 (rad)

MODEL = """
<mujoco model="humanoid_visual_servo_grasp">
  <option gravity="0 0 -10"/>
  <visual><global offwidth="1280" offheight="1024"/></visual>
  <worldbody>
    <light pos="0 3 6"/>
    <geom name="ground" type="plane" size="6 6 0.1" rgba=".85 .85 .85 1"/>
    <geom name="table" type="box" pos="0.28 0 0.40" size="0.4 0.4 0.05" rgba=".7 .7 .7 1" contype="1"/>

    <!-- 目标物：桌面上的红色立方体（静态，模拟被放置） -->
    <body name="object" pos="0.22 0 0.49">
      <geom name="obj_geom" type="box" size="0.03 0.03 0.03" rgba="1 0 0 1" group="1" contype="1"/>
    </body>

    <!-- 相机：前视，观察目标物 -->
    <camera name="camera" pos="-0.3 0 1.0" xyaxes="1 0 0 0 0 1" fovy="60"/>

    <!-- 人形：固定底座（不参与物理），单臂 + 两指夹具 -->
    <body name="root" pos="0 0 0">
      <geom name="pelvis" type="box" pos="-0.12 0 0.60" size="0.15 0.12 0.20" rgba=".4 .4 .8 1" contype="0"/>
      <geom name="torso" type="box" pos="-0.12 0 0.85" size="0.13 0.10 0.22" rgba=".5 .5 .9 1" contype="0"/>
      <geom name="head" type="sphere" pos="-0.12 0 1.15" size="0.09" rgba=".9 .9 .9 1" contype="0"/>
      <geom name="leg_L" type="capsule" fromto="-0.17 0 0.55 -0.17 0 0.15" size="0.05" contype="0"/>
      <geom name="leg_R" type="capsule" fromto="-0.07 0 0.55 -0.07 0 0.15" size="0.05" contype="0"/>

      <!-- 单臂：肩 roll(z) + 肩 pitch(y) + 肘 pitch(y)，末端夹具 -->
      <body name="shoulder" pos="-0.12 0 0.95">
        <geom name="shoulder_ball" type="sphere" size="0.045" rgba=".4 .4 .8 1" contype="0"/>
        <joint name="shoulder_roll" type="hinge" axis="0 0 1" pos="-0.12 0 0.95"/>
        <body name="upper_arm" pos="0 0 0">
          <geom name="upper" type="capsule" fromto="0 0 0 0.35 0 0" size="0.03" contype="0"/>
          <joint name="shoulder_pitch" type="hinge" axis="0 1 0" pos="0 0 0"/>
          <body name="forearm" pos="0.35 0 0">
            <geom name="fore" type="capsule" fromto="0 0 0 0.33 0 0" size="0.025" contype="0"/>
            <joint name="elbow" type="hinge" axis="0 1 0" pos="0 0 0"/>
            <body name="end_effector" pos="0.33 0 0">
              <geom name="tip" type="sphere" size="0.02" rgba="1 .8 0 1" contype="1"/>
              <body name="finger_L" pos="0.02 0.015 0">
                <geom name="fing_L" type="box" size="0.045 0.009 0.009" rgba=".3 .6 .3 1" contype="1"/>
                <joint name="finger_L_j" type="hinge" axis="0 0 1" pos="0.02 0 0"/>
              </body>
              <body name="finger_R" pos="0.02 -0.015 0">
                <geom name="fing_R" type="box" size="0.045 0.009 0.009" rgba=".3 .6 .3 1" contype="1"/>
                <joint name="finger_R_j" type="hinge" axis="0 0 -1" pos="0.02 0 0"/>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

J_SHOULDER_ROLL = 'shoulder_roll'
J_SHOULDER_PITCH = 'shoulder_pitch'
J_ELBOW = 'elbow'
J_FINGER_L = 'finger_L_j'
J_FINGER_R = 'finger_R_j'

# 肩基座在世界系的位置（root 固定于原点，肩在 (-0.12,0,0.95)）
SHOULDER_BASE = np.array([-0.12, 0.0, 0.95])
L1 = 0.35   # 上臂长
L2 = 0.33   # 前臂长
FINGER_EXT = 0.30  # 手指尖端到肘的偏移（含手指 0.045 约等效）


class ServoGraspController:
    """视觉伺服抓取控制器（standalone 与 ROS 节点共用）。"""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_string(MODEL)
        self.data = mujoco.MjData(self.model)
        self.obj_body = self.model.body('object')
        self.tip_body = self.model.body('end_effector')
        self.grasped = False
        self.reached = False

        self._qadr = {}
        for name in (J_SHOULDER_ROLL, J_SHOULDER_PITCH, J_ELBOW,
                     J_FINGER_L, J_FINGER_R):
            self._qadr[name] = self.model.jnt_qposadr[self.model.joint(name).id]

    def object_pos(self):
        return self.data.xpos[self.obj_body.id]

    def tip_pos(self):
        return self.data.xpos[self.tip_body.id]

    def _joint_angle(self, name):
        return self.data.qpos[self._qadr[name]]

    def _set_joint(self, name, val):
        self.data.qpos[self._qadr[name]] = val

    def _set_joint_vel_zero(self, name):
        jid = self.model.joint(name).id
        self.data.qvel[self.model.jnt_dofadr[jid]] = 0.0

    def ik_arm(self, target_world):
        """对单臂（肩 roll + 肩 pitch + 肘 pitch）做平面 IK。

        手臂在 x-z 平面内运动，肩基座世界位置 SHOULDER_BASE。
        返回 (roll, shoulder_pitch, elbow)。
        """
        t_local = target_world - SHOULDER_BASE
        x, z = t_local[0], t_local[2]
        dist = np.hypot(x, z)
        dist = np.clip(dist, 1e-4, L1 + L2 - 1e-3)
        cos_el = (dist**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_el = np.clip(cos_el, -1.0, 1.0)
        elbow = np.arccos(cos_el)
        alpha = np.arctan2(z, x)
        beta = np.arccos(np.clip((L1**2 + dist**2 - L2**2) / (2 * L1 * dist), -1.0, 1.0))
        # 绕 +y 轴正角使杆朝 -z（下方）；目标方向角 alpha（朝下时<0），
        # 杆指向 alpha 方向需 pitch = -(alpha + beta)
        shoulder_pitch = -(alpha + beta)
        # 目标方向使 pitch 朝下（当 z<0 时 alpha<0，肩 pitch 为负）
        roll = 0.0
        return roll, shoulder_pitch, elbow

    def servo_step(self, dt):
        """一个控制周期：视觉伺服 + IK 驱动手臂 + 到位后抓取。"""
        mujoco.mj_forward(self.model, self.data)
        obj = self.object_pos()
        tip = self.tip_pos()

        # 末端到达判定（以夹具闭合中心近似）
        self.reached = np.linalg.norm(tip - obj) < REACH_TOL

        if self.reached and not self.grasped:
            # 合拢夹具抓取
            self._set_joint(J_FINGER_L, -GRASP_ANG)
            self._set_joint(J_FINGER_R, GRASP_ANG)
            self.grasped = True

        # 视觉伺服 IK：末端目标 = 目标物位置（略偏上以对准夹具）
        target = obj + np.array([0.0, 0.0, 0.02])
        roll, sp, se = self.ik_arm(target)
        k = K_IK
        for name, target_angle in ((J_SHOULDER_ROLL, roll),
                                   (J_SHOULDER_PITCH, sp),
                                   (J_ELBOW, se)):
            cur = self._joint_angle(name)
            self._set_joint(name, cur + k * (target_angle - cur))

        # 归零关节速度，避免自由关节受重力加速（运动学驱动）
        for name in (J_SHOULDER_ROLL, J_SHOULDER_PITCH, J_ELBOW,
                     J_FINGER_L, J_FINGER_R):
            self._set_joint_vel_zero(name)

        err = np.linalg.norm(tip - obj)
        return err, self.reached, self.grasped
