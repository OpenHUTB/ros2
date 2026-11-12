# -*- coding: utf-8 -*-
"""参数化台阶场景 + 平面双足模型（纯 XML 生成，无外部网格）。

台阶由 ``step_height/step_depth/count`` 决定；机器人是平面模型：躯干只有
x/z/pitch 三个自由度，两条腿各 3 个关节（髋/膝/踝）。
"""

import humanoid_stair_zmp_preview.config as config


def build_scene_xml(step_height=config.STEP_HEIGHT, step_depth=config.STEP_DEPTH,
                    count=config.STEP_COUNT, direction="up"):
    """生成台阶场景。``direction="down"`` 时台阶向下延伸（下台阶）。"""
    h, d, n = float(step_height), float(step_depth), int(count)
    sign = 1.0 if direction == "up" else -1.0
    steps = []
    # 统一约定：**所有踏面都在 z >= 0**。
    #   上台阶：第 i 级踏面 (i+1)*h，台体从地面垒到踏面
    #   下台阶：从高度 n*h 的平台出发，第 i 级踏面 (n-i)*h，逐级降到 0
    # 早期把下台阶做成「向地面以下延伸」，结果脚一落到 z<0 就压进地面体块里
    # （实测 64% 的帧穿透、中位 -206 mm）。
    for i in range(n):
        x0 = config.STAIR_X0 + i * d
        z_top = (i + 1) * h if sign > 0 else (n - i) * h
        steps.append(
            f'    <geom name="step{i}" type="box" '
            f'pos="{x0 + d / 2:.4f} 0 {z_top / 2:.4f}" '
            f'size="{d / 2:.4f} 0.60 {max(z_top / 2, 0.05):.4f}" material="step_mat" '
            f'contype="1" conaffinity="1" group="0"/>')
    landing_x = config.STAIR_X0 + n * d + config.LANDING_DEPTH / 2
    steps.append(
        f'    <geom name="landing" type="box" pos="{landing_x:.4f} 0 0.025" '
        f'size="{config.LANDING_DEPTH / 2:.4f} 0.60 0.025" '
        f'material="step_mat" contype="1" conaffinity="1" group="0"/>')
    if sign < 0:
        # 出发平台：顶面在 n*h
        plat_z = n * h
        steps.append(
            f'    <geom name="platform" type="box" '
            f'pos="{config.STAIR_X0 - config.LANDING_DEPTH / 2:.4f} 0 {plat_z / 2:.4f}" '
            f'size="{config.LANDING_DEPTH / 2:.4f} 0.60 {plat_z / 2:.4f}" '
            f'material="step_mat" contype="1" conaffinity="1" group="0"/>')

    legs = []
    for side, y in (("l", 0.09), ("r", -0.09)):
        legs.append(f"""
      <body name="thigh_{side}" pos="0 {y} 0">
        <joint name="hip_{side}" type="hinge" axis="0 1 0" range="{config.JOINT_LIMITS['hip'][0]} {config.JOINT_LIMITS['hip'][1]}"/>
        <geom name="thigh_geom_{side}" type="capsule" fromto="0 0 0 0 0 -{config.THIGH}" size="0.045" mass="{config.LEG_MASS / 2}"/>
        <body name="shank_{side}" pos="0 0 -{config.THIGH}">
          <joint name="knee_{side}" type="hinge" axis="0 1 0" range="{config.JOINT_LIMITS['knee'][0]} {config.JOINT_LIMITS['knee'][1]}"/>
          <geom name="shank_geom_{side}" type="capsule" fromto="0 0 0 0 0 -{config.SHANK}" size="0.040" mass="{config.LEG_MASS / 2}"/>
          <body name="foot_{side}" pos="0 0 -{config.SHANK}">
            <joint name="ankle_{side}" type="hinge" axis="0 1 0" range="{config.JOINT_LIMITS['ankle'][0]} {config.JOINT_LIMITS['ankle'][1]}"/>
            <geom name="foot_geom_{side}" type="box" pos="{config.FOOT_LEN / 2 - 0.03:.3f} 0 -{config.FOOT_H / 2:.3f}"
                  size="{config.FOOT_LEN / 2:.3f} 0.05 {config.FOOT_H / 2:.3f}"
                  material="foot_mat" contype="1" conaffinity="1" group="0"/>
          </body>
        </body>
      </body>""")

    return f"""<mujoco model="stair_zmp_preview">
  <compiler angle="radian" coordinate="local" inertiafromgeom="true"/>
  <option timestep="{config.TIMESTEP}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual>
    <global offwidth="640" offheight="360"/>
    <quality shadowsize="2048"/>
    <map znear="0.02" zfar="40"/>
  </visual>
  <asset>
    <texture name="sky_tex" type="skybox" builtin="gradient"
             rgb1="0.42 0.52 0.66" rgb2="0.96 0.97 1.00" width="256" height="256"/>
    <texture name="grid_tex" type="2d" builtin="checker"
             rgb1="0.87 0.87 0.85" rgb2="0.93 0.93 0.91" width="256" height="256"/>
    <material name="floor_mat" texture="grid_tex" texrepeat="12 12" texuniform="true" rgba="1 1 1 1"/>
    <material name="step_mat" rgba="0.72 0.70 0.68 1"/>
    <material name="body_mat" rgba="0.22 0.52 0.82 1"/>
    <material name="leg_mat"  rgba="0.58 0.60 0.64 1"/>
    <material name="foot_mat" rgba="0.90 0.38 0.22 1"/>
  </asset>
  <default>
    <geom contype="0" conaffinity="0" group="1"/>
  </default>
  <worldbody>
    <light pos="2 -2 4" dir="-0.3 0.4 -1" diffuse="0.65 0.65 0.65"/>
    <light pos="5 2 3" dir="0.2 -0.3 -1" diffuse="0.35 0.35 0.38" castshadow="false"/>
    <geom name="floor" type="box"
          pos="{(config.STAIR_X0 - config.FLOOR_LEN / 2) / 2:.4f} 0 -0.25"
          size="{config.FLOOR_LEN / 2 + config.STAIR_X0 / 2:.4f} 2 0.25"
          material="floor_mat" contype="1" conaffinity="1" group="0"/>
{chr(10).join(steps)}
    <body name="torso" pos="0 0 {config.HIP_Z:.3f}">
      <joint name="root_x" type="slide" axis="1 0 0"/>
      <joint name="root_z" type="slide" axis="0 0 1"/>
      <joint name="root_pitch" type="hinge" axis="0 1 0"/>
      <geom name="torso_geom" type="capsule" fromto="0 0 -0.10 0 0 0.30" size="0.11"
            mass="{config.BODY_MASS - 2 * config.LEG_MASS}" material="body_mat"/>
{chr(10).join(legs)}
    </body>
    <camera name="side" pos="1.2 -3.2 1.1" xyaxes="1 0 0 0 0.32 0.95" fovy="42"/>
    <camera name="track" pos="-2.2 -2.6 1.5" mode="targetbody" target="torso" fovy="45"/>
  </worldbody>
  <actuator>
    <position name="hip_l"   joint="hip_l"   kp="400" kv="40" ctrlrange="-1.4 1.4"/>
    <position name="knee_l"  joint="knee_l"  kp="400" kv="40" ctrlrange="0.0 2.2"/>
    <position name="ankle_l" joint="ankle_l" kp="200" kv="20" ctrlrange="-0.9 0.9"/>
    <position name="hip_r"   joint="hip_r"   kp="400" kv="40" ctrlrange="-1.4 1.4"/>
    <position name="knee_r"  joint="knee_r"  kp="400" kv="40" ctrlrange="0.0 2.2"/>
    <position name="ankle_r" joint="ankle_r" kp="200" kv="20" ctrlrange="-0.9 0.9"/>
  </actuator>
  <sensor>
    <framepos objtype="body" objname="torso" name="torso_pos"/>
  </sensor>
</mujoco>
"""