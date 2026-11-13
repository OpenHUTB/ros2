# -*- coding: utf-8 -*-
"""参数化窄通道场景生成。

单一真相源：build_scene_xml(width) 生成 XML；main.py --export-model 写入
models/corridor_scene.xml。集成测试校验两者一致，防止漂移。

布局（沿 +x 前进）::

    y
    ^   +---------------- 侧墙 y = +0.90
    |   |      # 障碍A         # 障碍B
    |   |   =====+=== 窄门 x=4，净宽 w
    |   |      x 人形（足迹半径 0.30）->
    |   +---------------- 侧墙 y = -0.90
    +----------------------------------> x
       x=0                            x=8
"""

import humanoid_passage_width_sweep.config as config


def build_scene_xml(width=config.DEFAULT_WIDTH, add_obstacles=True):
    """按窄门净宽 width 生成 MuJoCo 场景 XML。"""
    w = float(width)
    half_gap = w / 2.0
    side = config.CORRIDOR_HALF_WIDTH
    gate_x = config.GATE_X
    thick = config.WALL_THICKNESS
    wall_h = config.WALL_HEIGHT
    seg_half = max((side - half_gap) / 2.0, 1e-3)
    seg_y = (side + half_gap) / 2.0

    obstacles = ""
    if add_obstacles:
        obstacles = "\n".join(
            '    <geom name="obs_%d" type="cylinder" pos="%.3f %.3f 0.40" '
            'size="%.3f 0.40" material="obs_mat" contype="1" conaffinity="1" group="0"/>'
            % (i, ox, oy, config.OBSTACLE_RADIUS)
            for i, (ox, oy) in enumerate(config.OBSTACLES))

    return """<mujoco model="passage_width_sweep">
  <compiler angle="radian" coordinate="local" inertiafromgeom="true"/>
  <option timestep="%(dt).4f" gravity="0 0 -9.81" integrator="implicitfast"/>
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
    <material name="floor_mat" texture="grid_tex" texrepeat="10 10" texuniform="true" rgba="1 1 1 1"/>
    <material name="wall_mat"  rgba="0.62 0.66 0.72 1"/>
    <material name="obs_mat"   rgba="0.85 0.55 0.25 1"/>
    <material name="torso_mat" rgba="0.22 0.52 0.82 1"/>
    <material name="leg_mat"   rgba="0.58 0.60 0.64 1"/>
    <material name="foot_mat"  rgba="0.90 0.38 0.22 1"/>
    <material name="goal_mat"  rgba="0.25 0.70 0.35 0.55"/>
  </asset>
  <default>
    <geom contype="0" conaffinity="0" group="1"/>
  </default>
  <worldbody>
    <light pos="2 -1.6 3.4" dir="-0.3 0.3 -1" diffuse="0.65 0.65 0.65"/>
    <light pos="6 1.4 2.8" dir="0.2 -0.4 -1" diffuse="0.35 0.35 0.38" castshadow="false"/>
    <geom name="floor" type="plane" size="8 4 0.1" material="floor_mat"
          contype="1" conaffinity="1" group="0" friction="1.2 0.02 0.002"/>
    <geom name="wall_north" type="box" pos="%(xmid).3f %(yn).3f %(wh).3f"
          size="%(xhalf).3f %(thalf).3f %(wh).3f"
          material="wall_mat" contype="1" conaffinity="1" group="0"/>
    <geom name="wall_south" type="box" pos="%(xmid).3f %(ys).3f %(wh).3f"
          size="%(xhalf).3f %(thalf).3f %(wh).3f"
          material="wall_mat" contype="1" conaffinity="1" group="0"/>
    <geom name="gate_left" type="box" pos="%(gx).3f %(gy_neg).3f %(wh).3f"
          size="%(thalf).3f %(seg_half).3f %(wh).3f"
          material="wall_mat" contype="1" conaffinity="1" group="0"/>
    <geom name="gate_right" type="box" pos="%(gx).3f %(gy_pos).3f %(wh).3f"
          size="%(thalf).3f %(seg_half).3f %(wh).3f"
          material="wall_mat" contype="1" conaffinity="1" group="0"/>
%(obstacles)s
    <geom name="goal_mark" type="cylinder" pos="%(gox).3f %(goy).3f 0.004"
          size="%(gr).3f 0.004" material="goal_mat"/>
    <body name="torso" pos="0 0 0">
      <joint name="root_x" type="slide" axis="1 0 0"/>
      <joint name="root_y" type="slide" axis="0 1 0"/>
      <joint name="root_yaw" type="hinge" axis="0 0 1"/>
      <geom name="footprint" type="cylinder" pos="0 0 %(fh).3f"
            size="%(fr).3f %(fh).3f"
            material="torso_mat" contype="1" conaffinity="1" group="0"/>
      <geom name="torso_geom" type="capsule" fromto="0 0 0.95 0 0 1.42" size="0.15" material="torso_mat"/>
      <geom name="head_geom" type="sphere" pos="0 0 1.60" size="0.115" material="leg_mat"/>
      <geom name="arm_l" type="capsule" fromto="0 0.30 1.32 0 0.34 0.95" size="0.055" material="leg_mat"/>
      <geom name="arm_r" type="capsule" fromto="0 -0.30 1.32 0 -0.34 0.95" size="0.055" material="leg_mat"/>
      <geom name="leg_l" type="capsule" fromto="0 0.12 0.95 0 0.12 0.10" size="0.075" material="leg_mat"/>
      <geom name="leg_r" type="capsule" fromto="0 -0.12 0.95 0 -0.12 0.10" size="0.075" material="leg_mat"/>
      <geom name="foot_l" type="box" pos="0.05 0.12 0.045" size="0.13 0.06 0.045" material="foot_mat"/>
      <geom name="foot_r" type="box" pos="0.05 -0.12 0.045" size="0.13 0.06 0.045" material="foot_mat"/>
    </body>
    <camera name="overview" pos="%(xmid).2f 0 11.0" xyaxes="1 0 0 0 1 0" fovy="52"/>
    <camera name="follow" pos="-2.6 0 1.9" mode="targetbody" target="torso" fovy="45"/>
  </worldbody>
  <sensor>
    <framepos objtype="body" objname="torso" name="torso_pos"/>
  </sensor>
</mujoco>
""" % dict(
        dt=config.TIMESTEP, xmid=config.CORRIDOR_X_MID,
        yn=side + thick / 2.0, ys=-side - thick / 2.0,
        wh=wall_h / 2.0, xhalf=config.CORRIDOR_LENGTH / 2.0, thalf=thick / 2.0,
        gx=gate_x, gy_neg=-seg_y, gy_pos=seg_y, seg_half=seg_half,
        obstacles=obstacles,
        gox=config.GOAL[0], goy=config.GOAL[1], gr=config.GOAL_RADIUS,
        fh=config.FOOTPRINT_HEIGHT / 2.0, fr=config.FOOTPRINT_RADIUS)


def gate_clearance(width):
    """窄门单侧净空 = (门宽 - 足迹直径) / 2；负值表示几何上不可能通过。"""
    return (float(width) - 2.0 * config.FOOTPRINT_RADIUS) / 2.0


def is_passable(width):
    """净宽是否大于足迹直径（留 5 mm 余量，避免浮点等价情形）。"""
    return gate_clearance(width) > 0.005