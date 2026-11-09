#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三足仿生机器人 MuJoCo 仿真：三角步态交替迈步（MuJoCo，不依赖 Gazebo）。

原理：
  模型为三足机器人（躯干 + 3 条腿，每条腿 hip_abd / hip_rot / knee 共 3 个旋转关节），
  9 个关节均由位置伺服执行器驱动。控制律让三条腿按 2π/3 相位差交替
  「抬脚前迈 → 落地支撑」，形成稳定的三角步态，并离屏渲染输出 MP4 视频。

运行：python3 main.py            # 输出 output/sim_video.mp4
      python3 main.py --preview  # 只跑 1.5 秒，输出 3 张 PNG 检查取景
      roslaunch tripod_mujoco main.launch  # ROS 环境
"""
import numpy as np

# 按包导入：源码树直跑由模块根目录的 main.py 先把 src/ 加入 sys.path 后转发，
# catkin 安装后由 setup.py / catkin 环境提供包路径。
from tripod_mujoco.common import setup_gl, run_sim, parse_args

setup_gl()  # 必须在 import mujoco 之前调用（无显示环境自动用 osmesa）

import mujoco  # noqa: E402

XML = """
<mujoco model="tripod_robot">
  <compiler angle="radian" inertiafromgeom="true" autolimits="true"/>
  <option timestep="0.002"/>

  <default>
    <joint armature="0.01" damping="1.0" limited="true"/>
    <geom condim="4" friction="1.0 0.1 0.1" material="mat_body"/>
  </default>

  <asset>
    <material name="mat_body" rgba="0.2 0.5 0.9 1"/>
    <material name="mat_leg" rgba="0.9 0.5 0.2 1"/>
    <material name="mat_foot" rgba="0.3 0.3 0.3 1"/>
    <material name="mat_ground" rgba="0.8 0.8 0.8 1"/>
    <texture type="skybox" builtin="gradient" rgb1="0.6 0.8 1.0" rgb2="1 1 1" width="512" height="512"/>
  </asset>

  <worldbody>
    <light directional="true" pos="0 0 5" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="ground" type="plane" size="5 5 0.1" material="mat_ground"/>
    <camera name="view0" mode="targetbody" target="torso" fovy="45"
            pos="1.15 -1.75 0.60"/>

    <body name="torso" pos="0 0 0.35">
      <freejoint/>
      <geom name="body_sphere" type="sphere" size="0.08" mass="1.0"/>
      <geom name="body_torso" type="cylinder" size="0.06 0.05" mass="0.5"/>

      <!-- 腿1: 前方 0度 -->
      <body name="hip_1" pos="0.07 0 0">
        <joint name="hip_abd_1" type="hinge" axis="0 1 0" range="-0.524 0.524"/>
        <joint name="hip_rot_1" type="hinge" axis="0 0 1" range="-0.785 0.785"/>
        <geom name="hip_ball_1" type="sphere" size="0.03"/>
        <body name="thigh_1" pos="0.05 0 -0.05">
          <joint name="knee_1" type="hinge" axis="0 1 0" range="-1.571 0.175"/>
          <geom name="thigh_geom_1" type="capsule" size="0.025 0.08" pos="0 0 -0.08" material="mat_leg"/>
          <body name="shin_1" pos="0 0 -0.16">
            <geom name="shin_geom_1" type="capsule" size="0.02 0.07" pos="0 0 -0.07" material="mat_leg"/>
            <geom name="foot_1" type="sphere" size="0.03" pos="0 0 -0.15" material="mat_foot"/>
          </body>
        </body>
      </body>

      <!-- 腿2: 左后方 120度 -->
      <body name="hip_2" pos="-0.035 0.0606 0" euler="0 0 2.0944">
        <joint name="hip_abd_2" type="hinge" axis="0 1 0" range="-0.524 0.524"/>
        <joint name="hip_rot_2" type="hinge" axis="0 0 1" range="-0.785 0.785"/>
        <geom name="hip_ball_2" type="sphere" size="0.03"/>
        <body name="thigh_2" pos="0.05 0 -0.05">
          <joint name="knee_2" type="hinge" axis="0 1 0" range="-1.571 0.175"/>
          <geom name="thigh_geom_2" type="capsule" size="0.025 0.08" pos="0 0 -0.08" material="mat_leg"/>
          <body name="shin_2" pos="0 0 -0.16">
            <geom name="shin_geom_2" type="capsule" size="0.02 0.07" pos="0 0 -0.07" material="mat_leg"/>
            <geom name="foot_2" type="sphere" size="0.03" pos="0 0 -0.15" material="mat_foot"/>
          </body>
        </body>
      </body>

      <!-- 腿3: 右后方 240度 -->
      <body name="hip_3" pos="-0.035 -0.0606 0" euler="0 0 4.1888">
        <joint name="hip_abd_3" type="hinge" axis="0 1 0" range="-0.524 0.524"/>
        <joint name="hip_rot_3" type="hinge" axis="0 0 1" range="-0.785 0.785"/>
        <geom name="hip_ball_3" type="sphere" size="0.03"/>
        <body name="thigh_3" pos="0.05 0 -0.05">
          <joint name="knee_3" type="hinge" axis="0 1 0" range="-1.571 0.175"/>
          <geom name="thigh_geom_3" type="capsule" size="0.025 0.08" pos="0 0 -0.08" material="mat_leg"/>
          <body name="shin_3" pos="0 0 -0.16">
            <geom name="shin_geom_3" type="capsule" size="0.02 0.07" pos="0 0 -0.07" material="mat_leg"/>
            <geom name="foot_3" type="sphere" size="0.03" pos="0 0 -0.15" material="mat_foot"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="hip_abd_1" joint="hip_abd_1" kp="30" ctrlrange="-0.524 0.524"/>
    <position name="hip_rot_1" joint="hip_rot_1" kp="50" ctrlrange="-0.785 0.785"/>
    <position name="knee_1" joint="knee_1" kp="50" ctrlrange="-1.571 0.175"/>
    <position name="hip_abd_2" joint="hip_abd_2" kp="30" ctrlrange="-0.524 0.524"/>
    <position name="hip_rot_2" joint="hip_rot_2" kp="50" ctrlrange="-0.785 0.785"/>
    <position name="knee_2" joint="knee_2" kp="50" ctrlrange="-1.571 0.175"/>
    <position name="hip_abd_3" joint="hip_abd_3" kp="30" ctrlrange="-0.524 0.524"/>
    <position name="hip_rot_3" joint="hip_rot_3" kp="50" ctrlrange="-0.785 0.785"/>
    <position name="knee_3" joint="knee_3" kp="50" ctrlrange="-1.571 0.175"/>
  </actuator>

  <keyframe>
    <!-- freejoint 7 项 + 9 关节：hip_abd 0、hip_rot 0、knee -0.35 rad（初始屈膝，脚贴地） -->
    <key name="stand" qpos="0 0 0.38 1 0 0 0 0 0 -0.35 0 0 -0.35 0 0 -0.35"/>
  </keyframe>

  <visual>
    <global offwidth="960" offheight="544"/>
  </visual>
</mujoco>
"""

FREQ = 0.8       # 步频 Hz（三角步态）
A_ROT = 0.16     # hip_rot 前后摆幅 rad（±9°，小步幅保证开环也能站稳）
KNEE_BASE = -0.35  # 屈膝基准 rad（-20°）
LIFT = 0.35      # 摆动相额外屈膝抬脚 rad


def make_controller(model):
    idx = {model.actuator(i).name: i for i in range(model.nu)}

    def ctrl(data, t):
        for i in range(3):
            ph = 2 * np.pi * FREQ * t + 2 * np.pi * i / 3
            n = i + 1
            # 外展保持中位；前后摆驱动迈步；摆动相（sin>0）屈膝抬脚，支撑相伸膝蹬地
            data.ctrl[idx["hip_abd_%d" % n]] = 0.0
            data.ctrl[idx["hip_rot_%d" % n]] = A_ROT * np.sin(ph)
            data.ctrl[idx["knee_%d" % n]] = KNEE_BASE + max(0.0, np.sin(ph)) * LIFT

    return ctrl


def build_model():
    """构建模型（主流程与单元测试共用）。"""
    return mujoco.MjModel.from_xml_string(XML)


def main():
    args = parse_args()
    model = build_model()
    controller = make_controller(model)
    out = args.out or "output/sim_video.mp4"
    run_sim(
        model, controller,
        duration=1.5 if args.preview else (8 if args.duration is None else args.duration),
        fps=30,
        video_path=None if args.preview else out,
        preview_dir="output/preview" if args.preview else None,
    )
    if not args.preview:
        print(f"仿真完成，视频已保存: {out}")


if __name__ == "__main__":
    main()
