# -*- coding: utf-8 -*-
"""简易人形机器人 MuJoCo 仿真：原地踏步 + 交替摆臂（MuJoCo，不依赖 Gazebo）。

原理：
  使用内联 XML 构建一个简易人形（躯干 + 头 + 双臂 + 双腿，4 个旋转关节）。
  关节执行器为位置伺服（position actuator），控制器让双腿交替踏步、
  双臂交替摆动（相位错开），形成拟行走的节律运动，并离屏渲染输出 MP4 视频。

运行：python3 main.py            # 输出 output/robot_sim.mp4
      python3 main.py --preview  # 只跑 1.5 秒，输出 3 张 PNG 检查取景
      roslaunch midterm_humanoid main.launch  # ROS 环境
"""
import numpy as np

# 按包导入：源码树直跑由模块根目录的 main.py 先把 src/ 加入 sys.path 后转发，
# catkin 安装后由 setup.py / catkin 环境提供包路径。
from midterm_humanoid.common import setup_gl, run_sim, parse_args

setup_gl()  # 必须在 import mujoco 之前调用（无显示环境自动用 osmesa）

import mujoco  # noqa: E402

XML = """
<mujoco model="human">
  <option timestep="0.01"/>
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1" rgba="0.85 0.85 0.85 1"/>
    <camera name="view0" mode="track" target="torso" fovy="50"
            pos="0 -1.7 1.4" xyaxes="1 0 0 0 0.26 0.97"/>
    <body name="torso" pos="0 0 1.2">
      <geom type="box" size="0.2 0.15 0.28" density="300" rgba="0.2 0.6 0.8 1"/>
      <!-- 原地踏步仿真：躯干无关节 = 固定在世界系，四肢相对躯干摆动 -->
      <body name="head" pos="0 0 0.5">
        <geom type="sphere" size="0.15" density="300" rgba="0.9 0.5 0.3 1"/>
      </body>
      <body name="left_arm" pos="-0.3 0 0.12">
        <joint name="j_larm" type="hinge" axis="0 1 0" range="-80 80" damping="0.8"/>
        <geom type="cylinder" fromto="0 0 0 -0.35 0 0" size="0.06" density="300" rgba="0.1 0.7 0.4 1"/>
      </body>
      <body name="right_arm" pos="0.3 0 0.12">
        <joint name="j_rarm" type="hinge" axis="0 1 0" range="-80 80" damping="0.8"/>
        <geom type="cylinder" fromto="0 0 0 0.35 0 0" size="0.06" density="300" rgba="0.1 0.7 0.4 1"/>
      </body>
      <body name="left_leg" pos="-0.12 0 -0.45">
        <joint name="j_lleg" type="hinge" axis="0 1 0" range="-57 57" damping="1.2"/>
        <geom type="cylinder" fromto="0 0 0 0 0 -0.45" size="0.07" density="300" rgba="0.5 0.3 0.7 1"/>
      </body>
      <body name="right_leg" pos="0.12 0 -0.45">
        <joint name="j_rleg" type="hinge" axis="0 1 0" range="-57 57" damping="1.2"/>
        <geom type="cylinder" fromto="0 0 0 0 0 -0.45" size="0.07" density="300" rgba="0.5 0.3 0.7 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="j_larm" joint="j_larm" kp="20" ctrlrange="-1.2 1.2"/>
    <position name="j_rarm" joint="j_rarm" kp="20" ctrlrange="-1.2 1.2"/>
    <position name="j_lleg" joint="j_lleg" kp="20" ctrlrange="-0.8 0.8"/>
    <position name="j_rleg" joint="j_rleg" kp="20" ctrlrange="-0.8 0.8"/>
  </actuator>
  <visual>
    <global offwidth="960" offheight="544"/>
  </visual>
</mujoco>
"""

FREQ = 0.5     # 节律频率 Hz（2 秒一个完整周期）
ARM_AMP = 0.7  # 手臂摆幅 rad
LEG_AMP = 0.55  # 腿部踏步摆幅 rad


def make_controller(model):
    import mujoco

    idx = {model.actuator(i).name: i for i in range(model.nu)}

    def ctrl(data, t):
        phase = 2 * np.pi * FREQ * t
        # 双腿交替踏步（相位差 π），双臂交替摆动并与腿部错相
        data.ctrl[idx["j_lleg"]] = LEG_AMP * np.sin(phase)
        data.ctrl[idx["j_rleg"]] = LEG_AMP * np.sin(phase + np.pi)
        data.ctrl[idx["j_larm"]] = ARM_AMP * np.sin(phase + np.pi / 2)
        data.ctrl[idx["j_rarm"]] = ARM_AMP * np.sin(phase + np.pi * 3 / 2)

    return ctrl


def build_model():
    """构建模型（主流程与单元测试共用）。"""
    return mujoco.MjModel.from_xml_string(XML)


def main():
    args = parse_args()
    model = build_model()
    controller = make_controller(model)
    out = args.out or "output/robot_sim.mp4"
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
