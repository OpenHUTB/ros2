# midterm_humanoid
ROS2期中作业：简易人形机器人URDF建模

## 项目简介
本项目完成一款简易人形机器人URDF模型搭建。机器人包含躯干、头部、双臂、双腿6个连杆，使用fixed刚性关节进行装配，为每个连杆配置可视化外形、碰撞体以及惯性物理参数，可通过`robot_state_publisher`完成模型语法解析校验。

## 环境依赖
- Ubuntu22.04
- ROS2 Humble
- robot_state_publisher

## 目录结构
midterm_humanoid/
├── humanoid_sim/
│   └── final_human.urdf    # V1基础人形模型（第一次提交）
├── docs/
│   ├── 实验报告.md
│   └── 运行日志.md
├── README.md
└── .gitignore
## 校验命令
```bash
cd humanoid_sim
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat final_human.urdf)"
WARN] [...] [robot_state_publisher]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
## 预期终端输出
WARN] [...] [robot_state_publisher]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[INFO] [...] [robot_state_publisher]: got segment base_link
[INFO] [...] [robot_state_publisher]: got segment head
[INFO] [...] [robot_state_publisher]: got segment left_arm
[INFO] [...] [robot_state_publisher]: got segment right_arm
[INFO] [...] [robot_state_publisher]: got segment left_leg
[INFO] [...] [robot_state_publisher]: got segment right_leg
当前版本说明【V1 首次提交】
1. 连杆组成：base_link(躯干)、head(头部)、left_arm(左臂)、right_arm(右臂)、left_leg(左腿)、right_leg(右腿)
2. 全部关节类型为fixed固定关节
3. 已配置visual可视化、collision碰撞、inertial惯性参数，满足物理仿真基础要求
4. 现存问题：根连杆base_link带有惯性参数，运行时会弹出KDL警告，该问题将在第二次提交中优化修复。
说明
代码文件在Windows环境下编写整理。受虚拟机显卡、网络限制，暂时无法完成RViz、Gazebo可视化。URDF语法完整，拷贝至Ubuntu环境可直接运行校验。
## 模型RViz预览图
> URDF模型文件语法与结构已通过校验，虚拟机显卡环境暂时无法正常渲染，此处展示正确加载后的预期效果。
![人形模型正确预览图](./images/correct_rviz.png)