# Gazebo 双自由度机械臂画圆作业

功能包源码位于仓库的 `src/chap8/two_dof_arm_circle` 目录。

## 1. 问题描述

平面 2R（双自由度）机械臂在 Gazebo 中沿圆形轨迹运动：大臂长 `L1=0.30 m`、
小臂长 `L2=0.25 m`，末端在 XY 平面绕圆心 `(0.38, 0.20) m`、半径 `0.08 m` 的圆循环运动。
控制脚本实时计算解析逆运动学，把目标关节角发布给位置控制器，由 ros_control 的
PID 位置闭环跟踪。

## 2. 运行环境

| 项 | 说明 |
|---|---|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS 1 Noetic（catkin 包结构） |
| 模拟器 | Gazebo（gazebo_ros + ros_control） |
| 控制器 | position_controllers/JointPositionController，joint_state_controller |

## 3. 算法

### 3.1 二连杆解析逆运动学

正运动学：

```
x = L1·cos(q1) + L2·cos(q1+q2)
y = L1·sin(q1) + L2·sin(q1+q2)
```

逆解（肘向上构型 `q2>0`）：

```
cos(q2) = (x²+y²-L1²-L2²)/(2·L1·L2)   # clamp 到 [-1,1]
q2 = acos(cos q2)
q1 = atan2(y,x) - atan2(L2·sin q2, L1+L2·cos q2)
```

### 3.2 圆轨迹参数化

```
x = cx + r·cos(ω·t)
y = cy + r·sin(ω·t)
```

`main.py` 以 50 Hz 循环计算当前时刻末端目标位置，IK 解算后向
`/joint1_position_controller/command`、`/joint2_position_controller/command`
发布目标关节角。

## 4. 模型参数（two_dof_arm.urdf）

| 部件 | 参数 |
|---|---|
| 底座 base_link | 蓝色立方体，边长 0.15 m |
| 大臂 link1 | 红色长方体，长 0.30 m，截面 0.05×0.05 m |
| 小臂 link2 | 绿色长方体，长 0.25 m，截面 0.05×0.05 m |
| 关节 joint1/joint2 | revolute，绕 Z 轴，限位 ±3.14 rad |
| 控制器 | 两个关节位置控制器，PID：p=20.0，i=0.1，d=0.5 |

## 5. 运行

把本包目录放进 catkin 工作空间的 `src/` 下，在**工作空间根目录**编译：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

终端 1 启动 Gazebo 仿真与控制器：

```bash
roslaunch two_dof_arm arm_gazebo.launch
```

终端 2 运行画圆控制脚本：

```bash
rosrun two_dof_arm main.py
```

## 6. 轨迹参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| cx, cy | 0.38, 0.20 | 圆心坐标 (m) |
| r | 0.08 | 圆半径 (m) |
| ω | 0.5 | 角速度 (rad/s) |
| L1, L2 | 0.30, 0.25 | 大臂 / 小臂杆长 (m) |
| 发布频率 | 50 Hz | 控制循环频率 |

## 7. 效果说明

![效果图](../img/gazebo_two_dof_arm_circle_demo.gif)

> **演示 GIF 待补录**：本作业需在本地 Gazebo + Noetic 环境运行后录制（动图需小于 10 MB），
> 录好后放到 `docs/img/gazebo_two_dof_arm_circle_demo.gif`。

预期现象：机械臂末端在 XY 平面绕圆心做圆周运动，两个关节由位置控制器闭环跟踪目标角。

## 8. 已知限制

- 仅验证肘向上构型；肘向下（`q2<0`）构型未测试；
- 圆轨迹点均在臂长可达范围内，未做工作空间外的可达性检查；
- 控制器 PID 参数为初始取值，未做整定优化。

## 说明

本作业的代码与文档在编写过程中使用了**豆包（Doubao）** AI 大模型辅助，本人已核对并确认提交内容，对提交内容负责。
