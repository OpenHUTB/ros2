# two_dof_arm

Gazebo 中平面 2R 双自由度机械臂逆运动学画圆仿真：大臂 0.30 m、小臂 0.25 m，
末端在 XY 平面绕圆心 `(0.38, 0.20) m`、半径 `0.08 m` 的圆循环运动。

## 运行环境

| 项 | 说明 |
|---|---|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS 1 Noetic（catkin 包结构） |
| 模拟器 | Gazebo（gazebo_ros + ros_control） |
| 控制器 | 两个关节位置控制器（JointPositionController）+ 关节状态控制器 |

## 编译与运行

1. 把本包目录整个放进 catkin 工作空间的 `src/` 下；
2. 在**工作空间根目录**编译：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. 终端 1 启动 Gazebo 仿真与控制器：

```bash
roslaunch two_dof_arm arm_gazebo.launch
```

4. 终端 2 运行画圆控制脚本：

```bash
rosrun two_dof_arm main.py
```

## 包结构

```
two_dof_arm_circle/
├── CMakeLists.txt
├── package.xml
├── config/arm_controller.yaml    # 两个关节位置控制器 PID + 关节状态控制器
├── launch/arm_gazebo.launch       # Gazebo 启动、模型生成、控制器加载
├── urdf/two_dof_arm.urdf          # 机械臂 URDF 模型
└── scripts/main.py                # 逆运动学 + 圆轨迹控制脚本
```

## 预期现象

1. Gazebo 中加载蓝色底座、红色大臂、绿色小臂的二连杆模型；
2. 两个位置控制器由 spawner 启动；
3. 控制脚本发布目标关节角，末端在 XY 平面绕圆运动。

## 已知限制

- 仅验证肘向上构型；肘向下（`q2<0`）构型未测试；
- 圆轨迹点均在臂长可达范围内，未做工作空间外可达性检查；
- PID 参数为初始取值，未做整定优化。

## 文档

详细说明见 [文档站页面](../../../docs/chapter/gazebo_two_dof_arm_circle.md)。

## 大模型使用声明

本作业的代码与文档在编写过程中使用了**豆包（Doubao）** AI 大模型辅助，本人已核对并确认提交内容，对提交内容负责。
