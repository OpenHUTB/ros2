<<<<<<< HEAD
# Ubuntu 20.04 配置 Gazebo Classic 环境完整指南

> **适用系统**：Ubuntu 20.04 LTS (Focal Fossa)  
> **ROS 版本**：ROS Noetic（ROS1 最终版本）  
> **Gazebo 版本**：Gazebo Classic 11  
> **文档说明**：本文档记录从零开始安装 ROS Noetic + Gazebo Classic 11 仿真环境的完整流程，包含常见问题排查与验证方法。

---

## 目录

- [一、环境说明](#一环境说明)
- [二、安装 ROS Noetic](#二安装-ros-noetic)
- [三、安装 Gazebo Classic 11](#三安装-gazebo-classic-11)
- [四、安装 gazebo_ros_pkgs（ROS 桥接包）](#四安装-gazebo_ros_pkgsros-桥接包)
- [五、配置环境变量](#五配置环境变量)
- [六、验证 ROS + Gazebo 集成](#六验证-ros--gazebo-集成)
- [七、实验结果](#七实验结果)
- [八、常见问题与解决方案](#八常见问题与解决方案)
- [九、一键检查脚本](#九一键检查脚本)
- [十、后续学习建议](#十后续学习建议)

---

## 一、环境说明

| 项目 | 版本/说明 |
|------|-----------|
| 操作系统 | Ubuntu 20.04 LTS (Focal Fossa) |
| ROS 版本 | ROS Noetic Ninjemys |
| Gazebo 版本 | Gazebo Classic 11.15.1 |
| 适用场景 | 机器人仿真、SLAM、导航、控制算法验证 |

> ⚠️ **重要提示**
> - Gazebo Classic 已于 **2025 年 1 月停止维护**，但 ROS Noetic 仍然使用它，现有教程和项目完全兼容。
> - Ubuntu 20.04 **必须搭配 Gazebo 11**，不要误装 `gazebo9`（那是 Ubuntu 18.04 + ROS Melodic 的标配）。
> - 后续若需迁移，可考虑 Gazebo Sim（新版）/ Ignition，但 ROS1 生态目前仍以 Classic 为主。

---

## 二、安装 ROS Noetic

### 2.1 配置 ROS 软件源

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-noetic.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

> 若官方源访问慢，可改用清华、中科大等国内镜像源。

### 2.2 更新并安装完整桌面版

**强烈建议安装 `ros-noetic-desktop-full`**，它已包含 Gazebo、RViz、rqt 等全套工具：

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

### 2.3 初始化 rosdep

```bash
sudo rosdep init
rosdep update
```

> 若因网络问题失败，可使用 `fishros` 一键配置工具或手动替换 `rosdistro` 为国内镜像。

### 2.4 配置环境变量

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.5 验证 ROS 安装

```bash
roscore
```

终端输出 ROS master 启动信息即表示成功（按 `Ctrl+C` 退出）。

---

## 三、安装 Gazebo Classic 11

> 若 `ros-noetic-desktop-full` 已安装成功，通常已包含 Gazebo 11。若终端输入 `gazebo` 提示 `Command not found`，则需要单独安装。

### 3.1 安装 Gazebo 11 本体

```bash
sudo apt update
sudo apt install gazebo11 libgazebo11-dev
```

> **关键点**：必须显式指定 `gazebo11`，不要只敲 `sudo apt install gazebo`，否则系统可能会提示安装 `gazebo9`（版本不匹配）。

### 3.2 验证 Gazebo 独立运行

```bash
gazebo
```

应弹出 Gazebo GUI 窗口，左侧有 `Insert` 标签，中央显示网格和坐标轴。

### 3.3 查看版本

```bash
gazebo --version
```

期望输出类似：

```
Gazebo multi-robot simulator, version 11.15.1
```

---

## 四、安装 gazebo_ros_pkgs（ROS 桥接包）

`gazebo_ros_pkgs` 是 **ROS 与 Gazebo 通信的桥梁**，提供 `gazebo_ros` 节点、传感器/执行器插件等。

### 4.1 安装核心桥接包

```bash
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

| 包名 | 作用 |
|------|------|
| `ros-noetic-gazebo-ros-pkgs` | 提供 `gazebo_ros` 节点、`gazebo_plugins` 插件集 |
| `ros-noetic-gazebo-ros-control` | ROS Control 与 Gazebo 的集成，用于控制器管理 |

### 4.2 验证安装

```bash
rospack find gazebo_ros
```

期望输出：

```
/opt/ros/noetic/share/gazebo_ros
```

---

## 五、配置环境变量

### 5.1 添加 Gazebo 模型路径

编辑 `~/.bashrc`，在末尾追加：

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/noetic/share/gazebo-11/models
```

### 5.2 source Gazebo 环境脚本

```bash
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
```

### 5.3 刷新命令缓存（可选）

若之前输入过 `gazebo` 提示找不到，执行以下命令刷新 bash 哈希表：

```bash
hash -r
```

---

## 六、验证 ROS + Gazebo 集成

### 6.1 用 ROS 方式启动 Gazebo

```bash
rosrun gazebo_ros gazebo
```

或使用 launch 文件（推荐）：

```bash
roslaunch gazebo_ros empty_world.launch
```

> **注意**：建议使用 ROS 命令启动 Gazebo，而不是直接敲 `gazebo`，这样能自动加载 ROS 插件、发布 `/clock`、`/gazebo/model_states` 等话题。

### 6.2 检查 ROS 话题

新开终端：

```bash
rostopic list | grep gazebo
```

应能看到：

```
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
```

### 6.3 在 Gazebo 中生成测试模型

新开终端：

```bash
rosrun gazebo_ros spawn_model -file `rospack find gazebo_ros`/worlds/empty.world -urdf -model test_box -x 1 -y 0
```

### 6.4 使用 launch 文件启动（推荐方式）

创建 `gazebo_test.launch`：

```xml
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
  </include>
</launch>
```

运行：

```bash
roslaunch your_package gazebo_test.launch
```

---

## 七、实验结果

### 7.1 实验环境

| 项目 | 内容 |
|------|------|
| 操作系统 | Ubuntu 20.04 LTS |
| ROS 版本 | ROS Noetic |
| Gazebo 版本 | Gazebo Classic 11.15.1 |
| 启动命令 | `rosrun gazebo_ros gazebo` |
| 启动时间 | 2026-09-20 19:49 |

### 7.2 实验结果截图

![ROS Gazebo 集成成功运行截图](../img/chapter/gazebo_ros_result.png)

> **图注**：通过 `rosrun gazebo_ros gazebo` 命令成功启动 Gazebo Classic 11，界面完整显示 3D 视图、坐标轴与仿真控制栏。

### 7.3 结果分析

| 检查项 | 结果 | 说明 |
|--------|------|------|
| Gazebo GUI 启动 | ✅ 成功 | 3D 视图、菜单栏、工具栏正常显示 |
| ROS 桥接加载 | ✅ 成功 | 由 ROS 命令启动，自动加载 `gazebo_ros` 插件 |
| 仿真引擎运行 | ✅ 正常 | 底部状态栏显示 `Real Time Factor: 1.00`、`FPS: 62.53` |
| 坐标系统 | ✅ 正常 | 红色 X 轴、绿色 Y 轴、蓝色 Z 轴正常显示 |
| 警告信息 | ⚠️ 可忽略 | `context mismatch in svga_surface_destroy`（虚拟机渲染警告）；`reaches end-of-life`（官方停维提示） |

### 7.4 结论

实验表明，在 Ubuntu 20.04 系统下，通过 ROS Noetic 成功配置了 Gazebo Classic 11 仿真环境：

- ✅ `gazebo_ros_pkgs` 桥接包安装正确，ROS 与 Gazebo 通信正常；
- ✅ Gazebo GUI 启动成功，渲染与物理引擎运行正常；
- ⚠️ 终端出现的两条警告信息（`svga_surface_destroy`、`end-of-life`）**不影响仿真功能**，可安全忽略；
- ✅ 环境已具备开展机器人仿真、SLAM、导航等开发任务的基础条件。

### 7.5 终端日志摘录

启动命令与输出：

```bash
$ rosrun gazebo_ros gazebo
context mismatch in svga_surface_destroy
context mismatch in svga_surface_destroy
```

GUI 顶部提示：

```
This version of Gazebo reaches end-of-life in January 2025. Consider migrating to the new Gazebo.
```

底部状态栏信息：

```
Steps: 1 - Real Time Factor: 1.00 - Sim Time: 00:00:05.01.214 - Real Time: 00:00:05.01.892 - Iterations: 301214 - FPS: 62.53
```

---

## 八、常见问题与解决方案

### 8.1 报错速查表

| 问题现象 | 原因 | 解决方案 |
|----------|------|----------|
| `Command 'gazebo' not found` | Gazebo 本体未安装 | `sudo apt install gazebo11 libgazebo11-dev` |
| 提示 `did you mean gazebo9` | 系统默认匹配了错误版本 | 显式指定 `gazebo11` 安装 |
| `[rospack] Error: package 'gazebo_ros' not found` | 环境变量未 source | `source ~/.bashrc`，检查 `ROS_PACKAGE_PATH` |
| `context mismatch in svga_surface_destroy` | 虚拟机显卡 OpenGL 渲染警告 | **可忽略**，不影响使用 |
| Gazebo 黑屏/卡顿 | 显卡驱动或 OpenGL 问题 | `export LIBGL_ALWAYS_SOFTWARE=1` 后重启 Gazebo |
| 模型加载失败 | `GAZEBO_MODEL_PATH` 未设置 | 在 `~/.bashrc` 中添加模型路径并 source |
| `hash: _: not found` | bash 缓存问题 | 执行 `hash -r` |
| `apt install gazebo11` 找不到包 | 系统版本不是 20.04 或源有问题 | `lsb_release -a` 确认版本 |

### 8.2 关于警告信息的说明

**Gazebo 启动时提示 `This version of Gazebo reaches end-of-life`**

这是 Gazebo 官方提示 Gazebo Classic 11 已停止维护。由于 ROS Noetic 官方适配的仍是 Gazebo 11，**可放心忽略**。

**终端出现 `context mismatch in svga_surface_destroy`**

这是虚拟机环境下 OpenGL 渲染的警告信息。**不影响 Gazebo 功能**，可正常进行机器人仿真。

### 8.3 已安装但命令找不到的排查步骤

```bash
# 1. 刷新命令缓存
hash -r

# 2. 检查可执行文件是否存在
which gazebo
ls /usr/bin/gazebo

# 3. 若 apt 认为已安装但文件丢失，强制重装
sudo apt install --reinstall gazebo11

# 4. 用 ROS 方式启动（推荐）
rosrun gazebo_ros gazebo
```

---

## 九、一键检查脚本

将以下内容保存为 `check_gazebo_ros.sh`：

```bash
#!/bin/bash
source /opt/ros/noetic/setup.bash

echo "=========== 环境检查 ==========="
echo "--- Ubuntu 版本 ---"
lsb_release -d

echo "--- ROS 版本 ---"
rosversion -d

echo "--- Gazebo 版本 ---"
gazebo --version

echo "--- gazebo_ros 包位置 ---"
rospack find gazebo_ros

echo "--- gazebo_ros_control 包位置 ---"
rospack find gazebo_ros_control

echo "=========== 检查完成 ==========="
```

赋予执行权限并运行：

```bash
chmod +x check_gazebo_ros.sh
./check_gazebo_ros.sh
```

---

## 十、后续学习建议

### 10.1 建模与仿真

- **URDF/Xacro 建模**：学习用 XML 描述机器人结构，通过 `spawn_model` 加载到 Gazebo。
- **传感器插件**：为机器人添加相机、激光雷达、IMU 等，参考 `gazebo_plugins` 包。
- **世界文件（World）**：自定义仿真场景，添加地面、障碍物、光照等。

### 10.2 控制与算法

- **ROS Control**：结合 `ros-noetic-gazebo-ros-control`，配置关节控制器（位置/速度/力矩）。
- **MoveIt!**：配合 Gazebo 做机械臂运动规划仿真。
- **SLAM/导航**：在 Gazebo 中使用 `gmapping`、`cartographer`、`move_base` 做仿真测试。

### 10.3 推荐参考资料

- [Gazebo Classic 官方文档](http://classic.gazebosim.org/tutorials)
- [ROS Wiki - gazebo_ros_pkgs](http://wiki.ros.org/gazebo_ros_pkgs)
- [ROS Wiki - URDF](http://wiki.ros.org/urdf)

---

## 附录：快速命令速查

```bash
# --- 安装 ---
sudo apt install ros-noetic-desktop-full
sudo apt install gazebo11 libgazebo11-dev
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# --- 环境配置 ---
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
hash -r

# --- 启动 ---
gazebo                                              # 独立启动
rosrun gazebo_ros gazebo                            # ROS 方式启动
roslaunch gazebo_ros empty_world.launch             # Launch 启动

# --- 验证 ---
rostopic list | grep gazebo
rospack find gazebo_ros
gazebo --version
```

---

**文档版本**：v1.1  
**适用系统**：Ubuntu 20.04 + ROS Noetic + Gazebo Classic 11  
**最后更新**：2026-09-21
=======
# 阿克曼四轮小车运动控制

本章实现一辆阿克曼（Ackermann）结构四轮小车的运动控制：使用 xacro 完成整车建模，通过 ros_control 加载后轮速度控制器与前轮转向控制器，编写键盘遥控节点完成阿克曼转角与轮速解算，并为小车配置激光雷达、摄像头和 IMU，在仿真环境中验证控制效果与传感器数据。本章完整工程代码位于仓库 `src/chap7` 目录，包含 `smartcar_description`（模型与配置）和 `smartcar_control`（控制节点）两个功能包。

## 1. 环境配置与工程结构

### 1.1 运行环境

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 20.04（虚拟机） |
| ROS 版本 | Noetic Ninjemys |
| 仿真环境 | Gazebo 11 |
| 可视化工具 | Rviz 1.14 |
| 工作空间 | `~/catkin_ws` |

### 1.2 工程结构

两个功能包的目录结构如下：

```
src/chap7/
├── smartcar_description/          # 小车模型、控制器配置与启动文件
│   ├── config/
│   │   ├── smartcar_joint.yaml    # 五个 ros_control 控制器配置
│   │   └── smartcar.rviz          # Rviz 显示配置（含 LaserScan）
│   ├── launch/xacro/
│   │   └── smartcar_gazebo.launch # 总启动文件
│   └── urdf/xacro/
│       ├── smartcar.xacro         # xacro 入口
│       ├── smartcar_body.xacro    # 车体模型
│       └── smartcar_sim.xacro     # 传感器与控制插件
└── smartcar_control/              # 运动控制节点
    ├── launch/
    │   └── smartcar_gazebo_controller.launch
    ├── scripts/
    │   └── front_wheel_jsp.py     # 前轮关节状态补全节点
    └── src/
        └── smartcar_control_gazebo.cpp  # 键盘遥控与阿克曼解算
```

整车模型共包含 16 个 link、15 个 joint。其中 6 个为运动关节：

| 关节名称 | 类型 | 作用 | 控制接口 |
| --- | --- | --- | --- |
| `left_back_wheel_joint` / `right_back_wheel_joint` | continuous | 后轮驱动 | VelocityJointInterface |
| `left_bridge_to_bridge` / `right_bridge_to_bridge` | revolute | 前轮转向 | EffortJointInterface |
| `left_front_wheel_to_bridge` / `right_front_wheel_to_bridge` | continuous | 前轮自转 | 无（由补全节点发布） |

## 2. 小车模型构建

### 2.1 xacro 模型组织

车体模型使用 xacro 宏描述，入口文件 `smartcar.xacro` 通过 `xacro:include` 分别引入车体宏与仿真插件宏，再实例化两个宏：

```xml
<?xml version="1.0"?>
<robot name="smartcar" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find smartcar_description)/urdf/xacro/smartcar_body.xacro"/>
  <xacro:include filename="$(find smartcar_description)/urdf/xacro/smartcar_sim.xacro"/>
  <xacro:smartcar_body/>
  <xacro:smartcar_sim/>
</robot>
```

`smartcar_body.xacro` 中定义了底盘、前后桥、四个车轮、上层支架以及激光雷达、摄像头、IMU 的安装结构，并以宏的形式封装了圆柱体惯性矩阵，例如：

```xml
<xacro:macro name="cylinder_inertial_matrix" params="m r h">
  <inertial>
    <mass value="${m}" />
    <inertia
      ixx="${m*(3*r*r+h*h)/12}" ixy="0" ixz="0"
      iyy="${m*(3*r*r+h*h)/12}" iyz="0"
      izz="${m*r*r/2}" />
  </inertial>
</xacro:macro>
```

### 2.2 xacro 旧语法修复

早期工程的 xacro 文件采用旧语法，在 Noetic 下无法正常解析，需要统一修复：

1. 文件包含标签由 `<include filename="..."/>` 改为 `<xacro:include filename="..."/>`；
2. 所有宏调用必须带 `xacro:` 前缀，例如 9 处圆柱体惯性矩阵调用由：

```xml
<cylinder_inertial_matrix m="0.1" r="0.025" h="0.05"/>
```

改为：

```xml
<xacro:cylinder_inertial_matrix m="0.1" r="0.025" h="0.05"/>
```

3. 入口文件中的宏调用同样需要前缀，即 `<xacro:smartcar_body/>` 与 `<xacro:smartcar_sim/>`。

## 3. 编译与启动

### 3.1 创建工作空间并编译

将两个功能包放入工作空间的 `src` 目录后，在工作空间根目录执行编译：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# 将 smartcar_description、smartcar_control 置于 src/chap7 下
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

编译成功后输出如下：

![编译成功](../img/code/chap7/03_catkin_make_success.png)

### 3.2 启动仿真环境

使用总启动文件启动 Gazebo、加载模型参数、加载控制器并将小车模型放入仿真环境：

```bash
roslaunch smartcar_description smartcar_gazebo.launch
```

启动文件中首先包含 Gazebo 的空世界模板，然后加载 xacro 模型与控制器配置：

```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
</include>

<param name="robot_description"
       command="$(find xacro)/xacro --inorder '$(find smartcar_description)/urdf/xacro/smartcar.xacro'" />
<rosparam file="$(find smartcar_description)/config/smartcar_joint.yaml" command="load"/>

<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
<node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"
      args="-urdf -model mrobot -param robot_description" output="screen"/>
```

启动过程中终端输出 Gazebo ROS 接口插件加载完成的提示：

```
Finished loading Gazebo ROS API Plugin.
```

此时 ROS 与 Gazebo 的通信已经打通，新开终端执行 `rostopic list`，可以看到 `/gazebo/model_states`、`/gazebo/set_model_state` 等 Gazebo 接口话题：

```bash
rostopic list
```

![Gazebo 接口话题列表](../img/code/chap7/02_rostopic_list.png)

随后模型生成节点输出成功信息：

![模型生成成功日志](../img/code/chap7/04_spawn_success_log.png)

Gazebo 窗口中出现一辆阿克曼四轮小车，车头沿 x 轴正方向：

![Gazebo 中的小车](../img/code/chap7/05_car_in_gazebo.png)

### 3.3 控制器加载

启动文件中的 `controller_spawner` 节点一次性加载五个控制器：

```xml
<node name="controller_spawner" pkg="controller_manager" type="spawner" output="screen"
      args="joint_state_controller
            rear_right_velocity_controller
            rear_left_velocity_controller
            right_bridge_position_controller
            left_bridge_position_controller"/>
```

终端依次输出五个控制器的 Loaded 与 Started 信息，即表示控制器全部就绪：

```
Loaded **joint_state_controller**
Started ...
Loaded **rear_right_velocity_controller**
Loaded **rear_left_velocity_controller**
Loaded **right_bridge_position_controller**
Loaded **left_bridge_position_controller**
```

启动日志中可能出现两行红色的 `No p gain specified for pid` 提示，这是 Gazebo 中 PID 增益未单独配置时的常规提示，不影响控制器正常运行。

五个控制器全部 Started 后，对应的指令话题即已注册，可执行 `rostopic list` 查看到 `/rear_left_velocity_controller/command`、`/rear_right_velocity_controller/command`、`/left_bridge_position_controller/command`、`/right_bridge_position_controller/command` 等话题。

## 4. 前轮 TF 树补全

两个前轮的自转关节没有配置 transmission，Gazebo 不会发布这两个关节的状态。若直接使用官方 `joint_state_publisher`，它又会与 Gazebo 争抢四个主动关节的发布权，导致终端持续刷出 `TF_REPEATED_DATA ignoring data with redundant timestamp for frame ...` 警告，Rviz 中 RobotModel 的 Status 报错。

![TF_REPEATED_DATA 警告](../img/code/chap7/17_tf_repeated_warning.png)

解决方法是编写一个专用节点 `front_wheel_jsp.py`，只发布两个前轮自转关节的零值，以 50Hz 补齐 TF 树：

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('front_wheel_jsp')
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
rate = rospy.Rate(50)

msg = JointState()
msg.name = ['right_front_wheel_to_bridge', 'left_front_wheel_to_bridge']
msg.position = [0.0, 0.0]
msg.velocity = [0.0, 0.0]

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rate.sleep()
```

运行该节点：

```bash
rosrun smartcar_control front_wheel_jsp.py
```

![前轮关节状态补全节点](../img/code/chap7/18_front_wheel_jsp.png)

该节点运行后无多余输出属于正常现象，此时整车 TF 树完整。

## 5. Rviz 模型可视化

新开一个终端启动 Rviz，并加载随工程提供的配置文件：

```bash
rviz -d $(rospack find smartcar_description)/config/smartcar.rviz
```

将 Fixed Frame 设置为 `base_link`，添加 RobotModel 并选择话题 `/robot_description`。TF 树完整后，Rviz 中显示完整的四轮小车模型，Global Status 与 RobotModel 均为绿色正常状态：

![Rviz 中的小车模型](../img/code/chap7/08_rviz_robotmodel_ok.png)

配置文件中已预置 LaserScan 显示项（Color Transformer 选择 AxisColor，Size 为 0.05 m，话题为 `/laser/scan`）。若 Rviz 中未出现该显示项，通常是功能包复制时旧目录未被完全覆盖，可点击 `Add` 手动添加 `LaserScan` 并指定话题。

## 6. 键盘遥控与阿克曼运动解算

### 6.1 控制器配置

五个控制器在 `smartcar_joint.yaml` 中配置。后轮采用速度控制器，两个转向桥采用位置控制器：

```yaml
joint_state_controller:
  type: "joint_state_controller/JointStateController"
  publish_rate: 50

rear_right_velocity_controller:
  type: "velocity_controllers/JointVelocityController"
  joint: right_back_wheel_joint
  pid: {p: 100.0, i: 0.01, d: 10.0}
rear_left_velocity_controller:
  type: "velocity_controllers/JointVelocityController"
  joint: left_back_wheel_joint
  pid: {p: 100.0, i: 0.01, d: 10.0}
right_bridge_position_controller:
  type: "effort_controllers/JointPositionController"
  joint: right_bridge_to_bridge
  pid: {p: 40.0, i: 0.0, d: 1.0}
left_bridge_position_controller:
  type: "effort_controllers/JointPositionController"
  joint: left_bridge_to_bridge
  pid: {p: 40.0, i: 0.0, d: 1.0}
```

### 6.2 阿克曼运动解算

阿克曼结构的特点是转向时内外前轮转角不同、内外侧车轮线速度不同，使所有车轮处于纯滚动状态。设转向桥输入角（虚拟中心转角）为 `angle`，车速为 `speed`，轮距（左右轮间距）为 `width = 0.16 m`，前后桥轴距为 `wheelbase = 0.1885 m`。

内外前轮转角分别为：

```
outer_angle = atan(wheelbase * tan(angle) / (wheelbase + 0.5 * width  * tan(angle)))
inner_angle = atan(wheelbase * tan(angle) / (wheelbase - 0.5 * width  * tan(angle)))
```

内外侧后轮线速度分别为：

```
outside_speed = speed * (1 + 0.5 * width * tan(angle) / wheelbase)
inside_speed  = speed * (1 - 0.5 * width * tan(angle) / wheelbase)
```

直行时 `angle = 0`，内外转角均为 0，两侧轮速相等；转向时内轮转角大于外轮转角、内侧轮速小于外侧轮速，与真实阿克曼底盘一致。

### 6.3 键盘遥控节点

键盘遥控节点 `smartcar_control_gazebo.cpp` 读取键盘输入，按上述公式实时解算四个控制器的目标值。节点通过四个话题发布控制指令：

| 发布话题 | 消息类型 | 作用 |
| --- | --- | --- |
| `/rear_right_velocity_controller/command` | std_msgs/Float64 | 右后轮目标转速 |
| `/rear_left_velocity_controller/command` | std_msgs/Float64 | 左后轮目标转速 |
| `/right_bridge_position_controller/command` | std_msgs/Float64 | 右前轮转向角 |
| `/left_bridge_position_controller/command` | std_msgs/Float64 | 左前轮转向角 |

启动键盘遥控节点：

```bash
roslaunch smartcar_control smartcar_gazebo_controller.launch
```

节点启动后循环打印四个控制量以及阿克曼内外侧解算结果，待机状态下均为 0：

![键盘遥控节点待机](../img/code/chap7/06_keyboard_teleop.png)

按键功能如下（终端窗口需获取焦点，并切换为英文输入法）：

| 按键 | 功能 |
| --- | --- |
| `w` | 加速前进 |
| `s` | 减速 / 后退 |
| `e` | 前进时向左转 |
| `q` | 前进时向右转 |
| `z` | 后退时向右转 |
| `c` | 后退时向左转 |
| `x` | 转向回正 |
| 空格键 | 紧急停车 |
| `Ctrl + C` | 退出节点 |

## 7. 激光雷达

### 7.1 雷达配置

激光雷达安装在车头顶部 `laser_link`，在 `smartcar_sim.xacro` 中以 ray 传感器配置，并加载 Gazebo 激光插件：

```xml
<sensor type="ray" name="head_hokuyo_sensor">
  <visualize>true</visualize>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.10</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <stddev>0.01</stddev>
    </noise>
  </ray>
  <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
    <topicName>/laser/scan</topicName>
    <frameName>laser_link</frameName>
  </plugin>
</sensor>
```

雷达每帧 720 束，扫描范围为车头前方 180°（±π/2），量程 0.10～30.0 m，更新频率 40 Hz，测量噪声标准差 0.01 m。

### 7.2 激光自扫故障排查

模型初次运行时，Rviz 中 LaserScan 以红色方块成片叠加在车体周围，说明雷达扫描到了自身结构：

![激光自扫故障](../img/code/chap7/09_laserscan_self_hit_rviz.png)

查看数据，每帧最近回波均为雷达量程下限 0.1 m：

![最近回波 0.1m](../img/code/chap7/10_laserscan_ranges_min.png)

进一步统计每帧含数值的回波数量，共 655 条有数值回波，其余 65 束为无回波的 `inf`；滚动输出还可见大量回波为量程下限 0.1 m，说明射线成片命中了自身结构：

![有效回波统计](../img/code/chap7/11_laserscan_valid_count.png)

排查发现 `laser_link` 自身配置了碰撞体，雷达射线与该碰撞体相交。删除 `laser_link` 中的 `<collision>` 块（保留 visual 与惯性参数）：

![删除 laser_link 的 collision](../img/code/chap7/12_grep_collision_removed.png)

修改后重新加载参数统计，整车 collision 数量为 14，laser_link 块中仅保留 visual：

![collision 数量验证](../img/code/chap7/13_rosparam_collision_count.png)

同时将传感器的 `<visualize>` 由 `false` 改为 `true`（模型中共有 2 处），便于在 Gazebo 中观察射线。修改 xacro 后必须彻底关闭 Gazebo 再重新启动：

```bash
killall gzserver gzclient
roslaunch smartcar_description smartcar_gazebo.launch
```

![重启并开启射线可视化](../img/code/chap7/14_killall_visualize.png)

### 7.3 点云与频率验证

重启后 Gazebo 中射线呈前向扇形正常展开，不再扫描到车体，空旷环境中最近回波距离明显增大：

![射线扇形展开](../img/code/chap7/15_ray_fan_open.png)

查看激光话题发布频率，稳定在 40 Hz 左右（实测约 39.8 Hz）：

```bash
rostopic hz /laser/scan
```

![激光话题频率](../img/code/chap7/25_laser_hz_40.png)

## 8. 摄像头与 IMU

### 8.1 摄像头

摄像头安装在车头 `camera_link`，分辨率 800×800，更新频率 30 Hz，发布 `/camera/image_raw` 与 `/camera/camera_info` 话题。使用 rqt 图像查看工具观察画面：

```bash
rqt_image_view /camera/image_raw
```

将方块放在车头前方时，画面中可以清晰看到障碍物：

![摄像头画面](../img/code/chap7/22_camera_rqt.png)

### 8.2 IMU

IMU 安装在车体上层 `imu_link`，更新频率 100 Hz，发布 `/imu` 话题。查看发布频率，实测约 98 Hz：

```bash
rostopic hz /imu
```

![IMU 发布频率](../img/code/chap7/23_imu_hz.png)

静止状态下查看 IMU 数据，角速度各分量接近 0，线加速度以 z 轴重力分量为主（约 9.8 m/s²），姿态四元数接近单位四元数：

```bash
rostopic echo /imu
```

![IMU 数据](../img/code/chap7/24_imu_echo.png)

## 9. 障碍物检测验证

为验证激光雷达对障碍物的检测效果，在 Gazebo 中放置标准方块并观察射线与点云。

### 9.1 放置方块

点击 Gazebo 顶部工具栏的 Box 图标，在地面上单击即可放置一个边长 1 m 的方块。在左侧 World 选项卡中展开 Models，选中方块后展开 Pose，按实际位置填写 x、y、z：方块边长 1 m、贴地放置时 z 应填 0.5。每个数值输入后按回车确认：

![Gazebo Pose 面板](../img/code/chap7/19_gazebo_pose_panel.png)

将方块放到车头正前方约 2 m 处，即 x=2、y=0、z=0.5：

![车头前方 2m 的方块](../img/code/chap7/20_box_at_2m.png)

### 9.2 观察检测结果

Gazebo 中激光射线在方块后方形成明显的楔形阴影区域：

![射线楔形阴影](../img/code/chap7/16_ray_fan_box_shadow.png)

在 Rviz 中将视图切换为 TopDownOrtho 俯视模式，点云与场景中的方块一一对应：车头正前方与侧前方方块的相邻两面在点云中呈现黄色的 L 形与 Γ（反 L）形轮廓，远处方块的侧面呈现蓝色与紫色竖直点线：

![Rviz 俯视点云](../img/code/chap7/21_rviz_pointcloud_top.png)

点云轮廓与真实障碍物的位置、形状完全吻合，表明小车的运动控制与激光雷达数据链路均工作正常。

## 10. 常见问题

**Q1：启动后 Gazebo 中看不到射线或传感器没有数据？**

确认是通过 `roslaunch` 启动，而不是先手动运行 `gazebo`。手动启动的无插件实例会被 Gazebo 单例机制复用，导致 ROS 插件无法注册。先执行 `killall gzserver gzclient` 关闭所有实例，再使用启动文件。

**Q2：修改 xacro 文件后效果没有变化？**

Gazebo 不会热加载模型，必须先 `killall gzserver gzclient`，再重新执行 roslaunch。

**Q3：终端持续刷 `TF_REPEATED_DATA` 警告？**

官方 joint_state_publisher 与 Gazebo 同时发布主动关节状态。关闭该节点，改用工程提供的 `front_wheel_jsp.py` 只补全两个前轮关节。

**Q4：Insert 选项卡中的在线模型库无法加载？**

在线模型库在国内网络环境下经常无法访问，属于网络问题。本章使用 Gazebo 内置几何体（Box 等）即可完成验证，不影响实验。

![在线模型库无法访问](../img/code/chap7/07_insert_online_unavailable.png)

**Q5：Rviz 中 LaserScan 显示项没有自动出现？**

功能包复制到已存在的同名目录时可能发生目录嵌套，导致加载的仍是旧配置。删除旧目录后重新复制，或在 Rviz 中手动 `Add` → `LaserScan`，话题选择 `/laser/scan`。

**Q6：键盘节点按键后小车没有反应？**

检查运行键盘节点的终端窗口是否获取鼠标焦点，并确认系统处于英文输入法状态；待机时终端持续打印全 0 数据属于正常现象。

---

本章完成了阿克曼四轮小车从 xacro 建模、xacro 旧语法修复、ros_control 控制器配置、阿克曼运动解算与键盘遥控，到激光雷达自扫故障排查和三传感器数据验证的完整流程，全部实验结果均在 Ubuntu 20.04 + ROS Noetic 环境下复现通过，工程代码位于仓库 `src/chap7` 目录。
>>>>>>> 3059729b304e5e3fe43079d4fcefd559bbfb79a2
