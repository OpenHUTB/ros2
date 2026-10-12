# Turtlebot3 ROS1自主避障仿真

## 一、作业背景与目标
使用有限状态机 FSM 与状态设计模式，实现Turtlebot3 自主避障项目。
## 二、仓库功能理解
* 机器人使用 Roomba 式随机行走策略：直行直到检测到障碍物，然后原地旋转寻找新方向，找到空旷路径后继续直行。
* 使用激光雷达扫描前方 90° 弧区来检测障碍物。
* 核心参数：前进速度 0.5 m/s，旋转速度 ±0.3 rad/s，障碍物检测阈值 0.8m。
* 代码采用状态设计模式，包含两个状态类：`ForwardState`（前进）和 `RotatingState`（旋转）。

## 三、环境准备

### 3.1 操作系统与 ROS1 安装
ROS1 推荐使用 Ubuntu 20.04 + ROS Noetic。


### 3.2 安装 Turtlebot3 相关包
```bash
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-gazebo
```
设置 Turtlebot3 模型：
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### 3.3 验证 Gazebo 和 Turtlebot3 仿真
```bash
# 终端1：启动 Gazebo 仿真环境
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
若 Gazebo 闪退，可尝试设置以下环境变量：
```bash
export SVGA_VGPU10=0
export MESA_GL_VERSION_OVERRIDE=3.3
```

## 四、创建 ROS1 工作空间和包

### 4.1 创建工作空间
```bash
mkdir -p ~/ros1_walker_ws/src
cd ~/ros1_walker_ws/src
```

### 4.2 创建 ROS1 包
```bash
catkin_create_pkg walker rospy roscpp std_msgs geometry_msgs sensor_msgs nav_msgs
```

### 4.3 包结构规划
```text
walker/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── walker.launch
├── scripts/
│   └── walker_node.py
└── src/
    └── walker_node.cpp
```

## 五、ROS2 → ROS1 代码移植

### 5.1 编写 python 文件
在 `scripts/walker_node.py` 中编写 ROS1 版本的避障节点


### 5.2 赋予执行权限
进入scripts文件夹
```bash
cd ~/ros1_walker_ws/src/walker/scripts
```

给python脚本添加执行权限
```bash
chmod +x walker_node.py
```

## 六、启动文件与编译

### 6.1 创建 launch 文件
在 `launch/walker.launch` 中


### 6.2 编译工作空间
```bash
cd ~/ros1_walker_ws
catkin_make
source devel/setup.bash
```

### 6.3 运行节点
```bash
roslaunch walker walker.launch
```

## 七、测试与验证
* 观察 Gazebo 中的 Turtlebot3 是否开始自主移动。
* 当机器人靠近障碍物时，是否切换为旋转状态，并原地旋转寻找新方向。
* 当路径通畅后，是否恢复前进状态。

```
