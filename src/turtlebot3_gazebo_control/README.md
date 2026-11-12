# Turtlebot3 Gazebo 键盘控制仿真

## 一、项目介绍
本模块在 Ubuntu 20.04 + ROS Noetic 环境下，利用 Gazebo 11 物理仿真器搭建 Turtlebot3 机器人环境。通过 Python 脚本发布 `/cmd_vel` 话题，实现**基于键盘速度指令的控制**

## 二、运行环境
| 配置项 | 版本/说明 |
| :--- | :--- |
| 操作系统 | Ubuntu 20.04 (VMware 虚拟机) |
| ROS 版本 | ROS Noetic |
| 编程语言 | Python 3.8 |
| 仿真器 | Gazebo 11 |
| 机器人型号 | Turtlebot3 (Waffle) |

## 三、获取源码到 catkin_make 的完整步骤

### 1. 获取源码并编译
打开终端，进入 ROS 工作空间的 `src` 目录：
```bash
cd ~/catkin_ws/src
```

克隆仓库：

```bash
git clone <https://github.com/Chen-pjun/ros2>
```

返回工作空间根目录编译并刷新环境：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 安装依赖并启动仿真

```bash
# 安装 xterm 用于提供键盘交互终端
sudo apt install xterm

# 设置环境变量，防止虚拟机渲染闪退
export TURTLEBOT3_MODEL=waffle
export LIBGL_ALWAYS_SOFTWARE=1

# 统一入口启动，同时启动 Gazebo 和键盘控制交互终端
roslaunch turtlebot3_gazebo_control main.launch
```

### 3. 键盘操作说明

将光标聚焦在 xterm 弹出的交互终端窗口，使用以下按键控制机器人：

· w：前进（增加线速度）
· x：后退（减小线速度）
· a：向左转（增加角速度）
· d：向右转（减小角速度）
· s 或 空格：停止
· Ctrl + C：退出键盘控制节点

```