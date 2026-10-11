---
title: 步骤 1：环境检查与 Gazebo 验证
---

# 步骤 1：环境检查与 Gazebo 验证

> 前置条件：已安装 Ubuntu 20.04，具备终端和 `sudo` 权限。
> 本步目标：确认 ROS Noetic、Gazebo Classic 11 和 `gazebo_ros` 可用于本教程。
> 预计时间：20-40 分钟。

- [1. 检查操作系统](#os)
- [2. 检查 ROS Noetic](#ros)
- [3. 检查 Gazebo](#gazebo)
- [4. 安装缺失依赖](#install)
- [5. 启动空世界](#empty-world)
- [6. 常见问题](#debug)
- [7. 本步验收](#acceptance)

## 1. 检查操作系统 <span id="os"></span>

```bash
lsb_release -a
```

预期输出：

```text
Distributor ID: Ubuntu
Description:    Ubuntu 20.04.x LTS
Release:        20.04
Codename:       focal
```

本教程使用 ROS Noetic + Gazebo Classic 11。Ubuntu 22.04 需要额外处理 Noetic 软件源，不建议初学者直接使用。

## 2. 检查 ROS Noetic <span id="ros"></span>

先加载 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
echo $ROS_DISTRO
rosversion -d
```

应分别输出：

```text
noetic
noetic
```

再检查常用命令：

```bash
which roscore
which roslaunch
which rosrun
```

如果命令不存在，需要先安装并正确配置 ROS Noetic。

## 3. 检查 Gazebo <span id="gazebo"></span>

检查 Gazebo 版本：

```bash
gazebo --version
```

应看到 Gazebo 11 系列版本信息。

检查 ROS Gazebo 包：

```bash
rospack find gazebo_ros
```

如果正确安装，会输出类似：

```text
/opt/ros/noetic/share/gazebo_ros
```

再检查本教程需要的插件库是否存在：

```bash
find /opt/ros/noetic -name "libgazebo_ros_diff_drive.so"
find /opt/ros/noetic -name "libgazebo_ros_laser.so"
find /opt/ros/noetic -name "libgazebo_ros_camera.so"
```

三个库都应能找到。找不到时，优先补装 `gazebo_ros` 包。部分 ROS Noetic 环境只提供 `libgazebo_ros_laser.so`，不提供 `libgazebo_ros_ray_sensor.so`，本教程按仓库现有插件版本使用前者。

## 4. 安装缺失依赖 <span id="install"></span>

```bash
sudo apt update
sudo apt install \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-xacro \
  ros-noetic-robot-state-publisher \
  ros-noetic-rqt-graph \
  ros-noetic-rqt-tf-tree \
  ros-noetic-rqt-image-view
```

安装完成后重新加载环境：

```bash
source /opt/ros/noetic/setup.bash
```

## 5. 启动空世界 <span id="empty-world"></span>

```bash
roslaunch gazebo_ros empty_world.launch
```

成功现象：

- Gazebo 窗口打开；
- 世界中出现地面和太阳；
- 终端没有持续报插件加载失败；
- 另开终端执行下列命令能看到 Gazebo 相关节点和话题。

```bash
rosnode list
rostopic list | grep -E "clock|gazebo|tf"
```

停止时在启动终端按 `Ctrl+C`。

## 6. 常见问题 <span id="debug"></span>

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `gazebo: command not found` | Gazebo 未安装或未加载环境 | 安装 `gazebo11` 和 `ros-noetic-gazebo-ros-pkgs` |
| `rospack find gazebo_ros` 找不到 | 未安装 Gazebo ROS 包 | 补装 `ros-noetic-gazebo-ros-pkgs` |
| Gazebo 窗口打不开 | 虚拟机显卡或 OpenGL 问题 | 开启 3D 加速，或先使用 `gui:=false` 测试 |
| ROS 命令找不到 | 未 source 环境 | `source /opt/ros/noetic/setup.bash` |
| 找不到插件 `.so` | ROS 包安装不完整 | 重新安装 Gazebo ROS 相关包 |

## 7. 本步验收 <span id="acceptance"></span>

- [ ] `echo $ROS_DISTRO` 输出 `noetic`；
- [ ] `gazebo --version` 显示 Gazebo 11；
- [ ] `rospack find gazebo_ros` 能找到路径；
- [ ] `roslaunch gazebo_ros empty_world.launch` 能打开空世界；
- [ ] 能解释 Roslaunch、Gazebo 和 ROS 节点之间的关系。

---

- 返回[教程总览](gazebo_platform_simulation.md)
- 下一步：[步骤 2：工作空间与功能包](gazebo_02_package.md)
