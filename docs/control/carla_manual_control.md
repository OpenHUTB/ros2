# CARLA 手动控制：设置并连接到 CARLA 模拟器

> **Abstract (English)**: This document records a verified CARLA 0.9.14 + ROS 2 Humble
> manual-driving setup. The CARLA simulator runs on a Windows host, while the Python API
> and `carla_ros_bridge` run inside an Ubuntu 20.04 virtual machine. Note that ROS 2
> Humble officially targets Ubuntu 22.04; the combination documented here is a
> source-built, non-officially-supported setup that has been validated end to end. The
> CARLA Python API **must** be installed from the PyPI `carla==0.9.14` cp38 wheel — the
> bundled `py3.7` egg cannot be imported by Python 3.8.

中文摘要：本文记录在 Windows 宿主机 + Ubuntu 20.04 虚拟机上，用 CARLA 0.9.14 与
ROS 2 Humble 实现 `ego_vehicle` 手动控制的完整流程，并给出各版本组件的实测验证方式。

## 一、项目概述

本项目基于 **CARLA 0.9.14** 和 **ROS 2 Humble**，实现对 CARLA 模拟器中 `ego_vehicle`
的手动控制。通过 `carla_manual_control` 节点，可以用键盘控制车辆行驶、转向、刹车和倒车。

### 1.1 系统架构

- **Windows 宿主机**：运行 CARLA 0.9.14 服务器（提供 3D 渲染和物理引擎）
- **Ubuntu 20.04 虚拟机**：运行 ROS 2 Humble 和 CARLA ROS Bridge（通过网络连接 CARLA 服务器）

### 1.2 功能

- 手动控制（键盘 W/A/S/D 控制车辆）
- 中文部署文档

## 二、环境准备

### 2.1 硬件与系统

| 项目 | 配置 |
| --- | --- |
| 宿主机 | Windows 10 / 11 |
| 虚拟机 | VMware Workstation |
| 虚拟机系统 | Ubuntu 20.04.6 LTS |
| 虚拟机内存 | 8 GB 以上 |
| 虚拟机处理器 | 4 核以上 |
| 显卡 | 支持 Vulkan 的独立显卡（宿主机） |

### 2.2 软件版本

本文档描述的是一组**已实测通过**的版本组合，它并不等同于 ROS 2 的官方支持矩阵，
请勿与其他版本混搭。

| 软件 | 版本 | 说明 |
| --- | --- | --- |
| 操作系统 | Ubuntu 20.04.6 LTS | 虚拟机 |
| ROS 2 | Humble | 源码编译安装（`dpkg` 中无 `ros-humble-*` 包）。Humble 官方目标平台是 Ubuntu 22.04，本组合属**非官方支持**，仅记录实测结果 |
| Python | 3.8.10 | Ubuntu 20.04 系统自带，ROS 2 节点与 CARLA Python API 共用此解释器 |
| CARLA | 0.9.14 | Windows 服务器 + Linux Python API |
| CARLA Python API | `carla==0.9.14`（**cp38** wheel，来自 PyPI） | 不能用发行包里自带的 `py3.7` egg，见 5.9 |
| CARLA ROS Bridge | 1.3.0 | `carla-simulator/ros-bridge` |

> ⚠️ 版本一致性是这套环境最容易踩的坑：CARLA 官方 Linux 包里只带 `py3.7` 的 egg，
> 而 Ubuntu 20.04 的 `python3` 是 3.8。两者 ABI 不兼容，必须另外安装 cp38 版本。

## 三、部署流程

### 3.1 下载 CARLA 0.9.14

Windows 版（用于宿主机服务器）：

```text
https://carla-releases.s3.us-east-005.backblazeb2.com/Windows/CARLA_0.9.14.zip
```

解压到 `D:\CARLA_0.9.14\WindowsNoEditor\`。

Linux 版（用于 Ubuntu 的 Python API）：

```text
https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.14.tar.gz
```

在 Ubuntu 上解压：

```bash
cd ~
tar -xzvf CARLA_0.9.14.tar.gz
```

### 3.2 安装 CARLA ROS Bridge

1. 创建工作空间并克隆代码：

```bash
mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
git clone https://gitclone.com/github.com/carla-simulator/ros-bridge.git src/ros-bridge
```

2. 修复子模块地址：

```bash
cd ~/carla-ros-bridge/src/ros-bridge
sed -i 's|https://github.com/|https://gitclone.com/github.com/|g' .gitmodules
git submodule sync
git submodule update --init --recursive
```

3. 安装依赖并编译：

```bash
cd ~/carla-ros-bridge
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r
colcon build --packages-skip rviz_carla_plugin carla_ad_demo pcl_recorder
```

### 3.3 安装并验证 CARLA Python API

**不要**把发行包里的 `carla-0.9.14-py3.7-linux-x86_64.egg` 加进 `PYTHONPATH`，
它无法被 Python 3.8 导入。正确做法是从 PyPI 安装 cp38 的 wheel：

```bash
python3 -m pip install --user --upgrade pip
python3 -m pip install --user --no-deps carla==0.9.14
```

验证 1 —— 确认模块来自 `site-packages`：

```bash
python3 -c "import carla; print(carla.__file__)"
```

实测输出：

```text
/home/user/.local/lib/python3.8/site-packages/carla/__init__.py
```

验证 2 —— 启动 CARLA 服务器后，打印客户端/服务端版本：

```bash
python3 -c "import carla; c = carla.Client('192.168.0.3', 2000); c.set_timeout(10); print(c.get_client_version(), c.get_server_version())"
```

预期输出：

```text
0.9.14 0.9.14
```

> 注意：CARLA 0.9.14 的 Python 模块**没有** `__version__` 属性，其
> `carla/__init__.py` 只有一行 `from .libcarla import *`。因此
> `carla.__version__` 会抛 `AttributeError`，这是正常现象，不能作为验证依据，
> 请使用上面的 `carla.__file__` 或 `get_client_version()`。

安装完成后**不需要**再设置 `PYTHONPATH`。如果 `~/.bashrc` 里残留了旧的行，请一并删除：

```bash
# 删除类似下面这样的残留（已废弃的写法）
# export CARLA_ROOT=$HOME/PythonAPI
# export PYTHONPATH=$CARLA_ROOT/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg:$CARLA_ROOT/carla:$PYTHONPATH
```

### 3.4 修改 CARLA 版本检查

ROS Bridge 默认要求 CARLA 0.9.13，需要修改为 0.9.14：

```bash
echo "0.9.14" > ~/carla-ros-bridge/src/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION
cd ~/carla-ros-bridge
colcon build --packages-select carla_ros_bridge
```

### 3.5 修改 ego_vehicle 车型

编辑 `objects.json`，把 `ego_vehicle` 的车型从 `vehicle.tesla.model3` 改成 `vehicle.audi.a2`：

```bash
nano ~/carla-ros-bridge/src/ros-bridge/carla_spawn_objects/config/objects.json
```

找到：

```json
"type": "vehicle.tesla.model3",
"id": "ego_vehicle",
```

改成：

```json
"type": "vehicle.audi.a2",
"id": "ego_vehicle",
```

重新编译：

```bash
cd ~/carla-ros-bridge
colcon build --packages-select carla_spawn_objects
```

### 3.6 配置 VMware 网络

1. 在 VMware 里设置桥接模式：

   `虚拟机 → 设置 → 网络适配器 → 桥接模式：直接连接物理网络`，并勾选「复制物理网络连接状态」。

2. 在 VMware 虚拟网络编辑器里，把 VMnet0 桥接到 WLAN 网卡：

   `编辑 → 虚拟网络编辑器 → 更改设置 → 选中 VMnet0 → 桥接到实际的无线网卡`。

3. 在 Ubuntu 里获取 IP：

```bash
sudo dhclient ens33
ip addr show ens33 | grep "inet "
```

4. 测试连接 Windows 上的 CARLA（`192.168.0.3` 换成宿主机当前 IP）：

```bash
nc -zv 192.168.0.3 2000
```

显示 `succeeded` 即可。

## 四、启动流程

### 4.1 启动 Windows 上的 CARLA 服务器

进入 `D:\CARLA_0.9.14\WindowsNoEditor\`，双击 `CarlaUE4.exe`，等 1–2 分钟让它完全加载。

确认监听 2000 端口：

```cmd
netstat -ano | findstr 2000
```

应该只有一行 `LISTENING`。

### 4.2 启动 ROS Bridge（终端 1）

```bash
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_ETC_DIR ROS_PACKAGE_PATH ROS_ROOT ROS_MASTER_URI PKG_CONFIG_PATH
export PATH=$(echo $PATH | tr ':' '\n' | grep -v noetic | grep -v catkin_ws | tr '\n' ':')
export PATH=${PATH%:}
export LD_LIBRARY_PATH=/opt/ros/humble/lib
source /opt/ros/humble/setup.bash
source ~/carla-ros-bridge/install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py host:=192.168.0.3 port:=2000 timeout:=60
```

### 4.3 启动 spawn objects 生成 ego_vehicle（终端 2）

```bash
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_ETC_DIR ROS_PACKAGE_PATH ROS_ROOT ROS_MASTER_URI PKG_CONFIG_PATH
export PATH=$(echo $PATH | tr ':' '\n' | grep -v noetic | grep -v catkin_ws | tr '\n' ':')
export PATH=${PATH%:}
export LD_LIBRARY_PATH=/opt/ros/humble/lib
source /opt/ros/humble/setup.bash
source ~/carla-ros-bridge/install/setup.bash
ros2 launch carla_spawn_objects carla_spawn_objects.launch.py host:=192.168.0.3 port:=2000
```

等它输出 `All objects spawned`。

### 4.4 启动手动控制（终端 3）

```bash
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_ETC_DIR ROS_PACKAGE_PATH ROS_ROOT ROS_MASTER_URI PKG_CONFIG_PATH
export PATH=$(echo $PATH | tr ':' '\n' | grep -v noetic | grep -v catkin_ws | tr '\n' ':')
export PATH=${PATH%:}
export LD_LIBRARY_PATH=/opt/ros/humble/lib
source /opt/ros/humble/setup.bash
source ~/carla-ros-bridge/install/setup.bash
ros2 run carla_manual_control carla_manual_control
```

### 4.5 开车操作

| 按键 | 功能 |
| --- | --- |
| B | 激活 / 关闭手动驾驶 |
| W | 油门 |
| S | 刹车（车停后再按可倒车） |
| A | 左转 |
| D | 右转 |
| 空格 | 手刹 |
| P | 切换自动驾驶 |
| H | 查看帮助 |

## 五、遇到的问题与解决方案

### 5.1 VMware Vulkan 支持不足

**问题**：CARLA 在 VMware 虚拟机里运行时报 `GameThread timed out waiting for RenderThread`，最终段错误崩溃。

**解决**：把 CARLA 服务器放在 Windows 宿主机上运行，Ubuntu 虚拟机通过网络连接。

### 5.2 ROS 1 Noetic 与 ROS 2 Humble 环境混用

**问题**：ROS Bridge 启动时报 `AttributeError: type object 'type' has no attribute '_TYPE_SUPPORT'`。

**解决**：在每个终端启动前，执行 `unset` 清理所有 ROS 相关变量，然后只 `source /opt/ros/humble/setup.bash`。同时修复 `carla-ros-bridge/install/setup.sh` 里写死的 `/opt/ros/noetic` 路径。

### 5.3 版本不匹配

**问题**：ROS Bridge 报 `CARLA python module version 0.9.13 required. Found: 0.9.14`。

**解决**：修改 `carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION` 文件，把 `0.9.13` 改成 `0.9.14`，重新编译。

### 5.4 依赖缺失

**问题**：编译时报 `ModuleNotFoundError: No module named 'derived_object_msgs'`。

**解决**：

```bash
cd ~/carla-ros-bridge/src
git clone https://gitclone.com/github.com/astuff/astuff_sensor_msgs.git
cd ~/carla-ros-bridge
colcon build --packages-select derived_object_msgs
```

### 5.5 tf2 库冲突

**问题**：报 `undefined symbol: _ZN3tf210BufferCore15allFramesAsDotB5cxx11Ev`。

**解决**：

```bash
sudo apt remove ros-noetic-tf2 ros-noetic-tf2-eigen ros-noetic-tf2-geometry-msgs ros-noetic-tf2-kdl ros-noetic-tf2-msgs ros-noetic-tf2-py ros-noetic-tf2-ros -y
```

### 5.6 Foxy 与 Humble 库冲突

**问题**：报 `undefined symbol: rcutils_expand_user`。

**解决**：

```bash
sudo mv /opt/ros/foxy/lib /opt/ros/foxy/lib.bak
```

### 5.7 VMware 桥接网络问题

**问题**：Ubuntu 无法连接 Windows 上的 CARLA，报 `Network is unreachable`。

**解决**：在 VMware 虚拟网络编辑器里，把 VMnet0 桥接到实际的 WLAN 网卡。

### 5.8 车辆卡在护栏上

**问题**：车辆生成后卡在护栏上，全油门也动不了。

**解决**：用 Python 脚本把车辆瞬移到空旷的生成点：

```python
import carla

client = carla.Client('192.168.0.3', 2000)
client.set_timeout(30)
world = client.get_world()
vehicles = world.get_actors().filter('vehicle.*')
for v in vehicles:
    if v.attributes.get('role_name') == 'ego_vehicle':
        spawn_points = world.get_map().get_spawn_points()
        for sp in spawn_points:
            if 0.2 < sp.location.z < 0.5:
                v.set_transform(sp)
                break
        break
```

### 5.9 `import carla` 成功但用不了（py3.7 egg 与 Python 3.8）

**问题**：按官方文档把 `carla-0.9.14-py3.7-linux-x86_64.egg` 加进 `PYTHONPATH` 后，
`python3 -c "import carla"` 不报错，但后面所有调用都失败，或报
`AttributeError: module 'carla' has no attribute ...`。

**原因**：Ubuntu 20.04 的 `python3` 是 3.8，而该 egg 是为 CPython 3.7 编译的，不能跨版本加载。
此时 `PYTHONPATH` 里的 `~/PythonAPI/carla` 目录会被 Python 当成 **namespace package**，
于是 `import carla` "成功"了，但里面是空壳——这也是为什么这个错误很容易被误判为"装好了"。

**解决**：改用 PyPI 上的 cp38 wheel（见 3.3），并删掉 `PYTHONPATH` 里的 egg 路径。验证方式是打印
`carla.__file__`，它应当指向 `site-packages` 而不是 `~/PythonAPI/carla`。

## 六、运行效果

以下是实际运行截图：

<img width="1368" height="940" alt="a6f7f79a6f7ad495036d0edf228eea8e" src="https://github.com/user-attachments/assets/c4878df1-c34c-4dc9-adff-c468d3f5b4d5" />
<img width="1368" height="940" alt="6f1c034cd4ced47e1bab9c74e9f7c8ff" src="https://github.com/user-attachments/assets/2df782ae-d37c-4090-83c0-96b13d504754" />
<img width="1368" height="940" alt="04925efc8308b365169c7fad3c7014f4" src="https://github.com/user-attachments/assets/3aa3b9d7-9915-4ef4-ae78-51841b5b5752" />
<img width="1368" height="940" alt="465a5d65260905200b94ea33eb1a2018" src="https://github.com/user-attachments/assets/c5d83adf-38f8-47a3-97e3-0bff78362fda" />
<img width="1227" height="789" alt="c86dfd4297d1fa23c96555ff11376c43" src="https://github.com/user-attachments/assets/f71978dd-864b-4d38-8f8e-3a9fe21a6add" />

## 七、总结

| 项目 | 状态 |
| --- | --- |
| ROS Bridge 编译 | ✅ |
| CARLA 服务器运行 | ✅ |
| ego_vehicle 生成 | ✅ |
| 手动控制开车 | ✅ |
| 中文部署文档 | ✅ |

文档作者：jasmine055
日期：2026-10-16
