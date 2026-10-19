# CARLA 停车场场景生成

## 概述

在 CARLA 0.9.16 仿真器的 Town04 地图中，通过 Python API 自动生成一个带车位线的停车场场景，包含 6 个垂直停车位，并自动将镜头移动到停车场上方俯视。

## 环境要求

- CARLA 0.9.16（服务端运行于 Windows 有 GPU 的宿主机）
- Ubuntu 22.04 + ROS 2 Humble + Python 3.10
- Python 依赖：`carla==0.9.16`

## 安装步骤

### 1. 下载安装 Windows 版 CARLA

从官方 Release 页面下载 CARLA 0.9.16 Windows 版：
[https://github.com/carla-simulator/carla/releases/tag/0.9.16](https://github.com/carla-simulator/carla/releases/tag/0.9.16)
下载 `CARLA_0.9.16.zip`，解压到任意目录（记为 CARLA 安装目录）。

### 2. 启动 CARLA 服务端

```
CarlaUE4.exe -quality-level=Low
```

### 3. 配置 Windows 防火墙

在 Windows Defender 防火墙中添加入站规则，放行 TCP 2000 和 UDP 2000 端口。

### 4. 编译 ROS2 包

```
cd ~/your_workspace
colcon build --packages-select carla_parking_lot
source install/setup.bash
```

## 运行

```
# 默认连接 192.168.43.1:2000
ros2 launch carla_parking_lot parking_lot.launch.py

# 指定 CARLA 服务器 IP
ros2 launch carla_parking_lot parking_lot.launch.py host:=192.168.1.100
```

## 运行效果

- 自动切换到 Town04 地图
- 在停车场位置绘制 6 个绿色车位线框
- 标注 P1-P6 车位编号
- 镜头自动跳到停车场上方俯视

## 技术说明

- 使用 `world.debug.draw_line()` 绘制车位边界线
- 使用 `world.debug.draw_string()` 标注车位编号
- 使用 `world.get_spectator()` 获取并移动镜头
- 车位坐标通过在 CARLA 窗口中飞镜头手动读取获得

## 目录结构

```
carla_parking_lot/
├── carla_parking_lot/
│   ├── __init__.py
│   └── parking_lot_node.py
├── launch/
│   └── parking_lot.launch.py
├── resource/
│   └── carla_parking_lot
├── package.xml
├── setup.py
├── main.sh
└── README.md
```

## 声明

本项目使用了大模型辅助编写代码。
