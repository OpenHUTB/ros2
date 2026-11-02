# OpenHUTB AIR 模式参数与端口速查

> 整理自 OpenHUTB 官方文档 (openhutb.github.io) 与 Carla-Air 快速上手指南, 供无人机仿真实验查阅。

## 1. 启动方式

```bash
# AIR 无人机模式 (2.3.0+ 默认同时支持 Carla 模式与 AirSim 模式)
python config.py --map Town10HD?GAME=AIR

# Carla 无人车模式 (对比)
python config.py --map Town10HD

# VR 座舱模式
python config.py --map Town10HD?GAME=VR
```

## 2. 常用端口

| 端口 | 用途 |
| --- | --- |
| 2000 / 2001 | Carla 模式 Python API (RPC) |
| 41451 | AirSim 模式 MultirotorClient (RPC, 无人机) |
| 41452 | AirSim 第二台载具 (多机仿真时递增) |

连接示例:

```python
import airsim
client = airsim.MultirotorClient(ip="127.0.0.1", port=41451)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync()
```

## 3. 键盘控制 (AIR 模式, 点击窗口捕获鼠标后)

| 按键 | 功能 | 按键 | 功能 |
| --- | --- | --- | --- |
| `W` / `A` / `S` / `D` | 前/左/后/右 | `Tab` | 释放/捕获鼠标 |
| `Space` / 左`Shift` | 上升 / 下降 | `1` / `2` / `3` | 传感器画面 |
| 鼠标 | 偏航转向 | `I` | 第一人称/默认视角 |
| 滚轮 | 调节飞行速度 | `B` | FPV 模式 |
| `N` | 切换天气 | `P` | 碰撞模式切换 |
| `H` | 帮助菜单 | | |

## 4. MultirotorClient 常用 API

| API | 说明 |
| --- | --- |
| `takeoffAsync()` | 异步起飞 (非阻塞) |
| `landAsync()` | 降落 |
| `hoverAsync()` | 悬停 (失控保护常用) |
| `moveToPositionAsync(x, y, z, v)` | 绝对位置移动 (NED, 速度 m/s) |
| `moveByVelocityAsync(vx, vy, vz, dur)` | 机体速度控制 (NED, YawMode 可选) |
| `getMultirotorState()` | 状态: 位置/速度/姿态 (kinematics_estimated) |
| `simGetCollisionInfo()` | 碰撞检测 (has_collided, position) |
| `armDisarm(True)` | 解锁电机 |
| `enableApiControl(True)` | 接管控制权 (与手动模式互斥) |

## 5. 坐标系提醒

- 模拟器内部为 **NED** (x 北, y 东, z 下, 偏航顺时针为正)；
- ROS 2 为 **ENU** (x 东, y 北, z 上, 偏航逆时针为正)；
- 位置转换: `x_ENU = x_NED, y_ENU = -y_NED, z_ENU = -z_NED`；
- 详见仓库模块 [ned_enu_transform](./ned_enu_transform.md)。

## 6. 硬件要求 (官方建议)

| 项 | 最低建议 |
| --- | --- |
| CPU | Intel i7 (9-11 代) / AMD Ryzen 7+ |
| 内存 | 16 GB+ |
| 显卡 | NVIDIA RTX 2070+ |
| 系统 | Windows 10+ / Ubuntu 18.04+ / macOS 12+ |

## 7. 大模型使用声明

本文档使用大模型工具（WorkBuddy）辅助整理，内容对照 OpenHUTB 官方文档 (openhutb.github.io/air_doc/dev/Carla_Air/) 核实，作者对提交内容负全部责任。
