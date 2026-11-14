# Docker / 虚拟机 / Windows 原生

## Docker 运行（控制节点容器化，仿真器在宿主机）

仓库根目录已提供 `Dockerfile`（基于 `osrf/ros:noetic-desktop-full`）。

```bash
# 1) 先在 Windows/Ubuntu 宿主机启动 AirSim 仿真器
# 2) 构建并运行容器，把宿主机网络暴露给它
docker build -t airsim_controller .
docker run --rm -it \
  --add-host=host.docker.internal:host-gateway \
  -e AIRSIM_HOST=host.docker.internal \
  airsim_controller bash

# 3) 容器内运行
python3 -m modules.task1_keyboard.main
```

> 说明：AirSim 仿真器是带 GPU/图形界面的 Unreal 可执行文件，不放进容器；容器只跑
> Python 控制与 ROS 节点，通过 41451 端口连接宿主机仿真器。

## 虚拟机（VMware / VirtualBox）

1. 在 Windows 上安装 Ubuntu 20.04 虚拟机，分配 ≥8 GB 内存、2 核 CPU；
2. 虚拟机内无法流畅渲染 Unreal，因此**仿真器仍跑在 Windows 宿主机**，
   虚拟机内只跑控制节点，网络设为「桥接模式」，并把 `AIRSIM_HOST` 指向宿主机 IP；
3. 若一定要在虚拟机内渲染，需启用 VMware 的 3D 加速并使用低画质地图。

## Windows 原生

无需 ROS，直接在 PowerShell 中：

```powershell
python -m venv .venv; .\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
.\scripts\main.bat keyboard
```

键盘依赖 `pynput`（已在 requirements 中）；任务3 的 SLAM/move_base 在 Windows 上不可用，
需切到 Ubuntu。

## 三种方式对照表

| 方式 | 任务1 | 任务2 | 任务3(SLAM) | 任务4 | 备注 |
| --- | --- | --- | --- | --- | --- |
| Windows 原生 | ✅ | ✅ | ❌ | ✅ | 最省心 |
| Ubuntu 原生 | ✅ | ✅ | ✅ | ✅ | 推荐 |
| Docker | ✅ | ✅ | ✅* | ✅ | *SLAM 需挂图形 X11 |
| 虚拟机 | ✅ | ✅ | ✅* | ✅ | *网络桥接，仿真器在宿主机 |
