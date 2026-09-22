# CARLA 0.9.16 作业一 · 无人车物理仿真 + 键盘运动控制

基于 **CARLA 0.9.16** 实现作业一：在 CARLA 中生成无人车，通过键盘实时控制其运动，并用前视相机 + pygame 显示车头画面。
可在 **Windows 原生** 或 **Ubuntu 20.04 (Noetic) / 22.04 (Humble)** 运行，支持 ROS launch 封装。

> 本分支当前提交 **作业一**（键盘控制）。作业二/三/四将后续以独立 PR 逐个提交。

## 运行环境

- 操作系统：Windows 10/11、Ubuntu 20.04 / 22.04
- CARLA 0.9.16（需启动服务端）
- Python 3.12（匹配 CARLA 0.9.16 的 cp312 wheel）
- 依赖：见 `requirements-core.txt`（numpy / opencv / pygame）

## 快速开始

```bash
# 1. 启动 CARLA 0.9.16 服务端（端口默认 2000）
./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low

# 2. 安装依赖（一次性）
pip install -r requirements-core.txt
pip install /path/to/Carla/PythonAPI/carla/dist/carla-0.9.16-*.whl

# 3. 运行作业一（弹出前视画面窗口，键盘控制）
python main.py --task control
```

## Windows 原生入口

```bat
main.bat control              :: 作业一 键盘控制（或 python main.py --task control）
main.bat control --follow      :: 让 CARLA 镜头跟随自车（便于录屏）
```

Unix 用 `bash main.sh control`。

## 操作说明

- W / ↑ = 油门，S / ↓ = 刹车，A / ← = 左转，D / → = 右转，ESC = 退出
- pygame 窗口实时显示车头相机画面与 HUD（坐标 / 速度 / 油门 / 转向 / 键位状态）

## 独立运行 / ROS launch

```bash
# 独立（需 CARLA 服务端）
python 01_control/main.py

# ROS2 Humble
ros2 launch carla_assignment 01_control.launch.py mode:=test

# ROS1 Noetic
roslaunch carla_assignment 01_control.launch mode:=test
```

## 目录结构

```
carla_assignment/
├── main.py / main.bat / main.sh  # 入口（跨平台）
├── common.py            # CARLA API 公共封装
├── 01_control/          # 作业一：键盘控制 main.py + launch/
├── docs/                # mkdocs 文档（含作业一运行截图）
└── scripts/             # 环境 / 资源工具
```

## 运行效果

见 `docs/assets/shot_control_*.png`（作业一实测截图）。
