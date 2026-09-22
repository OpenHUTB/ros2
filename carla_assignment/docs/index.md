# CARLA 0.9.16 作业一 · 无人车物理仿真 + 键盘运动控制

欢迎浏览《基于 **CARLA 0.9.16** 的无人车》作业一实现与文档。

> 本分支当前交付**作业一**（键盘运动控制，对应老师任务 1）。
> 作业二（感知+轨迹）、作业三（建图+导航）、作业四（端到端 CNN）后续以独立 PR 逐个提交。

## 作业导航

- [**作业一 · 键盘控制**](01_control.md)：在 CARLA 中物理仿真无人车，通过键盘实时运动控制（任务 1）。
- [**环境安装**](setup_guide.md)：从零配置 CARLA 0.9.16 + Python 环境。

## 运行

```bash
# 1. 启动 CARLA 0.9.16 服务端（另开终端，端口默认 2000）
./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low

# 2. 安装依赖（一次性）
pip install -r requirements-core.txt
pip install /path/to/Carla/PythonAPI/carla/dist/carla-0.9.16-*.whl

# 3. 运行作业一（弹出前视画面窗口，键盘控制；ESC 退出）
python main.py --task control
```

Windows 可用 `main.bat control`（详见 [README](../README.md)）。

## 操作说明

- W / ↑ = 油门，S / ↓ = 刹车，A / ← = 左转，D / → = 右转，ESC = 退出
- pygame 窗口实时显示车头相机画面与 HUD（坐标 / 速度 / 油门 / 转向 / 键位状态）

## 运行效果

见 `assets/shot_control_*.png`（作业一实测截图）。

## 目录结构

```
carla_assignment/
├── main.py / main.bat / main.sh  # 入口（跨平台）
├── common.py            # CARLA API 公共封装
├── 01_control/          # 作业一：键盘控制 main.py + launch/
├── docs/                # 本 mkdocs 文档
└── scripts/             # 环境 / 资源工具
```
