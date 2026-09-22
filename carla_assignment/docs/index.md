# CARLA 0.9.16 作业一 · 无人车物理仿真 + 键盘运动控制

欢迎浏览《基于 **CARLA 0.9.16** 的无人车》作业一实现与文档。

## 作业一内容

在 CARLA 中物理仿真无人车，通过键盘实时控制其运动，并用前视相机 + pygame 显示车头画面：

- **键盘控制**：W/S 油门刹车、A/D 转向、方向键同效、ESC 退出。
- **实时画面**：pygame 窗口显示车头相机前视画面与 HUD（坐标 / 速度 / 油门 / 转向 / 键位）。
- **ROS 封装**：提供 ROS1 Noetic / ROS2 Humble launch 文件。

## 运行

```bash
# 1. 启动 CARLA 0.9.16 服务端（另开终端，端口默认 2000）
./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low

# 2. 安装依赖（一次性）
pip install -r requirements.txt
pip install /path/to/Carla/PythonAPI/carla/dist/carla-0.9.16-*.whl

# 3. 运行作业一（弹出前视画面窗口，键盘控制）
python main.py --task control
```

Windows 可用 `main.bat control`（详见 README）。

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
