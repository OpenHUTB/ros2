# 环境安装向导

从零配置 CARLA 0.9.16 + Python + ROS 环境。

## 1. 需求

| 组件 | 版本 |
|---|---|
| 操作系统 | Ubuntu 20.04（ROS1 Noetic）或 22.04（ROS2 Humble） |
| CARLA | **0.9.16**（0.9.x 新版接口大体兼容） |
| Python | 3.8 以上（carla Python API 对版本要求宽） |
| 显卡 | **建议 NVIDIA**（CARLA 3D 渲染对 GPU 有要求） |
| ROS | Noetic 或 Humble（可选，仅用于 launch） |

> ⚠️ **无独显虚拟机跑 CARLA 会非常卡**（3D 渲染负担大）。
> 无显卡时请改用同仓库 `mujoco_assignment`（MuJoCo 方案），对显卡无硬性要求。

## 2. 安装 CARLA 0.9.16

1. 下载 CARLA 0.9.16 发行包（约 20GB，含地图与 Python API）。
2. 解压后目录结构大致为：
   ```
   CarlaUE4/
   ├── CarlaUE4.sh            # 服务端启动脚本
   └── PythonAPI/
       └── carla/dist/        # carla-0.9.16-*.whl（Python API）
   ```
3. 安装 Python API：
   ```bash
   pip install /path/to/Carla/PythonAPI/carla/dist/carla-*.whl
   # 或
   pip install carla==0.9.16
   ```

## 3. 一键安装本包依赖

```bash
cd carla_assignment
bash scripts/setup_env.sh
```
脚本会创建 `.venv` 并安装 numpy / opencv / pygame / tensorflow-cpu / carla。

> 💡 **只需跑作业（无需 tf/gpu）时，装最小依赖更快（几秒）**：
> ```bash
> pip install -r requirements-core.txt          # numpy + opencv + pygame，无需 big TF 包
> pip install <CARLA>/PythonAPI/carla/dist/carla-*.whl
> ```
> 作业一仅需 numpy / opencv / pygame，**不需要 tensorflow**。

## 4. 手动安装（等价命令）

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy opencv-python-headless pygame tensorflow-cpu carla==0.9.16
```

## 5. 验证

```bash
source .venv/bin/activate
python -c "import carla; print(carla.__file__)"
```

## 5.1 Windows 原生安装

本包支持 **Windows 10/11 原生运行**（无需 WSL / Docker）：

```bat
:: cmd
cd carla_assignment
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements-core.txt
:: carla 模块从 CARLA 发行包 PythonAPI 安装
pip install <CARLA>/PythonAPI/carla/dist/carla-0.9.16-*.whl
```

运行（`main.bat`）：`main.bat control`。

> 注意：CARLA 服务端对 GPU 有要求；若虚拟机 / 无独显跑起来很卡，请改用同仓库
> [MuJoCo 方案](../mujoco_assignment)。


## 6. 启动 CARLA 服务端

```bash
# 窗口模式（有 GPU）
./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low

# Headless（带 EGL 的 Linux）
# ./CarlaUE4.sh -carla-rpc-port=2000 -RenderOffScreen
```

首次启动会加载默认地图，再运行本包作业时会通过 `client.load_world("Town05")`
自动切换地图。

## 7. ROS（可选）

本包四个作业都可用纯 `python main.py` 运行，不必依赖 ROS。若老师要求 launch 启动：
- **Ubuntu 22.04 + Humble**：用 `launch/*.launch.py`。
- **Ubuntu 20.04 + Noetic**：用 `launch/*.launch`（CARLA ros-bridge 跑在 Noetic）。
- launch 本质只是调起对应 `main.py`，因此无需 `colcon build` 亦可运行。

## 8. 常见问题

| 现象 | 解决 |
|---|---|
| `ModuleNotFoundError: No module named 'carla'` | 安装 carla Python API，见第 2 节 |
| 连接超时 | 先启动 CarlaUE4.sh，端口默认 2000 |
| 画面很卡 | 无 GPU；改用 mujoco_assignment 方案 |
| `ros2 launch` 找不到包 | `source /opt/ros/humble/setup.bash` 后再 launch |
