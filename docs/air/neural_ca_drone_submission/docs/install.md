# 环境安装

## 1. 依赖一览

| 用途 | 依赖 | 版本要求 | 说明 |
| --- | --- | --- | --- |
| 核心仿真 / 网络 / 训练 | `numpy` | ≥ 1.20 | 全部数值计算 |
| 图像渲染 / GIF | `Pillow` | ≥ 7.0 | 画家算法、HUD、量化 |
| 结果图表 | `matplotlib` | ≥ 3.1 | 感知三联图、规划对比、训练曲线 |
| 文档 | `mkdocs`、`mkdocs-material` | mkdocs ≥ 1.4 | 本文档 |
| ROS1 | ROS Noetic | Ubuntu 20.04 | `catkin_make` |
| ROS2 | ROS Humble | Ubuntu 22.04 / 20.04 | `colcon build` |

!!! note "为什么不用 PyTorch"
    目标机器无 GPU、无外网，安装 PyTorch 不现实。项目自带
    `ca_drone/nn/`（自动微分、卷积、池化、Adam），因此**只依赖 NumPy** 即可训练与推理。
    如果你有 PyTorch，也可以把 `ca_drone/models/*.py` 的结构照搬到 PyTorch 中，
    权重形状一一对应。

## 2. 通用安装（仿真 / 训练 / 评测）

```bash
git clone <your-repo-url> neural_ca_drone
cd neural_ca_drone

python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

`requirements.txt` 内容：

```text
numpy>=1.20
Pillow>=7.0
matplotlib>=3.1
mkdocs>=1.4
mkdocs-material>=9.0
```

验证：

```bash
python -c "import numpy, PIL, matplotlib; print(numpy.__version__, PIL.__version__)"
python main.py --module sim --scene sparse
```

第二条命令应输出 `saved .../runs/<timestamp>_sim/frame_sparse.png`。

## 3. ROS1 Noetic（Ubuntu 20.04）

```bash
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
cd ros1_ws
catkin_make
source devel/setup.bash
roslaunch neural_ca_drone_ros main.launch module:=full scene:=sparse
```

## 4. ROS2 Humble（Ubuntu 22.04 / 20.04）

```bash
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select neural_ca_drone_ros2
source install/setup.bash
ros2 launch neural_ca_drone_ros2 main.launch.py module:=full scene:=sparse
```

## 5. 文档构建

```bash
pip install mkdocs mkdocs-material
mkdocs serve          # http://127.0.0.1:8000
mkdocs build --strict # 产物在 site/
```

## 6. 目录约定

| 路径 | 是否入库 | 说明 |
| --- | --- | --- |
| `ca_drone/`、`main.py`、`tools/` | ✅ | 全部源码 |
| `weights/*.npz` | ✅ | 训练好的权重（约 1.4 MB） |
| `data/*.npz` | ❌ | 数据集可再生成（约 20 MB），`.gitignore` 排除 |
| `runs/`、`site/` | ❌ | 运行产物 |
| `docs/assets/` | ✅ | 文档图片与 GIF |

重新生成数据集：

```bash
python tools/make_data.py --out data
```

## 7. 常见安装问题

??? question "Pillow 版本太旧，报 `rounded_rectangle` 不存在"
    Pillow < 8.2 没有圆角矩形。项目已做兼容（`_rrect` 回退为直角矩形），
    无需升级；若想更好看可 `pip install -U Pillow`。

??? question "Python 3.8 报 `unsupported operand type(s) for |`"
    这是 PEP 604 的联合类型语法（3.10+）。项目已改为 `typing.Union`，
    请拉取最新代码。

??? question "ROS2 `ros2 topic echo` 报 `!rclpy.ok()`"
    某些精简版 Humble 的 CLI daemon 不可用，与本项目无关。
    使用 `ros2_ws/src/neural_ca_drone_ros2/scripts/probe.py` 直接以 rclpy 订阅验证。
