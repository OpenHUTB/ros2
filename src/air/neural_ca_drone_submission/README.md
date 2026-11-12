# Neural CA Drone · 基于神经网络的无人机避障

> 低空经济 2502 · 陈康艺 · 2535030044
> 感知 → 规划 → 控制 → 端到端，四个模块全部为神经网络；纯 NumPy 实现，CPU 可训练、可复现。

![demo](docs/assets/gifs/full_corridor.gif)

*corridor 场景：深度图 → 在线栅格 → 神经启发式 A\* → 神经控制器，连续穿过 5 道门洞。*

![screen recording](docs/assets/gifs/recording_dense.gif)

*真实屏幕录制（Ubuntu 20.04 虚拟机，Tk 窗口 + `xwd` 抓帧）：dense 场景 429 步、零碰撞。*

## 这是什么

一个**可运行、可复现、可部署**的无人机避障课程项目：

- **CA-Sim**：自研轻量 3D 仿真器（解析射线投射、双积分动力学、动障碍、RGB-D 渲染），
  不依赖 GPU / Gazebo / AirSim；
- **四个神经网络**：深度图占用栅格（U-Net）、神经启发式 A\*（MLP）、
  模仿学习控制器（MLP）、端到端深度→速度（CNN）；
- **纯 NumPy 自动微分**：700 行实现反向传播、卷积、Adam，无 PyTorch；
- **ROS 1 / ROS 2 双支持**：`main.launch` / `main.launch.py` 一键启动；
- **完整实验**：5 场景 × 4 方法 × 3 种子 = 60 回合，含成功率、间隙、耗时。

## 关键结果

| 方法 | 成功率 | 平均最小间隙 | 规划耗时 | 推理耗时 |
| --- | --- | --- | --- | --- |
| expert（几何基线） | 100% | 1.49 m | 7.0 ms | 0.05 ms |
| control（真值图 + 神经控制） | 100% | 1.92 m | 7.0 ms | 0.05 ms |
| **full（完整神经链路）** | **100%** | 1.41 m | 15.3 ms | 0.22 ms |
| e2e（端到端） | 40% | 1.16 m | — | 0.82 ms |

- 感知：val IoU **0.956**，Dice 0.978；
- 规划：val MAE **2.52 m**，R² 0.910；dense 场景搜索量 **−49%**；
- 控制：val MAE **0.0356 m/s**，角度误差 1.9°；
- 端到端：dynamic 与留出的 unseen 成功，其余训练场景**零碰撞但未收敛**（如实的负结果）。

## 快速开始

```bash
# 1) 安装（只需 5 个依赖，无 GPU）
pip install -r requirements.txt
pip install -e .        # 可选：安装为可导入包；直接在本目录运行无需此步

# 2) 30 秒冒烟测试：渲染一帧
python main.py --module sim --scene sparse

# 3) 完整闭环 + 生成 GIF
python main.py --module full --scene corridor --gif
```

输出在 `runs/<时间>_<模块>/`，包含 `*.png`、`*.gif`、`metrics.json`。

### 模块入口

| 命令 | 说明 |
| --- | --- |
| `python main.py --module sim` | 只渲染一帧场景 |
| `python main.py --module perception` | 深度图 → 占用栅格三联图 + IoU |
| `python main.py --module planning` | 经典 A\* vs 神经 A\* 对比 |
| `python main.py --module control` | 真值地图 + A\* + 神经控制器 |
| `python main.py --module e2e` | 端到端策略单独运行 |
| `python main.py --module full` | 完整分层闭环 |
| `python main.py --module demo` | 实时窗口演示 |

## 环境

| 项 | 要求 |
| --- | --- |
| Python | 3.8 – 3.11（已在 3.8.10 / 3.10 验证） |
| 依赖 | numpy、Pillow、matplotlib |
| 硬件 | 4 核 CPU / 8 GB，**无需 GPU** |
| ROS（可选） | Noetic / Humble，Ubuntu 20.04 |
| 已验证 | Windows 10 + Python 3.10；Ubuntu 20.04 VM + Python 3.8 + Pillow 7.0 |

## 目录

```
neural_ca_drone/
├── main.py                 # 统一入口（main. 前缀）
├── ca_drone/               # 仿真、网络、建图、规划、控制
├── tools/                  # 数据、训练、评测、图表、打包
├── weights/                # 训练好的 npz（随仓库提交）
├── ros1_ws/  ros2_ws/      # ROS 1 / ROS 2 包与 launch
├── docs/                   # mkdocs 文档（GitHub Pages）
└── results/                # 评测 CSV / JSON
```

## 文档

- 在线文档：`mkdocs serve` → <http://127.0.0.1:8000>
- 构建：`mkdocs build`（输出 `site/`）
- 首页有指向“四个模块 / 实验 / ROS 部署”的卡片链接

## ROS 部署

```bash
# ROS 1 Noetic
cd ros1_ws && catkin_make && source devel/setup.bash
roslaunch neural_ca_drone_ros main.launch scene:=sparse module:=full

# ROS 2 Humble
cd ros2_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch neural_ca_drone_ros2 main.launch.py module:=full scene:=sparse
```

话题契约：订阅 `/camera/depth/image_raw`、`/odom`、`/move_base_simple/goal`；
发布 `/cmd_vel`、`/ca/path`、`/ca/occupancy`、`/ca/status`。
详见 [ROS1 文档](docs/ros1.md) 与 [ROS2 文档](docs/ros2.md)。

## 复现全部结果

```bash
python tools/make_data.py --out data --e2e-episodes 60 --e2e-limit 14000
python tools/train.py --module all --data data --weights weights   # ~12 min
python tools/evaluate.py --scenes all --modes expert,control,full,e2e --seeds 3
python tools/make_figs.py
```

## 结果可复现性

同一个权重 + 同一场景，**轨迹与指标是可复现的**（纯 NumPy 推理，无 GPU、无随机丢弃）：

| 场景 | seed 0/1/2 的步数与最小间隙 |
| --- | --- |
| dense | 429 步 / 1.66 m（三个种子完全一致） |
| sparse | 434 步 / 1.30 m（三个种子完全一致） |
| dynamic | 470–474 步 / 1.12–1.31 m（运动障碍相位随种子变化，均到达） |

在 Ubuntu 20.04 + Python 3.8 虚拟机中跑 dense 也是 **429 步 / 1.66 m**，
与 Windows + Python 3.10 一致，说明结果不依赖平台。

会有差别的地方：

- **遥测里的毫秒数**（plan/infer）随 CPU 快慢变化，其他人机器上数字不同；
- **GIF 像素**受字体（Windows 用 Consolas，Linux 回退 DejaVu Sans Mono）和
  Pillow 版本影响，文字排版可能差几个像素，但轨迹与数值相同；
- **dynamic 场景**换 `--seed` 会改变运动障碍相位，步数略有浮动；
- 用 `--render-every` 改抽帧间隔会改变 GIF 帧数（默认 4，录屏脚本用 6）。

## 已知限制

- 端到端策略在 sparse / dynamic / corridor 会停滞（零碰撞，但不收敛）；
- 神经启发式在稀疏场景反而增加搜索量（训练分布偏密集）；
- Gazebo 接入只提供话题契约与参数说明，未在无 GPU 虚拟机中做闭环验证；
- 感知网络对 < 0.1 m 的细杆可能漏检。

这些限制在文档中均有定量说明，不掩饰。

## 许可

课程作业用途。第三方素材（字体、图标）遵循其原始许可。
