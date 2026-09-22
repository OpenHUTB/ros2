# CARLA 0.9.16 无人车神经网络四次作业文档

欢迎浏览《基于 **CARLA 0.9.16** 的无人车神经网络课题》四次作业实现与文档。

本课题对应老师要求：
> 撰写（神经网络）实现感知、规划、控制算法，应用到 CARLA 0.9.16 的相关代码和文档。

**说明**：本包为纯 Python + CARLA Python API 实现，覆盖老师要求的前 4 项任务；每个模块
均提供 `main.py` 入口 + `launch/`（同时提供 ROS1 Noetic 的 `.launch` 与 ROS2 Humble 的
`.launch.py`）。

> 若你的环境**没有独立显卡**（虚拟机关闭 3D 加速导致 CARLA 很卡），请改用同仓库下的
> [MuJoCo 方案 `mujoco_assignment`](../mujoco_assignment)，对无显卡环境更友好。

---

## 作业导航

- [**作业一 · 键盘控制**](01_control.md)：在 CARLA 中物理仿真无人车，键盘运动控制（任务 1）。
- [**作业二 · 感知与轨迹**](02_perception.md)：RGB/深度相机 + 雷达感知，并按给定轨迹控制（任务 2，感知/控制为 NN）。
- [**作业三 · 建图与导航**](03_navigation.md)：LiDAR 建图 + 神经网络导航避障（任务 3，规划为 NN）。
- [**作业四 · 端到端 CNN**](04_end_to_end.md)：输入图像、输出控制指令的神经网络（任务 4）。
- [**作业五 · 整合与评价**](05_reports.md)：统一启动 + 性能评价。
- [**神经网络库说明**](nn_models.md)：`nn_models.py`（纯 numpy MLP，感知/控制/规划共用）。
- [**交付说明**](delivery_notes.md)：对照老师要求的覆盖与 Windows/Ubuntu/ROS 运行指引。

## 神经网络

为满足老师"感知、规划、控制、端到端算法需要为神经网络"的硬性要求，本包提供
**统一最小神经网络库 `nn_models.py`**（纯 numpy，离线可训练，无需 GPU）：

- 感知 NN：`MLPClassifier` — 特征 → 障碍/偏航类别
- 控制/规划 NN：`MLPPolicy` — (状态) → 连续控制量
- 端到端 CNN：`04_end_to_end` 的 `tf.keras` CNN — 图像 → steer

```bash
# 离线训练（仅 numpy，本机即可）
python 02_perception/main.py --mode train
python 03_navigation/main.py --mode train
```

## 环境与运行

- [环境安装](setup_guide.md)：从零配置 CARLA 0.9.16 + Python + ROS。
- [启动方式](../README.md)：总入口 `main.py` / 逐模块 `python X/main.py` / `roslaunch` / `ros2 launch`。

---

## 快速开始

```bash
# 1. 启动 CARLA 0.9.16 服务端（另开终端）
./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low

# 2. 配置环境
bash scripts/setup_env.sh
source .venv/bin/activate

# 3. 总入口运行作业一（键控小车，弹出窗口）
python main.py --task control

# 或逐模块用 ROS launch
source /opt/ros/humble/setup.bash
ros2 launch 02_perception/launch/main.launch.py
```

---

## 目录结构

```
carla_assignment/
├── main.py                    # 总入口：python main.py --task <name>
├── common.py                  # CARLA API 公共封装（连接/spawn/传感器/控制）
├── 01_control/  main.py + launch/{main.launch.py, main.launch}
├── 02_perception/ main.py + launch/...
├── 03_navigation/ main.py + launch/...
├── 04_end_to_end/ main.py + launch/...   # --mode collect/train/test
├── scripts/setup_env.sh       # 一键环境安装
└── docs/                      # 本 mkdocs 文档
```
