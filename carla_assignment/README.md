# CARLA 0.9.16 无人车神经网络四次作业

基于 **CARLA 0.9.16** 的无人车神经网络课题交付包，覆盖老师要求的前 4 项任务。
感知 / 规划 / 控制 / 端到端算法**均为神经网络**（`nn_models.py` 纯 numpy MLP / tf.keras CNN），
可在 **Windows 原生** 或 **Ubuntu 20.04 (Noetic) / 22.04 (Humble)** 运行，并支持 ROS launch 封装。

## 任务覆盖

| 作业 | 目录 | 内容 | 对应任务 | NN |
|---|---|---|---|---|
| 一 | `01_control/` | CARLA 键盘控制无人车（pygame + 前视相机） | (1) | – |
| 二 | `02_perception/` | RGB/深度相机 + 雷达感知 + 给定轨迹 NN 跟踪 | (2) | 感知/控制 |
| 三 | `03_navigation/` | LiDAR 占用栅格建图 + 神经网络导航避障 | (3) | 规划 |
| 四 | `04_end_to_end/` | 端到端 CNN（图像 → 控制指令） | (4) | CNN |
| 五 | `05_reports/` | 整合统一启动 + 性能评价 | 综合 | – |
| 库 | `nn_models.py` | 纯 numpy 神经网络（MLP 分类 / 策略） | 综合 | ✅ |
| 工具 | `scripts/gen_train_loss.py` | 采集后离线生成 CNN 训练 loss 曲线 | 综合 | – |

## 快速开始

```bash
# 1. 启动 CARLA 0.9.16 服务端
./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low

# 2. 装环境（一次性）
bash scripts/setup_env.sh
source .venv/bin/activate

# 3. 先离线训练两个神经网络（无需 CARLA，仅 numpy）
python 02_perception/main.py --mode train
python 03_navigation/main.py --mode train

# 4. 总入口运行作业一（键控小车，弹出窗口）
python main.py --task control
```

## Windows 原生入口（不需要 WSL/ROS）

```bat
main.bat control                          :: 作业一 键盘控制
main.bat perception --mode train          :: 作业二 训练感知/控制NN
main.bat navigation --mode train          :: 作业三 训练规划NN
main.bat end_to_end --mode test           :: 作业四 端到端
```
Unix `main.sh` 用法相同（`bash main.sh control`）。

## 独立运行 / ROS launch

```bash
# 独立（在线，需 CARLA 服务端）
python 01_control/main.py
python 02_perception/main.py --mode run --model models/nn_percept.json --waypoints "40,-8 40,12 25,20"
python 03_navigation/main.py --mode run --model models/nn_plan.json --goal "20,8"
python 04_end_to_end/main.py --mode test --model_path models/cnn.h5

# ROS2 Humble（先 ament 打包，见 setup.py / package.xml）
colcon build --symlink-install && source install/setup.bash
ros2 launch carla_assignment 02_perception_launch.py mode:=run
ros2 launch carla_assignment 03_navigation_launch.py mode:=run

# ROS1 Noetic（CARLA ros-bridge 跑在 Noetic）
source /opt/ros/noetic/setup.bash
roslaunch carla_assignment 02_perception.launch mode:=run
roslaunch carla_assignment 03_navigation.launch mode:=run
```

## 环境要求

- 操作系统：Ubuntu 20.04（Noetic）/ 22.04（Humble）/ Windows 10+
- CARLA 0.9.16（在线 run 需要；train 模式无需 CARLA）
- Python ≥ 3.8；依赖：numpy / opencv-python-headless / pygame / tensorflow-cpu / carla
- 建议 NVIDIA 显卡（无独显请改用同仓库 [MuJoCo 方案](../mujoco_assignment)）

## 文档与 GitHub Pages

- mkdocs：`docs/mkdocs.yml`（`mkdocs serve --config-file docs/mkdocs.yml`）
- GitHub Actions 自动部署：`.github/workflows/deploy_page.yml`
- PR 模板：`.github/pull_request_template.md`
- 交付说明（对照老师要求 / 运行指引）：`docs/delivery_notes.md`
- 神经网络库文档：`docs/nn_models.md`

## 目录结构

```
carla_assignment/
├── main.py / main.bat / main.sh  # 总入口（跨平台）
├── common.py            # CARLA API 公共封装
├── nn_models.py         # 纯 numpy 神经网络（感知/控制/规划）
├── package.xml / setup.py / resource/  # ROS2 ament 打包
├── 01_control/  02_perception/  03_navigation/  04_end_to_end/  05_reports/  # main.py + launch/
├── scripts/             # setup_env.sh + gen_train_loss.py
└── docs/                # mkdocs 文档
```
