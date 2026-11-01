# mujoco_ground_slam_nn_nav

基于 **ROS Noetic + MuJoCo 3.12.0** 的地面差速无人车模块：物理仿真、键盘遥控、360° 激光、前向 RGB-D/BEV 相机、在线占据栅格 SLAM、A* 导航，以及四个纯 NumPy 神经网络（激光去噪、控制、局部规划、端到端 BEV 驾驶）。

## 与仓库已有模块的区别

| 模块 | 仿真器 | 主要能力 |
| --- | --- | --- |
| `mujoco_car_avoidance` | MuJoCo | 射线感知 + 规则分级减速 |
| `mujoco_car_nn_avoidance` | MuJoCo | 射线 + MLP 端到端避障 |
| `carla_mapping_navigation` | CARLA | 激光建图 + 神经网络规划 |
| **本模块** | **MuJoCo** | **SLAM + A* + 纯跟踪 + 四个网络 + ROS Noetic 全链** |

本模块不是单一避障 demo：它同时包含在线建图、重规划、路径跟踪、神经感知/控制/规划和端到端图像策略，并提供三场景闭环指标与失败分析。

## 目录结构

```text
mujoco_ground_slam_nn_nav/
├── main.py / main.sh           统一入口（--launch / --test / --info）
├── gvnn/                       仿真、SLAM、规划、控制、NN 算法
│   ├── sim/                    MuJoCo 世界、传感器、渲染
│   ├── slam/                   占据栅格 + 增量扫描匹配
│   ├── planning/               A*、神经局部规划
│   ├── control/                纯跟踪、安全层
│   └── nn/                     MLP/CNN/Adam
├── scripts/
│   ├── nodes/                  9 个 ROS 节点
│   ├── setup_env.sh            项目内 Python 3.11 venv
│   └── build_ros1.sh           独立 catkin 工作区构建
├── launch/                     6 个 launch
├── models/                     4 个训练权重
├── tools/                      采集、训练、评估、GIF
├── tests/                      15 个单元测试
├── results/                    指标 JSON/CSV
├── package.xml
└── CMakeLists.txt
```

## 运行环境

| 组件 | 版本 |
| --- | --- |
| Ubuntu | 20.04.6 LTS（VMware） |
| ROS | Noetic（`/opt/ros/noetic`） |
| Python（计算节点） | 3.11，项目内 `.venv` |
| MuJoCo | 3.12.0 |
| NumPy / Pillow | 1.26.4 / ≥9 |
| OpenCV | 4.2（系统 Python，仅 viewer） |

> ROS Noetic 的 `roslaunch/roscore` 使用系统 Python 3.8；仿真、SLAM、规划、控制、E2E 节点由脚本自动 `exec` 到项目 `.venv` 的 Python 3.11。不会修改 `.bashrc`、PATH 或系统 Python。

## 安装与构建

```bash
# 1. 把模块放进 catkin 工作区
mkdir -p ~/catkin_ws/src
cp -r <仓库>/src/ground/mujoco_ground_slam_nn_nav ~/catkin_ws/src/

# 2. 项目内 venv（MuJoCo 3.12 需要 Python ≥3.10）
cd ~/catkin_ws/src/mujoco_ground_slam_nn_nav
bash scripts/setup_env.sh

# 3. 编译
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

也可以使用模块自带脚本构建到独立工作区：

```bash
bash scripts/build_ros1.sh ~/gvnn_ws
source ~/gvnn_ws/devel/setup.bash
```

## 启动

```bash
# 键盘遥控（W/A/S/D）
roslaunch mujoco_ground_slam_nn_nav teleop.launch scene:=warehouse

# SLAM + A* + 纯跟踪 + 指标
roslaunch mujoco_ground_slam_nn_nav slam_nav.launch scene:=warehouse viewer:=true

# 神经规划 + 神经控制
roslaunch mujoco_ground_slam_nn_nav neural_nav.launch scene:=warehouse

# 端到端 BEV-CNN
roslaunch mujoco_ground_slam_nn_nav e2e.launch scene:=crossing

# 无头
roslaunch mujoco_ground_slam_nn_nav slam_nav.launch viewer:=false
```

统一入口：

```bash
python3 main.py --info
python3 main.py --launch slam_nav --scene warehouse
python3 main.py --launch teleop --no-viewer
python3 main.py --test          # 使用 .venv/bin/python 运行
```

## 实测结果（Ubuntu 20.04 / 4 vCPU）

| 指标 | 数值 |
| --- | --- |
| 全链 SLAM + A*（warehouse） | 成功，35.1 s，0 碰撞，终点 0.846 m |
| SLAM ATE RMSE | 0.0796 m |
| 地图覆盖率 | 87.8% |
| 三场景 classical | 3/3 成功，0 碰撞 |
| 感知 CNN | 激光 RMSE 0.714 → 0.480 m（+32.8%） |
| 神经控制 MAE | v 0.039 m/s，ω 0.077 rad/s |
| 神经规划离线准确率 | 72.8% |
| 神经规划+控制（监督） | warehouse 成功，61.15 s，终点 0.157 m |

端到端纯行为克隆在 warehouse/obstacle_field 仍会碰撞；crossing 可行驶 23.3 m、0 碰撞但未到终点。该限制与失败分析已写入教程，不把离线指标当作在线闭环结论。

## 训练与测试

```bash
.venv/bin/python tools/collect_expert.py --episodes-per-scene 4 --duration 25
.venv/bin/python tools/train_perception.py
.venv/bin/python tools/train_control.py
.venv/bin/python tools/train_planner.py
.venv/bin/python tools/train_e2e.py --epochs 30
.venv/bin/python tools/evaluate.py --modes classical,neural,e2e --seeds 1
.venv/bin/python tests/run_all.py
```

模型与指标分别在 `models/`、`results/`。

## 已知限制

- SLAM 无回环检测，长距离闭环会保留少量漂移；
- 端到端 CNN 只做行为克隆，存在分布偏移；
- 神经规划器依赖 A* 回退与纯跟踪监督；
- 本模块只实测 ROS Noetic，未在 ROS 2 Humble 验证。

## 许可证

MIT。
