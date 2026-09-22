# AllocNet 键盘遥操作模块

本模块为 [AllocNet](https://github.com/KumarRobotics/AllocNet)（KumarRobotics，RA-L 2024）
避障轨迹规划器补齐**人机交互层**：用键盘精确下发航点、把一次性发布的地图转为
常驻话题、记录轨迹数据并生成交付图表。

> 课程文档（含原理、实测数据与完整操作说明）见
> [`docs/air/allocnet_teleop.md`](../../../docs/air/allocnet_teleop.md)。

## 1. 与上游的边界

明确区分本模块的工作与上游 AllocNet 的工作，避免混淆：

| 组成部分 | 来源 | 是否在本仓库 |
|---|---|---|
| AllocNet 规划器（`learning_planning`、libtorch 推理、QP 优化） | 上游 KumarRobotics/AllocNet | ❌ 需自行编译 |
| `param_env` 地图生成（`structure_map`） | 上游 `kr_param_env` | ❌ 需自行编译 |
| 键盘遥操作、地图 latched 转发、轨迹记录、图表绘制、素材采集 | **本模块** | ✅ |

本模块**不包含规划器本体**。规划器依赖 libtorch / OMPL / osqp-eigen，需按上游
README 自行编译。

## 2. 运行环境

| 项 | 版本 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 |
| ROS | Noetic（`rosversion` 1.16.0） |
| Python | 3.8（随 Noetic） |
| 规划器依赖 | libtorch (CPU)、OMPL、Eigen3、osqp + osqp-eigen |
| 绘图依赖 | `matplotlib`（仅 `plot_trajectory.py` / `offline_demo.py` 需要） |
| 素材采集（可选） | `xvfb`、`imagemagick`、Pillow |

无 GPU 直通的虚拟机必须使用 **CPU 版模型**（`*_cpu.pt`）；上游默认的 GPU 模型
在无 CUDA 环境加载会报错。

## 3. 快速开始

```bash
# 1. 准备 AllocNet 工作区（规划器需已编译通过）
git clone https://github.com/KumarRobotics/AllocNet.git ~/allocnet_ws/src/AllocNet
# ... 按上游 README 编译 planner 与 param_env ...

# 2. 把本模块装入工作区的 planner 包
cd src/air/allocnet_teleop
python3 main.py install --ws ~/allocnet_ws

# 3. 重新编译（脚本需注册进 CMakeLists）
cd ~/allocnet_ws && catkin_make --pkg planner

# 4. 环境自检
python3 main.py check

# 5. 启动
python3 main.py run              # 含 RViz
python3 main.py run --no-gui     # 无图形界面（服务器 / 虚屏）
python3 main.py run --no-record  # 不记录轨迹
```

`main.py` 是本模块的统一入口（对应仓库约定「入口为 `main.` 开头」），
支持 `install` / `check` / `run` / `capture` / `help` 五个子命令。

启动后通过 `rostopic pub` 或另开终端 `rosrun planner teleop_keyboard.py`
下发航点；完整按键表见课程文档 §4.4。

## 4. 自检与测试

本模块自带两处可独立运行的校验，均**不需要 ROS 环境**即可执行：

```bash
# 环境自检（需要 ROS）：逐项确认工作区、上游包、脚本与 launch 是否就位
python3 main.py check

# 单元测试：用 mock 的 rospy 验证键盘节点核心逻辑（18 项）
python3 test_teleop_keyboard.py
```

`test_teleop_keyboard.py` 覆盖高度与 `orientation.z` 的双向换算、各按键对游标
状态的影响、边界与高度裁剪、两段式 Goal 语义。

绘制链路可在无 ROS 的机器上复现：

```bash
python3 offline_demo.py --out-csv /tmp/demo.csv
python3 plot_trajectory.py --csv /tmp/demo.csv --out-dir /tmp/assets
```

> `offline_demo.py` 产出的是**算法层离线复现**，不是 ROS 仿真运行记录，
> 不作为实验证据；实验数据以真机运行导出的 CSV 为准。

## 5. 文件说明

| 文件 | 作用 |
|---|---|
| `main.py` | 统一入口：`install` / `check` / `run` / `capture` |
| `teleop_keyboard.py` | 键盘遥操作节点（游标、航点下发） |
| `map_republisher.py` | 地图 latched 转发（解决规划器错过地图） |
| `record_trajectory.py` | 轨迹数据记录 |
| `plot_trajectory.py` | 由 CSV 生成三维轨迹 / 俯视图 / 速度曲线 |
| `offline_demo.py` | 无 ROS 环境时的算法层离线复现 |
| `test_teleop_keyboard.py` | 单元测试（mock rospy，18 项） |
| `teleop_planning.launch` | 一键启动：地图 + 转发 + 规划器 + 键盘 + 记录 + RViz |
| `teleop_planner.rviz` | RViz 配置（正常实验用） |
| `capture_view.rviz` | RViz 配置（采集素材用，灰底高对比） |
| `capture_teleop_demo.py` | 演示素材采集（Xvfb 虚屏 + 抓屏） |
| `make_assets.py` | 把抓到的帧裁成静态图与 GIF |

## 6. 许可与致谢

人机交互层代码按上游 AllocNet 的许可条款提供。规划算法版权归 KumarRobotics 所有，
引用请见课程文档 §8。

---

## 人工智能使用声明

本模块的开发过程使用了 AI 编程助手（Claude）辅助代码编写、调试与文档撰写。
所有提交内容由提交者本人审阅并对正确性负全部责任。
