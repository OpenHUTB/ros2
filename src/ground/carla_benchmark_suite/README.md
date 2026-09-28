# carla_benchmark_suite —— CARLA 作业综合整合与性能评价

> 机器人操作系统及应用 · 课程作业五
>
> 把前四份作业**统一到一个入口**调度，并给出**真实测量**的性能评价指标
> （精度、时延、转向平滑度、建图覆盖率等）。所有评测在**无 CARLA 服务端**时也能完成。

## 1. 功能

### 1.1 统一调度

| target | 功能包 | 内容 |
|---|---|---|
| `control` | `carla_keyboard_control` | 任务(1) 物理仿真 + 键盘运动控制 |
| `perception` | `carla_perception_control` | 任务(2) 传感器感知 + 给定轨迹跟踪（NN） |
| `navigation` | `carla_mapping_navigation` | 任务(3) 建图 + 神经网络规划导航 |
| `end_to_end` | `carla_end_to_end_nn` | 任务(4) 端到端图像 → 控制（CNN） |

### 1.2 基准评测

`--benchmark` 会**真实训练并回放**三个模块，测量：

| 指标 | 含义 |
|---|---|
| `percept_acc` | 感知 NN 训练集准确率 |
| `control_mse` | 控制 NN 回归 MSE |
| `lateral_rmse` | 轨迹跟踪横向误差 RMSE（m） |
| `plan_mse` | 规划 NN 回归 MSE |
| `known_ratio` / `occupied_cells` | 建图覆盖率 / 占据格数 |
| `nav_min_dist` | 导航到目标的最近距离（m） |
| `reg_mae` / `reg_sign_acc` / `reg_corr` | 端到端 CNN 的 MAE / 方向一致率 / 相关系数 |
| `aos` | 转向平滑度（相邻转向指令平均绝对变化量，越小越平滑） |
| `latency_mean_ms` / `latency_p95_ms` | 单步推理时延均值 / P95（ms） |
| `steer_saturation_ratio` | 转向饱和比例（打到 ±1 边界的比例） |

## 2. 目录结构

```
src/ground/carla_benchmark_suite/
├── main.py                                   # 主入口（--list / --benchmark / --target）
├── main.sh / main.bat                        # 课程约定的 main.* 一键运行脚本
├── carla_benchmark_suite/
│   ├── __init__.py
│   ├── nn_models.py                          # 与其它功能包保持一致的副本
│   ├── carla_common.py                       # CARLA Python API 封装
│   └── benchmark_node.py                     # ROS 2 节点（发布评测指标 / 调度子模块）
├── launch/
│   ├── main.launch.py                        # ROS 2 Humble
│   └── main.launch                           # ROS 1 Noetic
├── config/sim_params.yaml
├── resource/carla_benchmark_suite
├── setup.py / setup.cfg / package.xml
├── requirements.txt
└── test/test_benchmark_logic.py              # 单元测试（11 项，无需 CARLA）
```

## 3. 快速开始

```bash
# ① 列出所有可调度模块（会显示哪些已就绪、哪些缺失）
python3 main.py --list

# ② 运行基准评测（真实测量；无需 CARLA 服务端；约 2~3 分钟）
python3 main.py --benchmark --save_dir ~/shots --out report.json

# ③ 只评测某一项
python3 main.py --benchmark --which end_to_end --samples 60

# ④ 调度子模块（"--" 之后的参数原样透传给子模块）
python3 main.py --target perception -- --headless --demo --save_dir ~/shots
python3 main.py --target end_to_end -- --mode train --epochs 60
python3 main.py --target navigation -- --mode run --host <宿主机IP> --goal "20,8"

# ⑤ ROS 2 / ROS 1
ros2 launch carla_benchmark_suite main.launch.py                    # 发布评测指标
ros2 launch carla_benchmark_suite main.launch.py target:=perception # 调度子模块
roslaunch carla_benchmark_suite main.launch target:=navigation host:=<宿主机IP>
```

一键脚本：`bash main.sh --benchmark` 或 Windows `main.bat --list`。

## 4. 测试

```bash
python3 test/test_benchmark_logic.py     # 11 项全部通过，无需 CARLA
```

其中 `test_sibling_nn_models_are_consistent` 是**副本一致性回归测试**：
`nn_models.py` 在每个功能包内各有一份副本（为了让每个包都能独立编译运行），
多副本有"修了一个包、忘了另一个包"的风险。该测试在副本出现分歧时失败并列出差异文件。

> 历史教训：作业四修复了 maxpool 反向传播与推理 tanh 两处缺陷后，
> 性能评价包仍用旧副本，基准测试中端到端 MAE 一度测得 **36.12**（正常应 <0.3）。

## 5. 实测指标（本机实测）

| 指标 | 数值 |
|---|---|
| 感知 NN 准确率 | 0.9850 |
| 控制 NN MSE | 0.01893 |
| 横向误差 RMSE | 0.9608 m |
| 规划 NN MSE | 0.00610 |
| 建图覆盖率 | 0.2919 |
| 占据格数 | 1387 |
| 导航最近距离 | 1.4539 m |
| 端到端 MAE | 0.10093 |
| 端到端方向一致率 | 0.9500 |
| 端到端相关系数 | 0.9920 |

## 6. 与已有示例的关系

本模块是 [`ad_demo`（自动驾驶演示）](https://openhutb.github.io/ros2/ground/ad_demo/)
的**拓展**：已有示例用 `roslaunch` 拉起 `carla_ad_demo` 做单次演示，
本模块把四份作业统一到一个可调度的入口，并补充**可复现的量化性能评价**。
**CARLA 服务端启动、宿主机 IP 与端口查看等通用配置步骤不在此重复**，
详见 [设置并连接到 Carla 模拟器](https://openhutb.github.io/ros2/set_up_and_connect_to_carla/)
与 [模块文档](https://openhutb.github.io/ros2/ground/carla_benchmark_suite/)。
