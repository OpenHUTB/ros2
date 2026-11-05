# mujoco_car_nn_avoidance

MuJoCo 地面两轮差速小车：**25 束前视射线 + 航向偏差 → 纯 numpy MLP → 转向与油门**。
感知、决策、控制整条链路由神经网络完成，推理阶段不含规则分支；专家策略只在离线采集阶段生成训练标签。

## 与仓库已有模块的区别

| 对比项 | `mujoco_car_avoidance`（规则基线） | 本模块 |
|---|---|---|
| 引擎 | MuJoCo | **MuJoCo** |
| 感知方式 | `mj_ray` 射线扇面 | **`mj_ray` 射线扇面 + 航向** |
| 决策方式 | 距离分级状态机（规则） | **纯神经网络（MLP）** |
| 推理时是否含规则分支 | 是 | **否** |
| 运行前提 | mujoco + numpy | **mujoco + numpy** |
| 无图形界面可用 | 是 | **是（默认无窗口）** |

本模块复用规则基线已经调通的差速底盘物理方案（速度执行器、质心前移、静定三点支撑、出生穿透量），差别只在决策层：规则状态机被替换成训练得到的 MLP。

## 目录结构

```
src/ground/mujoco_car_nn_avoidance/
├── main.py                     统一入口（--collect/--train/--eval/--demo/--test/--launch）
├── main.sh / main.bat          脚本入口（无参数时默认 --eval）
├── mujoco_car_nn_avoidance/
│   ├── config.py               路径解析与全部可调参数
│   ├── car_env.py              场景加载、步进、车辆状态、静置起步
│   ├── sensors.py               mj_ray 射线扇面与距离归一化
│   ├── expert.py               专家策略（仅作离线教师标签）
│   ├── kinematics.py           纯函数：差速解算为左右轮角速度
│   ├── model.py                纯 numpy MLP（前向、反传、训练、权重存取）
│   ├── ros_node.py             ROS 2 推理节点（订阅 scan/odom，发布 cmd_vel）
│   └── sim_bridge.py           ROS 2 仿真桥接节点（发布 scan/odom，订阅 cmd_vel）
├── models/                     三个自建 MJCF 场景（纯几何体，无外部网格资产）
├── config/params.yaml          ROS 2 桥接节点参数
├── launch/main.launch.py       ROS 2 双节点闭环启动文件
├── test/                       单元测试（网络 / 专家 / 运动学 / 场景与传感器）
├── tools/                      场景冒烟与诊断脚本
├── requirements.txt            Python 运行依赖
└── results/mlp_weights.npz     随包附带的训练权重（ROS 推理节点从这里加载）
```

## 运行环境

| 组件 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04（虚拟机，实测通过）；Windows 本地 Python 环境可跑纯仿真；20.04 见下 |
| Python | 3.10（虚拟机 3.10.12，本地 3.10.11） |
| MuJoCo | 3.15.0（虚拟机实测；课程要求 3.12.0，本模块只用稳定接口，3.12～3.15 可运行） |
| numpy | 1.26.4（虚拟机） |
| 可选依赖 | `matplotlib` 画评测曲线、`imageio` + `pillow` 合成演示 gif；缺失时自动跳过绘图，不算失败 |
| ROS | ROS 2 Humble（仅 `--launch` 与 ROS 节点需要） |
| 显卡 | 不需要；默认离屏无窗口运行 |

> Ubuntu 20.04 兼容说明（非实测环境）：20.04 上 ROS 2 Humble 的 `_rclpy_pybind11` 针对系统 Python 3.8 编译，ROS 节点须用 3.8 运行；仿真部分（`main.py --collect/--train/--eval/--demo/--test`）与 ROS 节点无进程内耦合，可单独用 3.10 解释器运行。本模块实测环境为 Ubuntu 22.04 + Humble，未在 20.04 上复测。

## 安装

```bash
# 系统依赖（无图形界面也需要）
sudo apt update && sudo apt install -y python3-pip libgl1

# Python 依赖
python3 -m pip install -r requirements.txt

# ROS 2 功能包（只跑 main.py 时不需要）
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ln -s <本仓库路径>/src/ground/mujoco_car_nn_avoidance .
cd ~/ros2_ws && colcon build --packages-select mujoco_car_nn_avoidance --symlink-install
source install/setup.bash
```

## 运行步骤

在模块根目录 `src/ground/mujoco_car_nn_avoidance/` 下执行：

```bash
python3 main.py --version        # 确认代码版本（当前 1.0.0）
python3 main.py --test            # 自检：纯逻辑单测 + 场景冒烟（需要 MuJoCo）
python3 main.py --test --logic-only   # 只跑纯逻辑单测，不加载 MuJoCo、不跑场景冒烟
python3 main.py --collect --seconds 20   # 探索式采集专家标签（准备数据集）
python3 main.py --train --epochs 300     # 训练 MLP（准备权重）
python3 main.py --eval            # 三场景闭环评测，导出报表与柱状图
python3 main.py --demo            # 离屏渲染录制演示 gif（450 帧、每帧 40 ms、总时长 18 s）
python3 main.py --launch          # ros2 launch 一键启动 ROS 2 闭环
bash main.sh                      # 等价于 --eval
```

仓库已附带训练好的 `results/mlp_weights.npz`，只想复现评测和演示时直接跑 `--eval` / `--demo`；重新采集、训练会覆盖该权重。

> Windows 环境请确保仓库路径不含中文，否则 MuJoCo 加载模型会报 `ParseXML: Error opening file`。

## 参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `lidar_angles_deg` | −60°…60°，5° 间隔（25 束） | 前视射线扇面 |
| `lidar_range_m` | 6.0 m | 射线最大探测距离 |
| `timestep` | 0.002 s | 物理步长（与 XML `option` 一致） |
| `control_hz` | 50 Hz | 决策 / 控制频率 |
| `settle_s` | 0.6 s | 起步静置：先零轮速推进，等接触建立后再进闭环 |
| `v_max` | 0.8 m/s | 最大目标线速度 |
| `omega_max` | 2.0 rad/s | 最大目标转向角速度 |
| `wheel_r` | 0.05 m | 轮半径（与 XML 一致） |
| `track` | 0.25 m | 轮距（与 XML 一致） |
| `max_wheel_omega` | 60 rad/s | 轮角速度上限（对应执行器 `ctrlrange`） |
| `max_wheel_force` | 8.0 N | 单驱动轮出力上限（对应 XML `forcerange`） |
| `cruise_throttle` | 0.85 | 空旷直行时的巡航油门 |
| `goal_x` | 6.0 m | 到达判定：车体 x 超过该值视为到达终点成功 |
| `input_dim` / `hidden` / `output_dim` | 26 / 32 / 2 | 网络输入、隐层、输出维度 |
| `epochs` / `lr` / `batch` / `seed` | 300 / 0.005 / 64 / 42 | 训练超参 |

## 版本记录

| 版本 | 内容 |
|---|---|
| 1.0.0 | 初版：25 束射线 + 航向输入的纯 numpy MLP 端到端避障；basic / slalom / dual 三场景闭环评测；ROS 2 Humble 双节点闭环在 Ubuntu 22.04 虚拟机实测通过 |

`python3 main.py --version` 可直接确认当前代码版本；`python3 main.py --test` 先打印版本再跑测试。

## 常见问题

**Q1：运行采集、评测、演示命令时报 `ModuleNotFoundError: No module named 'mujoco'`**
按上面"安装"一节装依赖，确认 `pip` 与运行脚本用的是同一个解释器（用 `python3 -m pip`）。纯逻辑自检不依赖 MuJoCo，见 Q6。

**Q2：虚拟机里 `--demo` 起不来**
虚拟机没有图形界面和 GLFW 上下文。本地验证改用 `--eval` 出评测曲线；演示 gif 在具备离屏渲染能力的环境录制。

**Q3：训练后车直行冲障碍、loss 降不下去**
先检查采集到的转向样本占比是否过低；确认 `--collect` 开启了探索噪声，必要时加大 `--seconds` 多采集贴近障碍和偏航的状态。

**Q4：ROS 节点启动报找不到 `mlp_weights.npz`**
确认用 `--symlink-install` 构建；权重由 `setup.py` 的 `data_files` 安装到 `share/mujoco_car_nn_avoidance/results/`，不要手动改节点里的加载路径。

**Q5：`--test` 里场景测试被跳过**
说明当前环境没装 MuJoCo，纯逻辑测试照常执行，场景与传感器测试自动跳过；装好 MuJoCo 后会全部运行。

**Q6：未装 MuJoCo 时入口起不来怎么办**
先用 `python3 main.py --test --logic-only` 跑纯逻辑单测；也可以绕开入口直接执行 `python3 -m unittest discover -s test -p "test_*.py"`。采集、评测、演示、场景冒烟命令需要先按 5.2 节装好 MuJoCo 才能运行。

## 已知限制

- 障碍物为静态，未覆盖动态目标；网络只在 basic / slalom / dual 三个训练场景内验证，未在全新障碍布局上做泛化测试。
- 射线扇面是二维水平面，对悬空或低矮目标不敏感，需要时可增加俯仰方向射线。
- 本模块为纯仿真，未接入实体车辆；ROS 节点与仿真器之间通过话题解耦，后续替换仿真底盘时只需更换桥接节点。
