# 人形机器人视觉通行感知与室内导航

在 MuJoCo 3.2.0 中使用 Unitree G1 的预训练步态，开发 RGB 地面分割、图像到速度控制及 ROS Noetic 同步 SLAM 导航。模块名：`humanoid_visual_patrol`。

完整原理、实验和限制见 [课程文档](../../../docs/humanoid/humanoid_visual_patrol/index.md)。

## 功能与边界

| 任务 | 入口 | 实现 |
|---|---|---|
| 键盘控制 | `main.py --mode teleop` | WASD/方向键、空格停止、加减速度，0.6秒未按键归零 |
| 感知控制 | `--mode perception` | RGB → FloorNet 地面分割 → MaskController 语义控制网络 → 行走速度 |
| 建图导航 | `launch/main.launch` 默认 navigation | 模拟雷达及积分里程计 → gmapping → CostNet 辅助 A* → move_base → G1 |
| 端到端 | `--mode end2end` | RGB → ImageController → 前进/转向速度 → 预训练步态 |

任务二的分割与语义控制网络分别训练，规则通道选择只用于生成示范及对照；部署时若分割图完全没有地面像素，额外的规则保护会将速度和转向置零，并不代表网络自身能处理这种情况。该保护不覆盖少量误分割或所有障碍。任务三使用经典 gmapping、CostNet 辅助全局 A* 和 move_base 局部避障，是混合规划方案，并非神经网络 SLAM。任务四直接输出高层速度，不是图像到关节力矩。底层步态是第三方预训练策略，不计作本项目训练成果。

本示例的终线是通行测试终点，场景标记不触发巡检事件。

## 已验证环境

- 当前复现步骤仅验证源码空间运行，不保证 catkin install 空间运行。
- `catkin_make` 源码空间构建通过；`catkin_make install` 构建通过，但安装空间只有 scripts/launch/config，不含 `models/` 与 `patrol` 包，因此仍按源码空间运行。
- Windows 11，Python 3.11.15，MuJoCo 3.2.0，PyTorch 2.15.0+cpu：物理仿真、图像采集、训练、离屏录像。
- WSL2 Ubuntu 20.04.5，ROS Noetic，Python 3.8.10：gmapping、move_base 和 ROS 桥。
- Windows 和 WSL 用子进程标准输入输出连接，不需要开放网络端口给仿真器。
- 本次使用 CPU 训练；未声明 GPU 性能。原生 Linux 仿真解释器路径受支持，但本次未验证该部署组合。

## 安装与获取资产

在本模块目录运行（以下 `python` 指独立仿真环境解释器）：

```bash
python -m pip install -r requirements.txt
python scripts/fetch_assets.py /path/to/unitree_rl_gym
```

资产固定到官方 `unitree_rl_gym` 提交 `276801e46c5d433564f24658bac64f254b7d2d4b`，含 G1 12 关节腿部模型、网格和 `deploy/pre_train/g1/motion.pt`。上半身模型固定，不能声称实现全身灵巧操作。许可证见 `third_party_notices.md`。

```bash
python main.py --assets /path/to/unitree_rl_gym --mode smoke --output results/smoke.json
python main.py --assets /path/to/unitree_rl_gym --mode teleop
python main.py --assets /path/to/unitree_rl_gym --mode perception --weights models --record --output results
python main.py --assets /path/to/unitree_rl_gym --mode end2end --weights models --record --output results
```

关闭可视窗口停止遥控，默认45仿真秒后退出，可用 `--seconds` 调整。需连续按键维持运动；已通过向真实 GLFW 窗口发送 Windows 按键事件验证前进、转向和停止，仍建议审核者实际操作复核手感。

## 独立训练与测试

```bash
python main.py --assets /path/to/unitree_rl_gym --mode collect --dataset data/visual_patrol --samples 80
python main.py --assets /path/to/unitree_rl_gym --mode train --dataset data/visual_patrol --weights models --epochs 20
python main.py --assets /path/to/unitree_rl_gym --mode train_planner --weights models
python main.py --assets /path/to/unitree_rl_gym --mode perception --weights models --seed 8 --output results --record
python main.py --assets /path/to/unitree_rl_gym --mode end2end --weights models --seed 8 --output results --record
```

训练/验证/测试分别使用场景种子 0–5、6、7–8，每个场景 80 张图像。随机化涉及光照、颜色和柜体侧向位置；两套测试场景并不代表不同建筑。标签由仿真几何 ID 产生，训练图像来自静态相机姿态采样，闭环测试中才使用真实步态运动。测试结果不能外推到楼梯、人群或真实机器人。

## ROS 启动（本次验证方式）

在 WSL 中，以本模块目录为当前目录：

```bash
bash scripts/run_navigation.sh \
  /mnt/f/path/to/.venv_humanoid/Scripts/python.exe \
  /mnt/f/path/to/unitree_rl_gym navigation
```

脚本自动转换 Windows 子进程使用的路径。`ROS_MASTER_URI` 默认为 `http://localhost:11331`；其他终端也需设置这个值。手动设置路径时：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11331
export ROS_PACKAGE_PATH="$PWD:$ROS_PACKAGE_PATH"
roslaunch humanoid_visual_patrol main.launch \
  sim_python:=/path/to/python sim_entry:=/path/to/main.py assets:=/path/to/unitree_rl_gym
```

先等待 `/map` 更新，再通过 RViz 的 2D Nav Goal 发送地图坐标目标，由神经辅助规划节点生成并执行航点；直接调用 `/move_base` action 则用于经典规划基线对照。导航与 SLAM 同时运行；位置容差 0.5m，终点朝向不约束。不要同时运行独立遥控和导航去控制同一个仿真实例。

`task:=teleop`、`task:=perception`、`task:=end2end` 可以经相同 launch 调用对应任务。后两项需传入 `weights`，输出路径通过 `output` 指定。

```bash
rosrun map_server map_saver -f /path/to/saved_map
```

Ctrl+C 退出 launch 后桥接进程会关闭其仿真子进程。

## 本地验证

```bash
python -m pip install -r requirements-dev.txt
# Linux: export HUMANOID_ASSETS=/path/to/unitree_rl_gym
# PowerShell: $env:HUMANOID_ASSETS='F:/path/to/unitree_rl_gym'
python -m pytest tests -q
```

包括无地面时停止、左右通行方向、神经平局排序不改变最优栅格路径代价、无路径报错，以及真实动力学行走和雷达自体回波排除。没有资产路径时，物理测试明确跳过。

## 大模型使用声明

本模块使用 OpenAI Codex 辅助方案、编程、调试、文档和实验整理。发布者需对内容复核并负责；预训练模型和第三方来源已单独说明。
