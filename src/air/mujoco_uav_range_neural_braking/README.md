# 室内无人机测距制动与定高仿真

## 功能

MuJoCo 纵向飞行模型保留重力、质量和碰撞。水平运动默认使用前向测距与距离差分速度进行连续制动，竖直方向采用重力补偿和 PD 定高反馈。朝向固定，不模拟姿态和电机动力学。控制基线用于检验制动行为与建立学习控制的实验参照。

## 环境

- Python 3.10～3.11、MuJoCo 3.12.0，依赖见 `requirements.txt`。
- Windows 可运行独立物理仿真；Ubuntu 22.04 与 ROS 2 Humble 用于 launch 启动。

在模块目录执行：

```
python -m pip install -r requirements.txt
python main.py --headless --output results
python main.py
```

也可从任意目录通过绝对路径运行 `main.py`，模型资源按源码位置解析。

ROS 环境中执行：

```
source /opt/ros/humble/setup.bash
ros2 launch ./launch/main.launch.py output:=/tmp/uav_braking
```

`trajectory.csv` 记录时间、间距、速度、高度和控制量，`summary.json` 汇总碰撞步数、最小间距和高度误差。

## 控制原理

目标速度：`v_ref = min(1.5, sqrt(2 × 0.8 × max(d − 0.8, 0)))`；加速度：`clip(3 × (v_ref − v), −2.5, 1.2)`；水平力：`mass × acceleration`。0.8 m 是减速参考间距，并非保证的最终停车间距。

原路径 `src/ground/mujoco_drone_distance_stop/main.py` 继续转发至本模块。 / The original entry point forwards to this module.

## 结果与文档

测距控制在初速度 0、1.5、3 m/s 的三组 10 s 场景均无碰撞，最小间距分别为 0.668362、0.668331、0.668360 m。神经控制在默认 1.5 m/s 场景无碰撞，最小间距 0.459629 m；3 m/s 外推场景记录 3 个接触步。21 项自动化检查通过。

完整方程、曲线和动图见[模块文档](../../../docs/air/mujoco_uav_range_neural_braking.md)。

容器环境：

```
docker build -t uav-braking:humble .
docker run --rm --mount type=bind,source="$PWD",target=/work/module uav-braking:humble bash -lc 'source /opt/ros/humble/setup.bash && ros2 launch ./launch/main.launch.py'
```

以上挂载命令在 Bash 中执行；Windows PowerShell 使用 `${PWD}` 表示当前路径。

## 数据与控制模式

```
python main.py --headless --controller range --output results
python main.py --headless --controller reference --output results/reference
python dataset.py --seed 2026 --episodes 30 --output datasets
python evaluate.py --controller range --render --output results/range
python -m pytest -q
```

`range` 使用前向测距和差分闭合速度；`reference` 使用物理真值作为参照。`launch` 支持 `controller:=range`、`controller:=reference` 或 `controller:=mlp`，`model` 指定权重路径。测距无效时施加最大反向制动；此措施仅适用于当前前进接近场景，不保证任意方向运动的安全性。首帧闭合速度初始化为零，随后按 `0.8 × 原估计 + 0.2 × 距离差分` 更新。

`range` 使用前向测距和差分闭合速度；`reference` 使用物理真值作为参照；`mlp` 加载训练权重。launch 接受三种控制器及模型路径。测距无效时施加最大反向制动；此措施仅适用于当前前进接近场景，不保证任意方向运动的安全性。首帧闭合速度初始化为零，随后按 `0.8 × 原估计 + 0.2 × 距离差分 / 步长` 更新。

数据默认 30 回合、每回合 12 s，训练/验证/测试为 18/6/6 回合，21600/7200/7200 行。特征为距离和闭合速度，标签为加速度；各回合参数与完整 CSV 的 SHA-256 写入 `manifest.json`。完整数据保留在 `datasets/`，仓库的 `sample_data/` 提供字段样本及汇总。

训练标签来自解析测距控制基线。MLP 使用测距与闭合速度两个特征，高度由独立 PD 回路保持。验证集误差用于选择训练轮次，并在达到耐心阈值后提前停止。/ Training labels come from the analytical ranging controller. The MLP uses range and closing speed; a separate PD loop holds altitude. Validation error selects the retained training epoch and triggers early stopping.

`evaluate.py --controller all` 可重复运行几何参照、测距控制和 MLP 三种控制器在 0、1.5、3 m/s 下的闭环比较，并输出各控制器曲线与 JSON 指标。
