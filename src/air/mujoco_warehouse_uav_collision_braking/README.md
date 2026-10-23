# 仓储通道巡检无人机测距与制动 / Warehouse UAV Ranging and Braking

MuJoCo 仓储通道中，固定姿态无人机通过九束测距获取障碍距离，参考控制器依据制动距离调整前进速度。
In a MuJoCo warehouse aisle, a fixed-attitude UAV measures obstacles with nine range beams and adjusts forward speed using a stopping-distance reference controller.

## 运行环境 / Environment

Ubuntu 22.04、ROS 2 Humble、Python 3.10、MuJoCo 3.12.0；Docker Desktop 使用 Linux 容器。
Ubuntu 22.04, ROS 2 Humble, Python 3.10 and MuJoCo 3.12.0; use Linux containers in Docker Desktop.

```bash
python3 -m pip install -r requirements.txt
python3 main.py --scenario static --duration 8 --output results/static
python3 main.py --scenario crossing --duration 8 --output results/crossing
python3 main.py --scenario narrow --duration 8 --output results/narrow
```

在模块目录构建并启动容器。输出目录通过挂载保存。
Build and run the container from this module directory. Mount an output directory to retain results.

```bash
docker build -t warehouse-uav:humble .
docker run --rm -v "${PWD}/results:/results" warehouse-uav:humble bash -lc "source /workspace/install/setup.bash && ros2 launch mujoco_warehouse_uav_collision_braking main.launch.py output:=/results"
```

PowerShell 挂载可写为 `--mount "type=bind,source=$($PWD.Path)\results,target=/results"`，运行前创建 `results` 目录。
In PowerShell, use `--mount "type=bind,source=$($PWD.Path)\results,target=/results"` after creating the `results` directory.

## ROS 接口 / ROS Interfaces

| 话题 / Topic | 类型 / Type | 内容 / Content |
| --- | --- | --- |
| `/warehouse/scan` | `sensor_msgs/LaserScan` | 九束距离，米 / Nine distances in meters |
| `/warehouse/pose` | `geometry_msgs/PoseStamped` | 仓储坐标位姿 / Pose in warehouse coordinates |
| `/warehouse/reference_velocity` | `geometry_msgs/TwistStamped` | 参考速度 / Reference velocity |
| `/warehouse/status` | `std_msgs/String` | 运行统计 / Run statistics |

`ros2 launch mujoco_warehouse_uav_collision_braking main.launch.py scenario:=crossing duration:=8.0`

## 模型范围 / Model Scope

无人机具有三轴平移自由度，姿态固定，垂直方向由高度控制器维持；这是一套巡检制动实验模型，不包含旋翼气动和姿态控制建模。
The UAV has three translational degrees of freedom, fixed attitude and altitude feedback. The inspection braking model does not model rotor aerodynamics or attitude control.

`simulation.py` 负责物理场景、测距与制动标签；`main.py` 保存轨迹；`ros_node.py` 发布数据；`main.launch.py` 启动整个运行过程。
`simulation.py` defines physics, ranging and reference braking labels; `main.py` saves trajectories; `ros_node.py` publishes data; `main.launch.py` starts the complete run.

参考制动控制用于数据采集与模型比较。神经网络的训练、独立测试和闭环评测是该模块的后续功能，不将参考控制结果视为神经网络结果。
Reference braking supports data collection and model comparison. Neural training, held-out testing and closed-loop evaluation are separate features; reference results are not neural results.

## 功能计划 / Feature Plan

1. 仓储场景、九束测距、参考制动与 ROS 数据接口。 / Warehouse scenes, nine-beam ranging, reference braking and ROS data interfaces.
2. 轨迹监督数据、隔离划分与质量检查。 / Supervised episodes, isolated splits and quality checks.
3. 多层感知机风险分类、速度回归与独立测试。 / MLP risk classification, speed regression and held-out evaluation.
4. 学习控制的 ROS 接口与参考控制对照。 / ROS integration of learned control and reference comparison.
5. 多场景指标、可视化与原理文档。 / Multi-scenario metrics, visualization and technical documentation.

## 图像与检查 / Visualization and Checks

`python3 main.py --scenario static --render --output results/static` 保存物理场景截图和动画。
`python3 main.py --scenario static --render --output results/static` saves a physics-scene screenshot and animation.

`python3 -m unittest discover -s tests -v` 检查射线几何、制动、接触、运动与高度保持。
`python3 -m unittest discover -s tests -v` checks ray geometry, braking, contact, motion and altitude hold.

原理、场景动画和运行指标见[模块文档](../../../docs/air/mujoco_warehouse_uav_collision_braking.md)。
See the [module documentation](../../../docs/air/mujoco_warehouse_uav_collision_braking.md) for principles, scene animation and run metrics.

## 监督数据 / Supervised Data

```bash
python3 main.py --collect --episodes 30 --seed 42 --duration 8 --output dataset
```

采集器输出 `train.npz`、`validation.npz`、`test.npz` 与 `manifest.json`。每个分区独占完整轨迹；九束距离、九束闭合速度、实际速度和巡航速度组成20维输入。目标为参考速度与三类风险标签，均来自制动距离规则。
The collector writes `train.npz`, `validation.npz`, `test.npz` and `manifest.json`. Each split owns complete episodes. Nine ranges, nine closing rates, actual speed and cruise speed form 20 inputs. Reference speed and three risk labels come from the stopping-distance rule.

固定种子42的30条轨迹得到训练5465、验证1816、测试2008个样本。数据在发生首次接触后停止采样，保留接触前高风险观测；NPZ文件可由命令重建，仓库保存清单与小型样例。
Thirty episodes with seed 42 yield 5465 training, 1816 validation and 2008 test samples. Sampling stops after first contact while retaining pre-contact high-risk observations. The command recreates NPZ files; the repository stores a manifest and a small sample.

闭合速度只使用上一帧，首帧归零且限幅至±10 m/s。完整轨迹隔离用于避免同一运动序列跨分区重叠；数据审计同时检查有限值、量程、目标范围和轨迹交集。
Closing rates use the previous frame, reset to zero at episode start and are clipped to ±10 m/s. Episode isolation prevents overlap of one motion sequence across splits. Auditing checks finite values, ranges, target bounds and episode intersections.

## 模型训练与 ROS 接口 / Model Training and ROS Interface

```bash
python3 main.py --train --dataset dataset --model model --seed 42
python3 main.py --evaluate --dataset dataset --model model
```

模型是共享两层隐藏层的 NumPy MLP，输出三类风险概率和 0 至 3 m/s 的速度参考。训练只读取训练分区；测试命令只读取独立测试分区，并保存准确率、速度平均绝对误差和混淆矩阵。
The model is a NumPy MLP with two shared hidden layers, producing three risk probabilities and a 0–3 m/s speed reference. Training reads only the training split; evaluation reads only the held-out test split and saves accuracy, speed MAE and a confusion matrix.

训练模型后可启动学习控制接口：
After training, start the learned-control interface:

```bash
ros2 launch mujoco_warehouse_uav_collision_braking learned.launch.py model:=model/risk_speed_mlp.npz
```

节点订阅 `/warehouse/scan` 和 `/warehouse/pose`，发布 `/warehouse/learned_velocity` 与 `/warehouse/learned_status`。首帧闭合速度为零，后续闭合速度只由相邻测距计算；模型输出始终限制在 0 至 3 m/s。
The node subscribes to `/warehouse/scan` and `/warehouse/pose`, then publishes `/warehouse/learned_velocity` and `/warehouse/learned_status`. The first closing-rate frame is zero; later rates use adjacent ranges only, and model speed is bounded to 0–3 m/s.
两个 launch 节点共享 `duration` 参数，设置有限时长后会共同完成并正常退出。
`duration` is shared by both launch nodes, so a finite launch completes and exits cleanly after the configured interval.

## 多场景对照 / Multi-Scenario Benchmark

```bash
python3 main.py --benchmark --model model --output results
```

学习速度经过参考制动速度的安全上限约束，再发送到 ROS 话题；这样既保留模型预测，又保留可解释的近障碍制动边界。
The learned speed is clipped by the reference braking speed before publication, retaining model prediction with an interpretable near-obstacle safety bound.

固定种子模型的 8 s 对照均未发生接触：静态、横穿、窄道的末速度分别约为 0、0、0 m/s；风险标签一致率分别为 1.000、0.968、0.000。窄道一致率较低时仍由安全上限控制速度，指标会在后续数据覆盖扩展后重新评估。
The fixed-seed model completed 8-second comparisons without contact. Final speeds for static, crossing and narrow scenes were approximately 0, 0 and 0 m/s; risk agreement was 1.000, 0.968 and 0.000. The narrow-scene agreement is low, while the safety bound still controls speed; it should be reevaluated after broader data coverage.

## 模型训练与 ROS 接口 / Model Training and ROS Interface

```bash
python3 main.py --train --dataset dataset --model model --seed 42
python3 main.py --evaluate --dataset dataset --model model
```

模型是共享两层隐藏层的 NumPy MLP，输出三类风险概率和 0 至 3 m/s 的速度参考。训练只读取训练分区；测试命令只读取独立测试分区，并保存准确率、速度平均绝对误差和混淆矩阵。
The model is a NumPy MLP with two shared hidden layers, producing three risk probabilities and a 0–3 m/s speed reference. Training reads only the training split; evaluation reads only the held-out test split and saves accuracy, speed MAE and a confusion matrix.

训练模型后可启动学习控制接口：
After training, start the learned-control interface:

```bash
ros2 launch mujoco_warehouse_uav_collision_braking learned.launch.py model:=model/risk_speed_mlp.npz
```

节点订阅 `/warehouse/scan` 和 `/warehouse/pose`，发布 `/warehouse/learned_velocity` 与 `/warehouse/learned_status`。首帧闭合速度为零，后续闭合速度只由相邻测距计算；模型输出始终限制在 0 至 3 m/s。
The node subscribes to `/warehouse/scan` and `/warehouse/pose`, then publishes `/warehouse/learned_velocity` and `/warehouse/learned_status`. The first closing-rate frame is zero; later rates use adjacent ranges only, and model speed is bounded to 0–3 m/s.
