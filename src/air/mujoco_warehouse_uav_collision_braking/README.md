# 仓储通道巡检无人机测距与制动

MuJoCo 仓储通道中，固定姿态无人机通过九束测距获取障碍距离，参考控制器依据制动距离调整前进速度。

## 运行环境

Ubuntu 22.04、ROS 2 Humble、Python 3.10、MuJoCo 3.12.0；Docker Desktop 使用 Linux 容器。

```bash
python3 -m pip install -r requirements.txt
python3 main.py --scenario static --duration 8 --output results/static
python3 main.py --scenario crossing --duration 8 --output results/crossing
python3 main.py --scenario narrow --duration 8 --output results/narrow
```

在模块目录构建并启动容器。输出目录通过挂载保存。

```bash
docker build -t warehouse-uav:humble .
docker run --rm -v "${PWD}/results:/results" warehouse-uav:humble bash -lc "source /workspace/install/setup.bash && ros2 launch mujoco_warehouse_uav_collision_braking main.launch.py output:=/results"
```

PowerShell 挂载可写为 `--mount "type=bind,source=$($PWD.Path)\results,target=/results"`，运行前创建 `results` 目录。

## ROS 接口

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/warehouse/scan` | `sensor_msgs/LaserScan` | 九束距离，米|
| `/warehouse/pose` | `geometry_msgs/PoseStamped` | 仓储坐标位姿|
| `/warehouse/reference_velocity` | `geometry_msgs/TwistStamped` | 参考速度|
| `/warehouse/status` | `std_msgs/String` | 运行统计|


## 模型范围

无人机具有三轴平移自由度，姿态固定，垂直方向由高度控制器维持；这是一套巡检制动实验模型，不包含旋翼气动和姿态控制建模。

`simulation.py` 负责物理场景、测距与制动标签；`main.py` 保存轨迹；`ros_node.py` 发布参考控制数据；`learned_node.py` 接收测距和位姿并发布学习控制；两个 launch 文件分别启动参考控制和学习控制。

参考制动控制用于生成监督标签和速度安全上限。基准分别记录纯参考控制、纯学习控制和受制动上限约束的学习控制结果。

## 功能组成

1. 仓储场景、九束测距、参考制动与 ROS 数据接口。
2. 轨迹监督数据、隔离划分与质量检查。
3. 多层感知机风险分类、速度回归与独立测试。
4. 学习控制的 ROS 接口与参考控制对照。
5. 多场景指标、可视化与原理文档。

## 图像与检查

`python3 main.py --scenario static --render --output results/static` 保存物理场景截图和动画。

`python3 -m unittest discover -s tests -v` 运行 16 项单元测试（含 CLI 默认路径 3 项），覆盖射线几何、制动、接触、运动、高度保持、数据划分、模型训练与基准输出。

原理、场景动画和运行指标见[模块文档](../../../docs/air/mujoco_warehouse_uav_collision_braking.md)。

## 监督数据

`--collect` 不传 `--output` 时，写入 `--dataset` 指定目录（默认 `dataset/`）；单次仿真与 `--benchmark` 仍默认写入 `results/`。同时指定 `--collect --benchmark` 时只执行基准测试，不执行采集。

```bash
python3 main.py --collect --episodes 30 --seed 42 --duration 8
```

采集器输出 `train.npz`、`validation.npz`、`test.npz` 与 `manifest.json`。每个分区独占完整轨迹；九束距离、九束闭合速度、实际速度和巡航速度组成20维输入。目标为参考速度与三类风险标签，均来自制动距离规则。

固定种子42的30条轨迹得到训练5465、验证1816、测试2008个样本。数据在发生首次接触后停止采样，保留接触前高风险观测；NPZ文件可由命令重建，仓库保存清单与小型样例。

闭合速度只使用上一帧，首帧归零且限幅至±10 m/s。完整轨迹隔离用于避免同一运动序列跨分区重叠；数据审计同时检查有限值、量程、目标范围和轨迹交集。三个分区的畅通/警戒/制动样本数分别为训练 2437/1195/1833、验证 854/309/653、测试 990/468/550。

两个 launch 节点共享 `duration` 参数，设置有限时长后会共同完成并正常退出。

## 多场景对照

```bash
python3 main.py --benchmark --model model --output results
```

基准将纯参考控制、纯学习控制和带制动上限的学习控制分开运行。学习控制节点发布的速度经过参考制动上限约束；基准中的纯学习结果用于单独衡量模型行为。

基准分别报告参考控制、纯学习控制和带制动上限的学习控制。固定种子下，纯学习控制在三个场景均发生接触；带制动上限的学习控制在三个场景均未发生接触。离线基准使用 MuJoCo 纵向速度真值构造模型输入和参考标签，ROS 学习节点则由 `/warehouse/pose` 相邻位姿差分估速；两者的速度输入口径不同。风险一致率表示与参考标签的一致程度，不表示碰撞概率。

## 模型训练与 ROS 接口

```bash
python3 main.py --train --dataset dataset --model model --seed 42
python3 main.py --evaluate --dataset dataset --model model
python3 main.py --benchmark --model model --output results
```

模型为共享两层隐藏层的 NumPy MLP，结构为 20→24→16→4。前三个输出对应畅通、警戒和制动概率，第四个输出经 Sigmoid 映射为 0 至 3 m/s 的速度建议。输入标准化参数只由训练分区计算，并与权重一同保存。训练只读取训练分区，评估只读取独立测试分区。

固定种子测试集含 2008 个样本，风险准确率为 0.770，速度平均绝对误差为 0.820 m/s。速度回归梯度按批次样本数归一化后，MAE 从旧运行的 0.522 m/s 变为 0.820 m/s；旧值的更新步长依赖样本数，不作为稳定基线。混淆矩阵行表示真实类别，列表示预测类别，类别顺序为畅通、警戒、制动：

| 真实类别 / 预测类别 | 畅通 | 警戒 | 制动 |
| --- | ---: | ---: | ---: |
| 畅通 | 915 | 71 | 4 |
| 警戒 | 220 | 162 | 86 |
| 制动 | 41 | 40 | 469 |

训练模型后可启动学习控制接口：

```bash
ros2 launch mujoco_warehouse_uav_collision_braking learned.launch.py model:=model/risk_speed_mlp.npz
```

节点订阅 `/warehouse/scan` 和 `/warehouse/pose`，发布 `/warehouse/learned_velocity` 与 `/warehouse/learned_status`。首帧闭合速度为零，后续闭合速度只由相邻测距计算；模型输出始终限制在 0 至 3 m/s。
