# 仓储通道巡检无人机测距与制动

MuJoCo 仓储通道内，固定姿态无人机通过九束水平测距获取障碍距离，并按制动距离规则调整前进速度。模型包含静态障碍、横穿障碍和窄通道场景。

## 运行环境

Ubuntu 22.04、ROS 2 Humble、Python 3.10、MuJoCo 3.12.0。Docker Desktop 使用 Linux 容器。

```bash
python3 -m pip install -r requirements.txt
python3 main.py --scenario static --duration 8 --output results/static
python3 main.py --scenario crossing --duration 8 --output results/crossing
python3 main.py --scenario narrow --duration 8 --output results/narrow
```

在仓库根目录构建 ROS 包并启动参考控制节点：

```bash
colcon build --packages-select mujoco_warehouse_uav_collision_braking
source install/setup.bash
ros2 launch mujoco_warehouse_uav_collision_braking main.launch.py scenario:=crossing duration:=8.0
```

Docker Desktop 运行方式：
在模块目录构建并启动容器。输出目录通过挂载保存。
Build and run the container from this module directory. Mount an output directory to retain results.

```bash
docker build -t warehouse-uav:humble .
docker run --rm -v "${PWD}/results:/results" warehouse-uav:humble bash -lc "source /workspace/install/setup.bash && ros2 launch mujoco_warehouse_uav_collision_braking main.launch.py output:=/results"
```

## ROS 接口

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/warehouse/scan` | `sensor_msgs/LaserScan` | 九束距离，米 |
| `/warehouse/pose` | `geometry_msgs/PoseStamped` | 仓储坐标位姿 |
| `/warehouse/reference_velocity` | `geometry_msgs/TwistStamped` | 参考速度 |
| `/warehouse/status` | `std_msgs/String` | 运行统计 |

## 监督数据与模型

每条完整轨迹只进入一个数据分区。20 维输入由九束距离、九束闭合速度、实际速度和巡航速度组成。MLP 结构为 20→24→16→4，输出三类风险概率和 0 至 3 m/s 的速度建议；标准化参数仅从训练分区计算。

```bash
python3 main.py --collect --episodes 30 --seed 42 --duration 8 --output dataset
python3 main.py --train --dataset dataset --model model --epochs 80 --seed 42
python3 main.py --evaluate --dataset dataset --model model
python3 main.py --benchmark --model model --output results
```

固定种子测试集包含 2008 个样本，风险准确率为 0.770，速度 MAE 为 0.820 m/s。完整混淆矩阵和机器可读指标见[测试指标](../../../docs/air/mujoco_warehouse_uav_collision_braking/test_metrics.json)。

## 验证

```bash
python3 -m unittest discover -s tests -v
```

测试覆盖射线几何、参考制动、接触检测、数据分区隔离、模型保存与加载、梯度更新、独立测试和三场景基准。场景原理、运行结果与控制组对照见[项目文档](../../../docs/air/mujoco_warehouse_uav_collision_braking.md)。

当前 ROS 节点提供参考制动速度接口；神经网络训练、测试及三场景比较由上述离线命令运行。
