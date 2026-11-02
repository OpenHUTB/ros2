# MuJoCo abandoned-park UAV visual steering

失落公园林荫步道场景下基于轻量卷积神经网络的 MuJoCo 无人机端到端视觉转向控制。

End-to-End Visual Steering Control for a MuJoCo UAV in an Abandoned-Park Tree-Lined Path Using a Lightweight CNN.

## Scope / 范围

The current module provides a lightweight MJCF park scene, a camera-equipped quadrotor, deterministic headless flight, and stable interfaces for RGB collection and steering commands. Neural-network training and ROS 2 runtime nodes are introduced in later stages.

当前模块提供轻量 MJCF 公园场景、带前视相机的四旋翼、确定性无窗口飞行，以及 RGB 采集和转向指令所需的稳定接口。神经网络训练与 ROS 2 运行节点在后续阶段接入。

## Environment / 环境

- Python 3.10
- MuJoCo 3.12.0
- NumPy, Pillow, PyYAML and pytest

```bash
cd src/air/mujoco_abandoned_park_uav_visual_steering
python -m pip install -r requirements.txt
```

## Run / 运行

The unified entry point runs a complete deterministic simulation episode and prints a bilingual JSON summary.

统一入口运行完整的确定性仿真回合，并输出中英双语 JSON 摘要。

```bash
python main.py --steps 120 --steering 0.0
```

Collect the reproducible RGB dataset and rerun its quality gate:

采集可复现 RGB 数据集并重新执行质量检查：

```bash
python main.py --mode collect --output data/park_rgb
python main.py --mode quality --output data/park_rgb
```

Train the lightweight CNN and replay frames through the same interface:

使用同一入口训练轻量 CNN 并回放图像：

```bash
python main.py --mode train --output data/park_rgb --checkpoint models/weights/lightweight_steering_cnn.pt --epochs 8
python main.py --mode replay --checkpoint models/weights/lightweight_steering_cnn.pt --images data/park_rgb/images
python main.py --mode evaluate --output evaluation/scenarios.json
```

The model consumes normalized RGB tensors and predicts four classes. A predicted class is mapped to `geometry_msgs/Twist`: LEFT and RIGHT use opposite `angular.z`, STRAIGHT uses zero angular velocity, and STOP sets `linear.x` to zero.

模型输入归一化 RGB 张量并预测四类标签。预测结果映射为 `geometry_msgs/Twist`：LEFT 与 RIGHT 使用相反的 `angular.z`，STRAIGHT 的角速度为零，STOP 将 `linear.x` 设为零。

The collector writes PNG frames, `metadata.csv`, `manifest.json`, and `quality_report.json`. Splits are assigned by episode to prevent adjacent frames from leaking across training, validation, and test sets. Generated data remains outside version control.

采集器写出 PNG 图像、`metadata.csv`、`manifest.json` 和 `quality_report.json`。数据按完整回合划分，避免相邻帧跨越训练集、验证集和测试集。生成数据不纳入版本控制。

Run the checks with:

执行检查：

```bash
python -m pytest tests -q
```

## Layout / 目录

- `models/scene.xml`: quadrotor and tree-lined park MJCF / 四旋翼与林荫公园 MJCF
- `config/simulation.yaml`: reproducible runtime parameters / 可复现运行参数
- `launch/module.yaml`: ROS 2 Humble integration contract / ROS 2 Humble 集成契约
- `mujoco_abandoned_park_uav_visual_steering/`: reusable Python interfaces / 可复用 Python 接口
- `tests/`: behavioral and entry-point checks / 行为与入口检查
- `train.py`, `model.py`: lightweight CNN training and checkpoint format / 轻量 CNN 训练与权重格式
- `ros_node.py`, `launch/main.launch.py`: ROS 2 Humble image-to-Twist interface / ROS 2 Humble 图像转 Twist 接口
- `evaluation.py`: straight, curve, and obstacle scenario comparison / 直道、弯道与障碍物场景对比

## Coordinate and command convention / 坐标与指令约定

The path extends along positive Y. Positive normalized steering turns the vehicle to its right and negative steering turns left. The camera output contract is RGB8 at 320 x 180. Paths are resolved relative to `main.py`, so the command does not depend on the caller's working directory.

步道沿 Y 轴正方向延伸。正归一化转向量使无人机右转，负值使其左转。相机输出接口为 320 x 180 的 RGB8 图像。所有路径均相对 `main.py` 解析，因此运行命令不依赖调用者的工作目录。



