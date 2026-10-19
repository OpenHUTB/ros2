# MuJoCo 失落公园无人机端到端视觉转向

该模块面向林荫步道低空飞行，采用 MuJoCo 3.12.0 构建轻量公园场景和带前视 RGB 相机的四旋翼。当前工程提供确定性仿真、统一运行入口和后续数据采集及 ROS 2 Humble 集成所需的接口边界。

The module targets low-altitude flight along a tree-lined path. MuJoCo 3.12.0 provides a lightweight park scene and a quadrotor with a forward RGB camera. The current implementation includes deterministic simulation, one unified entry point, and interface boundaries for data collection and ROS 2 Humble integration.

## 场景与接口 / Scene and interfaces

- 程序化 MJCF 场景，不依赖大型地图和外部资产。
- 前视相机接口固定为 RGB8、320 x 180。
- 转向指令范围为 `[-1, 1]`，右转为正，左转为负。
- 固定随机种子和相对路径确保运行结果可复现。

- Procedural MJCF scene without large maps or external assets.
- Forward-camera contract fixed to RGB8 at 320 x 180.
- Steering range is `[-1, 1]`, positive for right and negative for left.
- A fixed seed and relative paths keep runs reproducible.

## 运行 / Run

```bash
cd src/air/mujoco_abandoned_park_uav_visual_steering
python -m pip install -r requirements.txt
python main.py --steps 120 --steering 0.0
python main.py --mode collect --output data/park_rgb
python main.py --mode quality --output data/park_rgb
python -m pytest tests -q
```

成功运行后，入口输出包含状态、随机种子、步数、位置和航向角的 JSON 记录。The successful entry-point output is a JSON record containing status, seed, step count, position, and yaw.

## 数据集 / Dataset

前视相机生成 RGB8 PNG 图像。确定性专家根据路径中心偏差和航向误差产生连续转向量，并映射为 `LEFT`、`STRAIGHT`、`RIGHT` 和 `STOP` 标签。数据按回合划分为训练集、验证集和测试集，固定种子保证划分可复现。质量报告检查图像可读性、路径完整性、SHA-256 哈希、标签覆盖及回合泄漏。

The forward camera produces RGB8 PNG images. A deterministic expert converts path-center and heading errors into continuous steering and `LEFT`, `STRAIGHT`, `RIGHT`, or `STOP` labels. Whole episodes are assigned to train, validation, and test splits with a fixed seed. The quality report checks image readability, path integrity, SHA-256 hashes, label coverage, and episode leakage.

## CNN 训练与评价 / CNN training and evaluation

图像先缩放为 $160\times90$，再归一化到 $[0,1]$。网络包含三组卷积特征提取层、全局自适应池化和两层全连接分类器：

$$
\hat{y}=\operatorname{softmax}(W_2\,\operatorname{ReLU}(W_1\,\operatorname{GAP}(f_{conv}(x))+b_1)+b_2)
$$

训练使用带类别权重的交叉熵：

$$
\mathcal{L}=-\sum_{c=1}^{4}w_c\,y_c\log\hat{y}_c
$$

数据划分以完整 episode 为单位，测试输出 accuracy、macro-F1、混淆矩阵和单帧推理延迟。离线回放核心也可在未安装 ROS 2 的环境中运行；ROS 2 Humble 节点订阅 `sensor_msgs/Image` 并发布 `geometry_msgs/Twist`。

The images are resized to $160\times90$ and normalized to $[0,1]$. The network uses three convolutional feature blocks, adaptive global pooling, and a two-layer classifier. Episode-level isolation is preserved during evaluation. The report includes accuracy, macro-F1, a confusion matrix, and single-frame latency. The replay core works without ROS 2; the ROS 2 Humble node subscribes to `sensor_msgs/Image` and publishes `geometry_msgs/Twist`.

## 三类场景 / Three scenarios

`evaluation.py` evaluates straight, curve, and obstacle traces against the path-center rule controller. The result is saved to `evaluation/scenarios.json` and includes mean lateral error, success rate, and label counts for both controllers.

`evaluation.py` 对直道、弯道和障碍物轨迹与路径中心规则控制器进行评价，结果保存到 `evaluation/scenarios.json`，包含两种控制器的平均横向误差、成功率和标签计数。



