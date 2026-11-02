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

## 键盘控制与建图导航 / Keyboard control and mapping navigation

键盘模式使用 `W/A/S/D` 控制前进、左转、右转和后退，`X` 发送停止指令。启动方式如下：

The keyboard mode maps `W/A/S/D` to forward, left, right, and reverse commands; `X` sends a stop command:

```bash
python main.py --mode keyboard --steps 4
```

传感器命中点通过射线更新占用栅格：射线经过的单元标记为空闲，末端命中单元标记为占用；A* 仅在四邻域自由单元中搜索。设栅格代价为 $g(n)$，启发函数为曼哈顿距离 $h(n)$，优先级为 $f(n)=g(n)+h(n)$。

Sensor hit points update an occupancy grid: traversed ray cells become free and the terminal hit becomes occupied. A* searches four-connected free cells with $f(n)=g(n)+h(n)$, where $h(n)$ is the Manhattan distance. The same simulated sensor trace therefore updates the map and supplies the navigation planner, providing a deterministic SLAM-style mapping-and-navigation loop.

增量式 SLAM 模块进一步融合机体坐标系里程计与测距射线。位姿更新为
$(x',y')=(x+\cos\theta\,\Delta x-\sin\theta\,\Delta y,\ y+\sin\theta\,\Delta x+\cos\theta\,\Delta y)$，并把每次扫描的末端命中投影到全局栅格，轨迹和地图同时保存。

The incremental SLAM module fuses body-frame odometry with range rays. The pose update is $(x',y')=(x+\cos\theta\,\Delta x-\sin\theta\,\Delta y,\ y+\sin\theta\,\Delta x+\cos\theta\,\Delta y)$; each scan endpoint is projected into the global grid while the estimated trajectory and map are retained together.

## 启动、验证与证据 / Launch, verification, and evidence

Ubuntu 22.04 与 ROS 2 Humble 中可执行 `ros2 launch` 启动入口；不含 ROS 2 的环境仍可运行 `main.py` 的仿真、键盘、训练、回放和评估模式。验证包括单元测试、`compileall`、`flake8` 和三场景指标输出。文档页面通过 `mkdocs.yml` 与首页入口发布，运行截图或 GIF 应放在 `docs/img/air/mujoco_abandoned_park_uav_visual_steering/` 并在此处引用。

On Ubuntu 22.04 with ROS 2 Humble, use the `ros2 launch` entry point. Environments without ROS 2 can still run the simulation, keyboard, training, replay, and evaluation modes. Verification includes unit tests, `compileall`, `flake8`, and three-scenario metrics. The page is wired into `mkdocs.yml` and the homepage; run screenshots or GIF evidence belongs under `docs/img/air/mujoco_abandoned_park_uav_visual_steering/` and should be linked here.

演示动图 / Demonstration GIF: [建图与 A* 导航](../img/air/mujoco_abandoned_park_uav_visual_steering/mapping_navigation_demo.gif)。


### 同步 SLAM 与神经网络规划 / Synchronized SLAM and neural planning

每个仿真步执行 `odometry -> range scan -> occupancy update -> MLP action`，由 `SynchronizedSlamNavigation.step` 保证建图和规划使用同一时刻的地图状态。MLP 输入局部自由空间与目标相对坐标，隐藏层使用 `tanh`，输出 LEFT/STRAIGHT/RIGHT 三类控制；A* 只用于基线对照。
