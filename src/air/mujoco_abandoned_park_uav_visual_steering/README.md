# MuJoCo 失落公园无人机视觉转向

模块在林荫步道场景中提供确定性无人机运动、前视相机和转向控制接口，场景由 MuJoCo 3.12.0 加载。

## 功能范围

本分支整合数据采集、CNN 训练、ROS 2 图像控制接口及真实相机闭环评价，并提供建图与导航辅助接口。

## 坐标、时间与相机

步道沿世界坐标 (+Y) 延伸，横向坐标为 (X)。正归一化转向量增加航向角，使无人机向左转；负值向右转。指令范围为 `[-1, 1]`，对应 ROS 的正 `angular.z` 左转约定。无人机位置更新使用：

$$
\theta_{t+1}=\theta_t+u_t\omega_{\max}\Delta t,\qquad
x_{t+1}=x_t-v_t\sin\theta_{t+1}\Delta t,\qquad
y_{t+1}=y_t+v_t\cos\theta_{t+1}\Delta t.
$$

`timestep_seconds` 写入 MuJoCo 模型的物理步长，控制周期由 `control_hz` 与物理步数共同确定。基础运动过程没有随机扰动；`seed` 字段用于数据采集与划分阶段的可复现性，不改变基础运动方程。

MJCF 中的 `fovy=78` 表示垂直视场角。以默认 320 × 180 宽高比计算，水平视场角约为 110°：

$$
\mathrm{FOV}_h=2\arctan\left(\frac{W}{H}\tan\frac{\mathrm{FOV}_v}{2}\right).
$$

## 运行环境

- Python 3.10 或更新版本
- MuJoCo 3.12.0
- 训练和推理使用 PyTorch CPU 与 scikit-learn
- ROS 2 Humble 与 Ubuntu 22.04 用于 ROS 节点

```bash
cd src/air/mujoco_abandoned_park_uav_visual_steering
python -m pip install -r requirements.txt
python main.py --steps 120 --steering 0.0
python -m pytest tests -q
```

配置与场景路径默认相对模块目录解析，运行不依赖调用终端所在目录。

## RGB 数据与专家监督

采集器从前视相机读取 RGB8 PNG 图像，并生成 `metadata.csv`、`manifest.json` 和 `quality_report.json`。专家使用横向位置与航向误差形成负反馈：

$$
u_t=\operatorname{clip}(k_x x_t-k_\theta\theta_t,-1,1).
$$

因此，位于步道右侧的无人机得到正转向量和 `LEFT` 标签，位于左侧得到负转向量和 `RIGHT` 标签。安全边界外或到达终点时输出 `STOP`。数据同时包含常规起点与终点前起点；终点停车标签来自实际推进后到达终点的状态。

同一初始横向偏移的回合属于同一场景组，整组只分配至训练、验证或测试中的一个集合。质量检查同时检查回合隔离、场景组隔离、跨集合图像哈希、图像可读性及每个集合的四类标签覆盖。

```bash
python main.py --mode collect --output data/park_rgb
python main.py --mode quality --output data/park_rgb
```

## CNN 训练与权重契约

图像缩放为 160 × 90 并归一化至 `[0, 1]`。轻量 CNN 通过三组卷积、池化和全连接层预测四类标签，使用带类别权重的交叉熵训练：

$$
\hat y=\operatorname{softmax}(f_\theta(I)),\qquad
\mathcal L=-\sum_{c=1}^{4}w_c y_c\log\hat y_c.
$$

统一入口支持训练与离线图像回放：

```bash
python main.py --mode train --output data/park_rgb --checkpoint models/weights/lightweight_steering_cnn.pt --epochs 8
python main.py --mode replay --checkpoint models/weights/lightweight_steering_cnn.pt --images data/park_rgb/images
```

验证集用于选择权重，测试集用于最终分类评价。Accuracy、固定四类 macro-F1 和 4 × 4 混淆矩阵使用相同标签序号；即使某类没有出现在预测结果中，macro-F1 仍保留该类。加载权重时要求保存的标签严格为 `LEFT, STRAIGHT, RIGHT, STOP`，缺失或顺序不符时拒绝加载，防止控制含义被交换。

本次固定种子 CNN 训练运行 16 轮（19.60 s），按场景组隔离得到 312/78/78 张训练、验证和测试图像。独立测试集（78 张）准确率为 64.10%，四类 macro-F1 为 67.42%；单帧推理延迟均值为 0.356 ms，P95 为 0.416 ms。

## ROS 2 图像控制接口

节点订阅 `sensor_msgs/Image` 并发布 `geometry_msgs/Twist`：`LEFT` 对应正 `angular.z`，`RIGHT` 对应负值，`STRAIGHT` 的角速度为零，`STOP` 的线速度和角速度均为零。包安装包括节点、launch 文件与参数配置，launch 从安装后的包共享目录解析配置。

从仓库工作空间根目录构建，提供训练所得权重：

```bash
colcon build --packages-select mujoco_abandoned_park_uav_visual_steering
source install/setup.bash
ros2 launch mujoco_abandoned_park_uav_visual_steering main.launch.py \
  checkpoint:=/absolute/path/lightweight_steering_cnn.pt
```

缺少有效权重、图像解码失败、推理异常或图像流超过超时阈值时，节点发布零速度。图像问题消除后，下一张有效图像可恢复正常控制；缺少权重时保持零速，提供有效权重后重新启动节点。

## MuJoCo 三场景闭环评价

`evaluation.py` 在直道、正弦曲道和带中心柱障碍的三种独立 MuJoCo 场景中评价控制器。每个初始横向偏移分别运行规则控制与 CNN 控制；CNN 在每一步从当前前视相机图像推理，再将动作送回仿真，生成自己的独立轨迹。

```bash
python main.py --mode evaluate \
  --checkpoint models/weights/lightweight_steering_cnn.pt \
  --output evaluation/scenarios.json
```

每回合记录位置、航向、标签、横向误差、前进距离及 CNN 推理次数。成功回合须同时满足：最大横向误差小于安全走廊界限、无人机中心到柱体中心的水平距离始终大于 0.5 m，以及沿 Y 方向的实际进度至少达到该时长理论前进距离的 60%。报告由这些轨迹统计平均横向误差和成功率，并记录权重 SHA-256；缺少权重时不生成 CNN 分数。

## 建图与导航辅助接口

`navigation.py` 提供占用栅格射线更新与四邻域 A* 基线；`slam.py` 积分里程计并把测距射线端点投影到全局栅格。`SynchronizedSlamNavigation.step` 顺序更新里程计、扫描和地图，再调用 MLP 局部策略。该接口处理给定的里程计和测距数据，不包含扫描匹配或回环优化。

导航 MLP 由 `train_neural_navigation.py` 使用 A* 首步监督训练。固定种子 `250102` 下，训练集为 1,200 张随机可达地图，留出测试集为 300 张地图，各训练 24 轮；训练准确率为 83.94%，留出地图准确率为 82.78%。保存的权重为 3,457 字节。

[建图与 A* 导航动图](../../../docs/img/air/mujoco_abandoned_park_uav_visual_steering/mapping_navigation_demo.gif)
