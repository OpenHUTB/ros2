# MuJoCo 失落公园无人机视觉转向 / MuJoCo abandoned-park UAV visual steering

模块在林荫步道场景中提供确定性无人机运动、前视相机和转向控制接口，场景由 MuJoCo 3.12.0 加载。

The module provides deterministic UAV motion, a forward-camera model, and steering interfaces in a tree-lined path scene loaded by MuJoCo 3.12.0.

## 功能范围 / Scope

本分支提供数据采集、CNN 训练与离线回放，并将图像推理封装为 ROS 2 Humble 的 Image-to-Twist 节点。

This branch provides data collection, CNN training, and offline replay, and packages image inference as a ROS 2 Humble Image-to-Twist node.

## 坐标、时间与相机 / Coordinates, timing, and camera

步道沿世界坐标 (+Y) 延伸，横向坐标为 (X)。正归一化转向量增加航向角，使无人机向左转；负值向右转。指令范围为 `[-1, 1]`，对应 ROS 的正 `angular.z` 左转约定。无人机位置更新使用：

The path runs along world (+Y), with lateral position (X). Positive normalized steering increases yaw and turns left; negative steering turns right. The range is `[-1, 1]`, consistent with positive ROS `angular.z` for a left turn. Position updates follow:

$$
\theta_{t+1}=\theta_t+u_t\omega_{\max}\Delta t,\qquad
x_{t+1}=x_t-v_t\sin\theta_{t+1}\Delta t,\qquad
y_{t+1}=y_t+v_t\cos\theta_{t+1}\Delta t.
$$

`timestep_seconds` 写入 MuJoCo 模型的物理步长，控制周期由 `control_hz` 与物理步数共同确定。基础运动过程没有随机扰动；`seed` 字段用于数据采集与划分阶段的可复现性，不改变基础运动方程。

`timestep_seconds` sets the MuJoCo model timestep. The control period is determined by `control_hz` and the number of physics steps. Base motion contains no random disturbance; `seed` supports reproducible data collection and splitting without changing the motion equations.

MJCF 中的 `fovy=78` 表示垂直视场角。以默认 320 × 180 宽高比计算，水平视场角约为 110°：

The MJCF `fovy=78` specifies the vertical field of view. At the default 320 × 180 aspect ratio, the horizontal field of view is approximately 110°:

$$
\mathrm{FOV}_h=2\arctan\left(\frac{W}{H}\tan\frac{\mathrm{FOV}_v}{2}\right).
$$

## 运行环境 / Environment

- Python 3.10 或更新版本 / Python 3.10 or later
- MuJoCo 3.12.0
- NumPy、Pillow、PyYAML / NumPy, Pillow, and PyYAML
- 训练和推理使用 PyTorch CPU 与 scikit-learn / Training and inference use CPU PyTorch and scikit-learn
- ROS 2 Humble 与 Ubuntu 22.04 用于 ROS 节点 / ROS 2 Humble on Ubuntu 22.04 for ROS nodes

```bash
cd src/air/mujoco_abandoned_park_uav_visual_steering
python -m pip install -r requirements.txt
python main.py --steps 120 --steering 0.0
python -m pytest tests -q
```

配置与场景路径默认相对模块目录解析，运行不依赖调用终端所在目录。

Default configuration and scene paths resolve relative to the module directory, independently of the caller's working directory.

## RGB 数据与专家监督 / RGB data and expert supervision

采集器从前视相机读取 RGB8 PNG 图像，并生成 `metadata.csv`、`manifest.json` 和 `quality_report.json`。专家使用横向位置与航向误差形成负反馈：

The collector reads RGB8 PNG frames from the forward camera and writes `metadata.csv`, `manifest.json`, and `quality_report.json`. The expert uses lateral position and heading error in a negative-feedback controller:

$$
u_t=\operatorname{clip}(k_x x_t-k_\theta\theta_t,-1,1).
$$

因此，位于步道右侧的无人机得到正转向量和 `LEFT` 标签，位于左侧得到负转向量和 `RIGHT` 标签。安全边界外或到达终点时输出 `STOP`。数据同时包含常规起点与终点前起点；终点停车标签来自实际推进后到达终点的状态。

A UAV on the right of the path receives positive steering and the `LEFT` label; one on the left receives negative steering and `RIGHT`. The expert emits `STOP` outside the safe corridor or at the path end. Collection includes regular starts and starts near the endpoint; endpoint stop labels follow actual movement to the endpoint.

同一初始横向偏移的回合属于同一场景组，整组只分配至训练、验证或测试中的一个集合。质量检查同时检查回合隔离、场景组隔离、跨集合图像哈希、图像可读性及每个集合的四类标签覆盖。

Episodes sharing an initial lateral offset form one scenario group, assigned wholly to training, validation, or test. Quality checks cover episode isolation, scenario-group isolation, cross-split image hashes, image readability, and four-class coverage within each split.

```bash
python main.py --mode collect --output data/park_rgb
python main.py --mode quality --output data/park_rgb
```

## CNN 训练与权重契约 / CNN training and checkpoint contract

图像缩放为 160 × 90 并归一化至 `[0, 1]`。轻量 CNN 通过三组卷积、池化和全连接层预测四类标签，使用带类别权重的交叉熵训练：

Images are resized to 160 × 90 and normalized to `[0, 1]`. A lightweight CNN uses convolution, pooling, and fully connected layers to predict four labels and is trained with class-weighted cross-entropy:

$$
\hat y=\operatorname{softmax}(f_\theta(I)),\qquad
\mathcal L=-\sum_{c=1}^{4}w_c y_c\log\hat y_c.
$$

统一入口支持训练与离线图像回放：

The unified entry point supports training and offline image replay:

```bash
python main.py --mode train --output data/park_rgb --checkpoint models/weights/lightweight_steering_cnn.pt --epochs 8
python main.py --mode replay --checkpoint models/weights/lightweight_steering_cnn.pt --images data/park_rgb/images
```

验证集用于选择权重，测试集用于最终分类评价。Accuracy、固定四类 macro-F1 和 4 × 4 混淆矩阵使用相同标签序号；即使某类没有出现在预测结果中，macro-F1 仍保留该类。加载权重时要求保存的标签严格为 `LEFT, STRAIGHT, RIGHT, STOP`，缺失或顺序不符时拒绝加载，防止控制含义被交换。

Validation selects the checkpoint; the test split provides final classification metrics. Accuracy, fixed-four-class macro-F1, and the 4 × 4 confusion matrix share the same class indices. Macro-F1 retains every class even when a class is absent from predictions. Checkpoints must store the exact label order `LEFT, STRAIGHT, RIGHT, STOP`; missing or reordered labels are rejected to preserve control semantics.

## ROS 2 图像控制接口 / ROS 2 image-control interface

节点订阅 `sensor_msgs/Image` 并发布 `geometry_msgs/Twist`：`LEFT` 对应正 `angular.z`，`RIGHT` 对应负值，`STRAIGHT` 的角速度为零，`STOP` 的线速度和角速度均为零。包安装包括节点、launch 文件与参数配置，launch 从安装后的包共享目录解析配置。

The node subscribes to `sensor_msgs/Image` and publishes `geometry_msgs/Twist`: `LEFT` maps to positive `angular.z`, `RIGHT` to negative, `STRAIGHT` to zero angular velocity, and `STOP` to zero linear and angular velocity. Installation includes the node, launch file, and parameter configuration; launch resolves configuration from the installed package share directory.

从仓库工作空间根目录构建，提供训练所得权重：

Build from the repository workspace root and supply the trained checkpoint:

```bash
colcon build --packages-select mujoco_abandoned_park_uav_visual_steering
source install/setup.bash
ros2 launch mujoco_abandoned_park_uav_visual_steering main.launch.py \
  checkpoint:=/absolute/path/lightweight_steering_cnn.pt
```

缺少有效权重、图像解码失败、推理异常或图像流超过超时阈值时，节点发布零速度。图像问题消除后，下一张有效图像可恢复正常控制；缺少权重时保持零速，提供有效权重后重新启动节点。

The node publishes zero velocity when a valid checkpoint is unavailable, image decoding or inference fails, or the image stream exceeds the timeout threshold. A subsequent valid image can resume control after image-related faults. Without a checkpoint, the node remains stopped; restart it with a valid checkpoint to enable inference.



