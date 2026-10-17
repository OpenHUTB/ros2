# Carla PilotNet 端到端控制 ROS 封装

> **重要提示**：本模块默认权重 `pilotnet.pth` 为随机初始化，仅用于验证 ROS 话题链路通断，**无实际自动驾驶控制效果**。如需实际控制效果，需用采集的数据训练模型。详见「模型训练」章节。

`carla_pilotnet_ros` 是基于 PyTorch 实现的 NVIDIA PilotNet 端到端自动驾驶网络，封装为 ROS Noetic catkin 包。模块订阅相机图像，经 CNN 推理输出转向与油门，发布车辆控制指令。

## 功能

- **image_publisher**：从本地图片目录循环发布图像到 `/camera/image_raw`
- **pilotnet_inference**：订阅图像，PilotNet CNN 推理，发布 `/pilotnet/control` 和 `/pilotnet/annotated_image`
- **control_node**：订阅 `/pilotnet/control`，映射为 `/vehicle/control_cmd`（Twist）

## 话题

话题统一加命名空间 `carla_pilotnet_ros`，避免与其他相机节点冲突：

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/carla_pilotnet_ros/camera/image_raw` | `sensor_msgs/Image` | 发布 | 图像输入 |
| `/carla_pilotnet_ros/pilotnet/control` | `std_msgs/Float32MultiArray` | 发布 | [steering, throttle] |
| `/carla_pilotnet_ros/pilotnet/annotated_image` | `sensor_msgs/Image` | 发布 | 带控制值标注的图像 |
| `/carla_pilotnet_ros/vehicle/control_cmd` | `geometry_msgs/Twist` | 发布 | linear.x=throttle, angular.z=steering |

**中间消息类型说明**：`/pilotnet/control` 使用标准 `std_msgs/Float32MultiArray`，data[0] 为 steering，data[1] 为 throttle，无需自定义 msg。

## 神经网络原理

PilotNet 是 NVIDIA 提出的端到端自动驾驶 CNN。网络结构为 5 个卷积层 + 3 个全连接层，参数量约 25 万。

- **输入**：66×200×3 RGB 图像
- **卷积层**：5×5 stride 2 (24/36/48 filters) + 3×3 (64/64 filters)
- **全连接层**：100 → 50 → 10 → 2
- **激活函数**：ELU
- **Dropout**：0.2
- **输出**：2 维（steering, throttle）

损失函数（回归任务）：

$$
L = \frac{1}{N} \sum_{i=1}^{N} (\hat{y}_i - y_i)^2
$$

## 算法流程

1. `image_publisher` 从 `assets/` 目录循环读取图片，发布到 `/camera/image_raw`
2. `pilotnet_inference` 订阅图像，每 2 帧执行一次推理：
   - 图像预处理：BGR → RGB，resize 到 66×200，归一化到 [0,1]
   - CNN 推理：输出 (steering, throttle)
   - 发布到 `/pilotnet/control` 和 `/pilotnet/annotated_image`
3. `control_node` 订阅 `/pilotnet/control`，映射为 `Twist` 消息发布到 `/vehicle/control_cmd`

## 图像预处理

`image_publisher` 发布 bgr8 编码的原始图像。`pilotnet_inference` 在推理前执行：

1. **编码转换**：BGR → RGB（OpenCV 默认 BGR，PilotNet 要求 RGB）
2. **尺寸调整**：resize 到 66×200（网络输入尺寸）
3. **归一化**：像素值除以 255，归一化到 [0,1]

无论输入图像原始尺寸是多少，推理节点都会自动 resize，不会因尺寸不匹配报错。

## 运行环境

- Ubuntu 20.04 + ROS Noetic
- Python 3.8（conda 环境 `carla38`）
- PyTorch 2.0.1+cpu、Torchvision 0.15.2+cpu

## 依赖安装

### 方式 A（推荐）：conda 环境 + main.sh

    conda activate carla38
    pip install torch==2.0.1 torchvision==0.15.2 --index-url https://download.pytorch.org/whl/cpu --no-deps
    pip install opencv-python pillow -i https://mirrors.aliyun.com/pypi/simple/

启动时用 `bash main.sh`（会自动激活 conda 并调用 roslaunch）。

### 方式 B：让 roslaunch 使用 conda Python

ROS Noetic 默认调用系统 Python 3.8，若依赖只装在 conda 会报 `ModuleNotFoundError`。可以：

    export PYTHONPATH=/home/user/miniconda3/envs/carla38/lib/python3.8/site-packages:$PYTHONPATH
    roslaunch carla_pilotnet_ros main.launch

## 编译与运行

### 1. 编译 catkin 工作空间

    mkdir -p ~/carla_pilotnet_ws/src
    cd ~/carla_pilotnet_ws/src
    ln -sfn ~/ros2/src/ground/carla_pilotnet_ros carla_pilotnet_ros
    cd ~/carla_pilotnet_ws
    source /opt/ros/noetic/setup.bash
    catkin_make

### 2. 启动

    conda activate carla38
    source /opt/ros/noetic/setup.bash
    source ~/carla_pilotnet_ws/devel/setup.bash
    roslaunch carla_pilotnet_ros main.launch

或使用入口脚本 `main.sh`：

    bash src/ground/carla_pilotnet_ros/main.sh

**main.py 与 roslaunch 的区别**：

- `roslaunch carla_pilotnet_ros main.launch`：直接启动 ROS 节点，需要手动 source ROS 环境
- `python main.py` 或 `bash main.sh`：封装入口，自动 source ROS 环境 + 激活 conda 环境 + 调用 roslaunch
- 推荐用 `main.sh`，避免环境配置问题

## 运行验证

启动后，另开终端验证话题链路：

    source /opt/ros/noetic/setup.bash
    source ~/carla_pilotnet_ws/devel/setup.bash

    # 查看话题列表
    rostopic list | grep carla_pilotnet_ros

    # 查看控制量输出
    rostopic echo /carla_pilotnet_ros/pilotnet/control

    # 查看车辆控制指令
    rostopic echo /carla_pilotnet_ros/vehicle/control_cmd

    # 查看标注图像
    rosrun image_view image_view image:=/carla_pilotnet_ros/pilotnet/annotated_image

**正常输出示例**（inference 节点终端）：

    [INFO] Loaded weights: /.../carla_pilotnet_ros/pilotnet.pth
    [INFO] PilotNet inference node ready
    [INFO] steering=0.024 throttle=-0.267
    [INFO] steering=0.031 throttle=-0.251

## 模型训练

本模块默认提供随机初始化的权重文件用于演示。如需训练：

1. 在 Carla 中采集图像 + 控制指令数据集
2. 使用 `src/model.py` 中的 PilotNet 模型训练（损失函数 MSELoss）
3. 保存权重到 `pilotnet.pth`

## 异常处理与友好提示

- **torch 导入失败**：提示安装命令
- **权重文件缺失**：提示下载地址，使用随机初始化继续运行
- **订阅超时**：5 秒未收到图像时警告，提示检查 image_publisher
- **图像解码失败**：捕获异常，输出错误信息不崩溃
- **推理异常**：捕获异常，输出错误信息

## 已知限制

- 默认权重为随机初始化，仅用于演示 ROS 话题链路
- 图像源为本地图片，接入真实 Carla 相机时替换 `image_publisher` 为 Carla ROS bridge 的相机节点
