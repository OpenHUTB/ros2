# Carla PilotNet 端到端控制 ROS 封装（离线仿真演示）

> **注意**：本模块为离线仿真演示，输入为本地图像，输出为 Twist 控制话题，不直接接入 Carla 实时控制流。默认权重为随机初始化，仅用于验证 ROS 话题链路。

## 神经网络结构
PilotNet 是 NVIDIA 提出的端到端自动驾驶 CNN，本模块基于 PyTorch 实现。
- **输入**：66x200x3 RGB 图像
- **卷积层**：5 个卷积层（5x5 stride 2, 24/36/48 filters；3x3, 64/64 filters）
- **全连接层**：3 层（100 -> 50 -> 10 -> 2）
- **激活函数**：ELU
- **输出**：2 维向量（steering, throttle）
- **参数量**：约 25 万

## 图像预处理
- 编码转换：BGR 转换为 RGB
- 尺寸调整：resize 到 66x200
- 归一化：像素值除以 255，归一化到 [0,1]

## 依赖安装
请先安装 PyTorch CPU 版本（避免拉取 CUDA），再安装其他依赖：

    pip install torch==2.4.1+cpu torchvision==0.19.1+cpu --index-url https://download.pytorch.org/whl/cpu
    pip install -r requirements.txt

## 编译与运行
    mkdir -p ~/carla_pilotnet_ws/src
    ln -sfn ~/ros2/src/ground/carla_pilotnet_ros ~/carla_pilotnet_ws/src/carla_pilotnet_ros
    cd ~/carla_pilotnet_ws
    source /opt/ros/noetic/setup.bash
    catkin_make
    source devel/setup.bash
    conda activate carla38
    roslaunch carla_pilotnet_ros main.launch

## 话题验证
- 查看话题：rostopic list | grep carla_pilotnet_ros
- 查看控制量：rostopic echo /carla_pilotnet_ros/pilotnet/control
- 查看图像：rosrun image_view image_view image:=/carla_pilotnet_ros/pilotnet/annotated_image

## 源码解析
- **推理流程**：image_publisher 发布图像，pilotnet_inference 收到后使用 model.py 中的 CNN 进行推理。
- **异常处理**：对 torch 导入、模型加载、图像解码均做了 try/except 捕获。
- **控制映射**：control_node 将 [steering, throttle] 映射为 Twist。负油门截断为 0 并警告。

## 性能说明
- 输入为本地静态图，输出话题需外部节点转换后才能驱动 Carla。
- 缺少标注测试集，无法提供 mAP 指标。CPU 单帧推理延迟约 50ms。
