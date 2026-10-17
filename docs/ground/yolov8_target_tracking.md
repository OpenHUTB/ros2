# YOLOv8-based Visual Target Tracking for Mobile Robots

## 一. 项目简介
本项目基于 ROS Noetic 和 Gazebo 仿真环境，结合 YOLOv8 深度学习算法，实现移动机器人对特定目标的视觉感知与自主跟随。

## 二. 项目技术路线

### 模块1：仿真环境搭建与基础控制
1. 使用 `roslaunch turtlebot3_gazebo turtlebot3_simulation.launch` 启动包含 TurtleBot3 及障碍物的仿真世界。
2. 通过键盘控制节点验证机器人底盘 `/cmd_vel` 话题通信。
3. 验证 `/camera/rgb/image_raw` 图像话题正常发布。

### 模块2：视觉感知模块（YOLOv8 + ROS 封装）
1. 在 ROS 节点中集成 Ultralytics 库，加载 `yolov8n.pt` 权重。
2. 利用 `cv_bridge` 将 ROS 图像消息转换为 OpenCV 格式。
3. 将图像输入 YOLOv8 模型进行推理，获取检测框坐标、类别和置信度。
4. 通过 `/target_position` 话题将目标坐标偏差发布出去。

### 模块3：目标跟随控制模块
1. 订阅 `/target_position` 话题接收目标偏差。
2. 将偏差转化为机器人的角速度（`angular.z`）和线速度（`linear.x`）。
3. 发布速度指令到 `/cmd_vel`，驱动机器人跟随目标。

### 模块4：工程化封装与文档撰写
1. 编写 `main.launch`，一键启动所有节点。
2. 采用规范的代码结构，代码放于 `src` 目录。
3. 使用 Markdown 编写实验报告。

## 三. 环境搭建指引

在运行本项目之前，请确保已配置好以下运行环境：

1. **ROS 环境**：Ubuntu 20.04 + ROS Noetic。
2. **安装系统级依赖 (OpenCV、TurtleBot3仿真包)**：
   ```bash
   sudo apt update
   sudo apt install python3-opencv turtlebot3-gazebo
3. **安装 Python 依赖 (YOLO库)**：
   请运行以下命令安装 YOLOv8 运行所需的 Python 库：
   ```bash
   pip install ultralytics
   
##  四.文件结构

```text
src/yolov8-target-tracking/
├── src/
│   ├── main.py                     # YOLOv8 视觉检测节点（入口）
│   └── main_follower.py            # 目标跟随控制节点
├── launch/
│   └── main.launch                 # 一键启动文件
├── CMakeLists.txt
├── package.xml
└── requirements.txt
```

## 五. 运行说明

1.启动仿真环境：roslaunch turtlebot3_gazebo turtlebot3_simulation.launch

2.启动完整项目：roslaunch yolov8-target-tracking main.launch

注意： 请将 yolov8n.pt 权重文件下载后，放置在 src/yolov8-target-tracking/src/ 目录下，与 main.py 放在同一文件夹内。（权重文件较大，未随仓库上传）

## 六. 实验数据与系统指标

| 指标项 | 详细数据说明 |
| :--- | :--- |
| **相机话题** | `/camera/rgb/image_raw` |
| **ROS 版本** | ROS Noetic |
| **图像分辨率** | 384x640 |
| **是否需要显示器** | 是（需显示 Gazebo 仿真界面与 YOLO 检测窗口） |
| **推理耗时** | 在无 GPU 加速的虚拟机环境下实测，YOLOv8n 模型单帧推理耗时（Inference）在 111ms - 127ms 之间波动，预处理与后处理耗时约在 10ms - 20ms 左右。 |
| **控制频率** | 基于单帧推理约 130ms 左右的总耗时，视觉检测节点与目标跟随节点的通信及控制频率约为 7Hz - 8Hz。 |
| **跟随误差** | 由于采用简单比例控制（P控制），在目标匀速运动时，画面中心偏移误差能控制在图像宽度的 ±5% 以内。 |
## 七. 运行效果

![Gazebo仿真环境](turtlebot3_gazebo_simulation_world.png)
![YOLOv8检测效果](yolov8_ros_gazebo_detection.png)

```
