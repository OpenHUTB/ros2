# YOLOv8-based Visual Target Tracking for Mobile Robots

## 一. 项目简介
本项目基于 ROS Noetic 和 Gazebo 仿真环境，结合 YOLOv8 深度学习算法，实现移动机器人对特定目标的视觉感知与自主跟随。

## 二. 项目技术路线

### 模块1：仿真环境搭建与基础控制
1. 使用 `roslaunch turtlebot3_gazebo turtlebot3_world.launch` 启动仿真世界。
2. 通过键盘控制节点验证机器人底盘 `/cmd_vel` 话题通信。
3. 验证 `/camera/rgb/image_raw` 图像话题正常发布。

### 模块2：视觉感知模块（YOLOv8 + ROS 封装）
1. 在 ROS 节点中集成 Ultralytics 库，加载 `yolov8n.pt` 权重。
2. 利用 `cv_bridge` 将 ROS 图像转换为 OpenCV 格式。
3. 通过 `/target_position` 话题将目标坐标偏差发布出去

### 模块3：目标跟随控制模块
1. 订阅 `/target_position` 话题接收目标偏差。
2. 将偏差转化为速度指令，并发布到 `/cmd_vel`。若未识别到目标，则发送停止指令。

## 三. 环境搭建指引
1. **ROS 环境**：Ubuntu 20.04 + ROS Noetic。
2. **安装依赖**：
   ```bash
   sudo apt install python3-opencv turtlebot3-gazebo
   pip install ultralytics

##  四.文件结构
```text
src/yolov8-target-tracking/
├── scripts/
│   ├── main.py                     # YOLOv8 视觉检测节点（入口）
│   └── main_follower.py            # 目标跟随控制节点
├── launch/
│   └── main.launch                 # 一键启动文件
├── CMakeLists.txt
├── package.xml
└── requirements.txt
```
## 五. 运行说明
1.编译工作空间：
```
bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

2.启动仿真环境（新开终端）：
```
bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

3.启动完整项目（新开终端）：
```
bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch yolov8-target-tracking main.launch
```
(注意：请将 yolov8n.pt 放在 scripts/ 目录下与代码同层)


## 六. 实验数据与系统指标

1.相机话题：/camera/rgb/image_raw

2.ROS 版本：ROS Noetic

3.图像分辨率：384x640

4.是否需要显示器：是（需显示 Gazebo 与 YOLO 窗口）

5.推理耗时：在无 GPU 加速环境下（强制 CPU），单帧约 112ms。

6.控制频率：基于单帧约 120ms 的总耗时，频率约为 8Hz - 10Hz。

7.跟随误差：采用 P 控制，理论上画面中心偏移误差控制在图像宽度的 ±5% 以内。

## 七. 运行效果

![Gazebo仿真环境](turtlebot3_gazebo_simulation_world.png)
![YOLOv8检测效果](yolov8_ros_gazebo_detection.png)

以下截图为本人在重构“感知-控制”闭环代码后的最新实机复现效果。在无目标场景下，系统正确执行了停止指令：

![最新运行截图](yolov8_ros_cpu_inference_result.png.png)

```
