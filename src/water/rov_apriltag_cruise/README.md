# 水下 ROV AprilTag 视觉定位与定点巡航仿真

基于 ROS 2 Humble + Gazebo Classic 的水下机器人 AprilTag 视觉定位与定点巡航仿真模块。

## 运行环境

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo Classic 11
- OpenCV 4.x（ArUco 模块，随 `ros-humble-desktop` 提供）

## 编译

```bash
cd ~/ros2
colcon build --packages-select rov_apriltag_cruise
source install/setup.bash
```

## 运行

方式一（launch）：

```bash
ros2 launch rov_apriltag_cruise apriltag_cruise.launch.py
```

方式二（main 入口）：

```bash
./src/water/rov_apriltag_cruise/main.sh
# 或
python3 src/water/rov_apriltag_cruise/main.py
```

可选参数：`gui:=false`（无 Gazebo 界面）、`rviz:=false`、`show_camera_gui:=false`。

## 功能

- Gazebo 水下场景内置 3 块 AprilTag 标牌（ID 0/1/2）
- ROV 前视相机实时检测标签并发布位姿 TF
- 识别后自动调整姿态对准标签
- 依次飞往多个标记点循环巡航，到点悬停识别
- RViz 实时显示巡航轨迹、标签坐标系、ROV 位姿与航点

## 主要话题

| 话题 | 类型 | 说明 |
| :--- | :--- | :--- |
| `/rov/camera/image_raw` | sensor_msgs/Image | ROV 前视相机原始图像 |
| `/rov/camera/image_detected` | sensor_msgs/Image | 叠加识别框的图像 |
| `/rov/tag_detections` | std_msgs/Float32MultiArray | 标签 ID 与相对位姿 |
| `/rov/status` | std_msgs/String | 巡航状态机状态 |
| `/rov/trajectory` | nav_msgs/Path | 巡航轨迹 |
| `/rov/waypoints` | visualization_msgs/MarkerArray | 航点与标签坐标轴 |

详细文档见 [docs/water/rov_apriltag_cruise.md](../../../docs/water/rov_apriltag_cruise.md)。

> 本模块代码与文档由大模型辅助生成，核心算法与参数经人工调试验证。
