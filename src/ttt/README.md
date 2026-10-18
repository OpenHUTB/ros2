# 基于 ROS 2 的禁飞区（电子围栏）动态检测与安全预警系统

## 1. 简介
本项目是一个基于 ROS 2 的安全监控模块，支持实时接收载具位姿与动态更新的禁飞区多边形数据。
通过射线法（Ray Casting）实时判断载具位置，并提供 `SAFE`（安全）、`WARN`（接近预警）、`CRITICAL`（侵入告警）三级安全响应。

## 2. 环境依赖
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+

## 3. 编译与运行
### 3.1 编译工作空间
```bash
cd ~/geofence_ws
colcon build --symlink-install
source install/setup.bash
```
### 3.2 启动节点
```bash
ros2 launch geofence_detector geofence.launch.py
```
### 3.3 测试验证
在另一个终端发布模拟数据测试：
```bash
# 发布虚拟载具位置
ros2 topic pub /odom nav_msgs/msg/Odometry "{pose: {pose: {position: {x: 0.0, y: 0.0}}}}" -r 2
```
## 4. 话题接口
订阅：

/odom (nav_msgs/Odometry)：载具实时位置

/geofence (geometry_msgs/PolygonStamped)：禁飞区多边形

发布：

/geofence_status (std_msgs/String)：安全状态输出
