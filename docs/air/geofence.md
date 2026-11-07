# 基于 ROS 2 的禁飞区（电子围栏）动态检测与安全预警系统

## 1. 项目简介
本项目是一个基于 ROS 2 的独立安全监控层模块。系统旨在为无人机等移动机器人提供实时的禁飞区（电子围栏）检测。通过订阅载具的实时位姿与动态下发的禁飞区多边形边界，系统利用**射线法（Ray Casting）**实时判断载具状态，并输出 `SAFE`（安全）、`WARN`（接近预警）、`CRITICAL`（侵入告警）三级安全响应。

## 2. 目录结构
项目采用标准的 `ament_python` ROS 2 包结构：

```text
src/air/
└── geofence_demo/
    ├── geofence_demo/            # Python 核心源码目录
    │   ├── __init__.py           # 声明为 Python 模块
    │   └── geofence_node.py      # 核心检测节点代码
    ├── launch/                   # 启动脚本目录
    │   └── geofence.launch.py    # 一键启动节点脚本
    ├── resource/                 # ROS 2 索引资源目录
    ├── test/                     # 代码规范与自动化测试目录
    ├── package.xml               # 包依赖与元数据声明
    ├── setup.cfg                 # 编译配置文件
    └── setup.py                  # 编译脚本与程序入口点配置
```

## 3. 运行与验证过程
3.1 编译工作空间
```bash
colcon build --symlink-install
source install/setup.bash
```
3.2 启动节点与测试
打开终端 A 启动节点：
```bash
ros2 launch geofence_demo geofence.launch.py
```
打开终端 B 发布禁飞区 (X:2~4, Y:2~4 矩形)：
```bash
ros2 topic pub /geofence geometry_msgs/msg/PolygonStamped "{
  header: {frame_id: 'map'},
  polygon: {
    points: [{x: 2.0, y: 2.0}, {x: 4.0, y: 2.0}, {x: 4.0, y: 4.0}, {x: 2.0, y: 4.0}]
  }
}" -r 1
```
打开终端 C 发布载具位置：
```bash
# 正常状态 (SAFE)
ros2 topic pub /odom nav_msgs/msg/Odometry "{pose: {pose: {position: {x: 0.0, y: 0.0}}}}" -r 2

# 报警状态 (CRITICAL) - 按 Ctrl+C 停止上一条，再执行此条
ros2 topic pub /odom nav_msgs/msg/Odometry "{pose: {pose: {position: {x: 3.0, y: 3.0}}}}" -r 2
```
## 运行效果图

![安全状态运行效果图片](./img/air/test_safe_status.png)

![报警状态运行效果图片](./img/air/test_critical_alert.png)
