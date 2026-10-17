# AUV 水下机器人仿真功能包

## 功能说明
基于ROS2 + Gazebo实现的水下机器人仿真功能包，包含**视觉避障**与**矩形巡航**两个功能。

## 环境依赖
- ROS2 Humble
- Gazebo 11.10.2
- Python 3.10+
- OpenCV + cv_bridge

## 坐标系说明
采用 ENU 世界坐标系：x轴向前，y轴向右，z轴向上。

## 水动力模型
包含简化水下阻尼模型，模拟水体阻力与浮力效应。

## 运行方式
### 1. 视觉避障
```bash
ros2 launch auv_end2end auv_sim.launch.py
```

### 2. 矩形巡航

```
ros2 launch auv_end2end auv_rect_cruise.launch.py
```
