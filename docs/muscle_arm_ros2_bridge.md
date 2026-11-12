# ROS2 话题桥接（二连杆机械臂肌肉骨骼仿真）

## 概述
本模块实现 MuJoCo 肌肉骨骼二连杆机械臂仿真与 ROS2 Humble 的双向话题通信桥接，将仿真中的关节状态、肌肉张力实时发布到 ROS2 话题，同时接收外部 ROS2 控制指令驱动仿真环境，实现肌肉骨骼模型与自主 ROS2 导航/控制栈的无缝联动。

## 依赖环境
- Ubuntu 22.04
- ROS2 Humble
- MuJoCo 2.3+
- Python 3.10

## 功能特性
1.  话题发布：
    - `/muscle_arm/joint_states`：发布两个关节的角度、角速度
    - `/muscle_arm/muscle_forces`：发布主动肌、拮抗肌的实时张力
2.  话题订阅：
    - `/muscle_arm/effort_cmd`：接收关节力矩指令，驱动二连杆运动
3.  桥接延迟：< 1ms，支持 500Hz 实时仿真频率

## 运行步骤
```bash
# 1. 启动 ROS2 桥接节点
ros2 run muscle_arm_ros2 bridge_node

# 2. 查看关节状态话题
ros2 topic echo /muscle_arm/joint_states

# 3. 发送测试力矩指令
ros2 topic pub /muscle_arm/effort_cmd std_msgs/Float64MultiArray "data: [0.5, -0.3]"
```

## 注意事项
- 桥接节点需与 MuJoCo 仿真在同一局域网内运行，默认使用本地环回地址
- 首次运行需配置 ROS2 环境变量：`source /opt/ros/humble/setup.bash`
