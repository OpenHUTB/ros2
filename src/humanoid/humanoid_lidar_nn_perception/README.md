# humanoid_lidar_nn_perception

Unitree G1 人形机器人 2D 激光雷达 + 1D-CNN 感知 + 神经网络避障。

## 运行环境

- MuJoCo 3.3.7 (OSMesa headless)
- ROS 2 Humble (Docker)
- Python 3.10
- onnxruntime, torch (CPU)

## 运行

```bash
# colcon build
ros2 launch humanoid_lidar_nn_perception main.launch.py teleop:=false
# 另开终端
ros2 topic echo /scan
ros2 topic echo /obstacle_info
```

## 来源

- G1 模型: unitreerobotics/unitree_mujoco (BSD-3-Clause)
- ONNX 行走策略: discoververse/MuJoCo-LiDAR (MIT)
- CNN/PPO/节点: 本模块原创 (BSD-3-Clause)
