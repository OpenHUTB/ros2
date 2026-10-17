# 新增无人机飞行日志与状态识别 ROS 2 封装模块
基于 ROS 2 实现的无人机飞行轨迹识别节点，订阅飞行指令并实时绘图。

## 包含文件
* `main.py`: 发布节点
* `trajectory_analyzer.py`: 识别与绘图节点
* `launch/`: 启动文件

## 快速开始
```bash
colcon build --packages-select drone_trajectory_logger
source install/setup.bash
ros2 launch drone_trajectory_logger main.launch.py
```

(详细原理与实验步骤请参考仓库 docs/ 目录下的说明文档)
