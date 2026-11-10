# 机器视觉驱动的多机器人蚁群编队
本项目基于 ROS2 Humble，实现视觉目标识别 + 多机器人蚁群编队跟随。

## 项目目录结构
src/chap8/ant_colony_swarm/

├── ant_colony_swarm/

│   ├── __init__.py

│   ├── swarm_control_node.py

│   └── vision_detect_node.py

├── launch/

│   └── swarm_bringup.launch.py

├── resource/

│   └── ant_colony_swarm

├── package.xml

├── setup.cfg

└── setup.py

## 功能说明
1. **vision_detect_node.py**：订阅图像，识别红色目标色块，输出目标像素坐标
2. **swarm_control_node.py**：接收视觉目标坐标，计算偏差，发布速度指令控制机器人编队
3. **swarm_bringup.launch.py**：一键启动视觉检测节点与编队控制节点

## 编译与运行
```bash
# 进入工作空间
cd ~/ros2_ws
# 编译包
colcon build --packages-select ant_colony_swarm
# 加载环境
source install/setup.bash
# 启动整套节点
ros2 launch ant_colony_swarm swarm_bringup.launch.py
```
## 运行效果图
![视觉检测节点启动]https://github.com/Xavier-101666/ros2/blob/ant/docs/img/vision_detect_startup.png

![Gazebo 蚁群编队场景]https://github.com/Xavier-101666/ros2/blob/ant/docs/img/swarm_gazebo_scene.png
