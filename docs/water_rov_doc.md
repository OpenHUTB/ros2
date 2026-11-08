# water 功能包：水下ROV声呐图像CNN目标识别
## 概述
water是ROS2 Python功能包，用于水下ROV侧扫声呐图像目标识别，基于CNN卷积神经网络，对Gazebo仿真相机输出图像进行推理分类。
## 目录结构
src/water
├── water/

│   ├── __init__.py

│   └── sonar_infer_node.py

├── launch/

│   └── rov.launch.py

├── resource/

│   └── water

├── worlds/

│   └── underwater.world

├── package.xml

└── setup.py

```

## 环境依赖
- ROS2
- Gazebo
- pytorch
- opencv-python
- cv_bridge

## 编译与运行步骤
```bash
# 编译功能包
colcon build --packages-select water
# 加载环境
source install/setup.bash
# 一键启动Gazebo水下仿真 + CNN识别节点
ros2 launch water rov.launch.py
```

## 运行效果图
![运行效果图] Camera_Image_Raw_Data.jpg
https://github.com/Xavier-101666/ros2/blob/waterwater/Camera_Image_Raw_Data.jpg
![运行效果图] Underwater Target Recognition Demo.jpg
https://github.com/Xavier-101666/ros2/blob/waterwater/Underwater%20Target%20Recognition%20Demo.jpg


    
