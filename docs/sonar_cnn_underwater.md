# 基于CNN的声呐图像海底目标分类（水下）
## 项目简介
本项目基于ROS2搭建水下声呐图像目标识别功能包，使用CNN卷积神经网络对声呐图像做分类识别，可在Gazebo水下仿真环境中运行。
节点订阅仿真相机图像话题，对图像做预处理，送入训练好的CNN模型推理，在终端输出识别的目标类别。

## 文件结构
src/water/
├── launch/
│   └── rov.launch.py
├── resource/
│   └── water
├── water/
│   ├── **init**.py
│   ├── sonar_infer_node.py
│   ├── sonar_model.py
│   └── train_sonar.py
├── worlds/
│   └── underwater.world
├── package.xml
├── setup.cfg
└── setup.py

```

## 环境依赖
- ROS2 Humble
- OpenCV
- PyTorch
- torchvision
- cv_bridge

## 编译步骤
```bash
colcon build --packages-select water
source install/setup.bash
```

## 运行命令

```
ros2 launch water rov.launch.py
```
