# uav_neural_perception

无人机机载前视RGB摄像头图像 → CNN神经网络感知 → 运动控制。

## 运行
roslaunch uav_neural_perception main.launch

## 文件
- scripts/dataset.py: 数据采集与预处理
- scripts/model.py: CNN 回归网络
- scripts/train.py: 训练脚本
- scripts/perception_node.py: ROS 推理节点
- config/perception.yaml: 参数配置

## 声明
本项目使用大模型辅助撰写，作者对全部内容负责。
<img width="1167" height="876" alt="1  `      drone_keyboard_demo gif`" src="https://github.com/user-attachments/assets/9c60c49a-fe46-4bf6-9cfa-d65d963b071c" />

