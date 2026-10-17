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
