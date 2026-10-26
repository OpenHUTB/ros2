# uav_neural_perception

无人机机载前视RGB摄像头图像 → CNN神经网络感知 → 运动控制。

## 运行
roslaunch uav_neural_perception main.launch

## 安全机制
- 权重文件缺失或加载失败时，部署节点立即安全停止并给出明确错误日志，**不会**使用随机初始化模型驱动无人机；随机权重仅用于离线自检。
- 模型输出出现 NaN / inf 等非法值时，节点丢弃本帧指令，不下发速度量。
- 速度量统一限幅到配置的 `max_vx / max_vy / max_yaw` 安全范围。

## 文件
- scripts/dataset.py: 数据采集与预处理
- scripts/model.py: CNN 回归网络
- scripts/train.py: 训练脚本
- scripts/safety.py: 权重加载校验与输出合法性校验（不依赖 ROS）
- scripts/perception_node.py: ROS 推理节点
- scripts/diag_v2.py: 离线诊断（整体 MSE、分方向一致率、转向召回与幅值）
- tests/test_local.py: 本地回归测试（缺失权重、非法输出、限幅等）
- config/perception.yaml: 参数参考表；部署限幅默认值由 launch 参数与节点默认值给出

## 声明
本项目使用大模型辅助撰写，作者对全部内容负责。
