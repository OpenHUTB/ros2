# AirSim 无人机视觉端到端控制（行为克隆）

## 功能
- AirSim Blocks 场景多旋翼仿真
- MLP 神经网络：输入前视灰度图 (64x64) -> 输出速度控制 (vx, vy)
- 行为克隆：先人工/预设轨迹收集数据 -> 训练神经网络 -> 神经网络自主控制

## 文件结构
- collect_data.py: 数据收集（无人机飞预设矩形轨迹，自动拍前视图+记录控制量）
- train.py: MLP 训练脚本（100 轮，MSE loss）
- main.py: 推理节点（加载 model.pth，实时拍图 -> 神经网络 -> 控制无人机）
- launch/uav_vision.launch.py: launch 启动入口
- model.pth: 训练好的神经网络权重

## 运行
1. Windows 启动 AirSim Blocks.exe
2. 虚拟机运行: python3 src/uav_vision_nn/collect_data.py
3. 训练: python3 src/uav_vision_nn/train.py
4. launch 启动: ros2 launch src/uav_vision_nn/launch/uav_vision.launch.py
