# uav_vision_nn - AirSim 端到端视觉控制（行为克隆 + MLP）

基于 AirSim Blocks 仿真的无人机视觉端到端控制模块（ROS 课程任务4）。

## 与已有 #113 的区别（创新点）

#113 是 YOLO 目标检测 + 八字轨迹控制（先检测后规划）。本模块是纯端到端行为克隆：
- 输入：单张前视灰度图（64x64），无目标检测、无显式规划
- 输出：直接速度控制量 (vx, vy)
- 学习方式：神经网络直接从图像映射到控制

## 模型权重获取（model.pth）

仓库不提交权重文件，按以下步骤复现：

    python3 collect_data.py     # 采集数据
    python3 train.py            # 训练 100 轮
    python3 main.py             # 实时控制

## ROS2 启动

    ros2 launch uav_vision_nn uav_vision.launch.py

## 演示

![采集](../../img/uav_vision_nn_collect.gif)
![控制](../../img/uav_vision_nn_demo.gif)
![轨迹](../../img/uav_vision_nn_track.png)
