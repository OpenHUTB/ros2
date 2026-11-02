# 空域载具（Air）模块

本目录收录无人机等空域载具相关的 ROS 模块文档。站点完整页面导航以侧边栏为准，
下面列出本课程两个任务的模块入口。

## 任务①：无人机仿真 + 键盘控制

- [无人机键盘控制（uav_keyboard_control）](./uav_keyboard_control.md)
- [无人机与 ROS 的桥接模块（carlair_ros_bridge）](./carlair_ros_bridge.md)
- [建立虚拟机和空域载具之间的连接](./setup_and_connect.md)
- [空域模拟器的 ROS 封装器](./ros_pkgs.md)

## 任务②：前视 CNN 行为克隆视觉导航

- [无人机视觉导航（前视 CNN 行为克隆，uav_neural_perception）](./uav_neural_perception.md)

## 说明

- 各源码包的构建步骤、参数表与本地测试方法，见对应包目录下的 `README.md`。
- 数据集图像与训练权重不提交本仓库，需在本机采集数据并运行训练脚本后生成。
