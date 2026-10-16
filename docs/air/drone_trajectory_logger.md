
# 新增无人机飞行日志与状态识别 ROS 2 封装模块

## 作业概述与背景
本模块基于 ROS 2 实现了一套无人机飞行轨迹类型识别系统。系统通过发布节点模拟无人机飞行指令，接收节点实时订阅数据，并自动识别当前的飞行状态（直线、旋转、悬停），同时实时绘制飞行轨迹图并生成自动任务日志。

## 运行环境
* 操作系统：Ubuntu 22.04（虚拟机）
* ROS 2 版本：Humble
* 依赖库：rclpy, geometry_msgs, matplotlib

## 核心代码文件清单
本模块包含以下核心文件，均已提交至仓库：
```text
src/
└── drone_trajectory_logger/
    ├── main.py                 # 轨迹发布节点（入口）
    └── trajectory_analyzer.py  # 轨迹识别与绘图节点
```
## 核心代码内容

完整代码请点击下方链接查看：

* [查看 main.py 代码](src/drone_trajectory_logger/main.py)
* [查看 trajectory_analyzer.py 代码](src/drone_trajectory_logger/trajectory_analyzer.py)

## 运行步骤

1. 新建标准工作空间并进入 src 目录：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
2.从 ros2 仓库拷贝模块代码到工作空间：
```
bash
cp -r ~/OpenHUTB/ros2/src/drone_trajectory_logger ~/ros2_ws/src/
cd ~/ros2_ws
```
3.编译工作空间并部署环境：
```
bash
colcon build --packages-select drone_trajectory_logger
source install/setup.bash
```
4.运行发布节点（在第一个终端）：
```
bash
python3 src/drone_trajectory_logger/drone_trajectory_logger/main.py
```
5.运行效果：
![运行效果图片](../img/capture_2.png)

效果说明：发布节点成功发送不同的飞行指令，识别节点成功解析并打印出对应的飞行状态日志，同时基于实时数据绘制了飞行轨迹图（椭圆），验证了轨迹识别与自动日志记录功能的正确性。
备注：本模块使用模拟的飞行指令流进行验证，后续可接入真实的无人机仿真数据。

