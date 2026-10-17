# uav_key_control 使用说明
无人机键盘速度控制ROS2功能包

## 修改概述 / Overview
中文：新增无人机键盘速度控制节点。读取键盘按键，生成并发布Twist类型速度指令；无人机订阅该话题，根据收到的速度指令完成运动。
English: Add UAV keyboard velocity control node. It reads keyboard inputs, generates and publishes Twist velocity commands. The UAV subscribes to this topic and moves according to received commands.

## 运行环境
- 操作系统：Ubuntu 20.04
- ROS版本：ROS2 Foxy
- Python版本：3.8.10
## 核心代码文件

本模块包含以下核心文件，均已提交至仓库：

```text
src/
└── uav_key_control/
    ├── main_uav_track.py       # 无人机键盘速度控制节点
    ├── setup.py                # ROS 2 功能包配置文件
    ├── package.xml             # ROS2功能包描述文件
    └── resource/
        └── uav_key_control    # resource资源标识文件
    └── launch/
        └── uav_track.launch.py # 节点一键启动文件
```
## 核心代码内容

本模块的核心代码已上传至仓库 `src/air/uav_key_control/` 目录下。

完整代码请点击下方链接查看：

* [查看 main_key_control.py 代码](../../src/air/uav_key_control/main_key_control.py)
* [查看 package.xml 代码](../../src/air/uav_key_control/package.xml)
* [查看 setup.py 代码](../../src/air/uav_key_control/setup.py)
* [查看 key_control.launch.py 代码](../../src/air/uav_key_control/launch/key_control.launch.py)
## 边界行为说明 / Boundary behavior
中文：节点预设最大线速度0.2 m/s、最大角速度0.3 rad/s。输出速度会被钳位在[-0.2, 0.2]、[-0.3, 0.3]区间，防止超速失控。按下空格键，速度立刻置零。

English: The node sets max linear velocity 0.2 m/s and max angular velocity 0.3 rad/s. Output velocity is clamped within [-0.2, 0.2] and [-0.3, 0.3] to avoid overspeed. Press space key to set velocity to zero immediately.
  
## 启动步骤

1.进入ROS2工作空间
```bash
cd ~/ros2-cc
```
2.编译功能包
```bash
colcon build --packages-select uav_key_control
source install/setup.bash
```
3. 启动键盘控制节点
```bash 
ros2 run uav_key_control main_key_control
```
4. 新开一个终端窗口，同样进入工作空间并加载环境，监听速度话题  /cmd_vel ，查看节点发布的Twist速度指令 
```bash
cd ~/ros2-cc
source install/setup.bash
ros2 topic echo /cmd_vel
```
5.按键映射
- W：向前，输出X方向正向线速度
- S：向后，输出X方向反向线速度
- A：原地左转，输出Z方向正向角速度
- D：原地右转，输出Z方向反向角速度
- 空格：急停，所有速度置0
  
6.运行效果
  
节点成功启动后，选中节点所在终端，按下W/A/S/D按键，监听终端可以实时打印Twist速度消息，验证速度指令正常发布。
![运行效果图片](../img/_uav_key_control_package_build.png)
![运行效果图片](../img/_uav_key_control_topic_test.png)
