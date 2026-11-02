# AirSim无人机键盘遥控与机载云台控制模块

## 功能说明
基于pygame实现AirSim无人机手动遥控与机载云台相机控制，规避keyboard库在Linux需要root权限的问题；支持无人机全向飞行控制与云台俯仰角度调节，封装为标准ROS2 Python功能包，可直接通过ros2命令启动。

### 控制按键
- W/S：无人机前进、后退
- A/D：无人机左移、右移
- Q/E：无人机上升、下降
- Z/X：机头左转、右转
- I/K：云台向上俯仰、向下俯仰
- ESC：退出程序，触发自动降落

## 运行环境
- 操作系统：Ubuntu 20.04 虚拟机
- ROS版本：ROS2 Foxy
- Python版本：Python 3.8
- 仿真环境：AirSim Unreal仿真（AbandonedPark场景）
- 依赖库：pygame、airsim、opencv-python

## 运行步骤
1. Windows端先启动AirSim仿真器，确保无人机处于就绪状态
2. Ubuntu端进入ROS工作空间根目录，编译并激活环境
   ```bash
   colcon build
   source install/setup.bash
3. 启动无人机云台遥控节点
   ```bash
   ros2 run airsim_drone_gimbal_teleop drone_gimbal_teleop
