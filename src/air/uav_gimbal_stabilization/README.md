# uav_gimbal_stabilization

无人机机载相机云台的两轴稳定控制仿真。目标在相机视线内平滑摆动，PID 控制器输出
pan/tilt 角速度，并对角速度和死区做限制。该模块不依赖 CARLA、Gazebo 或真实云台。

## 运行环境

- Ubuntu 20.04
- ROS Noetic
- Python 3.8
- `python3-pil`（绘图可选）

## 编译与启动

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch uav_gimbal_stabilization main.launch
```

节点会发布：

- `/uav/gimbal/target`：`geometry_msgs/Vector3`，目标 pan/tilt 角度；
- `/uav/gimbal/cmd`：`geometry_msgs/Vector3`，pan/tilt 角速度指令；
- `/uav/gimbal/state`：`geometry_msgs/Vector3`，当前角度和二维误差；
- `/uav/gimbal/status`：`std_msgs/String`。

默认结果写入 `~/.ros/uav_gimbal_stabilization/`，包括 CSV 和响应曲线。

## 纯 Python 测试

```bash
cd src/air/uav_gimbal_stabilization
python3 test/test_gimbal_controller.py
```

测试覆盖 PID 速率限幅、死区、双轴误差、目标轨迹确定性和闭环收敛。

## 范围限制

这是云台姿态控制的教学仿真，不是实机云台实验。目标轨迹和云台动力学均为合成模型；
后续可替换为真实云台角度反馈和相机目标检测接口。
