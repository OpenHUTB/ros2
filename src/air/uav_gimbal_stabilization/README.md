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

`Vector3` 字段固定约定如下：

- `target`：`x=pan` 目标角，`y=tilt` 目标角，`z=0`；
- `cmd`：`x=pan` 角速度，`y=tilt` 角速度，`z=0`；
- `state`：`x=当前 pan`，`y=当前 tilt`，`z=二维综合角误差`。

默认结果写入 `~/.ros/uav_gimbal_stabilization/`，包括 CSV 和响应曲线。
`duration` 和 `rate` 必须是有限正数；无有效样本时不会创建空的 CSV 或图片。

## 纯 Python 测试

catkin 自动测试：

```bash
cd ~/catkin_ws
catkin_make run_tests_uav_gimbal_stabilization
catkin_test_results
```

也可以直接运行 unittest：

```bash
cd src/air/uav_gimbal_stabilization
python3 -m unittest discover -s test -v
```

11 项测试覆盖 PID 速率限幅、死区、双轴误差、目标轨迹确定性、闭环前后段
误差下降、CSV 首条时间、非法时长/频率、空结果和离线参数校验。

本次 50 Hz、12 s 闭环仿真的二维角误差 RMSE 为 `5.269°`，最大误差为 `7.764°`；
该平滑目标轨迹未触发 60°/s 速率限幅，因此饱和比例为 `0`，限幅行为由单元测试覆盖。

## 范围限制

这是云台姿态控制的教学仿真，不是实机云台实验。目标轨迹和云台动力学均为合成模型；
后续可替换为真实云台角度反馈和相机目标检测接口。

## AI 使用声明

使用 Codex（OpenAI）辅助代码、测试和文档整理；算法参数、运行结果与提交内容由作者复核并负责。

本包使用 MIT License，完整文本见 `LICENSE`。
