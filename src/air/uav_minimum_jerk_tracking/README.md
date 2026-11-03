# uav_minimum_jerk_tracking

无人机五次最小加加速度轨迹生成与世界系速度跟踪实验。

模块通过 `carlair_ros_bridge` 订阅 `/uav/odom`，按当前机头方向把
`[前, 左, 上]` 相对航点转换成世界 ENU 航点，生成位置、速度、加速度和
加加速度连续的五次轨迹，再发布 `/uav/cmd_vel` 完成闭环飞行。

## 与 waypoint_tracking 的区别

| 模块 | 参考输入 | 控制器 | 关注指标 |
|---|---|---|---|
| `uav_waypoint_tracking` | 逐个离散航点 | PID 到点后制动 | 到点时间、稳态误差、超调 |
| 本模块 | 多段连续五次轨迹 | 参考速度前馈 + 位置反馈 | 轨迹 RMSE、速度/加速度/加加速度峰值 |

本模块还提供 `linear` 参考模式，用相同控制器比较线性位置参考与五次轨迹。

## 五次最小加加速度

单个时间区间内使用归一化时间 $s=t/T$：

```text
p(s) = p0 + Δp(10s³ - 15s⁴ + 6s⁵)
```

该曲线满足起止点速度、加速度为零，并让全程平方加加速度最小。轨迹时长根据
允许的最大速度和最大加速度自动计算。

## 运行环境

- OpenHUTB/CarlaAir + AirSim；
- Ubuntu 20.04 + ROS Noetic；
- `carlair_ros_bridge`；
- `python3-numpy`。

## 一键启动

先在 Windows 启动 OpenHUTB，进入地图后按 `Ctrl+V` 切到 AirSim：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

roslaunch uav_minimum_jerk_tracking openhutb_test.launch \
  host:=192.168.94.1 \
  mission:=smooth_forward_6m \
  mode:=minimum_jerk
```

`host` 必须替换为 VMnet8 中 Windows 宿主机地址。

轨迹结束后若在 `final_wait_timeout` 内仍未达到终点容差，节点输出
`FAILED_STALLED` 并停止，不会无限重复控制日志。

## 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/uav/cmd_vel` | `geometry_msgs/Twist` | 世界 ENU 速度 |
| `/uav_minimum_jerk_tracking/status` | `std_msgs/String` | 段号、轨迹误差和参考量 |
| `/uav_minimum_jerk_tracking/desired_path` | `nav_msgs/Path` | 采样后的参考轨迹 |
| `/uav_minimum_jerk_tracking/actual_path` | `nav_msgs/Path` | 实际飞行轨迹 |

CSV 默认写入：

```text
~/.ros/uav_minimum_jerk_tracking/<mission>_<mode>.csv
```

## 测试

```bash
cd src/air/uav_minimum_jerk_tracking
python3 -m unittest discover -s test -v
```

测试覆盖五次边界条件、速度和加速度约束、多段航点、线性参考、机体系航点旋转、
`body_frame` fail-closed 和速度执行器限幅。

## OpenHUTB 实测结果

已在 OpenHUTB 中分别运行 `minimum_jerk` 与 `linear`：

| 模式 | 最终误差 | 运动段 RMSE | 参考时长 | 结果 |
|---|---:|---:|---:|---|
| minimum_jerk | 0.238 m | 2.023 m | 12.5 s | DONE |
| linear | 2.155 m | 3.297 m | 6.67 s | FAILED_STALLED |

五次轨迹最终收敛，但实际收敛时间约 37.9 s，说明当前外环控制在 OpenHUTB 的
速度执行滞后下仍需整定。线性参考在 5 s 最终等待窗口内未收敛；该结果作为负面对照
保留，不包装成成功。

## AI 使用声明

使用 Codex 辅助需求分析、代码编写、测试和文档整理；作者负责 OpenHUTB
飞行验证、数据采集和最终内容复核。
