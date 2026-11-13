# rov_thruster_allocation_ros1 — ROV 六推进器推力分配

把上层控制器给出的期望力/力矩 `/rov/wrench`（`geometry_msgs/Wrench`，机体坐标系）分配到
4 个 45° 布置的水平推进器和 2 个垂直推进器，发布 `/rov/thruster_cmd`
（`std_msgs/Float64MultiArray`，单位 N），同时发布实际可达的 `/rov/wrench_achieved`
（`Wrench`）。支持推进器饱和处理和单推进器故障重构。配套教程见
[docs/water/rov_thruster_allocation_ros1.md](../../docs/water/rov_thruster_allocation_ros1.md)。

## 运行环境

| 项目 | 配置 |
| ---- | ---- |
| 操作系统 | Ubuntu 20.04（ROS 节点） |
| ROS | ROS Noetic（Python 3） |
| 依赖 | rospy、geometry_msgs、std_msgs、numpy |
| 离线验证 | Windows / Linux，Python 3 + numpy + matplotlib（不需要 ROS） |

## 运行步骤

### 方式一：ROS 节点

```bash
# 1. 拷贝到工作空间并编译
mkdir -p ~/catkin_ws/src
cp -r <本仓库>/src/water/rov_thruster_allocation_ros1 ~/catkin_ws/src/
chmod +x ~/catkin_ws/src/rov_thruster_allocation_ros1/main.py
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# 2. 启动分配节点
roslaunch rov_thruster_allocation_ros1 main.launch

# 3. 另开终端手动发一个期望 wrench（向前的 20 N 推力）
rostopic pub -1 /rov/wrench geometry_msgs/Wrench \
  "{force: {x: 20.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}"

# 4. 查看分配结果
rostopic echo /rov/thruster_cmd
rostopic echo /rov/wrench_achieved
```

### 方式二：离线仿真（不需要 ROS）

```bash
cd <本仓库>
python -m pip install numpy matplotlib
python src/water/rov_thruster_allocation_ros1/offline_sim.py
```

脚本按一段预设任务发送多组 wrench，输出推进器指令、可达 wrench 和饱和/故障对比图。

## 参数

| 参数 | 默认值 | 说明 |
| ---- | ---- | ---- |
| `~max_thrust` | 40.0 | 单个推进器最大推力（N） |
| `~saturation_mode` | `scale` | 饱和策略：`scale` 按比例缩小，`clamp` 直接限幅 |
| `~failed_thruster` | -1 | 故障推进器下标（0～5），-1 表示正常 |
| `~control_rate` | 20.0 | 分配频率（Hz） |
| `~wrench_timeout` | 0.5 | 期望 wrench 超时（s），超时后推进器归零 |

## 话题接口

| 方向 | 话题 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 订阅 | `/rov/wrench` | `geometry_msgs/Wrench` | 期望力/力矩（机体坐标系，Z 向上） |
| 发布 | `/rov/thruster_cmd` | `std_msgs/Float64MultiArray` | 6 个推进器推力（N） |
| 发布 | `/rov/wrench_achieved` | `geometry_msgs/Wrench` | 限幅后实际可达的 wrench |

## 推进器布置

| 编号 | 名称 | 位置 (x, y) m | 推力方向 |
| ---- | ---- | ---- | ---- |
| T1 | 前左水平 | (0.22, 0.16) | 45° |
| T2 | 前右水平 | (0.22, -0.16) | −45° |
| T3 | 后左水平 | (−0.22, 0.16) | 135° |
| T4 | 后右水平 | (−0.22, −0.16) | −135° |
| T5 | 前垂直 | (0.18, 0.0) | +Z |
| T6 | 后垂直 | (−0.18, 0.0) | +Z |

该布置无法产生横滚力矩（Mx），横滚是不可控自由度，分配矩阵秩为 5。

## 预期效果

正常工况下，分配后的实际 wrench 与期望值误差在数值精度范围内（< 0.01%）；
期望推力超过推进器能力时，`scale` 模式按比例缩小并保持方向，`clamp` 模式会改变合力方向；
某个推进器故障后，其余推进器重新分配，仍能保持大部分自由度的控制能力。
