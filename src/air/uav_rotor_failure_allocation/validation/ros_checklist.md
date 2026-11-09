# ROS 2 验证清单（需要另一位使用者在 Ubuntu 22.04 + Humble 上执行）

离线部分（分配矩阵、单桨失效求解、可达力矩集合、19 项测试、`main.py` 出图）已经在
Windows 11 + Python 3.12 上跑通。**ROS 2 部分没有在作者机器上执行**（本机没有 ROS 环境，
只做了 `py_compile` 与参数边界单测），仓库要求换一个人、换一台机器复现，所以把步骤列在
这里，按顺序执行即可。

## 1. 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2                                            # 本仓库
colcon build --packages-select uav_rotor_failure_allocation
source install/setup.bash
ros2 pkg list | grep uav_rotor_failure_allocation    # 应能列出包名
```

## 2. 启动节点

```bash
ros2 launch uav_rotor_failure_allocation main.launch.py
```

`launch` 默认参数：`mass=2.0`、`arm_length=0.25`、`torque_ratio=0.016`、`max_thrust=0`
（按 T/W = 2 反推 9.81 N）、`failed_rotor=-1`（四桨完好）。

另起一个终端：

```bash
ros2 topic info /uav/rotor_thrust      # 订阅者 0、发布者 1，类型 Float64MultiArray
ros2 topic info /uav/allocation_status # 同上，类型 String
ros2 topic echo /uav/rotor_thrust
ros2 topic echo /uav/allocation_status
```

## 3. 发一条悬停请求

`uav/desired_wrench` 是 `geometry_msgs/WrenchStamped`，只取 `wrench.force.z` 作为总推力：

```bash
ros2 topic pub --once /uav/desired_wrench geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: 'base_link'}, wrench: {force: {x: 0.0, y: 0.0, z: 19.6133}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"
```

期望结果：

- `/uav/rotor_thrust` 约 `[4.903, 4.903, 4.903, 4.903]`（四桨各 mg/4，单位 N）；
- `/uav/allocation_status` 为 `ok healthy`；
- 节点日志无 `infeasible` 告警。

## 4. 换成单桨失效

```bash
ros2 launch uav_rotor_failure_allocation main.launch.py failed_rotor:=0
```

再发一次同样的悬停请求（`z: 19.6133`），期望：

- `/uav/rotor_thrust` 约 `[0.0, 9.807, 0.0, 9.807]`（只有对角线两桨出力，各 f_max）；
- `/uav/allocation_status` 为 `ok failed=0 yaw=0.3138`（被动偏航 ≈ κ·mg = 0.3138 N·m）。

把推力加大到 `z: 25.0`（超过失效后零力矩上限 1.0 mg）时，期望状态变成
`infeasible failed=0 yaw=…`，节点打一条 warning。

## 5. 参数边界

```bash
ros2 run uav_rotor_failure_allocation node --ros-args -p failed_rotor:=-2
```

期望：节点启动即抛 `ValueError: failed_rotor 必须是 -1（四桨完好）或 0..3（失效桨编号）`，
不会把 `-2` 当成四桨完好。

## 6. 回填结果

请把下面几项贴回
<https://github.com/OpenHUTB/ros2/pull/1031>：

- 系统与 ROS 版本（`lsb_release -d`、`echo $ROS_DISTRO`）、Python 版本；
- `colcon build` 是否成功、有没有 warning；
- 第 3、4 步的两个话题实际输出（四桨推力数值与状态字符串）；
- 第 5 步的实际报错文本。

执行完把 `validation/offline_validation.json` 里的 `ros_validation.checked` 改为 `true`、
`independent_machine_review` 改为 `true`，并补上机器信息即可。
