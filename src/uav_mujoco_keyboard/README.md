# uav_mujoco_keyboard（Mujoco 四旋翼无人机键盘遥控）

基于 Mujoco 3.15 的四旋翼无人机仿真模块：支持键盘运动控制与 ROS2 话题遥控。
无人机动力学（推力模型 + PID 悬停控制）在 Mujoco 物理引擎中实时仿真，
通过 ROS2 发布位置话题 `/uav/pose`、接收目标话题 `/uav/target`。

## 环境要求

- Ubuntu 22.04 + ROS2 Humble
- Python 依赖：
  - `mujoco`：`pip3 install mujoco -i https://pypi.tuna.tsinghua.edu.cn/simple`
  - `opencv-python`：`pip3 install opencv-python -i https://pypi.tuna.tsinghua.edu.cn/simple`
- ROS2 消息包：`std_msgs`、`geometry_msgs`（Humble 自带）

## 目录结构

```
uav_mujoco_keyboard/
├── main.py                    # ROS2 节点：仿真+控制+显示+话题（入口，main. 开头）
├── launch/
│   └── uav_keyboard.launch.py # launch 启动文件
└── models/
    └── drone.xml              # 四旋翼无人机 Mujoco 模型（自写）
```

## 运行步骤

1. 启动（第一个终端）：
```bash
cd ~/ros2/src/uav_mujoco_keyboard
source /opt/ros/humble/setup.bash
ros2 launch ./launch/uav_keyboard.launch.py
```
弹窗显示无人机，自动起飞到 z=1.0 悬停。

2. 键盘控制：点击 drone 窗口获得焦点后：
   - `W` 向前　`S` 向后　`A` 向左　`D` 向右　`R` 上升　`F` 下降　`Q` 退出

3. 查看位置（第二个终端）：
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /uav/pose
```

4. ROS 话题遥控（第二个终端）：
```bash
ros2 topic pub -1 /uav/target std_msgs/msg/Float64MultiArray "{data: [2.0, 1.0, 1.5]}"
```
无人机自动飞向目标位置 (2, 1, 1.5)。

## 控制原理

### 推力模型

四旋翼由 4 个旋翼产生垂直推力 `f1~f4`（N），作用于机体坐标系 Z 轴向上的旋翼点：

- 总升力：`F = f1 + f2 + f3 + f4`
- 滚转力矩：`Mx = L·(f1 + f2 - f3 - f4)`
- 俯仰力矩：`My = L·(-f1 + f2 + f3 - f4)`，其中 `L = 0.2 m` 为力臂

旋翼推力分配（控制量 → 旋翼力）：
```
f1 = F/4 + (Mx - My)/(4L)
f2 = F/4 + (Mx + My)/(4L)
f3 = F/4 - (Mx - My)/(4L)
f4 = F/4 - (Mx + My)/(4L)
```

### PID 控制

- 高度环：`F = mg + Kp_z·(z_des - z) - Kd_z·vz`（重力前馈 + 比例微分）
- 水平环：`Fx = Kp_xy·(x_des - x) - Kd_xy·vx`（同理 y）
- 姿态环：`Mx = -Kp_a·roll - Kd_a·ωx`（保持水平，同理 pitch）

控制输出经混合矩阵转换为 4 个旋翼力，由 Mujoco 物理引擎积分得到无人机运动。
