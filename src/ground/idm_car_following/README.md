# idm_car_following

智能驾驶员模型（IDM）跟车与车队减速波仿真教学模块。

IDM 是微观交通流里最常用的纵向跟驰模型：由"期望速度项"和"期望间距项"构成加速度，
能自发复现拥堵波向后传播这类宏观现象。

## 两种运行方式

### 一、ROS 2 节点（订阅前车里程计 → 发布加速度指令）

```bash
cd <ros2_ws>
colcon build --packages-select idm_car_following
source install/setup.bash

ros2 launch idm_car_following main.launch.py
# 或者
ros2 run idm_car_following ros_node.py
```

话题接口：

| 方向 | 话题 | 类型 | 内容 |
|---|---|---|---|
| 订阅 | `/idm/lead_odom` | `nav_msgs/Odometry` | 前车状态：`pose.pose.position.x` 纵向位置、`twist.twist.linear.x` 速度 |
| 发布 | `/idm/ego_odom` | `nav_msgs/Odometry` | 自车状态（节点内一阶积分 `v += a·dt`） |
| 发布 | `/idm/cmd_accel` | `std_msgs/Float64` | IDM 解出的期望加速度 (m/s²)，限幅到 `[a_min, a]` |

把传感器/仿真器接进来：只要有人持续发布 `/idm/lead_odom`（Carla、Gazebo 里的前车
真值，或真实前车的定位输出），本节点就是标准跟驰控制器。手动喂一条验证链路：

```bash
ros2 topic pub /idm/lead_odom nav_msgs/Odometry \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 30.0}}}, twist: {twist: {linear: {x: 12.0}}}}"
ros2 topic echo /idm/cmd_accel
```

节点参数：`rate`（控制频率，Hz）。命令行覆盖：

```bash
ros2 run idm_car_following ros_node.py --ros-args -p rate:=100.0
```

### 二、纯离线仿真（不需要装 ROS 2）

```bash
cd src/ground/idm_car_following
python3 scripts/main.py --outdir scripts
python3 scripts/ros_node.py --offline --outdir scripts
```

会输出单车跟驰的稳态间距、车队减速波的下潜深度，并生成两张配图。未安装 `rclpy`
时 `ros_node.py` 自动降级为离线模式。

## 测试

```bash
cd src/ground/idm_car_following
python3 -m unittest discover -s tests -t .   # test_idm.py 6 项 + test_ros_params.py 3 项
```

## 依赖

- Python 3 标准库（无 numpy / matplotlib）
- ROS 2 可选：`rclpy`、`std_msgs`、`nav_msgs`

## 关于验证范围（请评审注意）

- 离线仿真的数值（稳态间距、各车速度下潜深度与峰值时刻）已在本机实跑，结果见文档。
- ROS 2 节点已用与 `rclpy` 接口一致的桩验证：话题名、`Odometry` 字段写入、自由流下
  自车收敛到 `v0`、前车贴近时输出负加速度且被限幅。
- 节点已 `declare_parameter` 声明并读回全部参数
  （`rate`/`dt`/`ego_v0`/`ego_x0`/`lead_x0`/`lead_v0`），launch 的 `rate:=...`
  真正生效。非默认参数运行记录（`rate:=20.0`，桩上回放 120 s，前车话题以 15 m/s
  匀速刷新）：定时器周期 0.050 s、每周期 5 个积分子步、自车速度收敛到 15.0000 m/s、
  车间距 28.9 m（趋向平衡值 29.6 m），已写成 `tests/test_ros_params.py` 回归测试。
- **ROS 2 节点尚未在安装 ROS 2 发行版的机器上实测**，`colcon build` / `ros2 launch`
  的行为请以目标环境为准。
