# Mbot 雷达感知自主避障仿真（ROS 1 Noetic）

> ROS 1 Noetic + Gazebo 11 + Mbot 差速底盘。包内自带 URDF、world 与 launch，可独立运行，不依赖 `turtlebot3_*`。

## 1. 项目概述

节点订阅 `/scan` 2D 激光雷达数据，按**扇区最小距离 + 滞回状态机**输出 `/cmd_vel` 速度指令，让 Mbot 在未知静态环境中自主避障。

控制分为三个状态：

| 状态 | 触发条件 | 行为 |
| :--- | :--- | :--- |
| `cruise` 巡航 | 前方 ≥ `safe_distance`（0.80 m） | 直行；侧向过近时朝更宽一侧轻微让开 |
| `avoid` 避障 | 前方 < `safe_distance` | 减速并朝更宽一侧转向；前方 > `exit_distance`（1.00 m）才退出 |
| `escape` 脱困 | 前方 < `danger_distance`（0.30 m） | 倒车并转向远离更近一侧；后方两侧都堵时原地旋转 |

`safe_distance` 与 `exit_distance`、`danger_distance` 与 `danger_exit_distance` 成对使用构成**滞回**，避免在阈值边界反复切换造成转向抖动。

## 2. 环境版本

| 项目 | 版本 |
| :--- | :--- |
| 操作系统 | Ubuntu 20.04 LTS |
| ROS | ROS Noetic（catkin） |
| 仿真引擎 | Gazebo 11 |
| 语言 | Python 3.8 |

## 3. 依赖安装

```bash
sudo apt update
sudo apt install -y ros-noetic-gazebo-ros ros-noetic-gazebo-ros-pkgs \
                     ros-noetic-robot-state-publisher ros-noetic-urdf
```

## 4. 项目文件结构

本模块位于仓库的 `src/ground/` 下：

```text
src/ground/mbot_gazebo_lidar_avoid/
├── README.md                       # 环境 / 构建 / 启动 / 停止 / 参数说明
├── LICENSE                         # MIT
├── CMakeLists.txt                  # catkin 编译与安装规则（含 launch/config/urdf/worlds）
├── package.xml                     # 包元信息
├── config/
│   └── lidar_avoid.yaml            # 全部可调参数（rosparam）
├── launch/
│   └── main.launch                 # 一键启动 Gazebo + Mbot + 避障节点
├── urdf/
│   └── mbot_lidar.urdf             # Mbot 差速底盘 + 2D 激光雷达
├── worlds/
│   └── lidar_avoid_world.world     # 封闭场地 + 障碍物
└── scripts/
    └── main.py                     # 避障节点（可执行）
```

对应的教程文档即本文，位于 `docs/ground/mbot_lidar_avoid.md`。

## 5. 构建

本包必须放进 catkin 工作空间的 `src/` 下才能编译。**不要直接用仓库根目录 `catkin_make`**：
仓库 `src/` 中还有 ROS 2（ament）以及依赖 CARLA / MuJoCo 的包，会被一起配置并大概率失败。

方式一（推荐）：新建独立工作空间，只放入本包：

```bash
mkdir -p ~/mbot_ws/src
cp -r <仓库根目录>/src/ground/mbot_gazebo_lidar_avoid ~/mbot_ws/src/
cd ~/mbot_ws
catkin_make
source devel/setup.bash
```

方式二：已有 `~/catkin_ws`，把本包软链接进去：

```bash
mkdir -p ~/catkin_ws/src
ln -s <仓库根目录>/src/ground/mbot_gazebo_lidar_avoid \
      ~/catkin_ws/src/mbot_gazebo_lidar_avoid
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

方式三：确实要在多包仓库里原地编译时，只构建本包及其依赖：

```bash
cd <仓库根目录>
catkin_make --only-pkg-with-deps mbot_gazebo_lidar_avoid
source devel/setup.bash
```

验证安装空间（可选）：

```bash
cd ~/mbot_ws
catkin_make install
source install/setup.bash
```

`CMakeLists.txt` 已把 `launch/`、`config/`、`urdf/`、`worlds/` 安装到 share 目录，因此 `catkin_make install` 后仍可用 `roslaunch` 找到 launch 文件。

## 6. 启动与停止

启动：

```bash
roslaunch mbot_gazebo_lidar_avoid main.launch
```

可选参数：`gui:=false`（无界面）、`x:=` / `y:=` / `yaw:=`（初始位姿）。

停止：

- 在 `roslaunch` 终端按 `Ctrl+C`。节点注册了 `rospy.on_shutdown`，退出前发布零速度。
- 或 `rosnode kill /mbot_lidar_avoid_node`。
- 清理残留：`killall -9 gazebo gzserver gzclient roslaunch`。

## 7. 参数

参数全部放在 `config/lidar_avoid.yaml`，由 launch 以 rosparam 加载。
节点**只在启动时读取一次参数**，`rosparam set` 之后不会生效；修改后请重新 `roslaunch`，
或 `rosnode kill /mbot_lidar_avoid_node` 后重启节点。本模块未实现 `dynamic_reconfigure`。

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `cruise_speed` | 0.30 | 巡航线速度 (m/s) |
| `avoid_speed` | 0.10 | 避障线速度 (m/s) |
| `reverse_speed` | -0.12 | 脱困倒车速度 (m/s) |
| `turn_rate` | 1.20 | 避障转向角速度 (rad/s) |
| `rotate_rate` | 0.90 | 脱困旋转角速度 (rad/s) |
| `safe_distance` | 0.80 | 进入避障阈值 (m) |
| `exit_distance` | 1.00 | 退出避障阈值 (m) |
| `danger_distance` | 0.30 | 进入脱困阈值 (m) |
| `danger_exit_distance` | 0.50 | 退出脱困阈值 (m) |
| `side_distance` | 0.45 | 侧向障碍影响转向的阈值 (m) |
| `rear_block_distance` | 0.35 | 后方受阻阈值，决定倒车或原地旋转 (m) |
| `front_half_deg` | 20.0 | 前扇区半角 (°) |
| `side_deg` | 60.0 | 侧扇区外边界 (°) |
| `rear_deg` | 150.0 | 后扇区外边界 (°) |
| `scan_timeout` | 1.5 | `/scan` 超时停车秒数 |
| `range_max` | 3.5 | 距离上限裁剪 (m) |
| `control_rate` | 10.0 | watchdog 频率 (Hz) |

## 8. 核心算法

### 8.1 扇区划分（不依赖束数与 `angle_min`）

每个扫描点的角度用 `angle = angle_min + i * angle_increment` 换算，归一化到 `[-π, π]` 后按角度分扇区：

| 扇区 | 角度范围 | 用途 |
| :--- | :--- | :--- |
| 前方 | \|θ\| ≤ 20° | 判定直行 / 避障 / 脱困 |
| 左侧 | +20° ~ +60° | 转向方向、侧向让开 |
| 右侧 | −60° ~ −20° | 转向方向、侧向让开 |
| 左后 | +60° ~ +150° | 判断能否倒车 |
| 右后 | −150° ~ −60° | 判断能否倒车 |

因为按真实角度换算，360 束、720 束以及 `angle_min = -π` 的雷达都能正常工作，不会因束数不同而误判、也不会左右扇区错位。

### 8.2 决策流程

```
/scan
  │
  ▼
过滤 NaN/Inf/非正值 → 按角度归入五个扇区 → 各取最小距离
  │
  ▼
前方 < danger_distance ──► escape：倒车 + 转向；后方被堵则原地旋转
前方 < safe_distance   ──► avoid ：减速 + 朝宽侧转向（> exit_distance 才退出）
其他                   ──► cruise：直行（侧向过近时轻微让开）
  │
  ▼
/cmd_vel
```

### 8.3 安全保护

| 保护类型 | 触发条件 | 行为 |
| :--- | :--- | :--- |
| NaN/Inf 过滤 | 扫描点非有限值或 ≤ 0 | 跳过该点 |
| 数据不可用 | `ranges` 为空、`angle_increment` 非法、无任何合法点 | 停车并告警 |
| 扫描超时 | 超过 `scan_timeout` 秒未收到 `/scan` | 停车并告警（10 Hz 墙钟 watchdog） |
| 未就绪 | 节点启动后尚未收到扫描 | 停车 |
| 退出停车 | `Ctrl+C` / `rosnode kill` | `rospy.on_shutdown` 发布零速度 |

超时判断使用**墙钟** `time.time()`，watchdog 是独立的 `threading.Thread`，不读 `/clock`。
因此即使 `rosservice call /gazebo/pause_physics` 暂停物理、仿真时钟停走，或 `gzserver`
崩溃导致 `/clock` 不再更新，节点仍会在 `scan_timeout` 秒后持续发布零速度，
不会让最后一条 `cmd_vel` 滞留在控制器里。

## 8.4 话题与 TF

| 方向 | 话题 / TF | 发布者 | 说明 |
| :--- | :--- | :--- | :--- |
| 订阅 | `/scan` | `libgazebo_ros_laser.so` | 360 束，`angle_min = -π`，10 Hz |
| 发布 | `/cmd_vel` | `mbot_lidar_avoid_node` | 速度指令 |
| 发布 | `/odom` | `libgazebo_ros_diff_drive.so` | `child_frame_id = base_footprint`，`odometrySource = world` |
| 发布 | `/joint_states` | `libgazebo_ros_diff_drive.so` | `publishWheelJointState = true`，两个驱动轮 |
| 发布 | `/tf` | 差速插件 + `robot_state_publisher` | `odom → base_footprint → base_link → laser_link` 及左右轮 |

`base_footprint` 在地面（`base_footprint → base_link` 的 origin z = 0.11，等于轮子最低点到
`base_link` 的距离），所以 `/odom` 的 z 约为 0，后续接 `move_base` / `nav2` 时不会多出 10 cm 偏置。

## 9. 测试

仓库级 pytest 测试位于 `tests/test_mbot_lidar_avoid.py`，桩化 `rospy`，不需要 ROS / Gazebo：

```bash
python3 -m pytest tests/test_mbot_lidar_avoid.py -v
```

覆盖内容：

- **扇区换算**：360 束 / `angle_min=0`；720 束 / `angle_min=-π`；左右扇区不错位
- **回调与发布链路**：真实 `_scan_callback` → `Publisher.publish`
- **异常输入**：NaN/Inf 过滤、全 NaN、空 `ranges`、`angle_increment` 非法
- **控制行为**：滞回不抖动、脱困倒车、后方受阻原地旋转、侧向让开
- **安全保护**：扫描超时、未收到扫描、shutdown 停车
- **工程完整性**：launch 内容、CMakeLists 安装规则、`package.xml` 与 `LICENSE`

仿真集成测试 `tests/test_mbot_lidar_avoid_launch.py`（需要 ROS Noetic + Gazebo）：

```bash
# 静态校验：URDF 插件 filename、check_urdf、roslaunch --nodes 解析
python3 -m pytest tests/test_mbot_lidar_avoid_launch.py -v

# 端到端：自动启动 headless Gazebo，验证 /scan、/odom、/cmd_vel 有数据
MBOT_LAUNCH_TEST=1 python3 -m pytest tests/test_mbot_lidar_avoid_launch.py -v
```

## 10. 实验指标

| 指标项 | 数值 |
| :--- | :--- |
| 传感器 | 2D 激光雷达 `/scan`，360 束，量程 0.10–3.50 m |
| 避障触发距离 | 0.80 m（退出 1.00 m） |
| 脱困触发距离 | 0.30 m（退出 0.50 m） |
| 巡航线速度 | 0.30 m/s |
| 避障线速度 | 0.10 m/s |
| 脱困倒车速度 | −0.12 m/s |
| 转向 / 旋转角速度 | 1.20 / 0.90 rad/s |

## 11. 运行效果

以下截图来自 Ubuntu 20.04 + ROS Noetic + Gazebo 11 虚拟机的真实运行（`roslaunch mbot_gazebo_lidar_avoid main.launch`）：

![避障运行截图](./mbot_lidar_avoid_run.png)

截图只包含 Gazebo 窗口：封闭场地、三块障碍物，以及机器人位置周围的蓝色激光扫描扇形
（URDF 中 `<visualize>true</visualize>` 的效果），底部状态栏 `Real Time Factor: 0.91`、
`Sim Time: 00:00:45.321`。此截图不含 `rqt_graph` 与终端窗口。

同一台机器上的启动日志与话题实测（`roslaunch` 启动后采集）：

```text
$ grep -c WARN /tmp/launch.log
0
$ grep -E 'ERROR|KDL' /tmp/launch.log
（无输出）

$ rostopic list | grep -E 'scan|odom|cmd_vel|joint_states'
/cmd_vel
/joint_states
/odom
/scan

$ rostopic info /joint_states
Publishers:
 * /gazebo (http://ubuntu:39803/)
Subscribers:
 * /robot_state_publisher (http://ubuntu:35675/)

$ timeout 8 rostopic hz /scan | tail -2
average rate: 10.000
	min: 0.098s max: 0.102s std dev: 0.00082s window: 67

$ rostopic echo -n 1 /odom/pose/pose/position
x: -3.318942598991395
y: 3.0892870496548657
z: 1.3599388173635152e-06          # base_footprint 在地面，不再有 0.11 m 偏置

$ timeout 5 rosrun tf tf_echo odom base_footprint | grep Translation
- Translation: [-3.485, 2.199, 0.000]

$ timeout 5 rosrun tf tf_echo base_link left_wheel_link | grep Translation
- Translation: [-0.000, 0.190, -0.050]   # 轮子 TF 正常，旋转量随里程计变化
```

`WARN` 计数为 0，说明差速插件已按 ROS 1 参数名完整配置：`wheelTorque` 10、`wheelAcceleration` 3.0、
`updateRate` 100、`odometrySource` world、`publishTf` 1、`publishWheelJointState` true、
`publishOdomTF` / `publishWheelTF` true、`rosDebugLevel` na。若写成 `maxWheelTorque` /
`maxWheelAcceleration` / `publishOdom`（ROS 1 插件并不识别这些名字），插件会退回默认扭矩 5、
默认加速度 0，并逐项打印 `missing <wheelTorque> default is 5` 之类的 WARN。

TF 树为 `odom -> base_footprint -> base_link -> laser_link`：`publishOdomTF` 由差速插件发布
`odom -> base_footprint`（`robotBaseFrame` 设为 `base_footprint`），`robot_state_publisher`
再发布 `base_footprint -> base_link` 等固定关节，因此 `base_link` 不会出现两个父坐标系。

`publishWheelJointState` 打开后，差速插件自己发布 `/joint_states`（两个驱动轮），
`robot_state_publisher` 据此发布轮子 TF，RViz 中轮子会随里程计转动，无需额外的
`joint_state_publisher`。

### 11.1 暂停物理后的停车保护

watchdog 用墙钟线程，不依赖 `/clock`。实测把物理暂停后（`/scan` 停止更新），节点仍在
1.5 秒内把速度清零：

```text
$ rostopic echo -n 1 /cmd_vel          # 暂停前，外部手动下发
linear:
  x: 0.3

$ rosservice call /gazebo/pause_physics

$ sleep 4; rostopic echo -n 3 /cmd_vel  # 暂停后
  x: 0.0
  z: 0.0
  x: 0.0
  z: 0.0
  x: 0.0
  z: 0.0
```

端到端集成测试输出（`MBOT_LAUNCH_TEST=1`）：

```text
PASS  test_urdf_plugins_have_filename
PASS  test_diff_drive_plugin_is_gazebo_ros_impl
PASS  test_launch_refers_to_existing_files
PASS  test_urdf_valid_with_check_urdf
PASS  test_launch_nodes_resolvable
PASS  test_gazebo_topics_end_to_end

6 passed in 22.65 seconds
```

`test_gazebo_topics_end_to_end` 会真实启动 headless Gazebo，确认 `/scan`、`/odom` 有数据，发 `cmd_vel` 直行 3 秒后 `/odom` 位移增长 > 0.01 m，验证驱动插件与避障节点工作正常。

## 12. 许可证

MIT License，见 `src/ground/mbot_gazebo_lidar_avoid/LICENSE`。
