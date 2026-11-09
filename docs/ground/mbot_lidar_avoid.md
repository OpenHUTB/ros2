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

本包必须放进 catkin 工作空间的 `src/` 下才能编译。

方式一：以仓库根目录作为 catkin 工作空间：

```bash
cd <仓库根目录>
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

验证安装空间（可选）：

```bash
cd ~/catkin_ws
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

参数全部放在 `config/lidar_avoid.yaml`，由 launch 以 rosparam 加载，运行时可在线修改（`rosparam set /mbot_lidar_avoid_node/xxx`）。

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
| 扫描超时 | 超过 `scan_timeout` 秒未收到 `/scan` | 停车并告警（10 Hz watchdog） |
| 未就绪 | 节点启动后尚未收到扫描 | 停车 |
| 退出停车 | `Ctrl+C` / `rosnode kill` | `rospy.on_shutdown` 发布零速度 |

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

## 11. 许可证

MIT License，见 `src/ground/mbot_gazebo_lidar_avoid/LICENSE`。
