# mbot_gazebo_lidar_avoid

Mbot 差速底盘在 Gazebo 11 中的 2D 激光雷达阈值避障（ROS 1 Noetic）。
包内自带 URDF、world 与 launch，不依赖 `turtlebot3_*`，可独立运行。

完整教程见 [`docs/ground/mbot_lidar_avoid.md`](../../../docs/ground/mbot_lidar_avoid.md)。

## 环境版本

| 项目 | 版本 |
| :--- | :--- |
| 操作系统 | Ubuntu 20.04 LTS |
| ROS | ROS Noetic（catkin） |
| 仿真 | Gazebo 11（`gazebo_ros`） |
| 语言 | Python 3.8 |

## 安装依赖

```bash
sudo apt update
sudo apt install -y ros-noetic-gazebo-ros ros-noetic-gazebo-ros-pkgs \
                     ros-noetic-robot-state-publisher ros-noetic-urdf
```

## 构建

本包必须放进 catkin 工作空间的 `src/` 下才能编译。**不要直接用仓库根目录 `catkin_make`**：
仓库的 `src/` 里还有 ROS 2（ament）以及依赖 CARLA / MuJoCo 的包，会被一起配置并大概率失败。

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

安装到 install 空间（可选，用于验证资源文件是否正确安装）：

```bash
cd ~/mbot_ws
catkin_make install
source install/setup.bash
```

## 启动

```bash
roslaunch mbot_gazebo_lidar_avoid main.launch
```

该 launch 会依次：

1. 启动 Gazebo 并加载 `worlds/lidar_avoid_world.world`；
2. 从 `urdf/mbot_lidar.urdf` 生成 Mbot；
3. 启动 `robot_state_publisher` 发布 TF；
4. 加载 `config/lidar_avoid.yaml` 并以私有参数启动避障节点。

常用可选参数：

```bash
# 无界面运行
roslaunch mbot_gazebo_lidar_avoid main.launch gui:=false
# 指定初始位姿
roslaunch mbot_gazebo_lidar_avoid main.launch x:=1.0 y:=0.5 yaw:=1.57
```

## 停止

- 在运行 `roslaunch` 的终端按 `Ctrl+C`；节点注册了 `rospy.on_shutdown`，退出前会发布零速度。
- 或单独结束节点：`rosnode kill /mbot_lidar_avoid_node`。
- 残留进程：`killall -9 gazebo gzserver gzclient roslaunch`。

## 参数

所有参数在 `config/lidar_avoid.yaml` 中，键名与节点名一致，可运行时修改：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `cruise_speed` | 0.30 | 巡航线速度 (m/s) |
| `avoid_speed` | 0.10 | 避障线速度 (m/s) |
| `reverse_speed` | -0.12 | 脱困倒车速度 (m/s) |
| `turn_rate` | 1.20 | 避障转向角速度 (rad/s) |
| `rotate_rate` | 0.90 | 脱困旋转角速度 (rad/s) |
| `safe_distance` | 0.80 | 进入避障阈值 (m) |
| `exit_distance` | 1.00 | 退出避障阈值 (m)，与进入阈值构成滞回 |
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

在线调参示例：

```bash
rosparam set /mbot_lidar_avoid_node/safe_distance 1.0
```

## 控制逻辑

扇区按 `angle_min + i * angle_increment` 换算真实角度，不假定束数与 `angle_min`，
因此 360 束、720 束以及 `angle_min = -π` 的雷达都能正常工作。

| 状态 | 触发条件 | 行为 |
| :--- | :--- | :--- |
| `cruise` | 前方 ≥ `safe_distance` | 直行；侧向过近时轻微让开 |
| `avoid` | 前方 < `safe_distance` | 减速并朝更宽一侧转向；前方 > `exit_distance` 才退出 |
| `escape` | 前方 < `danger_distance` | 倒车并转向远离更近一侧；后方两侧都堵则原地旋转 |

`/scan` 超过 `scan_timeout` 秒未更新、扫描为空、`angle_increment` 非法或全部为 NaN/Inf 时，
节点发布零速度并告警。

## 测试

离线单元测试（桩化 rospy，不需要 ROS / Gazebo）：

```bash
python3 -m pytest tests/test_mbot_lidar_avoid.py -v
```

仿真集成测试（需要 ROS Noetic + Gazebo）。默认只做静态校验（URDF 插件属性、launch 节点解析）；
设置 `MBOT_LAUNCH_TEST=1` 后会真正拉起 Gazebo（headless），验证 `/scan`、`/odom`、`/cmd_vel` 是否连通：

```bash
# 静态校验
python3 -m pytest tests/test_mbot_lidar_avoid_launch.py -v

# 端到端：自动启动与关闭 Gazebo
MBOT_LAUNCH_TEST=1 python3 -m pytest tests/test_mbot_lidar_avoid_launch.py -v
```

## 许可证

MIT，见 `LICENSE`。
