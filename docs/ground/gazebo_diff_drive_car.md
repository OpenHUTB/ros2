# Gazebo 差速驱动小车物理仿真与多传感器闭环控制

> 对应课程任务（1）与（2）：在物理仿真环境中通过键盘对机器人进行运动控制；
> 使用传感器获取的数据进行感知并对机器人进行运动控制。
>
> **仿真平台**：Gazebo 11 ｜ **中间件**：ROS 2 Humble ｜ **底盘**：差速驱动
> ｜ **机器人**：自建 URDF 小车（含激光雷达与 RGB-D 深度相机）

## 1. 概述与目标

本模块在 Gazebo 中构建一台差速驱动小车，完成三项要求：

| 任务书要求 | 实现文件 |
|---|---|
| (1) 物理仿真 + 键盘运动控制 | `gazebo_diff_drive_car/teleop_keyboard.py` |
| (2) 传感器感知 → 控制闭环（**激光雷达**） | `gazebo_diff_drive_car/laser_avoid.py` |
| (2) 传感器感知 → 控制闭环（**深度相机**） | `gazebo_diff_drive_car/depth_camera_avoid.py` |

三项功能共用同一套底盘，通过统一入口的 `mode` 参数切换：

```bash
./main.sh                 # 键盘遥控（默认）
./main.sh laser           # 激光雷达避障
./main.sh camera          # 深度相机避障
./main.sh idle            # 仅加载仿真环境
./main.sh laser nogui     # 无图形界面
```

### 1.1 与同目录其他模块的关系

`src/ground/` 下已有多个仿真平台的载具实现，本模块补充 **Gazebo 平台的自建差速底盘**：

| 模块 | 仿真平台 | 说明 |
|---|---|---|
| `carla_keyboard_control` | CARLA | 车辆键盘控制 |
| `mujoco_diff_drive_slip_compensation` | MuJoCo | 差速小车轮滑补偿 |
| `turtlebot3_lidar_avoid` | Gazebo | TurtleBot3 现成模型的雷达避障 |
| **`gazebo_diff_drive_car`** | **Gazebo** | **自建差速底盘 + 雷达/深度相机双闭环** |

与 `turtlebot3_lidar_avoid` 的区别：后者直接使用 TurtleBot3 现成模型，
本模块则从 URDF 开始自建底盘、传感器安装位置与 Gazebo 插件配置，
并额外提供深度相机闭环与键盘控制。

## 2. 运行环境

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS | Humble |
| 仿真平台 | Gazebo 11 |
| 构建系统 | colcon（ament_python） |
| Python | 3.10 |
| 第三方库 | numpy（深度图处理） |

### 2.1 依赖安装

```bash
sudo apt install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-robot-state-publisher \
    python3-numpy
```

> 说明：本模块的机器人模型为**纯 URDF**（`urdf/car.urdf`），
> 不依赖 xacro 可执行文件；控制节点只依赖 rclpy 与 numpy，
> 无需 CARLA 等外部仿真器。

## 3. 编译与运行

```bash
# 编译
mkdir -p ~/ros2_ws/src
cp -r /path/to/src/ground/gazebo_diff_drive_car ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gazebo_diff_drive_car
source install/setup.bash
```

### 3.1 启动

```bash
# 统一入口脚本
./main.sh                 # 键盘遥控
./main.sh laser           # 激光雷达避障
./main.sh camera          # 深度相机避障

# 或直接用 launch
ros2 launch gazebo_diff_drive_car main.launch.py mode:=laser
ros2 launch gazebo_diff_drive_car main.launch.py mode:=camera gui:=false
```

### 3.2 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `mode` | `teleop` | `teleop` / `laser` / `camera` / `idle` |
| `gui` | `true` | 是否启动 Gazebo 图形界面 |
| `rviz` | `false` | 是否启动 RViz |
| `x` `y` `z` `yaw` | `-5.0` `-4.0` `0.15` `0.0` | 机器人初始位姿 |

> **相机模式的显示要求**：深度相机依赖 Gazebo 渲染管线。
> 在无 `DISPLAY` 的环境（纯 SSH）中渲染会被禁用，`/camera/*` 话题不会出现；
> 此时请使用带图形会话的终端，或先启动 Xvfb 再运行。
> 激光雷达为物理射线传感器，不受此限制。

### 3.3 键盘操作

| 按键 | 功能 | 按键 | 功能 |
|---|---|---|---|
| `w` / `s` | 前进 / 后退 | `a` / `d` | 左转 / 右转 |
| `q` / `e` | 前进 + 左转 / 右转 | `z` / `c` | 后退 + 左转 / 右转 |
| `空格` | 紧急停止 | `x` | 退出 |
| `+` / `-` | 调整线速度上限 | | |

采用增量式控制：松开按键后保持当前速度，需按空格停车。

## 4. 运行效果

### 4.1 键盘运动控制

![Gazebo 差速驱动小车键盘遥控模式](../img/ground/gazebo_diff_drive_car_teleop.png)

*图 1：键盘遥控模式。蓝色车体、黑色驱动轮、车头红色激光雷达。*

### 4.2 激光雷达避障

![激光雷达避障模式](../img/ground/gazebo_diff_drive_car_laser.png)

*图 2：激光雷达避障模式。可见激光束在圆柱、方块与墙面上的回波扇形，
以及障碍物背后的扫描阴影区。*

### 4.3 深度相机避障

![深度相机避障模式](../img/ground/gazebo_diff_drive_car_camera.png)

*图 3：深度相机避障模式。*

### 4.4 三种模式的轨迹对比

![三种模式轨迹对比](../img/ground/gazebo_diff_drive_car_trajectory.png)

*图 4：三种模式的轨迹对比。灰色为场地围墙（12×12 m），
红色圆形与蓝色方块为障碍物。*

## 5. 实测结果

在 Ubuntu 22.04 + ROS 2 Humble + Gazebo 11 环境中的实测数据：

### 5.1 话题频率

| 话题 | 类型 | 实测频率 |
|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | 9.97 Hz（360 束，360°） |
| `odom` | `nav_msgs/Odometry` | 99.46 Hz |
| `cmd_vel` | `geometry_msgs/Twist` | 9.98 Hz |
| `camera/depth/image_raw` | `sensor_msgs/Image` | 1.32 Hz（软件渲染） |
| `camera/image_raw` | `sensor_msgs/Image` | 5.50 Hz |

### 5.2 激光雷达模式

| 指标 | 实测值 |
|---|---|
| 巡航线速度 | 0.35 m/s |
| 20 秒位移 | **3.86 m** |
| 状态切换 | `cruise → avoid`（前方 0.97 m）→ `cruise`（1.53 m） |

### 5.3 深度相机模式

| 指标 | 实测值 |
|---|---|
| 深度图频率 | 1.32 Hz |
| 输出速度指令 | 后退 −0.10 m/s（进入紧急状态时） |
| 状态切换 | `emergency → avoid → cruise → emergency` |

> 深度相机在软件渲染下帧率较低（约 1.3 Hz），控制响应会比激光模式迟钝，
> 这是虚拟机无 GPU 的限制，非算法问题。

### 5.4 三种模式的性能对比（ROS 1 版本历史数据，供参考）

| 模式 | 行驶路程 | 净位移 | 路径效率 | 碰撞 |
|---|---|---|---|---|
| `teleop` | 6.34 m | 6.31 m | 0.997 | — |
| `laser` | 18.22 m | 9.07 m | 0.498 | 0 |
| `camera` | 12.82 m | 9.73 m | 0.758 | 0 |

**路径效率** = 净位移 / 实际路程，越接近 1 表示路径越直。

## 6. 实现原理

### 6.1 车体模型与离地高度

| 参数 | 值 |
|---|---|
| 车体 | 0.44 × 0.38 × 0.22 m，5.0 kg |
| 车轮半径 $r$ | 0.0975 m |
| 轮间距 $L$ | 0.335 m |
| 车体中心高度 | $r + h/2 = 0.2075$ m |
| 离地间隙 | 0.0975 m（等于车轮半径） |

**离地高度为何关键**：若车体碰撞盒底面与地面接触（车体中心取
$h/2 = 0.11$ m），Gazebo 会让车体与地面产生持续切向摩擦，
驱动轮虽在旋转但整车被「钉住」。实测表现为车轮转过 168.9 rad
而车体 30 秒仅移动 0.065 m，且 `cmd_vel` 无任何异常——极难排查。
抬高车体后仅驱动轮与地面接触，实测 20 秒可行驶 3.86 m。

万向轮在 URDF 中为 `fixed` 关节球体，**不赋予 collision 几何**，
仅作视觉件；否则会因与地面持续接触产生额外滑动摩擦导致转向迟滞。

### 6.2 差速驱动运动学

$$
v = \frac{v_r + v_l}{2}, \qquad \omega = \frac{v_r - v_l}{L}
$$

反向求解轮角速度：

$$
\omega_l = \frac{v - \omega L/2}{r}, \qquad \omega_r = \frac{v + \omega L/2}{r}
$$

控制节点只发布 `geometry_msgs/Twist`，由 `libgazebo_ros_diff_drive.so`
完成上述解算，使控制逻辑与底盘解耦。

### 6.3 激光雷达避障

**感知**：360 束扫描划分五个扇区，各取最小距离。取最小值而非均值——
均值会被远处空旷区域稀释，导致对近处障碍不敏感。

| 扇区 | 角度范围 |
|---|---|
| 前方 | $[-30°, +30°]$ |
| 左侧 / 右侧 | $[+30°, +90°]$ / $[-90°, -30°]$ |
| 左后 / 右后 | $[+90°, +180°]$ / $[-180°, -90°]$ |

**决策**：三态状态机

| 状态 | 进入条件 | 行为 |
|---|---|---|
| `cruise` | 前方 > 1.4 m | 直行 0.35 m/s |
| `avoid` | 前方 < 1.0 m | 减速并转向 |
| `emergency` | 前方 < 0.45 m | 后退 0.8 s |

进入与退出避障使用不同阈值（1.0 / 1.4 m）形成**滞回**，
避免距离在阈值附近波动时状态高频切换。

**控制**：势场法。转向方向由左右可通行度之差决定：

$$
\Delta = d_{\text{left}} - d_{\text{right}}, \qquad
\omega = k_\omega \cdot \text{clamp}\!\left(\frac{\Delta}{2.0},\; -1,\; +1\right)
$$

线速度随前方距离线性衰减；并施加最小转向力度约束
（$|\omega| < 0.45$ rad/s 时强制取 $\pm 0.45$），
用于规避纯势场法的局部极小值问题——避免车辆贴着障碍面平移而不真正转开。

### 6.4 深度相机避障

**感知**：640×480 深度图分左/中/右三条带（0~25%、37~62%、75~100%），
竖直方向只取 25%~75% 行（避开顶部天空与底部地面），
对每条带取**第 25 百分位深度**：用最小值易被单点噪声误判，
用均值会被远处空旷区域稀释，低百分位兼顾抗噪与敏感性。

**决策与控制**：与雷达版本同构的三态状态机，参数更保守
（$d_{\text{safe}} = 1.2$ m，$v_{\text{avoid}} = 0.15$ m/s），
因为相机水平视场仅 80°，侧向感知能力弱，需要更大安全余量。

## 7. 传感器数据处理中的两个关键问题

两个缺陷都不报错、不崩溃，只是让算法做出错误决策。

### 7.1 激光雷达自遮挡

**现象**：车辆处于空旷位置时，正前方仍持续返回 0.12~0.40 m 的虚假回波，
算法误判「前方被堵死」，车辆原地打转。

**原因**：雷达虽安装于车体轮廓外 0.06 m，但激光束以 360° 放射，
部分光束仍会扫到车体边缘。

**判别依据**：真实障碍会**连续遮挡多束**激光，自遮挡只影响孤立几束。

**方案**：对每束小于 0.30 m 的回波，检查邻域 ±5 束内是否也有近距回波，
连续数 ≥ 5 才判定为真实障碍，否则作为噪声剔除。

### 7.2 深度图地板值

**现象**：深度图统计结果恒为常数 1.000 m，算法判定「前方 1 m 有障碍」
（$< d_{\text{safe}} = 1.2$ m），车辆原地打转，30 秒仅移动 0.08 m。

**原因**：Gazebo 深度相机对**无回波区域**并非输出 0 或 inf，
而是输出固定常数 **1.005 m**。实测中区该值占比达 **66%**，
无论取 p15、p25 还是 p40，百分位统计都会被完全锁死。

**关键难度**：1.005 m 是**看起来完全合理**的深度读数，
min / median / max 中只有 max（8.76 m）暴露了异常，
直到绘制直方图才看清 66% 的像素压在同一个值上。

**方案**：直方图峰值检测——若某深度值占比超过 30%，判定为无效标记并剔除；
剔除后像素不足 30 个时退回原数据，避免过度剔除。

修复后中区 $P_{25}$ 从 1.000 m 恢复为 3.509 m，相机模式 30 秒移动 7.40 m
（修复前 0.08 m）。

## 8. 工程问题排查记录

| # | 问题 | 现象 | 根因 | 方案 |
|---|---|---|---|---|
| 1 | `/cmd_vel` 无消息 | 话题连接正常、无报错，车辆不动 | 看门狗只缓存指令未发布 | 直接发布 |
| 2 | 车轮空转、车辆不动 | 车轮转 168 rad，位移 0.065 m | 底盘离地间隙为零，车体与地面摩擦 | 车体中心抬至 $r + h/2$，驱动轮摩擦系数 1.0 |
| 3 | 深度读数恒为 1.0 | 读数「看起来正常」 | Gazebo 无回波区输出地板值 1.005 m（占比 66%） | 直方图峰值检测并剔除 |
| 4 | 主循环卡死 | 启动后数秒再无输出 | ROS 1 的 `rospy.Rate` 依赖未发布的仿真时钟 | ROS 2 改用 `create_timer` |
| 5 | 万向轮拖拽 | 转向迟滞 | `fixed` 关节球体与地面持续接触 | 移除 collision，保留视觉件 |
| 6 | 深度相机无话题 | `camera/*` 全部缺失 | gzserver 无 `DISPLAY` 时渲染被禁用 | 提供图形会话或 Xvfb |
| 7 | 迁移后 launch 找不到包 | `package not found` | 未 `source install/setup.bash` | 启动前载入工作空间 overlay |
| 8 | 依赖 xacro 而环境未装 | `xacro: command not found` | 部分 ROS 2 环境未安装 `ros-humble-xacro` | 改为纯 URDF，去掉该依赖 |

**最重要的一条经验**：不要相信「看起来正常」的数据。
深度值 1.005 m 看起来正常，实为无效标记；`cmd_vel` 话题连接正常，
实际从未发布；车轮正常转动，实际车辆没动。

## 9. 模块结构与接口

### 9.1 目录结构

```
gazebo_diff_drive_car/
├── main.py                      模块入口（课程 main.* 约定）
├── main.sh / main.bat           Linux / Windows 启动脚本
├── package.xml / setup.py / setup.cfg
├── launch/main.launch.py        ROS 2 launch 入口
├── urdf/car.urdf                车辆模型（纯 URDF，含 ROS 2 Gazebo 插件）
├── worlds/car_world.world       仿真场景（12×12 m + 障碍物）
├── config/params.yaml           参数配置
├── rviz/car.rviz                RViz 配置
└── gazebo_diff_drive_car/
    ├── main.py                  统一入口节点（按 mode 调度）
    ├── laser_avoid.py           激光雷达避障
    ├── depth_camera_avoid.py    深度相机避障
    ├── teleop_keyboard.py       键盘控制
    └── odom_recorder.py         轨迹记录与性能评价
```

### 9.2 话题接口

**订阅**

| 话题 | 类型 | 使用节点 |
|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | `laser_avoid` |
| `camera/depth/image_raw` | `sensor_msgs/Image`（32FC1） | `depth_camera_avoid` |
| `odom` | `nav_msgs/Odometry` | `odom_recorder` |

**发布**

| 话题 | 类型 | 发布者 |
|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | 各控制节点 |
| `odom` | `nav_msgs/Odometry` | Gazebo 差速驱动插件 |
| `scan` | `sensor_msgs/LaserScan` | Gazebo 射线传感器插件 |
| `camera/image_raw`、`camera/depth/image_raw`、`camera/points` | 相机相关 | Gazebo 相机插件 |

### 9.3 ROS 2 Gazebo 插件

| 功能 | 插件 | 关键参数 |
|---|---|---|
| 差速驱动 | `libgazebo_ros_diff_drive.so` | `left_joint`、`right_joint`、`wheel_separation`、`wheel_diameter` |
| 激光雷达 | `libgazebo_ros_ray_sensor.so` | `output_type=sensor_msgs/LaserScan`，`~/out:=scan` |
| 深度相机 | `libgazebo_ros_camera.so` | `camera_name`、`min_depth`、`max_depth` |
| 关节状态 | `libgazebo_ros_joint_state_publisher.so` | `joint_name`、`update_rate` |

> 与 ROS 1 的差异：ROS 1 使用 `libgazebo_ros_laser.so` 并把参数写在插件下；
> ROS 2（gazebo_ros_pkgs）改为 `libgazebo_ros_ray_sensor.so` + `output_type`，
> 差速驱动参数改为下划线命名，并通过 `<ros><remapping>` 映射话题。

### 9.4 TF 树

```
odom
 └── base_footprint
      └── base_link
           ├── left_wheel_link / right_wheel_link
           ├── front_caster_link / rear_caster_link
           ├── laser_link
           └── camera_link
                └── camera_optical_link
```

## 10. 参数汇总

### 10.1 激光雷达避障

| 参数 | 值 | 说明 |
|---|---|---|
| `safe_distance` / `clear_distance` / `danger_distance` | 1.0 / 1.4 / 0.45 m | 进入 / 退出 / 紧急阈值 |
| `cruise_speed` / `avoid_speed` / `back_speed` | 0.35 / 0.18 / −0.12 m/s | 速度 |
| `min_turn` / `max_turn` / `turn_gain` | 0.45 / 1.30 rad/s / 1.8 | 转向 |
| `emergency_hold` | 0.8 s | 紧急状态最短时长 |
| `self_occlusion_range` / `min_close_hits` | 0.30 m / 5 | 自遮挡判别 |

### 10.2 深度相机避障

| 参数 | 值 | 说明 |
|---|---|---|
| `safe_distance` / `clear_distance` / `danger_distance` | 1.2 / 1.6 / 0.5 m | 阈值（比雷达保守） |
| `cruise_speed` / `avoid_speed` / `back_speed` | 0.30 / 0.15 / −0.10 m/s | 速度 |
| `min_turn` / `max_turn` / `turn_gain` | 0.40 / 1.20 rad/s / 1.6 | 转向 |
| `depth_percentile` | 25.0 | 深度统计百分位 |
| `floor_value_ratio` / `floor_bin_width` | 0.30 / 0.01 m | 地板值检测 |
| `max_spin_duration` | 1.5 s | 原地转向超时 |

### 10.3 键盘与系统

| 参数 | 值 |
|---|---|
| 线速度增量 / 上限 | 0.1 / 0.8 m/s |
| 角速度增量 / 上限 | 0.2 / 1.2 rad/s |
| 发布频率 | 20 Hz |

## 11. 已知限制与改进方向

| 限制 | 说明 | 改进方向 |
|---|---|---|
| 无全局规划 | 纯反应式避障，存在局部极小值风险 | 引入 Nav2 / A* 生成全局路径 |
| 深度相机帧率低 | 软件渲染下约 1.3 Hz | 启用 GPU 渲染或降低分辨率 |
| 单传感器独立工作 | 雷达与相机各自独立闭环，未融合 | 传感器融合 |
| 参数硬编码 | 参数以内置默认值写在代码中 | 接入参数 YAML 统一加载 |

## 12. 声明

本模块的开发过程使用了 AI 大模型辅助（代码编写、文档撰写与问题排查），
提交者对全部内容负责。所有代码均已在 Ubuntu 22.04 + ROS 2 Humble +
Gazebo 11 环境中实际编译运行验证。

## 13. 许可证

MIT License
