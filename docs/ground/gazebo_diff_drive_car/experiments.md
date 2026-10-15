# 实验设置与操作步骤

> **任务书原文**
>
> (1) 仿真环境时需给出具体的仿真步骤，包括操作步骤和截图

本章提供**完整可复现**的实验操作步骤，包括环境准备、启动流程、
数据采集与结果验证。

---

## 8.1 实验环境

### 8.1.1 硬件配置

| 项目 | 配置 |
|---|---|
| 宿主机 | Windows PC，VMware Workstation |
| 虚拟机 | Ubuntu 20.04.6 LTS |
| CPU | 6 核 |
| 内存 | 8 GB（分配给虚拟机 8.78 GB） |
| 显卡 | VMware SVGA II（软件渲染） |
| 磁盘 | 70 GB SCSI |

### 8.1.2 软件环境

| 软件 | 版本 |
|---|---|
| Ubuntu | 20.04.6 LTS |
| 内核 | 5.15.0-84-generic |
| ROS | Noetic Ninjemys |
| Gazebo | 11.13.0 |
| Python | 3.8.10 |
| numpy | 1.17.4 |
| matplotlib | 3.1.2 |

### 8.1.3 环境验证

```bash
# 验证 ROS 安装
ls /opt/ros/
# 期望输出: noetic

# 验证 Gazebo
gazebo --version
# 期望输出: Gazebo multi-robot simulator, version 11.13.0

# 验证关键插件
ls /opt/ros/noetic/lib/libgazebo_ros_*.so | grep -E "laser|depth|diff_drive"
# 期望输出:
#   libgazebo_ros_depth_camera.so
#   libgazebo_ros_diff_drive.so
#   libgazebo_ros_laser.so
```

---

## 8.2 操作步骤

### 步骤 1：获取代码

```bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src

# 复制本项目包
cp -r /path/to/openhutb_car .

# 确认包结构
ls openhutb_car/
# 期望输出: CMakeLists.txt  README.md  config  launch  package.xml
#           rviz  scripts  urdf  worlds
```

### 步骤 2：编译

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

**期望输出**：

```
-- +++ processing catkin package: 'openhutb_car'
-- ==> add_subdirectory(openhutb_car)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- Configuring done
-- Generating done
-- Build files have been written to: /home/user/catkin_ws/build
####
#### Running command: "make -j6 -l6"
####
Scanning dependencies of target openhutb_car_scripts
[ 20%] Copying laser_avoid.py to devel space
[ 40%] Copying main.py to devel space
[ 60%] Copying teleop_keyboard.py to devel space
[ 80%] Copying depth_camera_avoid.py to devel space
[100%] Copying odom_recorder.py to devel space
[100%] Built target openhutb_car_scripts
```

### 步骤 3：载入环境

```bash
source ~/catkin_ws/devel/setup.bash

# 建议追加到 ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

**验证包已被 ROS 发现**：

```bash
rospack find openhutb_car
# 期望输出: /home/user/catkin_ws/src/openhutb_car

roslaunch --nodes openhutb_car main.launch
# 期望输出:
#   /gazebo
#   /gazebo_gui
#   /spawn_car
#   /robot_state_publisher
#   /joint_state_publisher
#   /openhutb_car_main
#   /odom_recorder
```

### 步骤 4：验证模型文件

```bash
cd ~/catkin_ws
xacro src/openhutb_car/urdf/car.xacro > /tmp/car.urdf

check_urdf /tmp/car.urdf
```

**期望输出**：

```
robot name is: openhutb_car
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
        child(1):  camera_link
            child(1):  camera_optical_link
        child(2):  front_caster_link
        child(3):  laser_link
        child(4):  left_wheel_link
        child(5):  rear_caster_link
        child(6):  right_wheel_link
```

### 步骤 5：启动仿真

**实验 A：键盘遥控**

```bash
roslaunch openhutb_car main.launch mode:=teleop
```

![键盘遥控模式启动截图](../images/G_teleop_a.png)

*图 8-1：键盘遥控模式启动后的 Gazebo 界面。
可见蓝色车体无人车、红色激光雷达、场地围墙与红色圆柱障碍物。*

启动后**在运行该命令的终端**按键控制：

```
============================================================
          OpenHUTB 无人车 —— 键盘运动控制
============================================================
   移动控制                     速度调节
   ----------------------       ----------------------
     w : 前进                     + : 提高最大速度
     s : 后退                     - : 降低最大速度
     a : 左转
     d : 右转                     其他
     q : 前进 + 左转              ------
     e : 前进 + 右转               空格 : 紧急停止
     z : 后退 + 左转                 x  : 退出程序
     c : 后退 + 右转
   ----------------------
   注意：松开按键后车辆保持当前速度，需按空格停车。
============================================================

  [速度]  线速度 v = +0.20 m/s   角速度 ω = +0.00 rad/s  上限 0.80 | 1.20
```

**实验 B：激光雷达避障**

```bash
roslaunch openhutb_car main.launch mode:=laser
```

![激光雷达避障截图](../images/H_laser_a.png)

*图 8-2：激光雷达避障模式。可见激光束在圆柱、方块与墙面上的
回波扇形，以及障碍物背后的扫描阴影区。*

**实验 C：深度相机避障**

```bash
roslaunch openhutb_car main.launch mode:=camera
```

![深度相机避障截图](../images/I_camera_a.png)

*图 8-3：深度相机避障模式运行场景。*

### 步骤 6：在另一终端观察数据

保持仿真运行，**新开一个终端**：

```bash
source ~/catkin_ws/devel/setup.bash

# 观察控制指令频率
rostopic hz /cmd_vel

# 观察激光数据
rostopic hz /scan

# 观察深度图频率
rostopic hz /camera/depth/image_raw

# 观察车辆位姿
rostopic echo /odom
```

### 步骤 7：采集轨迹数据

```bash
# 采集 60 秒的位姿数据
timeout 60 rostopic echo -p /odom/pose/pose/position > ~/traj_laser.csv

# 查看采样点数
wc -l ~/traj_laser.csv
```

### 步骤 8：分析结果

```bash
cd ~/results
python3 ~/catkin_ws/src/openhutb_car/scripts/analyze_trajectory.py \
    teleop.csv laser.csv camera.csv
```

**期望输出**：

```
====================================================================================
              OpenHUTB 无人车 —— 轨迹性能评价
====================================================================================
算法/模式        时长(s)   路程(m)   位移(m)  均速(m/s)  峰值(m/s)  转向(rad)
------------------------------------------------------------------------------------
teleop           58.43      6.34      6.31     0.108     4.001   1777.35
laser            57.08     18.22      9.07     0.319     7.006     51.05
camera           58.45     12.82      9.73     0.219     6.007    165.38
====================================================================================

路径效率 = 净位移 / 实际路程，越接近 1 表示路径越直：
  teleop       0.997   （X 范围 -0.62~5.69，Y 范围 -3.98~-3.82）
  laser        0.498   （X 范围 -5.21~5.07，Y 范围 -5.26~1.12）
  camera       0.758   （X 范围 1.56~5.58，Y 范围 -3.96~5.00）

轨迹对比图已保存：trajectory_plot.png
速度曲线图已保存：speed_profile.png
指标报告已保存：metrics_report.txt
```

### 步骤 9：查看生成的图表

![轨迹对比图](../data/trajectory_plot.png)

*图 8-4：三种模式的轨迹对比。灰色为场地围墙，
红色圆形与蓝色方块为障碍物。*

### 步骤 10：退出

在运行 `roslaunch` 的终端按 `Ctrl+C`。程序会：

1. 停止算法节点
2. 输出性能报告
3. 保存轨迹 CSV 到 `~/openhutb_car_results/`
4. 关闭 Gazebo 与所有节点

**退出时的性能报告**：

```
================================================================
          无人车运行性能评价报告
================================================================
  运行时长        :    58.43 s
  轨迹总长度      :     6.34 m
  平均线速度      :    0.108 m/s
  最大线速度      :    4.001 m/s
  平均角速度      :    0.145 rad/s
  轨迹采样点数    :     2915
  X 范围          :    -0.62 ~ 5.69 m
  Y 范围          :    -3.98 ~ -3.82 m
================================================================
```

---

## 8.3 无图形界面运行（推荐用于批量实验）

在服务器或需要批量采集数据时，可关闭 Gazebo GUI 以节省资源：

```bash
roslaunch openhutb_car main.launch mode:=laser gui:=false
```

**性能对比**：

| 配置 | 实时因子 (RTF) | CPU 占用 |
|---|---|---|
| `gui:=true` | 0.58 ~ 0.85 | 高（软件渲染） |
| `gui:=false` | **0.98 ~ 1.00** | 低 |

**结论**：无 GUI 模式下仿真几乎达到实时（RTF ≈ 1.0），
适合长时间数据采集。

---

## 8.4 批量实验脚本

本项目提供了批量采集脚本模板：

```bash
#!/bin/bash
# batch_experiment.sh —— 依次运行三种模式并采集数据

export DISPLAY=:0
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

run_exp () {
  local MODE=$1
  local TAG=$2

  # 清理残留进程
  pkill -f gzserver; pkill -f gzclient
  pkill -f rosmaster; pkill -f "main.py"
  sleep 3

  # 启动仿真
  setsid roslaunch openhutb_car main.launch mode:=$MODE gui:=false \
     > ~/exp_${TAG}.log 2>&1 < /dev/null &
  sleep 25                       # 等待 Gazebo 完全启动

  # 采集 60 秒轨迹
  timeout 60 rostopic echo -p /odom/pose/pose/position > ~/${TAG}.csv

  echo "=== $TAG ==="
  echo "samples: $(wc -l < ~/${TAG}.csv)"
}

run_exp teleop T_teleop
run_exp laser  T_laser
run_exp camera T_camera
```

**输出**：

```
=== T_teleop ===
samples: 2915

=== T_laser ===
samples: 2855

=== T_camera ===
samples: 2924
```

---

## 8.5 实验注意事项

### 8.5.1 启动时间

Gazebo 首次启动需要 **20~25 秒**（软件渲染下）。若在启动完成前
就采集数据，会得到大量零值。

**建议**：启动后 `sleep 25` 再开始采集。

### 8.5.2 出生点选择

launch 默认出生点为 `(-5.0, -4.0)`，位于场景西南侧空旷区
（`world` 文件中用绿色 `spawn_marker` 标记）。

**不要选在障碍物密集处** —— 车辆生成时可能与障碍物重叠，
导致物理引擎解算异常（车辆被弹开或卡住）。

### 8.5.3 残留进程清理

Gazebo 异常退出时会残留 `gzserver` / `gzclient` 进程，
占用端口导致下次启动失败。每次实验前应清理：

```bash
pkill -f gzserver
pkill -f gzclient
pkill -f rosmaster
pkill -f "main.py"
sleep 3
```

### 8.5.4 锁文件问题

虚拟机异常关机后可能残留 `.vmx.lck` 锁目录，
导致虚拟机无法启动。处理方式：

- **正常情况**：VMware 会自动清理
- **异常残留**：确认无 `vmware-vmx` 进程后手动删除

> **警告**：删除锁目录前必须确认虚拟机真的没有运行，
> 否则会损坏磁盘文件。

---

## 8.6 常见操作问题

| 现象 | 原因 | 解决方法 |
|---|---|---|
| `RLException: ... is neither a launch file` | 未 source 工作区 | `source devel/setup.bash` |
| `xacro: XML parsing error` | XML 注释含连续 `--` | 注释内不要用 `----` 分隔线 |
| Gazebo 启动后一片黑 | 软件渲染慢 | 等待 20~30 秒 |
| 车辆不动 | 车体触地摩擦 | 检查 `chassis_z` 与摩擦系数 |
| `/cmd_vel` 无消息 | 看门狗未发布 | 检查 `feed()` 是否调用 `publish()` |
| 相机深度全为 1.0 | 深度地板值 | 检查地板值剔除逻辑 |
| `rostopic echo` 无输出 | 仿真时间未启动 | 等待 `/clock` 开始发布 |
