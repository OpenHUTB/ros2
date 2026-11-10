# uav_perception_trajectory — 无人机多传感器感知与给定轨迹跟踪控制

对应课程任务（2）：*实现传感器的感知及运动控制，要求使用传感器获取到的数据进行感知
（如雷达、摄像头、深度摄像头等）对机器人进行运动控制（5·2：感知、给定轨迹的控制）*。

详细文档：[docs/air/uav_perception_trajectory.md](../../../docs/air/uav_perception_trajectory.md)

## 功能

让无人机**沿给定轨迹自主飞行**：用机载传感器（激光雷达 + 深度相机）做环境感知，
用轨迹跟踪控制器（前馈 + PID）计算速度指令，全程由传感器数据闭环驱动。

## 控制链路

```text
/lidar/points ─┐
               ├─► uav_perception_trajectory ──/uav/cmd_vel──► carlair_ros_bridge ──► CarlaAir
/uav/odom    ──┘   (感知 → 决策 → 控制)
```

## 任务解读

| 子项 | 含义 | 实现 |
|---|---|---|
| **感知** | 用传感器数据**理解环境**，而非只做数据转发 | 点云 → 16 扇形最近距离直方图；深度图 → 可通行性度量 |
| **给定轨迹的控制** | 沿**预设参考轨迹**飞行，用感知数据保证安全 | 参数化轨迹（line/helix/figure8）+ 前馈+PID + 感知安全缩放 |

## 运行环境

| 组件 | 版本 |
|---|---|
| 仿真器 | CarlaAir / AirSim（OpenHUTB v2.10.0） |
| 操作系统 | Ubuntu 20.04（虚拟机） |
| ROS | ROS 1 Noetic |
| Python | 3.8+，需要 `numpy` |

## 依赖安装

```bash
pip3 install numpy
```

## 编译

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/OpenHUTB/ros2.git
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

## 运行

```bash
# 终端 1：桥接（必须 body_frame:=false，本模块输出世界系速度）
source ~/catkin_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch publish_image:=false body_frame:=false

# 终端 2：轨迹跟踪
source ~/catkin_ws/devel/setup.bash
roslaunch uav_perception_trajectory main.launch traj:=figure8 radius:=5.0 omega:=0.4 z:=-5.0
```

可调参数：`traj`（line/helix/figure8）、`radius`、`omega`、`z`、`cycles`、
`kp`、`ki`、`kd`、`v_max`、`s_safe`、`s_min`。

### 离线自检（不连 ROS / 仿真器）

用合成传感器数据走一遍完整链路，验证感知与控制逻辑：

```bash
python3 scripts/main.py --dry-run
```

输出示例（正前方 4 m 处合成一片墙）：

```text
扇形直方图（16 扇，1=空旷）：
  0.33 0.34 1.00 1.00 1.00 1.00 1.00 1.00 1.00 1.00 1.00 1.00 1.00 1.00 1.00 0.34
  正前方扇形 [15,0,1] = 0.34 0.33 0.34
  安全系数 s = 0.238（1=全速，0=停止）
```

## 实现要点

* **每扇取最小值而非均值**：均值会被远处空旷点稀释，导致近处障碍被平均掉。
* **剔除地面点**：只统计无人机高度 ±1.5 m 内的点，否则地面会被误判为障碍。
* **导数解析给出**：轨迹导数用解析式而非数值差分，差分的相位滞后会造成持续跟踪偏差。
* **抗积分饱和**：速度饱和时冻结积分，否则误差累积会导致退出饱和后长时间超调。
* **感知修正是乘性缩放**：不是阈值硬切换，保证速度连续无跳变，飞行平滑。
* **启动校验 `body_frame`**：读到 `true` 立即 `logerr`，避免「会飞但方向不对、日志无提示」的隐蔽故障。
* **控制周期须与桥接一致**：`dt` 必须等于桥接 `control/cmd_duration`（0.1 s），否则等效增益改变会破坏稳定性。

## 文件结构

```text
uav_perception_trajectory/
├── CMakeLists.txt
├── package.xml
├── config/controller.yaml        # PID、轨迹、安全阈值
├── launch/main.launch            # 课程约定主入口
└── scripts/
    ├── main.py                   # ROS 节点 + 离线自检
    ├── perception.py             # 点云→直方图；深度图→可通行性（纯函数）
    ├── trajectory.py             # 轨迹生成（纯函数）
    └── controller.py             # 前馈+PID+抗饱和（纯计算类）
```

## 参数整定建议

1. 先用 `traj:=line` 调 `kp` 到无明显超调；
2. 再加 `ki` 消除稳态误差（从 0.02 起，过大易振荡）；
3. 最后加 `kd` 抑制超调（从 0.1 起，过大放大雷达噪声）；
4. 每次改 `dt` 或桥接 `cmd_duration` 后**必须重新整定**。
