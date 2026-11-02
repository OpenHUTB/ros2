# lidar_mcl_localization

ROS 1（Noetic）+ Gazebo 的**二维激光雷达蒙特卡洛定位（MCL / 粒子滤波）**原理实现。

课程第 6 章用 `amcl` 这个"黑盒"做定位；本模块把盒子打开，用不到 400 行纯 NumPy
把 MCL 的每一步显式写出来——运动模型采样、似然场观测模型、低方差重采样——
再回到 Gazebo 里和真实位姿对比，最后用一个**神经网络**学掉轮式里程计的运动残差。

## 与仓库已有模块的区别

| 模块 | 做法 | 定位方式 |
|---|---|---|
| `chap6/.../amcl.launch`（教材） | 调用现成 `amcl` 包 | 黑盒，参数在 yaml 里 |
| `ground/turtlebot3_lidar_avoid` | 用激光做避障 | **不做定位** |
| `ground/lateral_control_benchmark` | 横向控制对比 | 假设位姿已知 |
| **本模块** | **自己实现粒子滤波** | 似然场 + 运动模型，逐行可读 |

本模块不依赖 `amcl` / `map_server` / `gmapping` / `move_base`，
只用到 `gazebo_ros`、`gazebo_plugins`、`nav_msgs`、`sensor_msgs` 这些 Noetic 自带包。

## 目录结构

```
├── urdf/mcl_bot.urdf          差速机器人（两驱动轮 + 两个万向球 + 2D 激光）
├── worlds/mcl_room.world      Gazebo 场景（自动生成，纯内联几何，不需要联网下载模型）
├── maps/mcl_room.pgm/.yaml    占据栅格地图（由同一份墙体定义生成）
├── config/scene_layout.py     **唯一真实来源**：墙体几何 + 出生点 + 雷达参数
├── src/lidar_mcl_localization/
│   ├── geometry.py            角度归一化、位姿复合、圆周平均
│   ├── map_model.py           地图加载、射线投射、似然场（BFS 距离变换）
│   ├── motion_model.py        里程计增量模型（α₁~α₄ 采样）
│   ├── sensor_model.py        似然场观测模型 + 粗配准
│   ├── particle_filter.py     预测/加权/低方差重采样/Neff
│   └── nn_motion.py           MLP（4→24→24→3，纯 NumPy 手写反传）
├── scripts/mcl_node.py        定位节点（订 /scan /odom，发 /mcl_pose /particles /tf）
├── scripts/teleop.py          键盘遥控（termios）
├── scripts/goal_driver.py     用 MCL 位姿闭环走到目标点
└── tools/
    ├── gen_scene.py           由 scene_layout.py 生成 world + 地图（保证两者一致）
    ├── offline_demo.py        不开 ROS 的闭环自检
    └── run_experiment.py      采集数据 → 训练网络 → 四组对比 → 出图
```

## 运行环境

| 组件 | 版本（实测） |
|---|---|
| 系统 | Ubuntu 20.04（VMware，无 GPU） |
| ROS | Noetic |
| Python / numpy | 3.8.10 / 1.17.4 |
| Gazebo | 11.15.1 |
| 其它 | matplotlib（出图）、Pillow（可选） |

不需要 `pip install`：算法只用 NumPy，仿真用 Gazebo，可视化用 rviz / matplotlib。

## 编译与运行

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
ln -s <仓库路径>/src/ground/lidar_mcl_localization .
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

```bash
# ① 一条命令：Gazebo 场景 + 机器人 + MCL 定位节点（无头）
roslaunch lidar_mcl_localization main.launch

# ② 带界面（Gazebo + rviz）
roslaunch lidar_mcl_localization main.launch gui:=true rviz:=true

# ③ 键盘遥控（另开终端）
rosrun lidar_mcl_localization teleop.py

# ④ 用定位结果闭环走到目标点
rosrun lidar_mcl_localization goal_driver.py _goal_x:=2.0 _goal_y:=2.5

# ⑤ 不开 ROS 也能跑的三件事
python3 tools/gen_scene.py          # 重新生成场景与地图
python3 tools/offline_demo.py       # 闭环自检：MCL vs 纯里程计
python3 tools/run_experiment.py     # 训练网络 + 四组对比 + 出图
```

## 关键话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 360 束、8 m、1 cm 噪声 |
| `/odom` | `nav_msgs/Odometry` | **编码器积分**得到的里程计（会漂移） |
| `/mcl_pose` | `PoseWithCovarianceStamped` | map 系下的定位结果（含协方差） |
| `/particles` | `geometry_msgs/PoseArray` | 粒子云，rviz 里直接看 |
| `/mcl_diagnostics` | `Float32MultiArray` | `[Neff, 位置散布, 航向散布, 位置误差, 航向误差]` |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | 真值，**只用于算误差，不参与估计** |

## 实测结果

离线闭环（`tools/run_experiment.py`，同一张地图、同一套算法）：

| 方案 | 位置误差 均值 / 最大 | 航向误差 均值 / 最大 |
|---|---|---|
| ① 纯里程计 | 0.345 m / 0.662 m | 12.52° / 26.73° |
| ② 里程计 + 神经网络 | **0.195 m** / 0.348 m | **6.17°** / 10.89° |
| ③ MCL（不用网络） | **0.020 m** / 0.101 m | **0.19°** / 1.06° |
| ④ MCL + 神经网络 | 0.021 m / 0.099 m | 0.23° / 1.19° |

Gazebo 实跑（Ubuntu 20.04 无头）：`/mcl_pose` 稳定 10.0 Hz，
出生点初始误差 0.072 m，直行 + 转向 6 s 后降到 **0.020 m**。

**两个诚实结论**：

1. 神经网络把**纯里程计**的误差降了 43%（0.345 → 0.195 m），说明它确实学到了滑移规律；
2. 但在 MCL 里加上它几乎没差别（0.020 → 0.021 m）——因为观测更新本身就在吸收
   里程计的系统误差，网络的价值被掩盖了。把观测降到 2 Hz 时网络能再改善约 19%
   （0.0221 → 0.0179 m），但趋势不稳定，不宜过度解读。

## 已知限制

1. **定位精度受地图分辨率限制**：地图是 5 cm/像素，2 cm 的位置误差已经接近下限，
   想再提升要先提高地图分辨率，而不是调滤波器参数。
2. **只在仿真里验证**：真值来自 `/gazebo/model_states`，换真车需要人工测点或扫描匹配。
3. **地图与场景由同一份定义生成**：如果场景被手工改动而地图没重新生成，
   定位会稳定收敛到错误位置——这正是 `tools/gen_scene.py` 存在的理由。
4. **未做全局定位的鲁棒性测试**：`global_localization:=true` 可以在全图撒点，
   但在对称走廊里可能收敛到镜像位置，本模块没有系统评估这一点。
5. **神经网络只修正运动模型**，不参与观测模型；网络的输入是轮速／角速度，
   换到真车需要重新采集数据训练。
