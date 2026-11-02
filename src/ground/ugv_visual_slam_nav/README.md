# ugv_visual_slam_nav —— 地面无人车视觉 SLAM 建图与自主导航

差速底盘 + 单目 RGB-D。用视觉路标做 **EKF-SLAM**，用视觉深度做 **占据栅格**，
在栅格上跑 **A\*** 规划、**纯追踪**执行，另外并排训练一个 **端到端 CNN**
（图像 → 速度指令）做对照。

整个包只依赖 `numpy`（训练与推理都是手写前向/反向），不依赖 PyTorch、
Gazebo、CARLA。任何装了 ROS 2 的机器 `colcon build` 之后就能 `ros2 launch`。

## 对应作业要求

| 要求 | 实现 |
| --- | --- |
| 物理仿真 + 键盘运动控制 | `sim_node`（差速运动学 + 打滑噪声）订阅 `/ugv/cmd_vel`，`teleop.launch.py` 配合 `teleop_twist_keyboard` |
| 传感器感知 + 给定轨迹控制 | RGB-D → 颜色标签检测路标/墙体；A\* 规划出路径后纯追踪跟踪 |
| 建图与导航 / SLAM 同步仿真 | `slam_node` 在线估计位姿并增量建图，`nav_node` 用同一张栅格规划，二者同步 |
| 端到端模型（输入图像、输出指令） | `cnn.py` + `e2e_node`，输入 3 帧灰度 + 1 帧深度 + 1 帧目标方位，输出 (v, ω) |
| 全部程序支持 launch | `main.launch.py` / `teleop.launch.py` / `e2e.launch.py` |

## 目录

```
ugv_visual_slam_nav/
├── launch/        main / teleop / e2e 三个 launch
├── config/        params.yaml
├── models/        e2e.npz（训练产物，随包安装）
├── results/       指标 CSV、对比图、GIF
├── test/          离线单元测试 + ROS 替身闭环测试
└── ugv_visual_slam_nav/
    ├── world.py         场景（三房间 S 形走廊 + 12 个彩色路标 + 动态障碍）
    ├── camera.py        解析光线投射渲染 RGB-D（160×120，FOV 90°）
    ├── detector.py      饱和度/色度/列密度三重判据的路标检测
    ├── slam.py          EKF-SLAM（位姿 + 路标，标准增广式）
    ├── gridmap.py       对数几率占据栅格 + 真值 IoU
    ├── planner.py       A\* + 视线简化 + 离障碍距离代价
    ├── controllers.py   纯追踪 + 终点收敛 + 深度局部避障
    ├── cnn.py           纯 NumPy 卷积网络（im2col，手写反向）
    ├── policies.py      oracle / slam_nav / e2e 三种策略
    ├── sim.py           离线闭环、评测、数据采集
    └── *_node.py        4 个 ROS 节点
```

## 快速开始

```bash
# 在 ROS 2 Humble 工作空间里
colcon build --packages-select ugv_visual_slam_nav
source install/setup.bash

# 1) 完整流水线：仿真 + SLAM + 导航
ros2 launch ugv_visual_slam_nav main.launch.py
ros2 launch ugv_visual_slam_nav main.launch.py scene:=cluttered seed:=1

# 2) 键盘遥控
ros2 launch ugv_visual_slam_nav teleop.launch.py
# 另一个终端：
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ugv/cmd_vel

# 3) 端到端策略
ros2 launch ugv_visual_slam_nav e2e.launch.py
```

想可视化就另开 `rviz2`，添加 Map(`/ugv/map`)、Path(`/ugv/path`)、
PoseArray(`/ugv/landmarks`)、Image(`/ugv/camera/image_raw`)。

## 话题

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/ugv/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/ugv/odom` | `nav_msgs/Odometry` |
| 发布 | `/ugv/camera/image_raw` | `sensor_msgs/Image` (rgb8) |
| 发布 | `/ugv/camera/depth` | `sensor_msgs/Image` (32FC1) |
| 发布 | `/ugv/ground_truth` | `geometry_msgs/PoseStamped` |
| 发布 | `/ugv/map` | `nav_msgs/OccupancyGrid` |
| 发布 | `/ugv/slam_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| 发布 | `/ugv/landmarks` | `geometry_msgs/PoseArray` |
| 发布 | `/ugv/path` | `nav_msgs/Path` |
| TF | `map → odom → base_link` | — |

`/ugv/ground_truth` 只用于评测与画图，导航链路里没有任何地方读它。

## 训练与评测（离线）

```bash
cd src/ground/ugv_visual_slam_nav
python -m ugv_visual_slam_nav.main                 # 采集→BC→DAgger→评测→出图
python -m ugv_visual_slam_nav.main --quick         # 少种子少轮次
python -m ugv_visual_slam_nav.main --stage eval    # 只评测
python -m pytest test/ -q                          # 45 个用例
```

产物：`results/compare_summary.csv`、`compare.png`、`training_curves.png`、
`trajectories.png`、`corridor.gif`、`dynamic.gif`，权重写进 `models/e2e.npz`。

## 实现要点

**为什么单目标签用颜色而不是形状**：数据关联需要唯一 ID。12 根柱子用调色板里
两两可分的颜色编码，检测端做一次归一化色度最近邻即可拿到 ID，等价于一个视觉
标签系统（AprilTag 的简化版）。

**检测的鲁棒性靠三重判据**：饱和度门限（排除灰墙/地板）、通道极差门限
（排除暗光噪声）、每列像素密度（排除散点）。第三条最关键 —— 真实的竖直柱子在
每一列上都是连续的一整段像素，而噪声散点每列只有一两个。没有这条判据，夜间
每帧会多出三个假路标，EKF 直接被污染。

**EKF-SLAM 只提供相对约束**。初始位姿来自先验（起点已知），路标首次观测时按
标准公式做状态增广、保留与位姿的相关性。如果初始位姿本身是错的、路标又从它
初始化，整张地图会自洽地平移过去而不会自动收敛回真值 —— 这是规范自由度，不是
bug，`test_offline.py` 里专门写了注释说明。

**规划贴边 + 跟踪切角 = 擦碰**。A\* 的最短路径天然贴着膨胀边界走，纯追踪在转弯
处又会向内切，两者叠加就会蹭到路标。解决方式是给 A\* 加一层"离障碍距离"代价，
把路径推向通道中线，并且视线简化时用更保守的掩码，避免拉直的线段贴回边缘。

**卡住必须能自己出来**。跟踪误差会把车带进膨胀区，此时 A\* 从原地无解。规划器
会在 8 格内就近找可通行格当起点，策略层再兜一层"连续 15 步几乎没位移就
原地转向 + 查图后退"的恢复动作，到点前还有一次"自转复核"的到点确认。

**端到端为什么必须喂目标方位**。第一版端到端只吃 3 帧灰度，闭环成功率 0/12：
三房间 S 形走廊里同一幅画面既可能对应"直行"也可能对应"左转"，正确动作取决于
目标在哪儿，而目标根本不在图像里 —— 这不是训练不足，是**不可辨识**，行为克隆
只能学到几种情形的均值然后撞墙。补一路 HUD 式的目标方位通道（由里程计位姿与
给定目标算出，不需要地图、不需要 SLAM）之后闭环成功率才起来；
`e2e_nogoal` 用同一份权重把这一路置零做消融，用来证明成败确实来自目标信息。

## 结果

四场景 × 三种子，四种策略对比：oracle **12/12**、slam_nav **10/12**
（ATE 均值 0.22 m、建图 IoU 0.30、探索覆盖 0.92）、e2e **3/12**、
e2e_nogoal **0/12**（平均 8.7 s / 3.1 m 就撞墙）。完整表格、失败归因与
端到端讨论见 `docs/ground/ugv_visual_slam_nav.md`。

## 已知局限

- 数据关联依赖颜色编码，真实场景换成 AprilTag / 描述子即可，但需要重新标定
  检测门限；
- 动态障碍只在"局部反应式避障"这一层处理，没有做动态地图的擦除，长时间停留
  会在栅格里留下"鬼影"；
- 端到端策略依赖里程计给出的目标方位，里程计漂移（本仿真里 70 s 约 0.5 m /
  3°）会直接变成方位误差；长距离导航下协变量偏移仍然明显，成功率低于流水线，
  具体数字与归因见文档。
