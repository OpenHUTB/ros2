# rov_pipeline_follow —— 水下 ROV 海底管道巡检（A→B→A 往返）

一只能下潜的 ROV，靠一台广角前视相机 + 一路逐列深度，沿海底输油管道
**从 A 点巡检到 B 点再原路返回 A**。管道是弯的，水是浑的，海底有海流。

三种能力叠在一条链路上：

- **视觉 SLAM**：着底声应答器当路标，EKF-SLAM 在线估位姿并增量建占据栅格；
- **感知 + 控制**：相机做管道分割，二次曲线拟合出横向偏置与航向偏差，
  级联控制执行"离管道 0.9 m 平行跟管"；
- **端到端学习**：纯 NumPy 卷积网络，输入 3 帧灰度 + 1 帧深度 + 目标方位，
  输出 (v, ω)，与 SLAM 那条链路并排对比。

整个包只依赖 `numpy`（训练与推理都是手写前向/反向），不依赖 PyTorch、
Gazebo、HoloOcean、CARLA。任何装了 ROS 2 Humble 的机器 `colcon build`
之后就能 `ros2 launch`——仿真器就是包里那份 240×160 的解析海底世界。

## 对应作业要求

| 要求 | 实现 |
| --- | --- |
| 物理仿真 + 键盘运动控制 | `sim_node`（三自由度水下运动学 + 推进器一阶滞后 + 海流 + 浑水滑移），订阅 `/rov/cmd_vel`；`teleop.launch.py` 配合 `teleop_twist_keyboard` |
| 传感器感知 + 给定轨迹控制 | 广角相机逐列光线投射出 RGB + 深度，色度/密度三重判据分割管道，纯追踪 + 横向积分跟踪 0.9 m 偏置线 |
| 建图与导航 / SLAM 闭环仿真 | `slam_node` 与 `control_node` 同步运行：同一帧图像既喂 EKF 也喂控制器，栅格图由 SLAM 位姿增量写入 |
| 端到端模型（输入图像、输出指令） | `cnn.py` + `e2e_node`，5 通道输入 → (v, ω) 直接输出，BC + 3 轮 DAgger 训练 |
| 全部程序支持 launch | `main.launch.py` / `teleop.launch.py` / `e2e.launch.py` |
| 多环境运行 | 四种海底场景：直管 / 弯管 / 横流 / 浑水，`scene:=` 参数切换 |

## 目录

```
rov_pipeline_follow/
├── launch/        main / teleop / e2e 三个 launch
├── config/        params.yaml
├── models/        e2e.npz + percep.npz（训练产物，随包安装）
├── results/       指标 CSV、对比图、轨迹图、GIF
├── test/          49 个离线用例 + 6 个 ROS 闭环用例
└── rov_pipeline_follow/
    ├── world.py         海底场景（Catmull-Rom 弯管 + 10 色应答器 + 海流场）
    ├── camera.py        逐列光线投射渲染 RGB + 逐列常数深度（128×96，FOV 140°）
    ├── detector.py      色度最近邻分类应答器 / 管道掩膜 / 列段连续性筛噪
    ├── slam.py          EKF-SLAM（位姿 + 应答器，标准状态增广）
    ├── gridmap.py       对数几率占据栅格 + 管道胶囊真值 + IoU
    ├── planner.py       A\* 兜底（折返后重新捕获管道失败时用）
    ├── controllers.py   级联跟管控制（横向 + 航向 + 积分抗流 + 回转迟滞）
    ├── kinematics.py    三自由度水下运动学 + 推进器滞后 + 海流 + 滑移
    ├── cnn.py           纯 NumPy 卷积网络（im2col，手写反向）
    ├── policies.py      oracle / pipe_slam / pipe_cnn / e2e / e2e_nogoal
    ├── sim.py           离线闭环、指标、数据采集
    └── *_node.py        4 个 ROS 2 节点
```

## 快速开始

```bash
# 在 ROS 2 Humble 工作空间里
colcon build --packages-select rov_pipeline_follow
source install/setup.bash

# 1) 完整流水线：仿真 + SLAM + 跟管控制
ros2 launch rov_pipeline_follow main.launch.py
ros2 launch rov_pipeline_follow main.launch.py scene:=curve seed:=1
ros2 launch rov_pipeline_follow main.launch.py scene:=turbid use_cnn:=true

# 2) 键盘遥控
ros2 launch rov_pipeline_follow teleop.launch.py
# 另一个终端：
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/rov/cmd_vel

# 3) 端到端策略
ros2 launch rov_pipeline_follow e2e.launch.py
```

想可视化就另开 `rviz2`，添加 Map(`/rov/map`)、Path(`/rov/path`)、
PoseArray(`/rov/landmarks`)、Image(`/rov/camera/image_raw`)。

## 话题

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/rov/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/rov/odom` | `nav_msgs/Odometry` |
| 发布 | `/rov/camera/image_raw` | `sensor_msgs/Image` (rgb8) |
| 发布 | `/rov/camera/depth` | `sensor_msgs/Image` (32FC1) |
| 发布 | `/rov/ground_truth` | `geometry_msgs/PoseStamped` |
| 发布 | `/rov/map` | `nav_msgs/OccupancyGrid` |
| 发布 | `/rov/slam_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| 发布 | `/rov/landmarks` | `geometry_msgs/PoseArray` |
| 发布 | `/rov/path` | `nav_msgs/Path` |
| TF | `map → odom → rov_base_link` | — |

`/rov/ground_truth` 只用于评测与画图，导航链路里没有任何地方读它。

## 训练与评测（离线）

```bash
cd src/water/rov_pipeline_follow
python -m rov_pipeline_follow.main                 # 采集→BC→DAgger→评测→出图
python -m rov_pipeline_follow.main --quick         # 少种子少轮次
python -m rov_pipeline_follow.main --stage eval    # 只评测
python -m pytest test/ -q                          # 无 rclpy：55 passed；装了 rclpy：49 passed, 6 skipped
```

`test/test_nodes_ros.py` 的 6 个用例用 `fake_ros` 替身，检测到真实 `rclpy` 会自动
skip，所以在 Ubuntu + Humble 上看到 `49 passed, 6 skipped` 是正常的；这 6 个用例
对应的端到端行为用下面的 `ros2 launch` 验证。

训练数据只来自清水场景（`straight` / `curve`），浑水（`turbid`）与横流
（`current`）**只做测试**——检验的是泛化，不是背布局。

产物：`results/compare_summary.csv`、`compare.png`、`training_curves.png`、
`trajectories.png`、`curve.gif`、`turbid.gif`，权重写进
`models/e2e.npz` 与 `models/percep.npz`。

## 实现要点

**为什么是"离管道 0.9 m 平行飞"而不是"压着管道飞"**：这台 ROV 只有前视
相机、没有下视相机。贴着管道正上方时，管道在图像里只剩最后一小段、而且
很快就出画；偏置 0.9 m 之后整条管道在视野里斜着拉开，拟合出来的横向偏置
才有分辨力。代价是横向误差必须靠控制回路而不是视觉直接闭环，于是有了下面
那条积分项。

**横向积分项是抗流的唯一手段**：横流 0.18 m/s 对阵 0.55 m/s 的推力上限，
稳态偏差会稳定停在 0.3 m 左右，纯比例控制消不掉——这不是调参能解决的，
因为真正的偏置来源（流速）不在误差信号里。加一路 `KI_LAT` 积分后稳态偏差
被压到 0.1 m 以内，代价是折返瞬间必须清积分，否则积分项会推着 ROV 撞管。

**任务系与车体系的符号必须显式转换**：往返任务里"管道在左边"这句话在去程
和返程指的是两个相反的横向方向。第一版直接把 `lat`（管道在车体坐标系下的
横坐标）当误差用，去程 12/12、返程 0/12 —— ROV 一路把管道往右推，最后
压进管壁。修正成 `dx_err = mdir·(FOLLOW_OFFSET − lat)`（`mdir` 是当前巡检
方向）之后才是 12/12。这个 bug 只在"往返"任务里出现，单向跟踪永远碰不到。

**折返是一次显式机动，不是"误差反号自然转过去"**：管道到端点后视野里只剩
端头那一小段，拟合出的航向偏差是垃圾值，误差反号后控制器会让 ROV 原地打转
甚至蹭到管口。做法是在相位翻转的瞬间调 `begin_turnaround(dir)`：按"管道在
鼻子哪一侧"决定往哪边转，一直转到航向误差小于 0.45 rad 才交回正常控制。

**几何检测在浑水里会退化，但不会崩**：雾化把管道颜色按 0.55 的比例混进水色，
色度判据给出的是断断续续的列段，横向误差从 0.11 m 涨到 2.0 m、成功率 3/3 → 2/3。
我们试过用 CNN 直接回归 (dx_err, dtheta) 替代它（`pipe_cnn`），两版都失败：
纯专家数据上均值坍缩成恒输出 0（专家横向误差本来就≈0），加了起始扰动后能回归，
但逐帧抖动 ±0.3 m 会被回转判据放大成"转一下停一下"，闭环 0/12。所以主对比保留
几何检测，CNN 感知作为对照留在代码里——完整归因见文档 §7.3。

**检测端最难的不是分类而是拒识**：应答器是 10 种颜色编码的立柱，管道是
橙色。第一版用"橙通道占优"判管道，结果红色应答器整根被当成管道，ROV 从
起点就开始打转。改成归一化色度空间的距离判据（`‖c − c_pipe‖ < 0.13`）才
分开。另一半是散点噪声：49 个像素的随机斑块曾给出 11.6 m 外的假观测，
把 EKF 一把拉到 (16.4, 20.2)。三条硬判据——严格连续列段、每列 ≥2 像素、
距离 ≤8 m ——把这类观测全拒掉了。

**端到端为什么必须喂目标方位**：和地面那次一样，同一幅"管道在右下角"的
画面，去程意味着"左转跟上"，返程意味着"右转跟上"，图像本身不可辨识。
补一路由里程计算出的目标方位通道（不需要地图、不需要 SLAM）之后端到端才
能跑起来；`e2e_nogoal` 用同一份权重把这一路置零做消融。

## 结果

四场景 × 三种子，四种策略对比（完整表格、逐条归因见 `docs/water/rov_pipeline_follow.md`）：

| 策略 | 直管 | S 弯 | 横流 | 浑水 | 合计 |
| --- | --- | --- | --- | --- | --- |
| Oracle（真值） | 3/3 | 3/3 | 3/3 | 3/3 | **12/12** |
| 几何感知 + EKF-SLAM | 3/3 | 3/3 | 1/3 | 2/3 | **9/12** |
| 端到端 CNN | 3/3 | 3/3 | 3/3 | 0/3 | **9/12** |
| 端到端（目标通道置零） | 3/3 | 3/3 | 3/3 | 0/3 | **9/12** |

清水里经典流水线横向误差 0.11 m（直管）/ 0.27 m（S 弯）、ATE 0.29 / 0.50 m，
往返 34 m 之后直管回环闭合误差只有 0.04 m。两类策略拿到同样的分数，但失败维度
完全不同：流水线崩在横流（SLAM 定位），端到端崩在浑水（没见过的图像分布）。
另有 pipe_cnn（CNN 回归替代几何检测）作为负面结果记录在文档 §7.3。

"目标通道置零"那一行与不置零同为 9/12、逐场景方向不一致，**不能当作消融有效的
证据**——它说明的是端到端策略并不依赖该通道。

## 已知局限

- 应答器靠颜色编码做数据关联，真实海底换成 USBL 应答器或声学标签即可，
  但需要重新标定检测门限；
- 海流只建了横向常值 + 慢摆动的简化模型，没有涡流与垂向流；
- 浑水场景的雾化是全局线性混合，没有做前向散射的体积散射建模，
  因此 `pipe_cnn` 的表现偏乐观；
- 端到端策略依赖里程计给出的目标方位，长距离往返下协变量偏移仍明显，
  具体数字与归因见文档。
