# 基于三次样条插值的双足机器人多步态平滑切换仿真

本项目在 ROS 2 Humble 中实现一个平面双足机器人（躯干 + 双腿，每腿髋/膝/踝 3 个关节）的多步态仿真：
站立、行走、快步、跑步四种步态按时间表切换，切换瞬间用**三次 Hermite 样条**对 6 个关节角做位置与速度连续（C1）的过渡；
摆动脚轨迹由三次多项式（smoothstep 与 `16u²(1-u)²` 抬脚曲线）生成；关节由带速度前馈的 PD 伺服驱动。
控制只使用自身关节反馈，ZMP/质心用于事后评估。

## 与已有双足教程的区别

仓库已有《双足步态生成与 ZMP 稳定性》侧重 LIPM 与 ZMP 基础。本项目聚焦**多步态平滑切换**：

| 项目 | 已有教程 | 本项目 |
| --- | --- | --- |
| 运行环境 | ROS 1 / 纯 Python | ROS 2 Humble |
| 步态数量 | 单一行走步态 | 站立 / 行走 / 快步 / 跑步 |
| 切换方式 | 无 | 三次 Hermite 样条 + 直接切换对照 |
| 平滑性证据 | 无 | 切换时刻 qddes 左右极限差 + 相邻采样差对比 |
| 可视化 | 曲线 | 关节轨迹、ZMP、踏地轨迹、GIF |

## 环境

- Ubuntu 22.04，ROS 2 Humble，Python 3.10
- 依赖：`numpy`、`pyyaml`、`matplotlib`、`Pillow`（`python3-pil`）
- 无 Gazebo/MuJoCo 依赖，纯 Python 运动学 + 关节伺服仿真，可在无显示器环境运行

## 目录

```
biped_cubic_spline_gait_switch/
├── biped_cubic_spline_gait_switch/
│   ├── spline.py           # 三次 Hermite 插值与解析导数
│   ├── kinematics.py       # 双足 IK/FK、质心、支撑多边形
│   ├── gait.py             # 步态定义、调度、平滑切换
│   ├── controller_node.py  # 200 Hz 轨迹生成
│   ├── sim_node.py         # 500 Hz 关节伺服 + 质心/ZMP
│   └── logger_node.py      # 100 Hz 等间隔网格 CSV 记录
├── config/params.yaml      # 步态参数与调度
├── launch/main.launch.py
├── tools/analyze.py        # 指标、图表、GIF
├── test/                   # pytest
├── results/                # 本机运行输出（.gitignore，不入库）
├── main.py                 # 一键实验
└── README.md
```

## 编译

```bash
mkdir -p ~/biped_ws/src
ln -s /path/to/biped_cubic_spline_gait_switch ~/biped_ws/src/
cd ~/biped_ws
colcon build --packages-select biped_cubic_spline_gait_switch --symlink-install
source install/setup.bash
```

## 运行

```bash
cd ~/biped_ws/src/biped_cubic_spline_gait_switch
python3 main.py            # 依次运行 spline 与 direct 两组实验，并生成图表
```

也可单跑一组：

```bash
ros2 launch biped_cubic_spline_gait_switch main.launch.py \
  mode:=spline output_dir:=$PWD/results/run_spline duration:=34.0
```

`mode:=spline` 使用三次样条过渡，`mode:=direct` 在切换时刻直接跳变，作为平滑性对照。

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/joint_cmd` | `sensor_msgs/JointState` | 期望关节角与角速度（200 Hz） |
| `/joint_states` | `sensor_msgs/JointState` | 伺服后的实际关节角 |
| `/base_cmd` | `std_msgs/Float32MultiArray` | `[base_x, v, hip_z, lean]` |
| `/base_state` | `nav_msgs/Odometry` | 浮动基座状态 |
| `/com`、`/zmp` | `geometry_msgs/PointStamped` | 质心与 ZMP |
| `/biped_metrics` | `std_msgs/Float32MultiArray` | 质心、ZMP、裕度、双脚位置与接触 |
| `/gait_info` | `std_msgs/String` | 步态名、相位、切换标志 |

## 算法

### 步态参数

| 步态 | 周期 (s) | 支撑相占比 | 步长 (m) | 抬脚 (m) | 髋高 (m) |
| --- | --- | --- | --- | --- | --- |
| stand | 1.00 | 0.50 | 0.00 | 0.00 | 0.84 |
| walk | 1.20 | 0.62 | 0.22 | 0.10 | 0.82 |
| fast_walk | 0.90 | 0.58 | 0.30 | 0.13 | 0.80 |
| run | 0.60 | 0.38 | 0.42 | 0.18 | 0.78 |

支撑相占比 > 0.5 时存在双支撑（walk/fast_walk），< 0.5 时存在腾空（run）。

### 三次 Hermite 样条切换

切换时刻 `t0` 记录当前关节角 `q0` 与角速度 `dq0`，并预测过渡结束 `t1 = t0 + 0.6 s` 时目标步态的角度 `q1` 与角速度 `dq1`。过渡期内每个关节按下式插值：

```
q(t) = h00(u)·q0 + h10(u)·T·dq0 + h01(u)·q1 + h11(u)·T·dq1
u = (t - t0) / T
```

其中 `h00/h10/h01/h11` 为三次 Hermite 基函数，因而位置与速度连续（C1）；控制器还使用该式的**解析导数**作为 `/joint_cmd` 的速度前馈，切换首拍速度不会掉成 0。摆动脚在 `u∈[0,1]` 内用 `3u²-2u³` 平滑前进，抬脚高度用 `16u²(1-u)²`，两端高度与竖直速度均为 0。

> 项目名中的“三次样条”指上述分段三次 Hermite 插值。自然三次样条（natural cubic spline，C2 连续、端点二阶导为零）在本项目主流程中并未使用，`spline.py` 只保留 Hermite 实现。

### 评价指标

- 关节跟踪 RMS：`q_des - q_actual`
- 切换时刻 `qddes` 左右极限差（主结论）：取最后一个切换前样本与第一个切换后样本的差；相邻采样差会随网格变细而缩小，只作为辅助指标
- 切换窗口（`t0 ~ t0+0.6 s`）最大 jerk / 角加速度：在日志频率的均匀网格上重采样，再对**命令侧位置 `qdes`** 差分；这两个值依赖网格，另附 100/200/400 Hz 敏感性表
- 接触状态由控制器按规划相位给出（不再用踝高阈值），腾空比例与步数按规划接触统计
- ZMP 裕度：ZMP 到支撑多边形边界的最小距离；支撑多边形由期望关节角 `qdes` 的 FK 得到，是规划几何量
- 前进距离与平均速度：全程平均 = 距离 / 时长；行进段平均只统计 `base_v > 0.01` 的样本

## 实验结果

以下为本机实际运行数据（34 s 调度，100 Hz 记录，共 3401 条严格等间隔采样；对照组同一参数下运行直接切换）：

| 指标 | 三次 Hermite 切换 | 直接切换 |
| --- | --- | --- |
| 前进距离 / 平均速度（全程） | 16.800 m / 0.494 m/s | 同调度 |
| 平均速度（行进段） | 0.686 m/s | 同调度 |
| 关节跟踪 RMS | 0.0097 rad | — |
| **切换时刻 qddes 左右极限差（主结论）** | **0.530 rad/s（100 Hz 采样表观值；连续极限为 0）** | 157.375 rad/s |
| 切换窗口最大相邻采样命令速度差（qddes） | 0.550 rad/s（21.126 s） | 149.811 rad/s（21.011 s） |
| 切换窗口最大相邻采样速度差（qdes） | 1.658 rad/s | 30.622 rad/s |
| 切换窗口最大 jerk（100 Hz 差分，分辨率敏感） | 14140.4 rad/s³ | 161009.7 rad/s³ |
| 切换窗口最大角加速度（100 Hz 差分，分辨率敏感） | 107.91 rad/s² | 1553.87 rad/s² |
| ZMP 最小裕度（规划几何量） | -0.2002 m | — |
| 腾空比例 / 步数（规划相位） | 0.045 / 50 | — |

分步态统计：

| 步态 | 采样数 | ZMP 最小裕度 (m) | 跟踪 RMS (rad) |
| --- | --- | --- | --- |
| stand | 1001 | 0.1477 | 0.0004 |
| walk | 1200 | -0.1484 | 0.0075 |
| fast_walk | 600 | -0.1442 | 0.0113 |
| run | 600 | -0.2002 | 0.0168 |

切换发生在 3 / 9 / 15 / 21 / 27 s。pytest 对 **5 次切换逐次断言**切换首拍速度与上一拍相同（使用规划器解析导数）；CSV 中 0.530 rad/s 的最大表观左右差出现在 21.006 s（跑步→行走），是因为 100 Hz 记录点在切换后 6 ms，过渡已经开始演化，并非真实跳变。最大相邻采样 `qddes` 差 0.550 rad/s 出现在 21.126 s，属于 21.0 s 的切换窗口（跑步→行走）；直接切换同窗口为 149.811 rad/s。

jerk/角加速度是 100 Hz 网格上的差分量，会随网格变化：Hermite 只保证 C1，过渡起止点的加速度仍有跳变，因此 jerk 同样不收敛。下表把同一份 100 Hz 记录插值到更细网格，说明分辨率敏感性；主结论采用切换时刻 `qddes` 左右极限差，而不是 jerk 降幅。

| 分析网格 (Hz) | Hermite jerk | 直接 jerk | Hermite 角加速度 | 直接角加速度 |
| --- | --- | --- | --- | --- |
| 100 | 14140.4 | 161009.7 | 107.91 | 1553.87 |
| 200 | 78371.0 | 604825.0 | 526.54 | 3062.16 |
| 400 | 210616.0 | 2434512.0 | 970.32 | 6086.28 |

规划接触统计下 walk / fast_walk 的腾空比例为 0，run 段为 0.255（理论 0.24），整体 0.045；步数 50（按规划接触上升沿统计并排除切换边界）。

ZMP 裕度为负是因为本仿真采用开环轨迹跟踪，没有加入平衡控制器；该指标由期望关节角的 FK 与规划接触相位得到，只用于衡量规划轨迹的几何关系，不代表真实接触力。

图表见 `docs/img/biped_cubic_spline_gait_switch_*`（由 `results/run_spline` 生成）：

- `*_joints.png`：期望/实际关节角
- `*_velocity.png`：关节角速度
- `*_jerk.png`：加加速度
- `*_zmp.png`：质心、ZMP 与裕度
- `*_feet.png`：双脚轨迹
- `*_gait_timeline.png`：步态时间线
- `*_comparison.png`：三次样条 vs 直接切换
- `*_walking.gif`：行走动画

## 测试

```bash
cd src/humanoid/biped_cubic_spline_gait_switch
python3 -m pytest -q test
```

覆盖三次 Hermite 插值、解析导数与端点速度、IK/FK 往返、脚掌水平、支撑多边形、双支撑/腾空比例，以及完整 34 s 调度下全部 5 次切换的逐窗口速度连续性检查（使用规划器输出 `dq`，不是相邻角度差）。

## 复现说明

`results/run_spline` 与 `results/run_direct` 由 `main.py` 在本机实际运行产生（34 s、3401 条 100 Hz 采样，时间戳严格等间隔），再由 `tools/analyze.py` 生成指标与图表。运行输出已通过 `.gitignore` 排除，不入库；文档所需图表保存在 `docs/img/`。数据未手工修改。

## 大模型使用声明

代码与文档在 AI 编程助手辅助下完成，作者已逐行核对、运行测试并实际采集仿真数据。
