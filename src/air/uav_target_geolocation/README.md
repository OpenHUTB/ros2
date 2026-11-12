# uav_target_geolocation —— 目标地理定位与多站交会

无人机悬停在 80 m 高度，用云台视线反算地面目标的位置。这件事的难点不在
"交个点"，而在**误差怎么长**：视线角偏 1°，地面交点就偏 2.18 m——高度 80 m、
水平距离 60 m 时的敏感度是 `h / cos²θ`，越掠射放大得越狠。本模块把三个问题
拆开看：

1. **单站**（`core/geometry.py`）：一条视线与地面求交，误差 ≈ 敏感度 × 角误差；
2. **多站交会**（`core/fusion.py`）：几条视线在地面平面上做加权最小二乘，
   把各自的零位偏置摊薄——前提是站与站**张开得够开**；
3. **两种模式的对照**（`core/analysis.py` + `core/flyer.py`）：解析打点只用
   几何 + 噪声，MuJoCo 闭环里的位置与姿态来自刚体积分，两套数共用同一套
   统计口径，可以直接对照。

## 快速开始

```bash
# 依赖：Python 3.10+，numpy / mujoco / matplotlib / pytest
cd src/air/uav_target_geolocation
python main.py                 # 三个算例：解析 600 次打点 + 闭环各飞 20 个架次
python main.py --no-sim        # 只跑解析层，秒级
python main.py --case two_station_shallow
python main.py --seed 7 --reps 20   # 指定随机流与架次数（默认 seed=3、reps=20）
python -m pytest test/ -q      # 70 项测试（1 项需要 rclpy，无 ROS 2 时自动跳过）
python tools/make_figures.py   # 重新生成文档配图
```

ROS 2 环境下可 colcon 构建：

```bash
colcon build --packages-select uav_target_geolocation
source install/setup.bash
ros2 run uav_target_geolocation main       # 离线解算
ros2 run uav_target_geolocation geo_node   # 在线判定节点
```

## 算例与结果

三个算例的目标都是地面同一点（原点），差别只在观察站布局：

| 算例 | 站 | 开角 | 解析单站 | 解析交会 | 实测单站 | 实测交会 |
| --- | --- | --- | --- | --- | --- | --- |
| `single_station` | 1 | — | 0.46 m | — | 0.56 m | 0.56 m |
| `two_station_orthogonal` | 2 | 90° | 0.46 m | **0.36 m** | 0.56 m | **0.31 m** |
| `two_station_shallow` | 2 | 12° | 0.46 m | **1.61 m** | 0.56 m | **1.49 m** |

（解析列 = 蒙特卡洛 600 次；实测列 = MuJoCo 闭环 20 个架次的均值。
视轴抖动 σ = 0.30°，单站零位偏置 σ = 0.20°，悬停时叠加 1 N 水平外力。）

两条结论：

- **交会有用，但只在几何构型好时有用**。双站正交要同列比：实测 0.56 →
  0.31 m（−44%），解析 0.46 → 0.36 m（−22%），理想 1/√2 是 −29%。实测比
  理想还好，原因在分母——单站参照取 0 号站，它的零位偏置每架次重抽一次
  （20 个架次模长均值 0.30°、最大 0.72°），抽大的架次抬高了分母；逐架次
  比值与该偏置模长的相关系数 −0.48。开角缩到 12°，交会反而把误差放大到
  1.61 m——比单站还差 3.5 倍，法方程条件数 1.0 → 90，交会点沿"线方向"跑掉。
- **开角本身是个硬门槛**。要让交会真正占优，两站开角得开到 **约 45°**，
  而且这个门槛几乎不随单站偏置移动（bias 0.05° → 44.8°，0.20° → 46.4°）。
  12° 不是"运气不好挑了个坏角"，而是绝大多数两站夹角都还不够开。
- **"拿指令姿态当真实姿态"是最贵的错误**。悬停时为了抵消 1 N 水平外力，
  机体俯仰 6.1°；如果把指令姿态（水平）当真实姿态用，误差 14.7 m——
  是正确做法（0.46 m）的 32 倍。这也是为什么云台测量必须和 IMU 姿态做时间同步。

## 误差从哪来

| 误差源 | 量级 | 多站交会能救吗 |
| --- | --- | --- |
| 视轴逐帧抖动 σ = 0.30° | ~0.65 m | 能，120 帧平均后剩 0.03° |
| 单站零位偏置 σ = 0.20° | ~0.44 m | **这是交会的主要收益来源**，两站互相约束 |
| 机体倾角（1 N 外力） | 6.1° | 用真实姿态即可消除；丢弃则 14.7 m |
| 悬停位置偏离标称点 | < 0.08 m | 位置项影响不大 |

## 与仓库已有模块的关系

- `uav_mujoco_sim`：物理底座，复用同一份 `quadrotor.xml` 与 `GeometricController`；
- `uav_apriltag_landing` / `uav_gimbal_reacquisition`：解决"看到目标"，本模块
  解决"看到之后它在地面的哪个坐标"；
- `uav_moving_platform_landing` / `uav_drop_on_moving_target`：拿到目标位置
  之后才有的下游问题，本模块给它们提供带误差量的输入。

## 已知局限

- 视线模型是"云台已对准目标 + 小角度噪声"，没有做完整的相机投影（像素 →
  视线），像素量化噪声与标定残差合并计入了 σ 和偏置；
- 交会只在 z = 0 平面上做，未利用视线的俯仰分量做三维交会；
- ROS 节点是简化版：站点切换靠 `/target/geo/new_station` 手动触发，没有
  自动按"无人机移动了多远"来分站；节点对视线只做单站打点与逐站累积，
  不含目标运动模型；节点已做时间同步与坐标系检查（`max_stamp_skew_s` /
  `odom_frame_id` / `los_frame_id`），里程计与视线时间戳偏差过大或
  frame_id 不匹配时拒收该帧；
- 本机为 Windows + Python 3.13、没有 ROS 2，无法本地实跑 launch；CI 里
  有专门的 ROS 2 Humble job（`ci_uav_target_geolocation.yml` 的 `ros2`），
  用官方 `ros:humble-desktop` 容器 `colcon build` 后 `ros2 launch` 起
  geo_node，并跑 `tools/ros2_smoke_test.py` 实测 /uav/odom、/gimbal/los、
  /target/ground 的话题收发（待仓库 workflow 批准后可见）；
- 闭环里的外力直接写入 `xfrc_applied` 的力分量（每步改写一次）。注意
  MuJoCo 的 `xfrc_applied` 跨步保留，逐子步累加的写法会让力按子步数叠乘、
  几秒内飞散——`uav_mujoco_sim.apply_wind` 的注释与其行为不一致，本模块
  没有使用该接口。
