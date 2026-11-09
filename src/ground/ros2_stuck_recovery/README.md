# ros2_stuck_recovery —— 差速机器人卡死检测与自动脱困

差速底盘的机器人最容易出的一类故障是**静默卡死**：任务层照常下发速度指令，驱动板也不报错，
但车其实已经顶在墙上、轮子空转、或者一侧轮子悬空。上层看到的是"任务一直没完成"，
而不是"车动不了了"。本模块在 `/cmd_vel_nav` 与 `/cmd_vel` 之间做一层守护：

1. 用**指令 / 轮式里程计 / IMU / 前向测距**四路信号的一致性判断"是不是卡住了、卡在哪一类"；
2. 按故障类型执行**分级脱困动作**（倒车让位 → 换向 → 试探前进；打滑走"停顿-起步"脉冲）；
3. 脱困成功后把控制权交回任务层；连续 3 级失败则**停车上报**，不再无休止挣扎。

核心算法只依赖 NumPy，**无需 ROS 与仿真器即可离线复现全部结论**；ROS2 节点只做话题收发。

## 环境

| 项目 | 版本 | 说明 |
|---|---|---|
| Python | 3.8+（实测 3.13.14） | 离线复现与测试 |
| NumPy | ≥1.21（实测 2.5.4） | 核心算法 |
| matplotlib | ≥3.5（实测 3.11.2） | 仅生成文档配图，缺失时自动跳过 |
| pytest | ≥7.0 | 单元测试 |
| ROS 2 | Humble / Foxy（可选） | 在线运行需要 `rclpy` 等消息包 |

离线部分不依赖 ROS；`node.py` 在构造时才 import `rclpy`，因此没装 ROS 也能跑测试。

## 目录结构

```
ros2_stuck_recovery/
├── main.py                     # 入口：一键离线复现（python main.py）
├── launch/recovery.launch.py   # ROS2 启动文件
├── src/ros2_stuck_recovery/    # 算法包（无 ROS 依赖）
│   ├── robot.py                #   差速运动学、限幅、轮速正反解
│   ├── detect.py               #   三路一致性卡死检测
│   ├── recovery.py             #   分级脱困状态机
│   ├── sensors.py              #   激光扇区取值等消息规整
│   ├── guard.py                #   每周期决策（里程计可用性 → 检测 → 脱困），可离线测
│   └── node.py                 #   ROS2 节点（薄封装）
├── sim/                        # 仿真与实验
│   ├── stuck_world.py          #   含"顶住/打滑/单轮失抓地"故障的 2D 世界
│   ├── scenarios.py            #   7 个可复现场景
│   ├── experiments.py          #   闭环评估（真值评分，在线只用里程计）
│   ├── run_sim.py              #   跑场景 + 出图
│   └── benchmark.py            #   多种子对照表（Markdown/CSV）
├── tests/                      # 70 项单元/集成测试
├── ci/run_ci.sh                # 五阶段自检
└── docs_figs/                  # 生成的配图与基准表
```

## 快速开始（离线，不需要 ROS）

```bash
cd src/ground/ros2_stuck_recovery
pip install -r requirements.txt

python main.py                      # 7 个场景 × 3 个种子，打印对照表并生成配图
python main.py --scenario slip      # 只跑打滑场景
python main.py --no-plots           # 只看指标
bash ci/run_ci.sh                   # 语法/依赖/测试/闭环/基准 五阶段自检
```

控制台输出（10 个种子，节选）：

```
场景                    脱困    成功率           触发    脱困    检测/s      脱困耗时/s       脱困后里程/m
blocked_wall          关     0/10         0.0   0.0    8.45           -          0.00
blocked_wall          开     10/10        1.0   1.0    8.45        4.28          2.46
corner                开     10/10        2.0   2.0    2.80        4.39          3.00
slip                  开     10/10        2.0   1.0    2.80        2.40          0.52
one_wheel             开     10/10        1.0   1.0    0.80        4.57          2.64
wall_hug              开     10/10        0.0   0.0       -           -          0.00
```

对照实验的口径：关掉脱困层时，四类卡死场景**全部无法继续前进**（脱困后里程 0）；
开启后四类全部恢复运动，两个"正常行驶"场景的误报次数为 0。

## 在线运行（ROS 2）

```bash
# 1) 编译
colcon build --packages-select ros2_stuck_recovery
source install/setup.bash

# 2) 任务层改发 /cmd_vel_nav，底盘保持订阅 /cmd_vel
ros2 launch ros2_stuck_recovery recovery.launch.py
```

接线关系：

```
任务层/导航  ──/cmd_vel_nav──▶  stuck_recovery  ──/cmd_vel──▶  底盘驱动
                                    ▲
                     /odom  /imu  /scan
```

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/odom` | `nav_msgs/Odometry` | 位姿与机体速度（必需） |
| 订阅 | `/imu` | `sensor_msgs/Imu` | 只用 `angular_velocity.z`（可选，缺省时降级） |
| 订阅 | `/scan` | `sensor_msgs/LaserScan` | 取前向 ±12° 最小测距（可选） |
| 订阅 | `/cmd_vel_nav` | `geometry_msgs/Twist` | 任务层指令 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 经守护层后的底盘指令 |
| 发布 | `/stuck_recovery/status` | `std_msgs/String` | JSON：状态、故障类型、级别、触发/脱困次数 |
| 服务 | `/stuck_recovery/clear_abort` | `std_srvs/Trigger` | 人工确认后解除 ABORT |

参数（`ros2 param list /stuck_recovery` 可查）：`odom_topic`、`imu_topic`、`scan_topic`、
`task_topic`、`cmd_topic`、`rate_hz`、`max_level`。

> **注意**：`cmd_topic` 必须接底盘，`task_topic` 必须接任务层，否则本节点会订阅到自己的输出形成自环。

## 判定逻辑（要点）

三路证据并联，按"硬件可信度"排序，超过确认时长 `confirm=0.7 s` 才报警：

| 通道 | 需要的信号 | 判据 | 结论 |
|---|---|---|---|
| A | 指令 + 里程计 | 窗口内 `d_odom/d_cmd < 0.30` 且持续 | 被顶住（`blocked`）/ 原地打转（`spin`） |
| B | 里程计 + IMU | `|ω_odom − ω_imu| ≥ 0.20 rad/s` 且持续 | 单轮失抓地（`one_wheel`） |
| C | 里程计 + 前向测距 | 走了 ≥0.15 m 但前向距离几乎不减，且**最近接近过某个面** | 轮胎打滑（`slip`） |

通道 A 优先于 C：里程计"完全不动"和"编码器说在走"是互斥的两种证据。
通道 C 的"最近接近过"这一条是为了把**贴墙平行行驶**（测距恒定但从未接近）排除掉。

**里程计不可用时（含还未收到首条 `/odom`）一律 fail closed**：`OdomWatch` 用"是否收到过"
而不是"时间戳是否为 0"来表示可用性，未收到时年龄取 `inf`，直接落到 stale 分支输出零速；
同时清空检测器缓冲，避免黑屏期"有指令零位移"的假证据在里程计恢复瞬间被误判成卡死。
启动期任务层先发速度指令也不会被透传到底盘。

## 脱困策略（要点）

```
CRUISE ─检测到卡死─▶ BACK_OFF ─▶ TURN ─▶ PROBE ─成功─▶ CRUISE
   ▲                    │失败             │失败
   │                    └──── ESCALATE ◀──┘  （级别 +1，换动作族）
   └──── 人工 clear_abort ──── ABORT ◀── 3 级都失败
```

* **分级**：倒车距离 `0.22 + 0.10·(level−1)` m，转向角 0.9 → 1.6 → 2.6 rad；
* **换动作族**：打滑先试"停顿-起步"脉冲（利用静摩擦），失败就改倒车退出，不在同一类动作上加码；
* **记忆**：同位置 2 m / 20 s 内再次触发，直接从更高一级开始，避免"挣扎-卡住-再挣扎"；
* **有界退出**：`level > 3` 立刻停车并上报，交给人工，而不是继续乱动把车顶到更坏的姿态；
* **安全**：输出全部限幅（前进 0.30 / 倒车 0.15 m/s，转向 1.2 rad/s），里程计超时 0.5 s 立即停车。

## 验证范围与局限

* 全部指标来自**离线 2D 仿真**（20 Hz 闭环、含编码器/陀螺仪/测距噪声），**未在真实底盘上闭环验证**；
* 评估用仿真真值位姿，模块内部只拿里程计——这个差异是本模块要解决的问题，不是评估作弊；
* `slip` 检测依赖前向测距且目标面在 1 m 内；`one_wheel` 依赖 IMU；缺传感器时相应通道自动降级；
* 单线测距下"贴墙平行"与"原地空转"物理同构，靠"最近接近过"这条记忆区分，沿墙极近距离
  长时间平行行驶仍是已知误判风险；
* 只处理**暂时性**故障（异物、打滑、姿态被别住）；永久失效时模块的正确行为是停车上报。

## 大模型使用声明

本模块的代码、仿真、测试与文档在编写过程中使用了 AI 编程助手（WorkBuddy / Claude）辅助生成与
修改，作者对提交内容的正确性、可复现性与实验结果负全部责任。

## 许可证

MIT
