# uav_vertiport_queue — 低空起降场进离场排队与容量评估

## ① 题目与解决的实际问题

低空经济兴起后，一个 vertiport（垂直起降场）要在有限的起降资源下接待
密集的进离场请求。本模块解决三个问题：

1. **排队**：一个起降场有 2 条起飞位、2 个降落位、4 个停机坪，N 架 UAV
   （6~10 架）各有计划起飞/降落时刻。当请求间隔小于运行间隔时必须排队。
   提供三种策略：**FIFO 基线**、**优先级抢占**（医疗/警用 priority=10
   可插队）、**固定间隔**（相邻两次离场 ≥ 60 s、相邻两次进场 ≥ 90 s，
   可配置）。冲突判定采用题目定义：**同一起降位的占用区间重叠即冲突**。
2. **容量评估**：固定请求率扫描（12~88 架次/h，覆盖题目要求的 12~36），
   输出吞吐量曲线、平均等待、最大队列长度与饱和点；延误只计正值。
3. **闭环验证**：抽取排队前后的 UAV 用 MuJoCo（刚体四旋翼 + 几何控制器）
   真飞对比，统计**实际**离地/接地时刻与间隔，证明排队策略在真机上可执行。

## ② 安装

```bash
# Python 依赖（ROS 2 环境外）
pip install numpy mujoco matplotlib pytest

# colcon 编译（可选，源码目录直接运行亦可）
colcon build --packages-select uav_vertiport_queue
source install/setup.bash
```

依赖同目录的 [`uav_mujoco_sim`](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_mujoco_sim)
（MuJoCo 刚体模型 + 几何控制器）作为闭环底座；源码目录直接运行时
`main.py` 会自动把它加入 `sys.path`。

## ③ 编译

```bash
colcon build --packages-select uav_vertiport_queue
```

## ④ 运行（三种模式）

```bash
python main.py --no-sim              # 模式 1：仅算法层（排队 + 容量扫描，秒级）
python main.py                       # 模式 2：算法层 + MuJoCo 闭环真飞（约 3.5 min）
python main.py --scenario light      # 模式 3：只跑一个场景（light/rush/medical/saturation）
./main.sh                            # 或 main.bat（Windows）一行包装
ros2 launch uav_vertiport_queue main.launch.py   # 启动在线排队节点
```

运行结果自动写入 `results/vertiport_queue_result.json`，配图写入
`images/capacity_curve.png` 与 `images/wait_comparison.png`。

**ROS 节点**：`queue_node` 订阅 `<prefix>1..N/request`
（std_msgs/String，JSON：`{"uid","kind","t_request","priority"}`），
发布 `/uav_vertiport_queue/queue`（std_msgs/String，JSON：当前排队序列
+ 每架的排定时刻与预计等待时间）。判定逻辑全部在
`uav_vertiport_queue/core/queue.py`，节点是薄壳。

## ⑤ 测试

```bash
python -m pytest test/ -q
# 37 passed in ~11 s（其中 3 项为 MuJoCo 闭环冒烟测试，其余纯算法层无需 mujoco）
```

## ⑥ 结果（与 results/vertiport_queue_result.json 完全一致）

### 四个内置场景：排队前 vs 排队后

| 场景 | 架次 | 策略 | 起降位冲突（对） | 平均等待/s | 最大等待/s | 最小离场间隔/s | 最小进场间隔/s | 最大队列/架 |
|---|---|---|---|---|---|---|---|---|
| light（轻度流量） | 8 | interval | 3 → **0** | 0.0 → **65.0** | 180.0 | 20.0 → **60.0** | 30.0 → **90.0** | 2 |
| rush（早高峰） | 16 | interval | 12 → **0** | 0.0 → **140.0** | 385.0 | 35.0 → **60.0** | 35.0 → **90.0** | 5 |
| medical（急救插队） | 12 | priority | 4 → **0** | 0.0 → **118.75** | 250.0 | 15.0 → **60.0** | 20.0 → **90.0** | 3 |
| saturation（饱和压测） | 20 | interval | 28 → **0** | 0.0 → **247.5** | 630.0 | 20.0 → **60.0** | 20.0 → **90.0** | 10 |

排队前所有请求按计划时刻直飞（平均等待 0，但存在起降位占用冲突与间隔
违规）；排队后冲突清零、间隔全部达标，代价是正值延误。

medical 场景体现**优先级抢占**：急救机 UAV-3（priority=10）请求 30 s，
排队后仍在 30 s 起飞（延误 0），普通机 UAV-2 为其让行（15 s → 150 s）。

### 容量-请求率扫描（1 h 评估窗口，离场/进场交替的确定性请求流）

| 请求率/（架次/h） | 12 | 16 | 20 | 24 | 28 | 32 | 36 | 40 | 48 | 56 | 64 | 72 | 80 | 88 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 吞吐量/（架次/h） | 12 | 16 | 20 | 24 | 28 | 32 | 36 | 40 | 48 | 56 | 64 | 72 | 80 | 84 |
| 平均等待/s | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 87.95 |
| 积压/架次 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 4 |

**饱和点：80 架次/h**（离场间隔 60 s 与进场间隔 90 s 交替决定的容量
上限；请求率 88 时平均等待 87.95 s、出现 4 架次积压）。曲线见
`images/capacity_curve.png`。

### MuJoCo 闭环真飞（每场景抽 3 架：2 离场 + 1 进场，排队前后各飞一遍）

实际离地/接地时刻与排定时刻的偏差 ≤ 0.55 s，全程最大跟踪误差 3.52 m
（起飞瞬态，巡航段 < 1 m）：

| 场景 | 抽飞机 | 排队前实际离场间隔/s | 排队后实际离场间隔/s |
|---|---|---|---|
| light | UAV-1, UAV-2（离场）+ UAV-5（进场） | 20.0（违规） | **60.0** |
| rush | UAV-1, UAV-2（离场）+ UAV-1（进场） | 35.0（违规） | **60.0** |
| medical | UAV-2, UAV-3（离场，UAV-3 为急救）+ UAV-1（进场） | 15.0（违规） | **120.0**（UAV-3 30 s 起飞、UAV-2 150 s 让行） |
| saturation | UAV-1, UAV-2（离场）+ UAV-1（进场） | 20.0（违规） | **60.0** |

进场接地偏差均为 -0.55 s（159.45 s vs 160.0 s 计划）。排队策略给出的
排定时刻经刚体动力学闭环后仍然成立。

## ⑦ 与已有模块的区分

| 模块 | 关注点 | 与本模块的关系 |
|---|---|---|
| [`uav_mujoco_sim`](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_mujoco_sim) | 单机物理仿真底座 | 本模块在其上做起降场排队与容量层 |
| [`uav_deconflict`](https://github.com/usernamekkyy/ros2/tree/homework/uav_deconflict/src/air/uav_deconflict)（配套模块） | 空中航迹**冲突检测与解脱**（圆柱间隔模型） | 不建模起降位资源；本模块建模**地面起降位占用与排队**，冲突定义是"同位占用重叠"而非机机间隔 |
| [`uav_formation_flight`](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_formation_flight) | 编队保持（队形误差） | 不涉及起降场资源分配 |
| 航点巡航类模块 | 单机沿航路飞行 | 假设起降时刻不受约束 |

本模块补的是**起降场面流量管理**这一层：给定一批"都要用这块场地"的
架次请求，产出一份起降位不冲突、进离场间隔达标的排队序列，量化场地
容量与饱和点，并用真物理仿真证明排定时刻真的可执行。

## 目录结构

```
uav_vertiport_queue/
├── main.py / main.sh / main.bat     # 一键入口（--no-sim / --scenario）
├── config/uav_vertiport_queue.yaml  # 起降位数量、占用时长、60/90 s 间隔、扫描请求率
├── launch/main.launch.py            # 启动 queue_node
├── uav_vertiport_queue/
│   ├── cli.py                       # 命令行编排
│   ├── core/                        # 纯算法（只依赖 numpy）
│   │   ├── queue.py                 # 请求/起降位/三种策略/冲突判定/指标
│   │   ├── capacity.py              # 请求率扫描、吞吐量、饱和点
│   │   ├── track.py                 # 三段匀速直线航迹（60 m 过渡高度）
│   │   ├── scenarios.py             # 4 个内置场景与场地布局
│   │   ├── pipeline.py              # 端到端流水线（含闭环抽飞选架）
│   │   ├── sim.py                   # MuJoCo 闭环真飞与实际时刻提取
│   │   └── figures.py               # matplotlib（Agg）配图
│   └── ros/queue_node.py            # 在线排队节点（薄壳）
├── test/                            # 37 项 pytest（含 3 项 MuJoCo 冒烟）
├── tools/make_figures.py            # 从结果 JSON 重新生成配图
├── results/vertiport_queue_result.json
└── images/capacity_curve.png, wait_comparison.png
```
