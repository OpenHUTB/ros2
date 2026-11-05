# uav_noise_footprint — 城市低空噪声足迹评估与航线降噪

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.12-green)](https://mujoco.readthedocs.io/)

城市上空密集无人机飞行带来的**噪声扰民**是低空经济落地的主要阻力之一。
本模块把"这批航线到底多吵、怎么飞才不吵"变成一个可复现的工程闭环：

> 声源模型 → 沿航迹能量积分（敏感点 LAeq）→ 噪声足迹等值线 →
> 三种降噪手段按代价择优 → MuJoCo 真飞验证 + 真实高度回灌噪声模型

## 1. 这个模块解决什么问题

仓库里已有的模块解决的是别的层面的问题：

| 已有模块 | 解决的问题 | 与本模块的区别 |
| --- | --- | --- |
| [`uav_mujoco_sim`](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_mujoco_sim) | 单机物理仿真底座 | 本模块在其上做噪声评估与降噪闭环 |
| [`uav_deconflict`](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_deconflict) | 机机**间隔**（4D 冲突检测与解脱） | 不涉及地面噪声暴露与敏感点 |
| [`uav_formation_flight`](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_formation_flight) | 编队**保持**（队形误差） | 不关心航迹对地面的噪声影响 |
| `no_fly_zone_check` | 静态禁飞区（几何） | 二维空域约束，不含声学传播模型 |

本模块补的是**环境噪声**这一层：给定航线与敏感点（学校/医院/居民区），
量化每点的等效连续 A 声级 LAeq,T 与 45/55/65 dB 足迹范围，产出一份
"敏感点噪声达标、航程代价受控"的降噪方案，并用真物理仿真证明它真的有效。

## 2. 方法

### 2.1 声学模型（`core/acoustic.py`）

桨盘功率 P（W）→ A 计权声功率级，经验幂律（声功率与输入功率近似成正比，
参 sUAS 实测声功率标度关系，此处为工程近似）：

```
Lw = 10·log10(P) + K
```

`K = 70.73` 为标定常数：取基准机型 P = 300 W，使 **100 m 处约 55 dB**。
传播取两项物理效应（自由场球面扩散 + 大气吸收 α ≈ 0.5 dB/100 m @ 500 Hz，
ISO 9613-1 量级）：

```
Lp(r) = Lw − 20·log10(r) − α·r
```

### 2.2 暴露评估（`core/exposure.py`）

航迹按 **2 m** 采样（爬升/巡航/下降全程），每个采样点对每个敏感点算 Lp，
能量叠加合成敏感点 **LAeq,T**（多机直接能量相加；按场景的小时航班密度
归一到 1 小时观测期）。足迹在航迹包络外扩 150 m 的矩形上取 **200×200**
栅格逐格算同一 LAeq 场，再用**自实现的 marching squares**（纯 numpy）
提取 45/55/65 dB 等值线段与超限面积。

### 2.3 降噪优化（`core/mitigate.py`）

三种手段按"目标改善量 / 航程代价"贪心择优，每架飞机每种手段至多一次：

1. **抬升巡航高度** +30/+60 m —— 球面扩散直接拉低敏感点 Lp；
2. **侧向平移航线** —— 巡航段余弦台阶横移，与敏感点保持 ≥ 200 m 缓冲；
3. **起降爬升段更陡** 8° → 12° —— 更快离开起降点附近的敏感点。

目标 = Σ(敏感点 LAeq 的能量 × 权重)（能量域加权，最吵的敏感点主导）；
约束 = **航程增加 ≤ 15 %**（几何不可行候选自动跳过并记录原因）。

### 2.4 MuJoCo 闭环（`core/sim.py`）

每个场景抽对敏感点噪声贡献最大的一架，优化前后各真飞一次
（`uav_mujoco_sim` 的 Quadrotor + GeometricController；参考速度前馈加
一阶低通 τ=0.5 s、姿态阻尼 kw=−0.18，与 `uav_deconflict` 的闭环写法一致）。
验收两条：爬升角 12°/巡航 90 m 下**全程最大跟踪误差 < 8 m**；用**真实
位置日志**（20 Hz）回灌声学模型重算敏感点 LAeq，证明结论来自闭环数据。

## 3. 目录结构

```
uav_noise_footprint/
├── package.xml  setup.py  setup.cfg  resource/
├── main.py  main.sh  main.bat        # 一键入口（薄壳）
├── config/params.yaml                # 声学/评估/优化/节点参数
├── launch/main.launch.py             # 启动在线噪声告警节点
├── uav_noise_footprint/
│   ├── cli.py                        # 命令行编排
│   ├── core/                         # 纯算法，只依赖 numpy
│   │   ├── acoustic.py               # 声源 + 传播模型
│   │   ├── track.py                  # 飞行剖面（斜爬升几何）
│   │   ├── exposure.py               # LAeq 积分 + 栅格 + marching squares
│   │   ├── mitigate.py               # 三手段贪心降噪
│   │   ├── scenarios.py              # 三个场景
│   │   ├── sim.py                    # MuJoCo 闭环
│   │   └── pipeline.py               # 端到端流水线与指标
│   ├── figures.py                    # 足迹图（matplotlib Agg）
│   └── ros/noise_node.py             # 在线噪声告警节点（薄壳）
├── test/                             # 44 项单元测试（含 MuJoCo 冒烟）
├── tools/make_figures.py             # 从 results JSON 重生成文档图
├── results/*.json                    # 运行结果（README 数字与之一致）
└── images/footprint*.png             # 噪声足迹等值线（优化前后并排）
```

## 4. 安装、编译、运行、测试

依赖：`numpy`、`mujoco`、`matplotlib`（pip），ROS 2 Humble（rclpy /
std_msgs / nav_msgs）。`uav_mujoco_sim` 与本包同级放置，源码目录直接运行
会自动挂路径，无需先 colcon install。

```bash
# 安装依赖
pip install numpy mujoco matplotlib

# 编译（ROS 2 工作空间）
colcon build --packages-select uav_noise_footprint

# 三种运行模式
python main.py --no-sim              # 仅算法层（约 1.5 min，不需要 mujoco）
python main.py                       # 全流程：算法层 + MuJoCo 闭环（约 2 min）
python main.py --scenario school     # 只跑一个场景

# 测试
python -m pytest test/ -q            # 44 项全过（约 7 s）

# ROS 节点（在线噪声告警）
ros2 launch uav_noise_footprint main.launch.py
# 或源码目录：python -m uav_noise_footprint.ros.noise_node
```

## 5. 实测结果（3 场景 × 基线/优化后 × MuJoCo 闭环）

以下数字来自 `results/all_result.json`（`python main.py` 自动生成），
脚本路径：`uav_noise_footprint/core/pipeline.py`。LAeq 口径：1 小时观测期
× 场景小时航班密度（residential 60 架/h，school/mixed 45 架/h）。

### 5.1 敏感点 LAeq,T 对比表（dB(A)）

| 场景 | 敏感点 | 类型 | 限值 | 优化前 | 优化后 | Δ |
| --- | --- | --- | --- | --- | --- | --- |
| residential | 小区A | 居民区 | 60 | 58.1 | 52.9 | −5.1 |
| residential | 小区B | 居民区 | 60 | 55.5 | 52.9 | −2.6 |
| residential | 小区C | 居民区 | 60 | 56.5 | 53.9 | −2.6 |
| school | 实验小学 | 学校 | 55 | **56.0（超标）** | 51.1（达标） | −4.8 |
| school | 小区D | 居民区 | 60 | 53.8 | 51.9 | −1.9 |
| mixed | 中心医院 | 医院 | 55 | 53.5 | 50.3 | −3.1 |
| mixed | 中学 | 学校 | 55 | **55.0（超标）** | 51.7（达标） | −3.3 |
| mixed | 小区E | 居民区 | 60 | 54.7 | 54.5 | −0.2 |

加权目标（能量域合成）：residential **62.5 → 58.7 dB（−3.8）**，
school **60.9 → 56.8 dB（−4.1）**，mixed **61.7 → 59.3 dB（−2.4）**。

### 5.2 降噪动作与航程约束（≤ 15 %）

| 场景 | 动作 | 航程增幅 |
| --- | --- | --- |
| residential | UAV-1 高度 30→90 m + 爬升角 12°；UAV-2 侧移 155 m | +2.04 % |
| school | UAV-1/UAV-2 高度 30→90 m；UAV-1 爬升角 12° | +0.00 % |
| mixed | UAV-2 高度 30→90 m + 爬升角 12°；UAV-1 侧移 170 m | +2.43 % |

### 5.3 噪声足迹（200×200 栅格，≥55 dB 面积变化）

| 场景 | 45 dB 面积 前→后 (m²) | 55 dB 面积 前→后 (m²) | 65 dB 面积 前→后 (m²) |
| --- | --- | --- | --- |
| residential | 803300 → 803501 | 359413 → 208321 | 1370 → 1169 |
| school | 1284368 → 1284644 | 172861 → 51610 | 829 → 691 |
| mixed | 890513 → 891020 | 217308 → 102252 | 737 → 645 |

足迹等值线图：`images/footprint.png`（两行=优化前/后，三列=场景；
另有 `images/footprint_<场景>.png` 单场景并排版）。

### 5.4 MuJoCo 闭环（每场景优化前后各真飞 1 架）

| 场景 | 复飞 | 仿真时长 (s) | 墙钟 (s) | 最大跟踪误差 (m) | 终点误差 (m) |
| --- | --- | --- | --- | --- | --- |
| residential | UAV-1 前 / 后 | 161.4 / 162.9 | 6.1 / 6.0 | 3.86 / 3.86 | 0.84 / 0.85 |
| school | UAV-1 前 / 后 | 161.4 / 162.9 | 5.9 / 6.1 | 3.86 / 3.86 | 0.84 / 0.85 |
| mixed | UAV-2 前 / 后 | 169.4 / 170.9 | 6.3 / 6.3 | 3.86 / 3.86 | 0.84 / 0.85 |

全流程 `python main.py` 总墙钟 **2 min 02 s**（< 6 min 红线）。
爬升角 12° + 巡航 90 m 的优化航迹**全程最大跟踪误差 3.86 m < 8 m**，
闭环可控。真实位置日志回灌噪声模型重算的 LAeq 与计划值最大偏差
**≤ 0.02 dB**（6 次真飞 × 全部敏感点，两位小数下最大 0.02）——结论由
闭环数据支撑，不是拿计划值凑数。

## 6. 测试

`python -m pytest test/ -q` —— **44 项全过，约 7 s**，0 fail：

| 文件 | 覆盖 |
| --- | --- |
| `test_acoustic.py` (8) | 幂律公式、100 m 标定、传播公式逐项、衰减单调性、大气吸收、能量叠加 |
| `test_track.py` (10) | 斜爬升几何、匀速性、地面起终点、侧移保持起终点、不可行构造拒绝、字典往返 |
| `test_exposure.py` (10) | 采样步长、LAeq 上界、对称性、交通密度 +10 dB、栅格与单点一致性、marching squares 线性场精确解、面积单调性、节点判定与告警 |
| `test_mitigate.py` (8) | 三手段各自有效、侧移方向与缓冲区、双侧敏感点无解、航程约束、每手段一次 |
| `test_scenarios.py` (5) | 场景注册、几何可行、基线真实超标 |
| `test_sim.py` (3) | MuJoCo 冒烟：跟踪误差 < 8 m、结果形状、真实轨迹回灌噪声模型偏差 < 1 dB |

## 7. 已知限制

- 声学模型是工程近似（幂律 + 球面扩散 + 单一吸收系数），不含桨叶谐波、
  地面反射与气象；`K` 由"100 m 处约 55 dB"标定，不同机型需重标。
- LAeq 口径假设航班事件相互独立且机型相同；多机同空域的时刻相关噪声
  会低估。
- 降噪优化是贪心逐轮择优，不保证全局最优；侧移只处理敏感点同侧的情形。
- ROS 节点用内置敏感点表演示，真实部署应从地图/规划服务加载。

## 8. 作业信息

- 课程仓库：[OpenHUTB/ros2](https://github.com/OpenHUTB/ros2)
- 仿真底座：[uav_mujoco_sim](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_mujoco_sim)
- 同层参考模块：[uav_deconflict](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_deconflict)
