# 减速带通过平顺性与建议车速评估

仓库里已有 AEB、盲区监测、横向控制对比等纵向/横向功能，但缺少对**垂向平顺性**的
量化：同一辆车以不同车速压过同一条减速带，车身冲击、轮荷波动和悬架行程差多少？
本模块用两自由度四分之一车模型回答这个问题，并给出"在自定义阈值下建议的最高通过
车速"，可作为园区无人车限速提示或减速带设计评估的依据。

模块不依赖 ROS 2、CARLA、AirSim 或 GPU，`python main.py demo` 一条命令就能跑出
指标与运行效果图；ROS 2 节点只是把同一套计算接到话题上。

## 目录结构

```text
src/ground/speed_bump_ride_comfort/
├── main.py                       # 入口：demo / sweep / ros 三种模式
├── launch/main.launch.py         # ROS 2 启动文件
├── package.xml                   # 依赖与包描述
├── setup.py / setup.cfg          # ament_python 安装配置
├── requirements.txt              # 离线演示依赖（numpy、Pillow）
├── resource/speed_bump_ride_comfort
├── speed_bump_ride_comfort/
│   ├── bump.py                   # 减速带几何（半正弦 / 梯形）
│   ├── vehicle.py                # 四分之一车模型 + RK4 积分
│   ├── comfort.py                # 峰值 / RMS / VDV / 轮荷 / 行程指标
│   ├── analysis.py               # 扫速、建议车速、演示数据集
│   ├── visualize.py              # Pillow 2×2 运行效果图
│   ├── ros_node.py               # ROS 2 适配节点
│   └── cli.py                    # 命令行分派
├── tests/                        # 36 个单元测试
└── results/                      # demo 输出（指标 JSON 与 PNG）
```

## 运行环境

离线演示在以下环境实测通过：

| 项目 | 版本 |
|---|---|
| 操作系统 | Windows 11 |
| Python | 3.12.14 |
| NumPy | 2.3.5 |
| Pillow | 12.3.0 |

ROS 2 节点按 ROS 2 Humble 的 `ament_python` 约定编写，**未在本机验证**（开发机没有
ROS 2 环境），需要按下面的"ROS 2 节点"一节在 Ubuntu 上实机确认。

## 快速开始

```bash
# 1. 安装离线演示依赖
python -m pip install -r src/ground/speed_bump_ride_comfort/requirements.txt

# 2. 进入模块目录（命令行输出与 results 路径都相对于该目录）
cd src/ground/speed_bump_ride_comfort

# 3. 跑完整演示：打印指标表，写出 results/demo_screenshot.png 与 results/metrics.json
python main.py demo
```

可选参数：

```bash
python main.py demo --speeds 1.5 3.0 8.0     # 自选对比车速（m/s）
python main.py demo --dt 0.002               # 积分步长（默认 1 ms）
python main.py demo --out /tmp/bump          # 指定输出目录
python main.py sweep --speed 5.0             # 只算单车速
python main.py sweep                         # 打印完整扫速表与建议车速
```

`python main.py` 不带参数等价于 `demo`。

## 运行效果

`results/demo_screenshot.png`（约 180 KB，1600×1200）：

![减速带通过平顺性与建议车速评估](results/demo_screenshot.png)

- (a) 三档车速下的车身垂向加速度时程，虚线为舒适 / 可接受阈值；
- (b) 轮胎动载比：1.0 为静载，≤ 0 表示车轮离地；
- (c) 峰值加速度随车速的变化，竖线标出两级阈值下的建议车速；
- (d) 三档车速的峰值、RMS、VDV、最小轮荷比与动行程。

本次实测（0.06 m 高、1.2 m 宽半正弦减速带）：

| 车速 | 峰值加速度 | RMS | VDV | 最小轮荷比 | 动行程 |
|---|---|---|---|---|---|
| 5.4 km/h (1.5 m/s) | 2.60 m/s² | 1.00 m/s² | 1.77 m/s^1.75 | 0.72 | 24.9 mm |
| 10.8 km/h (3.0 m/s) | 5.63 m/s² | 1.84 m/s² | 3.42 m/s^1.75 | 0.54 | 57.4 mm |
| 28.8 km/h (8.0 m/s) | 9.41 m/s² | 1.87 m/s² | 4.47 m/s^1.75 | **-0.17（离地）** | 50.9 mm |

建议通过车速：**舒适 ≤ 5.4 km/h，可接受 ≤ 10.8 km/h**（阈值 3.0 / 6.0 m/s²）。
完整扫速 1–20 m/s 时峰值加速度从 1.33 增至 19.16 m/s²，**7.0 m/s 起出现车轮离地**。

## 模型与指标定义

**模型**：两自由度四分之一车，状态为 `[z_s, ż_s, z_u, ż_u]`（相对静平衡位置）：

```text
m_s * z̈_s = -k_s (z_s - z_u) - c_s (ż_s - ż_u)
m_u * z̈_u =  k_s (z_s - z_u) + c_s (ż_s - ż_u) - k_t (z_u - z_r)
```

路面激励 `z_r(t) = z_r(v t)` 由减速带几何给出；积分用 1 ms 定步长 RK4，前置 0.2 s
接近段、后置 2.0 s 回落段。

**指标**（统一在 `t ≥ 0` 即车轮接触减速带之后到仿真结束的窗口内计算）：

| 指标 | 定义 | 单位 |
|---|---|---|
| 峰值加速度 | `max |a_s|` | m/s² |
| RMS | `sqrt(mean(a_s²))` | m/s² |
| VDV | `(∫ a_s⁴ dt)^(1/4)`，梯形积分 | m/s^1.75 |
| 轮胎动载比 | `F_z / F_z,static`，`F_z = (m_s+m_u)g + k_t(z_u - z_r)` | — |
| 悬挂动行程 | `max |z_s - z_u|` | mm |

**阈值说明**：3.0 m/s²（约 0.3 g）与 6.0 m/s²（约 0.6 g）是本模块自定义的经验阈值，
用于给出"建议通过车速"，**不是** ISO 2631 的限值；本模块也**没有**实现 ISO 2631
的频率加权（Wk）滤波器，因此指标不与标准限值直接比较。

## 参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `height_m` | 0.06 | 减速带峰高（m） |
| `width_m` | 1.2 | 沿行驶方向宽度（m） |
| `kind` | `half_sine` | 截面形状，可选 `trapezoid` |
| `ramp_ratio` | 0.25 | 梯形上下坡各占宽度的比例 |
| `sprung_mass_kg` | 350 | 车身（悬挂）质量 |
| `unsprung_mass_kg` | 45 | 非悬挂质量 |
| `suspension_stiffness_n_per_m` | 30000 | 悬架刚度 |
| `suspension_damping_n_s_per_m` | 1800 | 减振器阻尼 |
| `tire_stiffness_n_per_m` | 250000 | 轮胎径向刚度 |
| `travel_limit_m` | 0.090 | 悬架动行程限位，仅用于超限判断 |

默认参数对应偏频 1.47 Hz / 12.55 Hz、阻尼比 0.278 的乘用车单角，属于常见量级。

## ROS 2 节点

```bash
cd ~/ros2_ws
colcon build --packages-select speed_bump_ride_comfort
source install/setup.bash
ros2 launch speed_bump_ride_comfort main.launch.py
# 另开终端发布车速
ros2 topic pub -r 5 /vehicle/speed std_msgs/msg/Float64 "{data: 6.0}"
ros2 topic echo /vehicle/ride_comfort
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/vehicle/speed` | `std_msgs/Float64` | 订阅 | 当前车速（m/s），非法值（负数/NaN）会告警并忽略 |
| `/vehicle/ride_comfort` | `std_msgs/String` | 发布 | JSON：峰值、RMS、VDV、最小轮荷比、动行程、是否超限、是否离地、建议车速与结论 |

节点参数：`bump_height_m`、`bump_width_m`、`bump_kind`、`comfort_limit_mps2`、
`publish_period_s`（默认 0.2 s，即最高 5 Hz）、`simulation_dt_s`（默认 1 ms）。
参数含义与类别见 `ros_node.py` 的 `declare_parameter`。

## 单元测试

```bash
cd src/ground/speed_bump_ride_comfort
python -m unittest discover -s tests -v
```

36 个用例覆盖：减速带几何与解析面积、静平衡与受力方向、偏频与阻尼比、速度单调性、
回落收敛、确定性、RMS/峰值/VDV 的解析值、指标标记、扫速与建议车速逻辑。

## 已知限制

- 线性模型：不含悬架限位撞击、减振器速度特性、衬套刚度与轮胎非线性，因此高车速下
  的峰值加速度会偏乐观；
- 只有垂向自由度：没有俯仰/侧倾，无法评估点头、侧倾与前后轴相位差；
- 未实现 ISO 2631 频率加权，指标不能直接与标准舒适性限值比较；
- 路面激励是刚性几何，未考虑轮胎包络（高宽比大的减速带会被"包"过去）；
- ROS 2 节点未在真机/仿真闭环中验证，本机只验证了它调用的同一套计算函数。
