# autonomous_car_stanley_tracking

无人车**横向控制**模块：用 Stanley 前轴横向偏差算法（含曲率前馈）让地面载具稳定跟踪给定参考路径，
并与**纯跟踪（pure pursuit）**、**预瞄 PID** 两种常见基线做同场对比。

模块的最大特点是**不依赖 CARLA / Gazebo / AirSim，也不依赖 ROS**：
核心算法与闭环仿真只用 NumPy，跑一条命令就能在本机复现全部数值与效果图，
适合作为横向控制算法本身的验证与调参工具；同时提供 ROS 2 节点封装，可直接接到实车或仿真器上。

* 在线文档：<https://openhutb.github.io/ros2/ground/autonomous_car_stanley_tracking/>
* 完整说明（公式推导、算法流程、源码解析、性能评价、问题记录）见 `docs/ground/autonomous_car_stanley_tracking.md`

## 1. 运行环境

| 项目 | 说明 |
|---|---|
| 操作系统 | Windows 10/11、Ubuntu 20.04/22.04（均已验证离线模式） |
| Python | 3.8 及以上（实测 3.13.7） |
| 必需依赖 | `numpy`（核心算法与仿真） |
| 可选依赖 | `matplotlib`（出效果图）、`imageio`（出 GIF 动图） |
| ROS 2 | 可选。仅 `--mode ros` 与 launch 启动需要（Humble 实测） |

## 2. 运行步骤

```bash
# ① 安装依赖（核心只需 numpy）
python3 -m pip install -r requirements.txt

# ② 离线单元测试：56 项，不需要 ROS、不需要模拟器、不需要联网
python3 test/test_stanley.py

# ③ 一键跑全部场景并导出效果图（等价于 main.sh / main.bat）
python3 main.py --mode demo

# ④ 只跑某个场景 / 某个控制器
python3 main.py --mode demo --scenario circle
python3 main.py --mode single --controller stanley --scenario sine

# ⑤ 横向增益 k 的扫描实验
python3 main.py --mode sweep --parameter k --values 0.8,1.6,2.4,4.0 --scenario circle

# ⑥ 控制点对比：后轴（默认）vs 前轴（经典 Stanley）
python3 main.py --mode compare

# ⑦ 只输出数值指标，不画图（无 matplotlib 的最小环境）
python3 main.py --mode demo --no-figures
```

Windows 下可直接双击 `main.bat`，Linux/WSL 下执行 `./main.sh`，两者都会依次完成
"装依赖 → 跑测试 → 跑全部场景并出图"。

ROS 2 方式（需要先 `source /opt/ros/<distro>/setup.bash`）：

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
cp -r <本模块目录> .
cd ~/ros2_ws
colcon build --packages-select autonomous_car_stanley_tracking --symlink-install
source install/setup.bash

# 启动节点（内置圆轨迹参考路径，等待 /odom）
ros2 launch autonomous_car_stanley_tracking main.launch.py \
    scenario:=circle k:=1.6 target_speed:=5.0

# 另开终端观察跟踪状态：[/cmd_vel, 期望前轮转角, 横向偏差/航向偏差/转角/速度]
ros2 topic echo /stanley/tracking_error
```

话题接口：

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/odom` | `nav_msgs/Odometry` | 车辆位姿与速度 |
| 订阅 | `/reference_path` | `nav_msgs/Path` | 参考路径（不发布则用内置场景） |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 线速度 + 横摆角速度，可直接驱动差速小车 |
| 发布 | `/stanley/steering` | `std_msgs/Float64` | 期望前轮转角，供阿克曼底盘或调试观察 |
| 发布 | `/stanley/tracking_error` | `std_msgs/Float64MultiArray` | `[横向偏差, 航向偏差, 转向角, 期望速度]` |

## 3. 目录说明

```
main.py                    统一入口：demo / single / sweep / compare / replay / ros
main.sh / main.bat         一键入口（装依赖 + 测试 + 出图）
stanley_tracking/
  geometry.py              角度归一化、参考折线最近点投影、单调前向投影、曲率估计
  reference.py             参考路径生成：直线、圆弧、正弦、双移线、CSV 航点
  controllers.py           Stanley 控制器 + 纯跟踪 / 预瞄 PID 两种基线
  simulator.py             差速底盘闭环仿真、速度曲线（含弯道限速）、性能指标
  benchmark.py             场景 × 控制器矩阵实验、参数扫描、CSV/JSON 导出
  visualization.py         轨迹图、误差曲线、指标柱状图、跟踪过程 GIF
  stanley_node.py          ROS 2 节点封装（订阅 /odom，发布 /cmd_vel）
launch/main.launch.py      ROS 2 启动入口
config/params.yaml         节点参数
test/test_stanley.py       56 项离线测试（几何 / 控制律 / 仿真 / 指标 / 导出 / 控制点对比）
results/                   运行产物：效果图、GIF、指标 CSV/JSON、轨迹时序 CSV
```

## 4. 效果与结果

本模块**不在仓库中保存图片**，全部结果都由脚本现场生成，便于复核与复现：

```bash
cd src/ground/autonomous_car_stanley_tracking
python3 main.py --mode demo       # 生成轨迹图、误差曲线、指标柱状图与跟踪动图
python3 main.py --mode compare    # 生成后轴/前轴控制点对比图与 CSV
python3 main.py --mode sweep --parameter k --values 0.8,1.6,2.4,4.0 --scenario circle
```

生成物（目录 `results/`，均为可再生的运行产物）：

| 文件 | 内容 |
|---|---|
| `trajectory_<场景>.png` | 参考路径与三个控制器的轨迹对比 |
| `error_<场景>.png` | 横向偏差与航向偏差随时间的变化 |
| `metrics.png` | 四项指标的柱状图对比 |
| `tracking.gif` | 跟踪过程动图（车辆位姿随时间推进） |
| `metrics.csv` / `metrics.json` | 四场景 × 三控制器的 13 项指标 |
| `control_point_compare.csv` | 后轴/前轴控制点对比 |
| `trace_<场景>.csv` | 单条轨迹的逐步时序，可用 Excel 复现曲线 |

四场景 × 三控制器的完整指标（速度 5 m/s，初始偏差 1.5 m / 0.1 rad）：

| 控制器 | 场景 | 平均绝对横向偏差 / m | 最大偏差 / m | **稳态偏差 / m** | 收敛时间 / s |
|---|---|---|---|---|---|
| **stanley** | 直线 | 0.177 | 1.575 | **0.0002** | 1.70 |
| pure_pursuit | 直线 | 0.194 | 1.576 | 0.0001 | 2.06 |
| lookahead_pid | 直线 | 0.245 | 1.577 | 0.0268 | 2.68 |
| **stanley** | 圆（R=15 m） | 0.018 | 1.371 | **0.0123** | 1.34 |
| pure_pursuit | 圆（R=15 m） | 0.026 | 1.371 | 0.0199 | 1.56 |
| lookahead_pid | 圆（R=15 m） | 0.789 | 1.371 | 0.7868 | 240（未收敛） |
| **stanley** | 正弦（幅值 3 m） | 0.126 | 1.658 | **0.0184** | 3.00 |
| pure_pursuit | 正弦（幅值 3 m） | 0.332 | 1.659 | 0.2204 | 18.46 |
| lookahead_pid | 正弦（幅值 3 m） | 0.556 | 1.657 | 0.5869 | 18.94 |
| **stanley** | 双移线 | 0.086 | 1.566 | **0.0006** | 1.70 |
| pure_pursuit | 双移线 | 0.099 | 1.567 | 0.0071 | 2.04 |
| lookahead_pid | 双移线 | 0.149 | 1.568 | 0.0210 | 2.74 |

结论：Stanley 在四种场景下的稳态偏差都最小，尤其在**连续变曲率**（正弦）与**定曲率**（圆）场景上，
因为有曲率前馈 `atan(L*kappa)`，稳态偏差比没有前馈的预瞄 PID 小 1～2 个数量级。

### 4.5 控制点对比：后轴（默认）vs 前轴（经典 Stanley）

| 场景 | 后轴控制点稳态偏差 / m | 前轴控制点稳态偏差 / m | 改善 |
|---|---|---|---|
| 直线 | 0.0002 | 0.0000 | — |
| 圆（R=15 m） | **0.0123** | 0.2127 | 约 17 倍 |
| 正弦 | **0.0184** | 0.2855 | 约 16 倍 |
| 双移线 | **0.0006** | 0.0088 | 约 15 倍 |

原因：经典 Stanley 在前轴中心取横向偏差与曲率前馈，而前轴比车辆实际到达的位置**超前一个轴距**；
转弯时前馈取到的是车辆前方 L 处的曲率，等于用"更外侧"的弯曲信息去转向，于是留下稳态偏差。
把控制点放回运动学模型的原点（后轴中心，即本模块默认）后，偏差与该处曲率一致，稳态偏差下降一个数量级。
两种控制点都保留在代码里（`control_point="rear_axle" | "front_axle"`），可用 `--mode compare` 复现。

## 5. 参数说明与调参建议

| 参数 | 默认值 | 作用与建议 |
|---|---|---|
| `k` | 1.6 | 横向增益。越大收敛越快、稳态偏差越小；过大时转角变化剧烈。圆轨迹上 k 从 0.8 → 4.0，稳态偏差 0.024 → 0.005 m 单调下降 |
| `k_soft` | 1.2 | 软化项。低速时抑制 `atan` 项，避免停停走走时的蛇行；高速时影响很小 |
| `feedforward` | 开 | 曲率前馈。定曲率与连续变曲率场景下是精度的主要来源，建议保持开启 |
| `control_point` | `rear_axle` | 控制点。追求稳态精度用后轴；若必须沿用经典前轴写法，可切到 `front_axle` |
| `target_speed` / `a_lat_max` | 5.0 / 2.5 | 纵向速度与侧向加速度约束。仿真按 `v <= sqrt(a_lat/kappa)` 自动弯道限速 |

## 6. 单元测试

```bash
python3 test/test_stanley.py     # 56 项，全部离线
```

覆盖内容：

1. **几何**：角度归一化边界、横向偏差符号、投影弧长、曲率估计、闭合路径首点曲率（防止历史缺陷回归）；
2. **控制律**：横向偏差项、航向偏差项、曲率前馈、转角限幅、转向速率限幅的解析值校验；
3. **对照控制器**：前视距离上下限、修正方向、积分累积；
4. **闭环仿真与指标**：收敛性、初始对准、侧向风扰下的有界性；
5. **参考场景**：闭合圆弧、双移线、正弦、航点路径长度；
6. **批量实验与导出**：矩阵实验、指标表格、CSV/JSON 与轨迹时序写盘；
7. **控制点回归**：后轴控制点的稳态偏差必须显著优于前轴。

## 7. 常见问题

| 现象 | 原因与解决 |
|---|---|
| 车辆绕圈飞出去、横向偏差越来越大 | 横向偏差项与航向偏差项符号不一致导致的**正反馈**。两者的符号必须满足 `delta = psi_e - atan(k*e/(k_soft+v))`，其中 `psi_e = path_yaw - yaw`。见文档第 8 章的问题记录 |
| 静止起步时蛇行 | `k_soft` 太小。低速时 `atan(k*e/(k_soft+v))` 会接近饱和，把 `k_soft` 调到 1～2 |
| 偏差总是差一点收敛不到 0 | 弯道上先确认 `feedforward` 是否开启、控制点是否用后轴；直线上残余偏差来自侧向风扰等外部常值扰动（本控制器无积分项） |
| 报错 `缺少 matplotlib` | `pip3 install matplotlib`，或用 `--no-figures` 只跑数值 |
| 报错 `无模块 named 'rclpy'` | `--mode ros` 与 launch 需要 ROS 2 环境，先 `source /opt/ros/<distro>/setup.bash` |
| 生成的图里中文变方块 | 系统缺少中文字体。安装 `Noto Sans CJK` 或使用 Windows 自带的微软雅黑（脚本会自动尝试） |

## 8. 大模型使用声明

本模块的代码、文档与实验脚本在编写过程中使用了 AI 编程助手（DeepSeek）辅助生成与调试；
所有公式、参数与性能数据均由本机实际运行结果得出，提交内容由提交者负责。
