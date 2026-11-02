# lateral_control_benchmark

地面载具**横向控制算法对比实验台**：把三种最常用的横向控制器放在**同一车辆模型、同一参考轨迹、同一套指标**下做定量比较。

| 控制器 | 类型 | 核心参数 | 每步用到的信息 |
|---|---|---|---|
| `pure_pursuit` 纯跟踪 | 几何法 | 前视距离（随速度自适应） | 前视点坐标 |
| `stanley` | 几何法 | 横向增益 k、软化项 k_soft | 控制点横向偏差、航向偏差、曲率 |
| `lqr` | 最优控制 | Q/R 权重、积分增益 Ki | 横向偏差、航向偏差、横摆角速度 |

模块**不依赖 CARLA / Gazebo / 任何外部仿真器**，也**不需要 ROS** 就能跑完整对比；
同时提供 ROS 1 (Noetic) 节点与 `roslaunch` 启动入口，可直接在 `/cmd_vel`、`/odom` 层对接。

* 运行环境：Ubuntu 20.04 + ROS Noetic（离线部分任何有 Python 3.8+ 的机器都能跑）
* 依赖：`numpy` 必需；`matplotlib` 仅用于出图

## 1. 快速开始（离线，推荐先用这个）

```bash
cd src/ground/lateral_control_benchmark

# ① 单元测试：66 项，不需要 ROS、不需要仿真器
python3 test/test_controllers.py

# ② 完整对比：四场景 × 三控制器，输出指标表、综合排名与图表
python3 main.py --mode bench

# ③ 换车辆模型 / 速度：
python3 main.py --mode bench --model dynamic --speed 15

# ④ 只跑单场景 / 单控制器：
python3 main.py --mode bench --scenario arc
python3 main.py --mode single --controller stanley --scenario sine

# ⑤ 参数扫描（看单个参数的影响）：
python3 main.py --mode sweep --param stanley.k --values 1.0,2.0,4.0

# ⑥ 无绘图环境只出数值：
python3 main.py --mode bench --no-figures
```

生成物在 `results/`：`metrics.csv`、`metrics.json`、各场景 `trajectory_*.png` / `error_*.png`、`metrics.png`。

## 2. ROS 1 运行

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
cp -r <本模块目录> .
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

roslaunch lateral_control_benchmark main.launch                     # 默认 stanley + sine
roslaunch lateral_control_benchmark main.launch controller:=lqr
roslaunch lateral_control_benchmark main.launch scenario:=arc model:=dynamic
```

观测：

```bash
rostopic echo /lateral_control/styles    # [横向偏差, 航向偏差, 前轮转角, 车速]
rostopic echo /lateral_control/metrics   # [mean|e|, max|e|, RMS, 步数]
rqt_plot /lateral_control/styles/data[0] # 横向偏差随时间曲线
```

节点自带车辆模型（不依赖 Gazebo），因此一条 `roslaunch` 就能看到完整闭环。

## 3. 目录说明

```
main.py                       统一入口：bench / single / sweep / ros
scripts/lateral_control_node.py  ROS 1 节点入口（roslaunch 用）
launch/main.launch            ROS 1 启动文件
config/params.yaml            节点参数
test/test_controllers.py      66 项离线单元测试
lateral_control_benchmark/
  geometry.py                 参考轨迹（直线/圆弧/正弦/双移线）与投影
  vehicle_model.py            运动学自行车 + 二自由度动力学模型
  controllers.py              纯跟踪 / Stanley / LQRI 三种控制器
  lqr_solver.py               纯 NumPy 的 Riccati 方程求解器（不依赖 scipy）
  simulation.py               闭环仿真、速度策略、13 项指标、排名
  benchmark.py                场景×控制器矩阵、导出、汇总
  viz.py                      图表（可选 matplotlib）
  ros_node.py                 ROS 1 节点实现
results/                      运行产物（CSV/JSON/PNG），均由脚本现场生成
```

## 4. 评测方法

为保证对比公平，三者共用：

1. **同一车辆模型**：默认运动学自行车模型（`--model dynamic` 可切二自由度动力学）；
2. **同一纵向速度策略**：巡航速度 + 按曲率自动限速 + 一阶加减速响应；
3. **同一初始位姿**：路径起点前方 2 m，横向偏移 1.2 m、航向偏差 0.10 rad，初速 3 m/s；
4. **同一套指标**（13 项，定义见文档）：平均/最大/RMS/稳态横向偏差、平均航向偏差、
   转角变化率 RMS（平顺性）、最大转角、收敛时间、离道占比（|e|>1 m 的时间占比）等；
5. **各自调好参数**：三个控制器都用自己的较优参数（纯跟踪前视增益 0.6、Stanley k=4.0、
   LQR Q=diag(20,2,0.5)/R=1/Ki=0.8），避免"给对手用坏参数"。

## 5. 主要结果（巡航 8 m/s，运动学模型）

| 控制器 | 平均稳态偏差 / m | 平均 RMS / m | 平均收敛时间 / s | 综合平均名次 |
|---|---|---|---|---|
| pure_pursuit | 0.0221 | 0.2580 | 1.35 | **1.81** |
| stanley | **0.0011** | 0.2455 | 2.04 | 1.91 |
| lqr | 0.0163 | 0.2459 | 1.89 | 2.28 |

结论要点：

* **稳态精度**：Stanley 最好（毫米级），因为它同时用了横向偏差、航向偏差与曲率前馈；
  纯跟踪在连续变曲率（正弦）上残差最大（0.083 m），因为几何法只看前视点、没有曲率先验。
* **收敛速度**：纯跟踪最快（1.35 s），因为前视点提供了天然的阻尼；LQR 次之（1.89 s）。
* **平顺性**：纯跟踪与 Stanley 的转角变化率 RMS 更小（20～29 °/s），
  LQR 因为增益较大达到 33～41 °/s。
* **实现细节的影响可能大于算法本身**：Stanley 若沿用教材的**前轴**取点，
  在半径 20 m 圆弧上稳态偏差会升到 **0.48 m**；改为**后轴**（与运动学模型参考点一致）
  后降到 **0.001 m**。用 `--mode sweep` 可复现这组对照。

### 5.1 速度边界与文件编码

* **倒车（负速度）**：本模块的横向指标都按前进定义，因此默认不允许倒车。
  配置阶段对 `cruise_speed <= 0`、`initial_speed < 0` 等非法值**直接抛 `ValueError`**，
  不再静默钳位；运行中若真的出现负速度，会钳到 0 并把次数记入指标的
  `reverse_blocked_steps` 列（正常场景恒为 0）。需要真实倒车时，
  用 `VehicleParams(allow_reverse=True)`。
* **CSV 编码**：`results/*.csv` 用 **UTF-8 无 BOM** 写出（列名全为 ASCII，
  BOM 会让部分 Excel 把首列表头显示成乱码）；读取旧文件时仍兼容带 BOM 的版本。


## 6. 与仓库已有地面载具模块的区别

仓库里已经出现过"纯跟踪 / 横向控制"这些词，为免误解，这里把差异说清楚：

| 已有模块 | 它做了什么 | 与本模块的关系 |
|---|---|---|
| `carla_perception_control` | 在 CARLA 中做**感知 + 给定轨迹跟踪**，横向用纯跟踪作为其中一环 | 纯跟踪只是它的一个构件；本模块把**三种算法并列**做成基准对比 |
| `carla_nn_car_following` | 在 CARLA 中做**自适应跟车（ACC）**，横向用纯追踪 / 面包屑跟踪 | 同上；本模块不做 ACC，只比横向算法 |
| `car_2d_target_tracker` | 2D 平面内的**目标追踪** | 任务是"追目标"，不是"跟轨迹" |
| `carla_keyboard_control` / `mujoco_car_project` | 仿真器内的**键盘遥控演示** | 本模块无遥控、无仿真器 |

因此本模块的定位是**"横向控制算法之间的对比基准"**，而不是再实现一个纯跟踪、也不是再做一个跟车任务。
它也是仓库里目前唯一一个**不依赖任何外部仿真器**就能完整复现的地面载具模块：
只需要 Python 与 NumPy，没有 GPU、没有 CARLA、没有 Gazebo 也能得到全部指标。

## 7. 大模型使用声明

本模块的代码、文档与实验脚本在编写过程中使用了 AI 编程助手（DeepSeek）辅助生成与调试；
所有指标均由 `python3 main.py --mode bench` 现场生成（`results/` 已加入 .gitignore，不入库），提交内容由提交者负责。
