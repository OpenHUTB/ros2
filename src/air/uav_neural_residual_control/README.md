# uav_neural_residual_control

**神经残差动力学与学习式轨迹跟踪。**

用一个小型前馈网络学习"标称模型 − 真实动力学"的差值，再用递推最小二乘
在线重写最后一层。覆盖四旋翼上最常见的四类未建模效应：**气动阻力、阵风、
挂载载荷、地面效应**。

```text
             ┌──────────────── 离线 ────────────────┐
  观测 ──▶ 特征 φ(x) ──▶ MLP(14→64→64→3) ──▶ Δa_hat ──┐
             └──────────────────────────────────────┘   │
                                                        ▼
             ┌──────────────── 在线 ────────────────┐  减法
   实测 Δa ─▶ RLS（只更新最后一层，65×3 参数）──────┘   │
                                                        ▼
   几何控制器：  f_des = m·(a_d − Δa_hat + Kp·e_p + Kd·e_v + g·e_z)
```

**核心结果**（螺旋 + 八字两条轨迹 × 3 个风种子平均，稳态 RMSE，单位 m。
4 载荷 × 3 风况 × 3 轨迹 × 4 控制器 × 3 种子 = 432 次闭环运行，无一发散）：

| 载荷 | 风况 | 位置 PID | SE(3) 几何 | + 残差网络 | + 在线 RLS |
|---|---|---|---|---|---|
| 空载 | 无风 | 0.0164 | **0.0098** | 0.0305 | 0.0187 |
| 空载 | 阵风 3.2 m/s | 0.0700 | 0.1039 | 0.0609 | **0.0235** |
| +0.25 kg | 阵风 3.2 m/s | 0.0790 | 0.1654 | 0.0610 | **0.0219** |
| +0.50 kg | 阵风 3.2 m/s | 0.0976 | 0.2794 | 0.0747 | **0.0232** |
| **+0.75 kg（训练集外）** | 阵风 3.2 m/s | 0.1216 | 0.4037 | 0.1295 | **0.0261** |

三点值得注意：

* 几何控制器对常值载荷的静差 $=\Delta a/K_p$，与风况基本无关；空载时风扰才是主要误差来源（无风 0.0098 m，阵风 0.1039 m）。
* 残差网络不是免费的：无扰动时它自身的 0.281 m/s² 预测误差就是新的误差下限
  （0.0305 m，比纯几何的 0.0098 m 还差）。
* 在线自适应把整表压平：`geo_rls` 在 12 格里落在 0.0187 ~ 0.0261，跨度仅 0.007 m，
  且训练集外的 +0.75 kg 档与标称档几乎相同。

复现：`python main.py --pipeline`（约 15 分钟，纯 CPU）。

---

## 1. 为什么需要"残差"这一层

几何控制器假设模型已知：`a = R·e_z·T/m + g`。真实机体上这个等式差四项：

| 未建模项 | 量级 | 特点 |
|---|---|---|
| 气动阻力 | 10 m/s 时约 0.6 倍重力 | 与**速度平方**成正比，机体系各向异性 |
| 阵风 | 平均 3 m/s + σ=1.8 m/s 的 OU 过程 | 时变、不可测 |
| 挂载载荷 | +0.75 kg → 垂直残差 **7.7 m/s²** | 常值，但直接吃掉推力裕度 |
| 地面效应 | 贴地推力增益 12%，衰减尺度 0.25 m | 只依赖高度 |

反馈能压住其中一部分：位置 PID 靠积分消掉常值载荷，代价是相位裕度；
几何控制器没有积分，静差严格等于 `|Δa|/kp`（实测 0.5 kg → 0.259 m，
解析值 0.258 m）。**前馈补偿不消耗任何裕度**，这是本模块的出发点。

## 2. 为什么不引 PyTorch

网络是 14→64→64→3、5,315 个参数，纯 numpy 前向一次 30 µs，反向一次 90 µs，
训练 9 万样本 60 轮约 12 s，与 torch 同级。但仿真侧（Python ≥3.10）与
ROS 侧（Humble 的 Python 3.8）都要能 import，纯 numpy 省掉一个 700 MB 依赖，
ROS 节点也不需要额外装任何东西。

反向传播手写，`--selftest` 里用数值梯度逐点校验（相对误差 < 1e-5）。

## 3. 四个控制器

四者**共用同一套内环**（姿态几何 PD + 逆混控 + 限倾角 + 饱和处理），
只差在期望加速度 `a_cmd` 怎么算。做法是把底座的几何控制器 `kp/kd` 置零，
让它退化为"加速度 → 推力/力矩"的执行器。

| 名字 | 外环 | 抗常值扰动 | 抗时变扰动 |
|---|---|---|---|
| `pid` | 位置 PID（含积分、条件积分抗饱和） | 积分消掉 | 积分把阵风低频分量也当偏差追 |
| `geo` | SE(3) 几何控制 + 轨迹前馈 | 静差 = Δa/kp | 反馈带宽内可压 |
| `geo_res` | 几何 + 离线残差网络前馈 | 前馈抵消 | 网络预测依赖状态，可泛化 |
| `geo_rls` | 几何 + 离线残差 + 在线 RLS | 在线辨识 | 在线辨识 |

## 4. 目录结构

```
uav_neural_residual_control/
├── main.py                       # 统一入口（作业要求入口以 main 开头）
├── launch/main.launch.py         # ros2 launch 入口
├── config/residual.yaml          # 桥接节点参数
├── uav_neural_residual_control/
│   ├── cli.py                    # --selftest/--collect/--train/--eval/--plot/--pipeline/--demo/--server/--node
│   ├── selftest.py               # 34 项数值自检（不需要 ROS、不需要显示器）
│   ├── plotting.py               # 由结果直接生成文档配图
│   ├── paths.py                  # 资源定位（源码布局 & colcon 安装布局）
│   ├── core/                     # ← 纯 Python：除 plant/dataset/experiment 外只需 numpy
│   │   ├── attitude.py           #   姿态数学、混控、几何控制器（无 mujoco，供 ROS 侧复用）
│   │   ├── disturbances.py       #   阻力 / 风 / 载荷 / 地面效应
│   │   ├── plant.py              #   ResidualPlant：底座四旋翼 + 未建模效应
│   │   ├── remote_plant.py       #   RemotePlant：把"跨 TCP 的机体"伪装成本地机体
│   │   ├── trajectories.py       #   悬停 / 圆 / 螺旋 / 八字（解析 pos/vel/acc/yaw）
│   │   ├── residual.py           #   MLP + 特征 + 限幅（手写前向/反向 + Adam）
│   │   ├── rls.py                #   递推最小二乘（遗忘因子、协方差上限、泄漏选项）
│   │   ├── controllers.py        #   四个跟踪器
│   │   ├── dataset.py            #   数据采集（含带限激励）
│   │   ├── training.py           #   训练 + 早停
│   │   ├── metrics.py            #   RMSE / P95 / IAE / 残差精度
│   │   └── experiment.py         #   4 载荷 × 3 风况 × 3 轨迹 × 4 控制器
│   ├── sim/plant_server.py       # 带未建模效应的仿真服务端（供 ROS 用）
│   └── ros/residual_node.py      # ROS2 桥接节点（控制律跑在 ROS 侧）
├── artifacts/                    # 训练产物（数据集 / 权重 / 结果 / 配图）
└── test/                         # pytest 冒烟测试
```

## 5. 运行

```bash
# 0) 依赖：numpy + mujoco + 底座包 uav_mujoco_sim
#    仿真侧 Python ≥ 3.10
export PYTHONPATH=<仓库>/src/uav_mujoco_sim:$PYTHONPATH

# 1) 数值自检（不需要 ROS / 显示器，约 4 s）
python main.py --selftest                    # 期望 34/34

# 2) 一条命令跑完整条流水线（约 15 分钟，纯 CPU）
python main.py --pipeline
#    等价于 --collect → --train → --eval → --plot

# 3) 单次运行看指标
python main.py --demo --payload extra --wind windy --traj helix

# 4) 打开 MuJoCo 窗口肉眼看
python main.py --demo --viewer
```

`--pipeline` 产出：

| 文件 | 内容 |
|---|---|
| `artifacts/residual_dataset.npz` | 90450 条 (特征, 残差) 样本 |
| `artifacts/residual_net.npz` | 43 KB 权重 |
| `artifacts/results/grid.json` | 全部运行的指标 + 展示工况的逐时刻序列 |
| `artifacts/figures/*.png|gif` | 文档配图 |

## 6. ROS2 用法

仿真侧（需要 mujoco，Python ≥3.10）与 ROS 侧（Python 3.8）分开跑，
中间走 TCP —— 与底座的 `carlair_ros_bridge` 架构一致：

```bash
# 终端 1：仿真（可跑在宿主机）
python main.py --server --payload heavy --wind windy

# 终端 2：ROS 节点（控制律在这里）
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run uav_neural_residual_control residual_node \
  --ros-args -p net:=<包目录>/artifacts/residual_net.npz -p controller:=geo_rls
```

或一键：

```bash
ros2 launch uav_neural_residual_control main.launch.py \
  payload:=heavy wind:=windy controller:=geo_rls traj:=helix
```

> ⚠️ 本节点**替代**底座的 `bridge` 节点运行，二者同时开会互相抢控制权。

发布的额外话题（底座没有的）：

| 话题 | 类型 | 含义 |
|---|---|---|
| `/uav/reference` | `geometry_msgs/Point` | 当前参考位置 |
| `/uav/tracking_error` | `geometry_msgs/Vector3`（z 分量为范数） | 位置误差 |
| `/uav/residual` | `geometry_msgs/Vector3` | 网络预测的世界系残差加速度 |
| `/uav/disturbance` | `geometry_msgs/Vector3` | 真值残差加速度（仅仿真侧可得，用于对照） |
| `/uav/wind` | `geometry_msgs/Vector3` | 当前风速 |

可切换：`/uav/trajectory`（`std_msgs/String`）、`/uav/controller`、`/uav/reset`。

**同一份控制器代码两侧共用**：`Tracker` 只依赖 `state()/dt/nominal_params/
last_acc/nominal_accel` 这几个方法，仿真侧由 `ResidualPlant` 提供，ROS 侧由
`RemotePlant`（只吃里程计与自己下达的推力）提供。因此"仿真里调好的增益
搬到 ROS 上行为一致"是可验证的，不需要维护两套实现。

## 7. 验证

```bash
python main.py --selftest        # 34/34，含与底座数学实现的逐位一致性校验
pytest -q                        # 26 项 pytest，约 12 s
```

自检覆盖：阻力耗散性、OU 过程统计量、载荷注入、**悬停实测残差 = Δm·g/m_nom
解析值**、四条轨迹的解析 pos/vel/acc 中心差分校验、反向传播数值梯度校验、
网络拟合能力与限幅、RLS 收敛性与低激励保护、采集数据无 NaN、四个控制器的
执行器范围、本包与底座混控的数值一致性。

**ROS2 端到端**（在课程虚拟机里实测，仿真与节点同在虚拟机内）：

| 项 | 结果 |
|---|---|
| `colcon build` | 两个包装配通过 |
| 自检（VM 内，Python 3.12） | 34/34 |
| 话题频率（50 Hz 控制回路，TCP 往返） | `/uav/odom` 等 **43.7 ~ 45.2 Hz** |
| 跟踪误差（`extra` 档 + 3.2 m/s 风，25 s，螺旋，三次运行） | 均值 **0.030 ~ 0.036 m**，最大 **0.075 m** |

最后一个数字值得注意：这是**训练集外**的载荷、跑在 **Python 3.8 + TCP** 上，
误差与进程内实验（0.026 m）同量级，说明"仿真里调好的控制器搬到 ROS 上行为一致"
这句话是站得住的。

## 8. 已知限制

- **偏航力矩受混控预算约束。** 合推力 `T` 下可用的偏航力矩上限是
  `k_m·min(4·T_max − T, T)`，标称档位悬停时只有 0.14 N·m。偏航误差超过
  约 13° 就会吃光预算、把滚转/俯仰的权限按比例压掉，进而翻机（实测倾角
  177°）。因此所有轨迹都从**轨迹自身的初始偏航**起飞。
- **内环角速率阻尼取的是底座默认值的 2 倍。** 底座按 0.95 kg 空机整定，
  挂到 +0.75 kg 时转动惯量涨约 1.6 倍、阻尼比从 0.41 掉到 0.32，起步的
  35° 倾角阶跃会过冲到 50° 以上并翻机。加倍后 0.95~1.7 kg 全载荷域稳定。
- **`+0.75 kg` 档刻意不进训练集**，用来分开"泛化"（`geo_res`）与
  "在线辨识"（`geo_rls`）两种能力。它的绝对值也接近本机体的推力上限
  （起飞重量 1.7 kg，总推力 24 N），再重就要考虑饱和了。
- 残差网络外推能力有限：训练速度包线是 0~3 m/s，超出后阻力项误差按平方增长。
  生产环境应在目标包线内补采数据。
- 在线 RLS 只改最后一层。整网微调在本场景下不稳定（在线数据时序强相关），
  且 65×3 参数已足够覆盖载荷与气动偏差。
