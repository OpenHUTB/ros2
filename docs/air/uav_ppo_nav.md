# 基于 PPO 的无人机自主导航与避障（uav_ppo_nav）

本模块用 **PPO（近端策略优化，Proximal Policy Optimization）** 训练一个神经网络策略，
让无人机**自主飞向目标点并避开障碍物**。训练在轻量质点环境完成（快），部署在
CarlaAir/AirSim 仿真器（真实），两侧观测/动作接口完全一致，策略可直接迁移。

## 1. 运行架构

```text
/lidar/points ─┐
               ├─► uav_ppo_nav ──/uav/cmd_vel──► carlair_ros_bridge ──► CarlaAir/AirSim
/uav/odom ─────┘   (策略推理)                     (坐标换算+执行)
```

## 2. 观测与动作（全部使用世界系 ENU）

> **为什么用世界系而不是机体系？** 实测 CarlaAir 里 SimpleFlight 的**偏航保持不生效**：
> 即使命令 `yaw_rate = 0`，机头仍以约 9°/s 漂移并 ±25° 摆动。若用机体系速度控制，
> 前进方向会随漂移的机头一起转，轨迹变成**绕圈**（实测半径 ≈ |v|/ω ≈ 0.8 m）。
> 改用世界系后，机头怎么转都不影响轨迹，策略才稳定收敛。

### 2.1 观测（22 维）

| 部分 | 维度 | 说明 |
|---|---|---|
| 激光雷达扇形直方图 | 16 | 360° 均分 16 个扇形（22.5°/个，**扇形 0 = 正东**，逆时针），每扇取最近障碍距离，归一化 `[0,1]`（1=无遮挡）。垂直方向只统计无人机高度 ±1.5 m 内的点，避免地面误报 |
| 目标相对位置 | 3 | 目标点相对无人机的**世界系**偏移（东/北/天），除以 8 m 归一化 |
| 世界系速度 | 3 | 东/北/天速度，分别除以 3 / 3 / 1.5 m/s 归一化 |

扇形直方图公式（以无人机为原点，世界系点 `(x, y)` 为东/北）：

```text
r   = sqrt(x² + y²)
θ   = atan2(y, x)                    # -π..π，0 = 正东
i   = ⌊(θ + Δ/2) / Δ⌋ mod 16        # Δ = 2π/16
h[i] = min over sector i of clip(r / R_max, 0, 1)      # R_max = 12 m
```

### 2.2 动作（4 维，`[-1,1]`）

```text
vx = a[0] · 3.0        # 世界系东向速度 (m/s)
vy = a[1] · 3.0        # 世界系北向速度 (m/s)
vz = a[2] · 1.5        # 世界系天向速度 (m/s)
ω  = a[3] · 1.0        # 偏航角速度 (rad/s，不影响轨迹)
```

映射到 `geometry_msgs/Twist` 的 `linear.x/y/z` 与 `angular.z`。
**因为是世界系速度，桥接必须以 `body_frame:=false` 启动**（否则会被当成机体系）：

```shell
roslaunch carlair_ros_bridge main.launch publish_image:=false body_frame:=false
```

> 节点启动时会**自动校验**桥接的 `control/body_frame` 参数：读到 `true` 会打印
> `logerr` 明确报错，读不到会 `logwarn` 提示，避免出现"无人机会飞但方向不对、
> 日志里却什么都没有"的隐蔽故障。
>
> 桥接的 `control/cmd_duration`（单条速度指令持续时长）默认 **0.1 s**，与本模块训练
> 环境的 `dt = 0.1 s` 对齐；若设成 0.2 s，指令作用时间翻倍，会造成过冲震荡。

## 3. 训练

训练环境 `scripts/env.py` 是**质点 + 速度环一阶滞后**模型：

```text
vel += (cmd_vel − vel) · dt / τ          # τ = 0.45 s，模拟真实速度环滞后
pos += vel · dt
```

随机摆放圆柱障碍物与随机目标，用射线-圆求交生成合成激光雷达直方图。
奖励 = 「向目标靠近的进度 − 动作平滑 − 时间惩罚」，到达 +20、撞障碍/越界 −20。

> **为什么要建模速度滞后？** 真实飞行器的速度环不是瞬时到达指令速度。早期版本
> 用瞬时质点模型训练，策略在 CarlaAir 里会因惯量过冲而**来回震荡**；加入一阶滞后后
> 明显改善。

```shell
pip install "stable-baselines3" "gymnasium" numpy
cd src/air/uav_ppo_nav/scripts
python3 main.py --train --total 400000 --obstacles 16 --out ../models
# 产出 models/policy_weights.npz（部署用）与 models/best_model.zip
```

> 注意 `--out ../models`：训练脚本默认输出到当前目录下的 `models`，而部署时读的是
> `$(find uav_ppo_nav)/models/policy_weights.npz`，所以要用 `--out ../models` 指回包内目录。

环境规模（贴近 CarlaAir 城镇尺度）：场地 ±10 m、目标距离 2~8 m、16 个半径 0.6 m 的障碍、
控制周期 `dt = 0.1 s`。训练评估见 §5。

PPO 超参：`n_steps=2048, batch_size=256, lr=3e-4, gamma=0.99, gae_lambda=0.95,
ent_coef=0.005, net_arch=[64,64]`。

## 4. 部署

部署节点 `scripts/ppo_nav_node.py` 用训练导出的 MLP 权重做**纯 numpy 前向**，
因此虚拟机**不需要安装 torch / stable-baselines3**：

```shell
# 终端 1：桥接（必须 body_frame:=false，因为策略输出世界系速度）
source ~/bridge_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch publish_image:=false body_frame:=false

# 终端 2：PPO 导航
source ~/bridge_ws/devel/setup.bash
roslaunch uav_ppo_nav main.launch goal_x:=16.6 goal_y:=5.4 goal_z:=-10.0
```

推理前向（MLP `[64,64]`）：

```text
h0 = tanh(W0·x + b0)
h1 = tanh(W1·h0 + b1)
a  = clip(W2·h1 + b2, -1, 1)
```

## 5. 验证

### 5.1 本地桩测试（无需 ROS / 仿真器）

```shell
python3 tests/test_ppo_nav_local.py        # 19 项：直方图/世界系变换/动作映射/环境/策略
python3 tests/test_ppo_nav_node_local.py   # 20 项：部署节点（mock 掉 rospy 也能测）
```

另外可以校验「部署侧纯 numpy 推理」与训练框架（SB3）完全一致：

```python
from policy import MlpPolicy; from stable_baselines3 import PPO
# 同一观测下 MlpPolicy.forward 与 model.predict 的最大绝对误差 < 1e-5
# （两者都是 float32，实测量级 1e-8~1e-6，属正常舍入误差）
```

### 5.2 训练环境评估

`main.py --eval --episodes 100 --obstacles 16`：

| 指标 | 数值 |
|---|---|
| 成功率 | **78/100（78.0%）** |
| 失败（碰撞/越界） | 17 |
| 超时 | 5 |
| 平均步数 | 38.4 |
| 平均奖励 | 16.5 |

> **必须显式带 `--obstacles 16`**：`--eval` 的 `--obstacles` 默认值是 6，不传会得到
> 不同的成功率，容易误判。
>
> 训练共 40 万步，但 `train.py` 故意导出的是**评估集最优检查点** `best_model.zip`
> （约 30 万步处）；已逐层比对确认 `policy_weights.npz` 与 best 权重完全一致
> （与 final 差 1.04e-1）。只有 `policy_weights.npz` 入库，两个 `.zip` 训练产物不入库。

### 5.3 CarlaAir 真机联调（同一策略、同一观测代码）

把训练好的策略直接部署到 CarlaAir（Town01），从悬停点给出目标点，记录全程：

| 目标距离 | 飞行用时 | 最终误差 | 说明 |
|---|---|---|---|
| 6.71 m | 3.4 s | 0.59 m | 开阔空域 |
| 8.94 m | 4.2 s | 0.38 m | 楼群高度（z ≈ −10） |
| 16.12 m | 6.4 s | **0.22 m** | 楼群高度，激光雷达 2567~3228 点 |
| 11.00 m | 5.0 s | 0.35 m | 楼群高度 |

**结论：训练环境（纯 numpy 质点模型）中学到的策略，可直接迁移到 CarlaAir 的
真实六自由度无人机上完成自主导航，位置误差均在 0.6 m 以内。**

### 5.4 工程经验（sim-to-real 差距）

联调过程中定位到三个真实的落差来源，均已修正：

1. **偏航不可控 → 改用世界系控制**。CarlaAir 的偏航保持不生效（漂移+摆动），
   机体系控制会导致绕圈飞行。
2. **速度环滞后 → 训练环境建模一阶滞后**。否则策略在真机上因惯量过冲震荡。
3. **控制周期必须一致**。训练 `dt = 0.1 s`，部署也须用 0.1 s（用 0.25 s 会震荡）。

另外，CarlaAir 的仿真器在**高密度几何 + 大点云**场景下（楼群中激光雷达 9000+ 点）
单步 RPC 可达数秒，属平台性能上限，与策略无关。

**该限制已通过传感器配置缓解**：原先 `settings.json` 的雷达没设 `Range` 且
`PointsPerSecond: 100000`，桥接又把滤波上限放到 60 m，而策略实际只用到 12 m ——
等于每帧都传输并解包大量用不上的远点。现改为 `Range: 15.0`、`PointsPerSecond: 40000`，
并把 `lidar/max_range` 收到 15 m，显著降低每帧射线投射与传输开销。
若在楼群中仍偏慢，可继续下调这两项。

## 6. 效果图

![PPO 自主导航演示](img/air/uav_ppo_nav/ppo_nav_demo.gif)

上图：PPO 策略驱动 CarlaAir 无人机自主飞向目标点（第一视角，前视相机）。

## 7. 参考

* [carlair_ros_bridge 桥接模块](../air/carlair_ros_bridge.md)
* [Proximal Policy Optimization (Schulman et al., 2017)](https://arxiv.org/abs/1707.06347)
* [Stable-Baselines3 文档](https://stable-baselines3.readthedocs.io/)
