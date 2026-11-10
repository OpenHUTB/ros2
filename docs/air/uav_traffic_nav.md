# 城市动态交通下的 PPO 避障（uav_traffic_nav）

> 本页对应当前开发分支。**这是一个仍在迭代中的模块**，下面区分了"已验证"与"尚未验证"。

## 1. 要解决的问题

`uav_ppo_nav` 的障碍是 16 个随机静态圆柱；真实低空场景里最大的风险来自**会动的车**。
本模块把障碍换成沿车道行驶的车辆，要求无人机从随机起点穿过车流到达随机目标点。

与静态障碍相比，难点在于**时机**：从车道间穿过时，安全窗口只有 1 秒左右，
只看"当前距离"不够，必须根据车辆速度预判。

## 2. 方法

### 2.1 观测（34 维）

| 部分 | 维度 | 说明 |
| --- | --- | --- |
| 激光直方图 | 16 | 世界系固定方向，0 = 正东，1 = 无障碍 |
| 目标相对位置 | 3 | 除以 8 m 归一化 |
| 世界系速度 | 3 | 除以 3 m/s、1.5 m/s |
| **最近 3 辆车** | **12** | 每车 4 维：相对位置 (2) + **相对速度 (2)** |

可以用 `--no-traffic-features` 训练去掉最后 12 维的消融版本（22 维，只能靠激光直方图感知车辆当前位置）。
本 PR 不保留消融权重，因此最终结果表只比较"带交通特征"的策略与不学习的基线。

### 2.2 动作

4 维：`[vx, vy, vz, yaw_rate]`，世界系 ENU 速度 + 偏航角速度，归一化到 `[-1, 1]`。
高度由底层执行器保持（本模块只做水平面避障）。

### 2.3 环境

* 质点 + **速度环一阶滞后**（`vel_tau = 0.45 s`），与 `uav_ppo_nav` 一致；
* 两条车道（`y = ±4 m`），车辆 2.5–5 m/s，按难度 1–2 辆/车道；
* 车辆圆形近似（半径 1.0 m），无人机半径 0.3 m（碰撞判定圆半径合计 1.3 m）；
* 起点/终点采样时保证与车辆至少相距 2.5 m（避免"出生即撞"污染训练信号）。

### 2.4 奖励

```
进度奖励  +1.0 / 米（朝目标靠近）
碰撞终止  -30
TTC 惩罚  -1.0 × Σ(1 - ttc/2)²   （ttc < 2 s 的每辆车）
平滑/步数 -0.02 / -0.005
```

TTC（碰撞时间）用二维匀速模型计算；它让"贴着车头抢行"这种高风险的省时行为代价更高。

## 3. 实验结果（多种子，难度 1.0，100 回合/条件，评测 seed 11）

| 策略 | 混合起终点 | 强制跨路 |
| --- | --- | --- |
| 规则专家（读真值，上界参照） | 72% | 75% |
| **行为克隆 + DAgger（3 个训练种子）** | **61.0% ± 3.0%** | **48.0% ± 3.6%** |
| greedy（无视车辆） | 57% | 39% |
| 速度障碍法 | 53% | 19% |

对照（单种子）：PPO 从零训练 61% / 44%，BC+DAgger+PPO 微调 68% / 41%，
纯行为克隆（无 DAgger）23% / 13%。

"强制跨路"分布：起点固定在路一侧、终点在另一侧，必须横穿两条车道。

## 4. 示范引导（本模块的关键做法）

PPO 从零训练卡在 44%（强制跨路），因为"等窗口"这类时机决策几乎采不到正样本。
改为两段式：

1. **规则专家**：在车道前 3 m 悬停，检查是否有车会在 1.2 s 内经过穿越走廊；
   安全则全速穿过（`baselines.expert_action`）。强制跨路 75%。
2. **行为克隆 + DAgger**（`scripts/bc_train.py`）：
   * 按 6:4 的混合/强制跨路分布采集约 9k 条专家示范；
   * 3 轮 DAgger：学生自己飞，在每个到达状态贴专家标签再训练；
   * 3 个种子的学生成功率分别为 20→42→50%、32→30→58%、10→42→60%。
3. **可选 PPO 微调**（`train.py --init-from`）：同分布 +4，跨路 -11，故不作默认导出。

## 5. 结论与局限

* 示范引导有效且稳定：3 个种子 **48.0% ± 3.6%**（强制跨路），
  高于 PPO 从零（44%）、贪心（39%）、速度障碍法（19%）。
* 与规则专家（75%）仍差 27 个百分点，瓶颈是"何时开始穿越"的时机精度。
* 纯 BC 只有 13%：没有 DAgger 的纠偏样本，学生一偏离专家轨迹就崩。
* 训练环境是质点模型、车辆圆形近似；**尚未在 CarlaAir 上做端到端在线飞行**。
* 评测种子会显著影响绝对值（seed 7 时贪心 26%、seed 11 时 39%），
  因此本表固定 seed 11 并同时给出 3 个训练种子的标准差。

## 6. 复现

```bash
# 1) 3 个种子跑专家示范 + BC + DAgger（约 6 分钟，可并行）
powershell -File scripts/run_dagger_seeds.ps1
# 2) 可选 PPO 微调
python3 scripts/train.py --total 300000 --difficulty 1.0 --init-from models/bc_policy_s42.npz --lr 1e-4 --ent 0.003 --name pol_bc_ppo
# 3) 多种子评测（出表 + 误差棒图）
python3 scripts/eval_multiseed.py
```

## 7. 下一步

1. CarlaAir 在线飞行（`traffic_feed_node` + `traffic_nav_node`）验证 sim-to-sim；
2. 固定 200 回合评测集，进一步压低评测噪声；
3. 把专家规则升级成"安全窗口 + 速度规划"，缩小与 75% 的差距。

## 附录 A 关键公式

**观测归一化**（`policy.build_observation`）：目标相对位置除以 8 m，速度分别除以 3.0 m/s 与 1.5 m/s；
车辆特征的位置除以 12 m、相对速度除以 10 m/s，并裁剪到 `[-1, 1]`。

**动作到速度**（`policy.action_to_velocity`）：

```
[vx, vy, vz, wz] = a ⊙ [3.0, 3.0, 1.5, 1.0],   a ∈ [-1, 1]^4
```

**碰撞时间**（`policy.time_to_collision`，二维匀速模型）：

```
r = p_car − p_drone,  v = v_car − v_drone
t* = −(r·v) / |v|²
若 0 < t* ≤ T_h 且最近距离小于 (r_drone + r_car)，记为有碰撞风险
```

**奖励**（`traffic_env.step`）：

```
r = w_p·(d_{t−1} − d_t) − w_s·||a_t − a_{t−1}||² − w_step
    − w_ttc·Σ_i (1 − ttc_i / T_h)²            # 只对 ttc_i ≤ T_h 的车辆
    + 20·1[到达] − 30·1[碰撞]
```

默认 `w_p = 1.0`、`w_s = 0.02`、`w_step = 0.005`、`w_ttc = 1.0`、`T_h = 2.0 s`。

## 附录 B 源码解析

| 文件 | 关键函数 | 作用 |
| --- | --- | --- |
| `policy.py` | `build_observation` | 把 16 扇区直方图、目标、速度、3 辆车特征拼成 34 维观测 |
| | `traffic_features` | 按距离取最近 3 辆车，输出相对位置与相对速度，不足补零 |
| | `time_to_collision` | 二维匀速模型下的 TTC |
| | `MlpPolicy.forward` | 纯 numpy 推理：tanh 两层 + 线性输出 + clip |
| `traffic_env.py` | `_sample_task` | 先放车、再挑不与车重叠的起点/终点（避免"出生即撞"） |
| | `_ray_dist` / `_histogram` | 16 条射线与圆求交，得到激光直方图 |
| | `step` | 一阶速度滞后 → 位置更新 → 车辆推进 → 碰撞/到达判定 → TTC 惩罚 |
| `baselines.py` | `greedy_action` / `vo_action` / `expert_action` | 贪心、速度障碍法、规则专家（示范来源） |
| `bc_train.py` | `collect_expert` / `rollout_student` / `train` | 专家示范、DAgger 采样、加权行为克隆 |
| `traffic_nav_node.py` | `build_obs` / `emergency_stop` | 拼装真实观测、车辆进入安全半径时悬停兜底 |

## 附录 C 复现与验证步骤

```bash
# 1) 依赖
python3 -m pip install -r requirements.txt
python3 -m pytest tests -q            # 期望 28 passed

# 2) 复现多种子结果（约 6 分钟，可并行）
powershell -File scripts/run_dagger_seeds.ps1

# 3) 评测（需先有 models/bc_policy_s4{2,3,4}.npz）
python3 scripts/eval_multiseed.py     # -> results/multiseed.{json,md,png}
python3 scripts/final_report.py       # -> results/final_report.{json,md,png}

# 4) 部署（ROS Noetic，先起桥接且 body_frame:=false）
roslaunch uav_traffic_nav main.launch live_json:=/path/live.json          calibration_json:=/path/calibration.json goal_x:=8.0 goal_y:=0.0
```

部署自检：

```bash
rosparam list | grep -E "traffic_nav|traffic_feed"   # 应看到 /traffic_nav/weights、/traffic_feed/live_json
rosparam get /traffic_nav/weights                    # 应与 launch 传入的一致
rostopic echo /traffic/vehicles                      # 车辆流是否在发布
rostopic echo /uav/cmd_vel                           # 策略是否在发速度
```

判定标准：`/traffic_nav/weights` 落点正确、`/traffic/vehicles` 持续有数据、`/uav/cmd_vel` 随车辆位置变化；
车辆进入 `safety_stop_m` 时速度指令应变为 0。

> 在线闭环飞行尚未验证：评测机没有 ROS 1 环境，CarlaAir 的 `ready=false` 也意味着速度接口不可用，
> 因此本模块目前只提供**离线评测 + 部署接口**，不代表已完成实飞。
