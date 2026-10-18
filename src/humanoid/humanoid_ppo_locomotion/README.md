# humanoid_ppo_locomotion

Unitree G1 人形机器人的双足控制模块：**解析平衡基元 + PPO 残差策略**，在 **MuJoCo** 物理仿真中
训练，通过 **ROS 2** 话题接收键盘指令控制上半身姿态，输出 12 个腿部关节目标角以维持平衡。

- 仿真：MuJoCo 3.3+（无头环境用 OSMesa 离屏渲染）
- 框架：Stable-Baselines3 PPO（PyTorch）
- 机器人：Unitree G1 29 自由度人形（12 个腿部关节由神经网络控制，17 个上半身关节由键盘控制）
- ROS：ROS 2 Humble

> **交付范围**：**稳定站立平衡**（20 s 跑满、跌倒率 0%、静止速度误差 0.013 m/s）。
> **行走未实现**，原因与实测证据见第 5.3 节。请据此评估是否符合预期。

## 1. 模型与代码来源

| 内容 | 来源 | 许可证 |
|------|------|--------|
| G1 机器人 MJCF 模型与网格 | [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) `unitree_robots/g1` | BSD-3-Clause |
| PPO 算法实现 | [DLR-RM/stable-baselines3](https://github.com/DLR-RM/stable-baselines3) | MIT |
| 环境封装、平衡基元、奖励设计、ROS 2 节点、训练与评价脚本 | 本模块原创 | BSD-3-Clause |

网格文件（约 32 MB）不进入仓库，首次运行前用 `scripts/fetch_assets.py` 从官方仓库下载。

## 2. 环境要求

已验证环境：

| 项 | 版本 |
|----|------|
| 操作系统 | Ubuntu 22.04（Docker 容器） / Windows 11（仅训练与离线演示） |
| ROS | ROS 2 Humble |
| Python | 3.10 |
| MuJoCo | 3.3.7 |
| PyTorch | 2.x（CPU 即可） |
| Stable-Baselines3 | 2.3.2 |

## 3. 安装

```bash
# 1) 放入工作空间
cd ~/ros2_ws/src
# 拷贝本模块目录 humanoid_ppo_locomotion

# 2) 下载 G1 网格资源（约 116 MB 压缩包，解压出 32 MB 网格）
cd humanoid_ppo_locomotion
python3 scripts/fetch_assets.py
# 若本地已有 unitree_mujoco 仓库，可离线复制：
# python3 scripts/fetch_assets.py --from-local /path/to/unitree_mujoco

# 3) 安装 Python 依赖
pip3 install --ignore-installed "mujoco==3.3.7" "numpy<2" gymnasium \
    "stable-baselines3==2.3.2" torch

# 4) 构建
cd ~/ros2_ws
colcon build --packages-select humanoid_ppo_locomotion
source install/setup.bash
```

> `pip3 install --ignore-installed` 是必需的：ROS 镜像里的 numpy 由 apt 安装，
> 新版 pip 会因 `uninstall-distutils-installed-package` 报错。

## 4. 运行

### 4.1 一键启动（仿真 + 策略 + 键盘）

```bash
ros2 launch humanoid_ppo_locomotion main.launch.py
```

另开一个终端查看状态：

```bash
ros2 topic echo /cmd_vel
ros2 topic hz /humanoid/obs
ros2 run tf2_tools view_frames
```

无图形界面（如 Docker）时关闭键盘节点，改用话题直接驱动上半身（17 维，弧度）：

```bash
ros2 launch humanoid_ppo_locomotion main.launch.py teleop:=false

# 抬起双臂（左右肩 pitch = 1.2 rad ≈ 69°），其余关节置 0
ros2 topic pub --once /humanoid/upper_body std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0, 1.2,0,0,0,0,0,0, 1.2,0,0,0,0,0,0]}"
```

17 维顺序：`waist_yaw, waist_roll, waist_pitch, left_shoulder_pitch, left_shoulder_roll,
left_shoulder_yaw, left_elbow, left_wrist_roll, left_wrist_pitch, left_wrist_yaw,
right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow, right_wrist_roll,
right_wrist_pitch, right_wrist_yaw`。

确认机器人在站：`ros2 topic echo /odom --field pose.pose.position.z` 应稳定在 0.78 左右。

### 4.2 键盘操作（上半身姿态，腿部由神经网络保持平衡）

| 按键 | 作用 | 键位上限 | 实测说明 |
|------|------|----------|----------|
| `W` / `S` | 腰部前倾 / 后仰 | 0.50 rad | 关节限位 0.52，指令再大也只是顶住限位 |
| `A` / `D` | 腰部左转 / 右转 | 0.60 rad | 实测 1.00 rad 仍稳定 |
| `Q` / `E` | 腰部左倾 / 右倾 | 0.40 rad | 关节限位 0.52，稳定 |
| `1` / `2` | 左肩前抬 / 放下 | **1.00 rad** | 1.20 起临界（重复实验中 0/3 ~ 2/3 跌倒），1.50 必倒 |
| `3` / `4` | 左肩外展 / 内收 | 1.40 rad | 稳定 |
| `5` / `6` | 左肘屈 / 伸 | 1.80 rad | 稳定（关节限位 2.09） |
| `7` / `8` | 右肩前抬 / 放下 | **1.00 rad** | 同左肩 |
| `9` / `0` | 右肩外展 / 内收 | 1.40 rad | 稳定 |
| `-` / `=` | 右肘屈 / 伸 | 1.80 rad | 稳定 |
| `R` | 上半身姿态复位 | — | |
| `I` / `K` | 速度指令 $v_x$ ±0.1 m/s | −0.3 ~ +0.5 | 通道通，策略不响应（见 5.3） |
| `J` / `L` | 速度指令 $\omega_z$ ±0.1 rad/s | ±0.4 | 同上 |
| `空格` | 速度指令清零 | — | |
| `X` | 退出 | — | |

上半身动作会推动质心偏移，平衡基元实时补偿——这是本模块在仿真中可见、物理真实的演示内容。

> **指令变化速率比幅度更关键**。肩前抬目标从 0 直接跳到 1.50 rad 时 2/2 跌倒（1.3 s）；
> 同样 1.50 rad 若用 0.8 s 平滑过渡则 2/2 稳定跑满 20 s。键盘每次只改 0.20 rad，
> 属于准平滑，所以键位上限可以比「话题一次性跳变」的上限高一些。用 `ros2 topic pub`
> 直接下发大角度时请先小步过渡。

### 4.3 训练 / 导出 / 评价 / 录屏

```bash
# 训练（默认 300 万步、8 并行环境，权重与曲线写到 saved_models/stage1/）
python3 main.py --train --timesteps 3000000 --n-envs 8

# 导出 TorchScript（ROS 2 推理节点用，无需 SB3 依赖）
# 仓库已自带 weights/policy_g1.pt，只有重训后才需要重跑这一步
python3 main.py --export --weights saved_models/stage1/ppo_g1.zip --out weights/policy_g1.pt

# 评价：跌倒率、速度误差、推理延迟（直接读仓库自带的 .pt，不必先训练）
python3 main.py --eval --weights weights/policy_g1.pt --episodes 5 --vx 0.0

# 录制演示动图（离屏渲染，无需显示器；--upper-demo 播放上半身动作序列）
python3 main.py --gif --weights weights/policy_g1.pt --upper-demo \
    --out docs/img/humanoid/humanoid_ppo_locomotion_demo.gif
```

仓库里入库的只有 `weights/policy_g1.pt`（187 KB，TorchScript，评测与 ROS 2 推理共用同一份）。
`saved_models/stage1/training_log.csv` 是训练曲线，`saved_models/eval.csv` 是评测明细。
Stable-Baselines3 存档（`saved_models/stage1/ppo_g1.zip`，1.1 MB，其中 719 KB 是 Adam 优化器
状态）**不入库**，只在你自己重训、需要 `--resume` 续训时才有意义。

## 5. 控制架构：为什么是「解析基元 + 神经网络残差」

### 5.1 只用神经网络（纯 PPO）为什么不行

G1 的踝关节 PD 刚度 $K_p = 40\ \mathrm{N\cdot m/rad}$，而倒立摆的失稳刚度是

$$m g h \approx 35.11 \times 9.81 \times 0.80 \approx 275\ \mathrm{N\cdot m/rad}$$

踝关节最大力矩只有 $\pm 50\ \mathrm{N\cdot m}$（刚好撑住约 10° 倾角）。因此**只给固定目标角的
PD 控制在数学上不可能让机器人站住**：踝刚度永远小于失稳刚度，而且力矩早就饱和了。

实测完全印证这一点：

| 踝关节 $K_p$ | 40 | 80 | 120 | 200 | 320 |
|---|---|---|---|---|---|
| 站立时长 | 1.86 s | 1.28 s | 1.12 s | 1.02 s | 1.02 s |

增益越大反而越差（因为力矩饱和后只剩硬碰硬的振荡）。失稳形态也很典型：`roll`/`yaw` 恒为 0，
`pitch` 从 2° 单调漂到 50°，质心前移 0.6 m —— 纯粹的倒立摆前倾，不是关节塌陷。

纯 PPO 同样打不过这个物理约束：训练 150 万步后，确定性策略只能站 **2.69 s**
（随机采样 6.22 s，零动作基线 1.28 s）。原因是回合在 1.2 s 内就终止，随机探索根本来不及
学到有用的步态。

### 5.2 解决方案

把「不摔倒」从策略里剥离出来，做成解析的**平衡基元**；神经网络只输出**残差偏置**：

$$
\begin{aligned}
q^{\text{target}} =\; &\underbrace{q^\star + \Delta q_{\text{balance}}}_{\text{解析基元}}
  \;+\; \underbrace{s_r \cdot a}_{\text{神经网络残差}}
\end{aligned}
$$

平衡基元用质心位置/速度反馈：

$$\Delta q_{\text{balance}} = K_c\, e_{\text{com}} + K_v\, \dot{x}_{\text{com}}$$

其中 $e_{\text{com}}$ 是质心相对**双脚中心**的前向偏差（平移不变，不是世界系绝对坐标），
修正量按 1 : 0.5 分配到 `ankle_pitch` 与 `hip_pitch`。

**符号很重要**：`ankle_pitch` 正向 = 身体后仰，所以前倾时必须给**正**修正；写成 $-K_c e_{\text{com}}$
会变成正反馈，越修越倒。

力学上这构成一个良性分工：神经网络摆动腿使双脚中心前移 → $e_{\text{com}}$ 变负 → 基元自动
把身体前倾跟上 → 理论上就是行走。基元管平衡，网络管步态。

效果对比（5 回合，每回合上限 20 s）：

| 配置 | 站立时长 | 说明 |
|------|----------|------|
| 零动作（纯 PD） | 1.30 s | 无任何反馈 |
| 纯 PPO 150 万步（确定性） | 2.69 s | 旧架构 |
| 纯 PPO 150 万步（随机采样） | 6.22 s | 旧架构 |
| **平衡基元 + 零残差** | **20.00 s（5/5 跑满）** | 本模块 |
| 平衡基元 + 随机残差 ±0.15 rad | 17.65 s（3/5 跑满） | 训练初期探索量级 |
| 平衡基元 + 随机残差 ±0.30 rad | 2.34 s | 残差过大反而盖过基元 |

> ±0.30 rad 一档回合间波动较大（1.9 ~ 2.4 s），`scripts/smoke_balance.py` 复现时可能略有出入。

容器内 ROS 2 实测（`teleop:=false`）：`/odom` 的 z 在 t=0 为 **0.771 m**，t=+6 s 为
**0.769 m**，发布抬臂指令后再测为 **0.762 m**，无累积漂移。

### 5.3 为什么没有实现行走（如实记录）

上述分工**只在双脚支撑时成立**，抬脚之后就不成立了。

1. **策略收敛到「站着不动」的局部最优。** 奖励里速度项权重 1.5，而姿态项 + 存活项在站立时已能
   拿满；迈步一旦失稳就全丢。PPO 在这种风险结构下必然选择不动。评测也印证了：
   `--vx 0.3` 时速度 MAE 恰好等于 0.3（即原地不动），`--vx 0.5` 时 MAE 等于 0.5。
2. **平衡基元在单腿支撑相是负作用的。** 双脚着地时抬高踝关节确实能把质心推回去；抬起一只脚后
   支撑多边形退化成一条线，同一个反馈律会把身体推向没有支撑的一侧。实测：给手写步态叠加开环
   侧向重心转移，站立时长从 1.4 s 掉到 **1.0 s**，倾倒方向变成左右交替。
3. **增益扫描无可行解。** 在 $K_c \times$ 髋分配比 $\times$ 残差幅度共 18 组组合上扫描，
   判据是「站得住（≥15 s）且抬脚 ≥5 cm」：11 组站得住但抬脚全部 ≤ **1.8 cm**（等于没抬），
   7 组抬得起脚但 1.4 s 内跌倒。**两个条件没有交集**，不是调参能解决的。

结论：双足行走需要单腿支撑相的重心轨迹规划（LIPM/MPC 落脚点规划）或数千并行环境的采样量级
（Isaac Gym 级别），本次预算（8 环境 CPU）做不到。因此交付范围收敛为**站立平衡 + 上半身键盘控制**。
下一步的可行方向是用 LIPM 落脚点规划器替换解析基元，再让 PPO 学残差。

## 6. 算法细节

### 6.1 观测（40 维）

| 分量 | 维度 | 说明 |
|------|------|------|
| 投影重力 | 3 | 机体系下的重力方向，等价于姿态 |
| 基座线速度 | 3 | **机体系** m/s |
| 基座角速度 | 3 | rad/s |
| 基座高度偏差 | 1 | $h - h^\star$，$h^\star = 0.78$ m |
| 关节角偏差 | 12 | $q - q^\star$ |
| 关节角速度 | 12 | rad/s |
| 速度指令 | 2 | $(v_x^\star, \omega_z^\star)$ |
| 步态相位 | 2 | $(\sin 2\pi\phi, \cos 2\pi\phi)$ |
| 质心偏差 | 1 | $e_{\text{com}}$，相对双脚中心 |
| 质心速度 | 1 | $\dot{x}_{\text{com}}$ |

### 6.2 动作（12 维，残差）

网络输出 $a \in [-1,1]^{12}$，作为残差叠加到平衡基元输出上（$s_r = 0.35$ rad）：

$$q^{\text{target}} = \text{ComBalance}(q^\star, e_{\text{com}}, \dot{x}_{\text{com}}) + 0.35\,a$$

关节力矩由 PD 律给出：

$$\tau = K_p (q^{\text{target}} - q) - K_d \dot{q}$$

$K_p$、$K_d$ 按关节类型取值（髋 100/2、膝 150/3、踝 40/1）。

> 初始动作标准差设为 $\sigma_0 = e^{-1.5} \approx 0.22$（对应残差 ±0.078 rad），
> 落在基元能兜住的 ±0.15 rad 以内，保证训练初期回合不会秒结束。

### 6.3 奖励

$$
\begin{aligned}
r =\;& \underbrace{1.5e^{-(v_x - v_x^\star)^2/0.25}}_{\text{线速度跟踪}}
  + \underbrace{0.8e^{-(\omega_z - \omega_z^\star)^2/0.25}}_{\text{角速度跟踪}} \\
&+ \underbrace{0.5e^{-5\|\mathbf{g}_{xy}\|^2}}_{\text{姿态}}
  + \underbrace{0.5e^{-20(h-h^\star)^2}}_{\text{高度}} + 0.2 \\
&- 10^{-4}\|\boldsymbol\tau\|^2 - 0.01\|a_t - a_{t-1}\|^2
  - 0.5|v_y| - 0.3\,e_{\text{com}}^2
\end{aligned}
$$

### 6.4 PPO 目标

$$
\begin{aligned}
L^{\text{CLIP}} = \mathbb{E}_t\Big[\min\big(&r_t(\theta)\hat A_t,\\
  &\text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon)\,\hat A_t\big)\Big]
\end{aligned}
$$

其中 $r_t(\theta) = \pi_\theta(a_t|s_t) / \pi_{\theta_{\text{old}}}(a_t|s_t)$，$\epsilon = 0.2$，
优势估计采用 GAE($\lambda=0.95$)。

### 6.5 课程学习

| 阶段 | 指令范围 | 目的 | 实际结果 |
|------|----------|------|----------|
| stage0 | $v_x^\star = 0,\ \omega_z^\star = 0$ | 原地站立 | 平衡基元已覆盖，跳过 |
| stage1（默认） | $v_x^\star \in [-0.3, 0.5]$ | 尝试低速行走与转向 | **站住了，但没走**（见 5.3） |
| stage2 | $v_x^\star \in [-0.3, 1.0]$（`--vx-max 1.0`） | 扩展到全速域 | 未进行 |

实际只跑了 stage1（300 万步）。

## 7. ROS 2 接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 发布 | 29 个关节角与角速度 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 基座位姿与速度（twist 在 base_link 下） |
| `/humanoid/obs` | `std_msgs/Float64MultiArray` | 发布 | 40 维观测 |
| `/humanoid/joint_targets` | `std_msgs/Float64MultiArray` | 订阅 | 12 维**残差动作**（-1~1，非最终关节角） |
| `/humanoid/upper_body` | `std_msgs/Float64MultiArray` | 订阅 | 17 维上半身关节目标角（键盘节点发布） |
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 速度指令 |
| TF `odom→base_link` | — | 发布 | 由 `publish_tf` 参数控制 |

数据流：

```
keyboard_teleop --(17维上半身角)--> /humanoid/upper_body --> sim_node
keyboard_teleop --(vx, wz)-------> /cmd_vel -----------> sim_node --(40维obs)--> policy_node
                                                                  |                    |
                                                                  |      (12维残差 a)  |
                                                                  v                    v
                                              q_target = ComBalance(...) + 0.35·a <-- /humanoid/joint_targets
                                                        |
                                                        v
                                              PD 力矩 --> MuJoCo
```

平衡基元在 `sim_node` 侧执行，`policy_node` 只负责神经网络推理，两者通过话题解耦，
便于单独替换或调试。上半身 17 关节由 `sim_node` 直接跟踪 `/humanoid/upper_body`，不经神经网络。

## 8. 性能指标

PPO 训练 300 万步（8 并行环境、50 Hz、CPU 约 19 分钟）。回合长度上限 1000 步 = 20 s：

| 训练步数 | 0.49 M | 0.80 M | 1.18 M | 2.03 M | 2.99 M |
|----------|--------|--------|--------|--------|--------|
| 回合长度（步） | 359 | 170 | **1000** | 1000 | 1000 |
| 回合回报 | 758 | 317 | 2425 | 2472 | 2713 |

> 0.80 M 处有一次策略崩塌（170 步），之后自行恢复。旧架构（纯 PPO）训练 150 万步只到 393 步。

评测（确定性策略，每档 5 回合）：

| 指令 $v_x$ | 存活 | 跌倒率 | 平均位移 | 平均 $v_x$ | 速度 MAE | 单帧推理 |
|------------|------|--------|----------|-----------|----------|----------|
| 0.0 m/s | 20.00 s | 0% | −0.044 m | −0.002 m/s | **0.013 m/s** | 0.10 ms |
| 0.3 m/s | 20.00 s | 0% | −0.011 m | −0.000 m/s | 0.301 m/s | 0.10 ms |
| 0.5 m/s | 20.00 s | 0% | +0.005 m | +0.000 m/s | 0.500 m/s | 0.10 ms |

结论：**站立平衡达标**（20 s 跑满、零跌倒、20 s 内漂移 < 5 cm）；**速度指令未被跟踪**
（MAE 等于指令值本身），原因见 5.3 节。

复现命令：

```bash
python3 main.py --eval --weights weights/policy_g1.pt --episodes 5 --vx 0.0
```

> 上表的推理延迟是 TorchScript 路径（0.10 ms）。用 SB3 存档评测时框架开销更大，
> 同一策略约 0.25 ms；ROS 2 的 `policy_node` 走的是 TorchScript 路径。

### 8.1 上半身幅度标定

加载训练好的策略，固定上半身目标角跑 20 s，3 个种子统计跌倒次数（用 `G1WalkEnv` 离线环境，
与 `scripts/smoke_balance.py` 同源）：

| 动作（双侧同时） | 目标角 | 跌倒 | 实际到位 |
|------------------|--------|------|----------|
| 静止 | — | 0/3 | — |
| 腰部俯仰 | 0.50 rad | 0/3 | 0.53（顶到关节限位） |
| 腰部偏航 | 0.60 rad | 0/3 | 0.61 |
| 腰部偏航 | 0.80 rad | 0/3 | 0.80 |
| 腰部侧倾 | 0.15 rad | 0/3 | 0.52（顶到关节限位） |
| 肩前抬 | 1.00 rad | 0/3 | 0.96 |
| 肩前抬 | **1.20 rad** | **2/3** | 1.14 |
| 肩前抬 | 1.50 rad | 3/3 | 1.47 |
| 肩前抬 | 1.80 rad | 3/3 | 1.83 |
| 肩外展 | 1.40 rad | 0/3 | 1.31 |
| 屈肘 | 1.80 rad | 0/3 | 1.84 |
| 腰部俯仰 0.50 + 屈肘 1.50 | — | 0/3 | 1.53 |

由此得到两条结论：**肩前抬 1.0 rad 是稳定上界**（超过 1.2 就开始倒）；腰部俯仰/侧倾的
可动范围本来就只有 ±0.52 rad，指令再大也只是顶住限位，所以腰部动作对平衡的威胁不大。

## 9. 已知问题

1. **必须加载 `scene_29dof.xml`，不能加载 `g1_29dof.xml`**。后者只是机器人本体，**不含地面**；
   用它仿真时机器人会一直自由落体，表现为「高度从 0.80 掉到 0.41、但姿态始终直立
   （$R_{22} \approx 1.0$）」，极易被误判成「站不稳」。`scene_29dof.xml` 才 `include` 了本体
   并加了 `<geom name="floor" type="plane"/>`。
2. **行走未实现**，只有站立平衡。诊断与证据见 5.3 节。
3. **控制频率必须 ≥ 50 Hz**。25 Hz 时 PD 与平衡基元都会失稳（实测只能站 0.36 s）。
4. **Windows 下路径不能含中文**：MuJoCo 的 XML 加载器无法打开含非 ASCII 字符的路径，
   需把模型复制到 `C:\g1_assets\g1` 之类的纯英文路径再运行。Ubuntu 下无此问题。
5. **CPU 训练较慢**：8 并行环境下约 1.6k steps/s，300 万步约 19 分钟。
   本仓库提供训练好的权重，评审机无需重新训练即可复现演示。
6. **仿真与真实机器人存在差距**：未做域随机化，策略仅在 MuJoCo 中验证，未上真机。
7. **平衡基元抗冲击较弱**：0.5 m/s 的外力冲击即会倒地（纯质心反馈没有迈步恢复能力），
   这正是需要神经网络残差学习步态的原因。
8. **上半身指令不能跳变**：一次性把肩前抬从 0 跳到 1.5 rad 会在 1.3 s 内跌倒，同样的目标用
   0.8 s 过渡就稳。键盘每次 0.20 rad 属于准平滑，安全；用话题直接下发大角度时需自行加过渡。

## 10. 许可证

本模块代码采用 BSD-3-Clause。G1 模型资源版权归 Unitree Robotics，按其 BSD-3-Clause 许可证使用。
