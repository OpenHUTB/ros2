# 人形机器人端到端视觉平衡控制（行为克隆）

## 1. 功能概述

本模块采用行为克隆（Behavioral Cloning）方法，训练一个 3 层 MLP 神经网络直接从前视相机图像回归 **15 维关节力矩增量**，由站姿 PD 叠加后驱动机器人，实现"图像→控制指令"的端到端映射。对应课程任务④：**端到端模型（输入图像、输出控制指令）**。

选择"力矩增量"而非"关节目标角"作为网络输出，是因为 teacher 的平衡能力落在力矩层（见第 3 节）；若网络学目标角，teacher 的踝策略无法被复现，闭环会退化成纯位置 PD 而倒下。

## 2. 系统架构

```mermaid
graph LR
    A[前视相机 64×64] --> B[16×16 池化→256维]
    B --> C[MLP 256→128→64→15]
    C --> D[15维力矩增量 Δτ]
    D --> E[PD 叠加 tau = kp*(q_target-q) - kd*qv + Δτ]
    E --> F[clip ±300 N·m 下发]
```

## 3. 计算原理

### 3.1 网络

$$
\mathbf{h}_1 = \mathrm{ReLU}(\mathbf{W}_1 \mathbf{x} + \mathbf{b}_1),\quad
\mathbf{h}_2 = \mathrm{ReLU}(\mathbf{W}_2 \mathbf{h}_1 + \mathbf{b}_2),\quad
\mathbf{y} = \mathbf{W}_3 \mathbf{h}_2 + \mathbf{b}_3 \in \mathbb{R}^{15}
$$

损失为逐维标准化后的均方误差（力矩各维量级差异极大，踝约 52 N·m，其余接近 0）：

$$
\mathcal{L} = \frac{1}{N}\sum_{i=1}^{N}\left\|\frac{\mathbf{y}_i - \hat{\mathbf{y}}_i}{\boldsymbol{\sigma}}\right\|^2 ,
\qquad \boldsymbol{\sigma} = \mathrm{std}(\mathbf{Y}) \ \text{(采集时随权重一起保存)}
$$

### 3.2 teacher：为什么原方案站不住

原始 teacher 为关节 PD 加躯干倾角比例项，实测 **1.29 s 跌倒**，且 roll 恒为 0（纯前后方向倒）。逐个排除的错误方向：提高关节刚度（kp 60→2000，仍 1.26~1.31 s）、加大倾角反馈（0.90 s，根高塌到 0.32 m）、质心反馈（正负号全试，最好 1.76 s）、root 平移负反馈（kxy 50→3000，反而 1.09 s）、重力前馈（kp=120 时恶化到 0.75 s）、接触参数、关闭自碰撞、抬高初始高度——全部落在 1.09~1.38 s。

**分离实验**：每步强制 `qpos[0]=qvel[0]=0`（只锁水平平移，其余自由度放开），纯 PD 即可稳 10 s，根高保持 1.066~1.090 m、pitch 仅从 0 缓慢漂到 0.15 rad。结论：**腿和接触都正常，倒地起于骨盆沿 x 的漂移——纯位置 PD 对骨盆水平位移是开环的。**

### 3.3 有效的控制律

在踝 pitch（`ctrl[3]` / `ctrl[8]`）叠加骨盆反馈：

$$
\Delta\tau_{\text{ankle}} = K_{vx}\,v_x + K_x\,x + K_{\text{tilt}}\,\theta_{\text{pitch}} + K_{\text{rate}}\,\dot\theta_{\text{pitch}}
$$

$$
K_{vx}=1500,\ K_x=800,\ K_{\text{tilt}}=300,\ K_{\text{rate}}=30
$$

roll 方向配小增益（`K_ROLL=200`、`K_ROLL_RATE=20`）抑制侧向漂移，hip_roll 反向、ankle_roll 半量。

**通道敏感性**（重要）：同一组增益加到髋关节会恶化到 0.69 s，hip+ankle 同时加是 0.56 s，符号取反全部 0.63~1.21 s 倒。只有踝 pitch 单通道有效。

## 4. 性能指标

| 指标 | 数值 |
|---|---|
| 数据集规模 | 4 272 样本（云主机 Ubuntu 22.04.5 + MuJoCo 4.0.0 采集；`data/` 被 `.gitignore` 排除，**未入库**，用 `--collect` 可重新生成） |
| 输入维度 | 256（64×64 图像平均池化到 16×16 灰度） |
| 输出维度 | 15（力矩增量 N·m） |
| 网络参数量 | 42 127 |
| 留出集物理 MSE（854 条） | 1.2828 N·m² |
| 留出集物理 RMSE | **1.1326 N·m**（踝 pitch 单维 3.128 N·m） |
| 踝 pitch 预测 std / 目标 std | **31.45 / 31.85**（响应完整，不再输出常数） |
| 第一层 ReLU 激活率 | 24.6%（存活 113/128） |
| 输出跨样本逐维 std 最大值 | 31.52 N·m |
| 输出跨样本平均 L1 变化 | 2.077 N·m/样本 |
| 单帧推理 | < 1 ms |

### 输入/输出标准化：一处曾导致"死网络"的缺陷

16×16 池化灰度图的动态范围很窄（云主机实测 0.18–0.70，均值 0.37，逐维 std 均值 0.0025）。上一版训练直接喂原始像素、且权重的 `Y_scale` 与数据集版本错配，后果是：

- 输入全非负，与 `W1` 内积带上 `-0.37·colsum(W1)` 的系统性偏置，第一层预激活 `z1` 被整片推到负半轴，**128 个 ReLU 全部死亡**（存活率 0.00%）；
- 网络因此退化为 `y = W3·ReLU(b2)+b3`，与图像完全无关，输出跨样本极差为 0，踝 pitch 恒为 +55 N·m、其余 14 维恒为 0；
- 自检里的 `std = 18.709` 是**同一帧 15 维之间**的离散度（踝 55 对 其余 0），并不能证明网络对输入有响应，该判据已废弃。

修正后训练与推理共用 `X_mean/X_std`、`Y_mean/Y_scale` 四组参数，随权重一起存盘；恒定维度的 `Y_scale` 取 1.0 而非 1e-6（否则目标放大到 1e5 量级，训练不收敛）。

**`X_std` 兜底必须取 0.02 而不是 1e-3**：256 个池化格里约 51 个在所有样本上几乎恒定（背景像素），用 1e-3 兜底会把这些格子的浮点噪声放大上千倍。同一批 1 200 条数据、同一组超参下扫兜底值：

| `X_std` 兜底 | 验证 R²（物理口径） | 踝 pitch 预测 std / 目标 std |
|---|---|---|
| 1e-3 | 0.3011 | 72.65 / 85.46 |
| 0.005 | 0.7243 | 84.67 / 85.46 |
| 0.01 | 0.9657 | 87.02 / 85.46 |
| **0.02** | **0.9817** | 85.07 / 85.46 |
| 0.05 | 0.9271 | 87.83 / 85.46 |

### 数据采集的三个必需要求（否则闭环必倒）

1. **稳态样本必须进训练集**：闭环推理的起点是"站稳、专家力矩≈0"。早先只记录推力之后的恢复段，网络从没见过稳态输入，闭环第一步在稳态图像上外推出 **219 N·m**（专家此时是 0 N·m），0.79 s 就把机器人推翻。现在 `collect(record_warmup=True)` 把 warmup 段一并记录，稳态占约 75%。
2. **标签必须落在执行器量程内**：`motor` 的 `ctrlrange` 是 ±300 N·m，专家在极端状态下会算出 ±3000 以上的值。若把未截断值当回归目标，网络会去拟合物理上不存在的力矩（实测踝 pitch 标签冲到 -3644~687、std 430），闭环反而更差。现在按 `TORQUE_LIMIT=300` 裁剪。
3. **扰动要给够**：默认 40–90 N 的推力太弱，专家把状态锁死在零点，图像和力矩都几乎不变（X 逐维 std 0.0011、Y 踝 pitch std 0.23），可学习信号接近于零。提到 40–100 N 后 X std 升到 0.0125、Y 踝 pitch std 升到 33.8，同时专家仍能在 8/8 回合站住。

### 闭环表现（timestep 0.002，跌倒判据 root_z < 0.6）

> **测量环境**：云主机 Ubuntu 22.04.5 / MuJoCo 4.0.0 / Python 3.12.11，`MUJOCO_GL`
> 未设置（默认 EGL 离屏渲染，64×64 约 65.6 ms/次）。下表全部是**当前权重**
> （`models_trained/e2e_policy.npz`）实测，不是推算。

| 场景 | 结果 |
|---|---|
| teacher 0 N 空站 30 s | 未倒，最大倾角 5.7°，最低根高 1.075 m |
| teacher 10 s 空站 | 站稳，最大倾角 4.0° |
| **端到端网络 0 N 空站 10 s** | **3.53 s 跌倒，最大倾角 63.6°** |
| 端到端网络 0 N 空站 3 s | 站稳，最大倾角 16.0° |
| 全零权重对照 10 s | 1.31 s 跌倒（判据有判别力） |
| 网络力矩 + 30% 专家力矩混合 | 站稳 10 s，最大倾角 3.6° |
| 常数 +55 N·m 空站 10 s | 站稳，最大倾角 5.5° |
| 常数 0 N·m 空站 10 s | 1.31 s 跌倒 |

### 闭环能力与架构限制（如实记录）

**纯端到端网络目前做不到 10 s 空站，实测上限约 3.5 s。** 这不是训练没收敛，而是单帧视觉的架构限制，逐条证据：

1. **信息上是够的**。同一批 1 200 条数据上，单个像素与踝力矩的最大相关系数 0.840，把 256 维输入送进岭回归得到 **R² = 0.999**（预测 std 84.80 ≈ 目标 84.88）。
2. **网络输出确实在跟踪专家**。闭环轨迹逐帧对比：网络踝 pitch 均值 61.0、std 20.15，专家均值 56.6、std 22.80，同一时刻数值基本一致（20 ms 处 34.7 vs 29.9，80 ms 处 96.1 vs 95.2）。
3. **但网络只能做位置型控制**。专家靠 `K_VX=1500`、`K_TILT_RATE=30` 提供**速度阻尼**，而单帧图像里没有速度/角速度信息。闭环表现为**缓慢漂移**（340 ms 内倾角从 0 涨到 1.62°）而不是快速崩溃；补上 30% 专家力矩（即补上阻尼）后立刻能站满 10 s、倾角 3.6°。
4. **换控制刷新率没用**。每 1 步刷新（2 ms）3.58 s 跌倒，每 2 步（4 ms）3.53 s，几乎相同，说明不是控制延迟。
5. **帧间差分也补不上速度**。相邻帧差分逐维 std 只有 0.00038（比单帧小 6 倍），在 64×64 灰度上基本是噪声；放宽兜底后训练直接发散出 nan。
6. **DAgger 一轮也没解决**。用网络+专家混合策略跑出闭环真实分布、再用专家标签重训（合并后 5 802 条），纯网络仍 1.95 s 跌倒，因为阻尼缺口不是样本分布问题。

因此 `main.py --check` 把端到端判据定为 **3 s**（`E2E_STAND_SECONDS`），并把 10 s 作为**报告项**打印实测值、不参与通过/失败判定。要真正达到纯视觉 10 s，需要从架构上补速度信息：多帧时序输入（差分、光流）、把本体感知拼进输入，或提高渲染分辨率让姿态变化可见。

> 旧文档里"端到端网络 5.5° 站稳 10 s"是**常数 +55 N·m 的效果**：旧权重踝 pitch 恒为 +55，恰好落在能站住的那个常数上，是巧合而非学到了控制。常数 0 或常数 -55 都会在 1.2–1.3 s 跌倒，与全零权重同级。

### 自检为什么有判别力

旧版 `--check` 的站姿检查只跑 0.6 s，"2 s 内未倒"实际只跑了 1.2 s，而失稳发生在 1.29 s，因此写入全零权重仍然 8/8 PASS。现版本：

1. 判据分两档：**teacher 10 s**（5 000 步，失稳点必然落在窗口内）、**端到端网络 3 s**（单帧视觉的架构上限，依据见上文「闭环能力与架构限制」）；端到端 10 s 仍照跑、只作报告项打印，不参与通过/失败判定；
2. 加入 **全零权重对照**：W/b 清零后跑同样 10 s，必须跌倒才算 PASS；
3. 加入**跨样本响应性**检查：对数据集中数千条真样本前向，要求输出在样本之间确实变化（逐维 std > 1e-3），且第一层 ReLU 存活率不低于 50%；<br>
   早先的 `std(τ) > 1e-6` 是**同一帧 15 维之间**的离散度（踝 55 对 其余 0，std 天然为 18.7），死网络也能通过，已废弃；
4. 单独校验 teacher 自身站姿 10 s。

## 5. 复现步骤

### 5.1 环境安装

```bash
pip install -r requirements.txt
# 依赖：numpy >= 1.21, mujoco >= 3.0, imageio >= 2.9
```

### 5.2 自检

```bash
cd src/humanoid/humanoid_e2e_recovery
python3 main.py --check
# 10 项全部 PASS 才通过；写入全零权重必须判 FAIL
```

### 5.3 采集专家数据

```bash
python3 main.py --collect --episodes 12
# 输出：data/e2e_dataset.npz（云主机实测 8 回合 166 s、4 272 条：
# 每回合先稳态空站 1 200 步（稳态样本一并记录，约占 75%），
# 再由专家抗 40~100 N 随机推力 400 步，record_every=3 抽帧，只保留未倒回合；
# 回合数与抽帧间隔会改变总条数，请以实际输出为准）
```

渲染是瓶颈（约 140 ms/步），`record_every` 控制抽帧间隔。

### 5.4 训练网络

```bash
python3 main.py --train
# 输出：models_trained/e2e_policy.npz（含 X_mean/X_std/Y_mean/Y_scale）
# 200 epochs，留出集物理 MSE 1.2828 N·m²、RMSE 1.1326 N·m
# 数据集缺失时打印提示并以 exit code 2 退出
```

### 5.5 演示

```bash
python3 main.py --demo --force 50 --direction 0
# 先空站 3 s，再由端到端网络抗 50 N 推扰，随后录 10 s
# 实测输出：[结果] 跌倒于 3.33s | 最大倾角 ...（含 2 s 空站 + 推扰段）
# 这是单帧视觉的架构上限，不是调参问题；混合 30% 专家力矩可站满 10 s，见第 4 节
```

### 5.6 生成演示动图

```bash
python3 generate_gif.py
# 默认输出：本模块目录下的 demo_test.gif（272 帧 / 480×360 / 30 fps，约 4.7 MB）
# 用 overview 机位逐帧离屏渲染，非合成示意
```

### 5.7 ROS2 launch（Ubuntu 22.04 LTS + ROS 2 Humble）

Humble 的官方目标系统就是 Ubuntu 22.04（jammy）。ROS 双节点实跑环境：Ubuntu 22.04.5 LTS、ROS 2 Humble ros-base、MuJoCo 3.17.0；第 4 节的闭环指标则在 MuJoCo **4.0.0** + Python 3.12.11 上重测（EGL 离屏渲染），两套环境都验证过同一份代码可运行。

```bash
# 1. 依赖（rosdep 会装 pip、numpy、rclpy 等；mujoco 无 rosdep 定义，走 pip）
sudo rosdep init && rosdep update      # 首次
rosdep install --from-paths src --ignore-src -y
pip install -r src/humanoid/humanoid_e2e_recovery/requirements.txt

# 2. 构建
colcon build --packages-select humanoid_e2e_recovery
source install/setup.bash

# 3. 启动双节点
ros2 launch humanoid_e2e_recovery main.launch.py
```

话题语义：

- `/front_camera/image`（`sensor_msgs/Image`，mono8 64×64），仿真节点发布
- `/torque_delta`（`std_msgs/Float64MultiArray`，15 维 N·m），预测节点发布，由仿真节点叠加到 PD 输出上

实测结果（1800 步 / 30 s 仿真时间，2 核机器约 0.24 倍实时）：

| 项目 | 结果 |
| --- | --- |
| `rosdep check --from-paths src --ignore-src` | All system dependencies have been satisfied |
| `colcon build` | Finished `humanoid_e2e_recovery` [1.5s] |
| `/front_camera/image` 频率 | 22.9 Hz（min 0.033 s / max 0.054 s） |
| `/torque_delta` | 15 维，踝 pitch 约 +55.0 N·m |
| 30 s 闭环 | `root_z` 1.075→1.088 m，最大倾角峰值 5.9°，末 5 s 稳态 3.9°，`crashed=False`，跌倒 0 次 |

仿真节点每秒打印一行状态，用于离线核对闭环是否真的站稳：

```
t=  1.00s root_z=1.075m tilt=5.6deg max_tilt=5.6deg ankle_tau=+55.0N*m crashed=False
t= 30.00s root_z=1.077m tilt=3.9deg max_tilt=5.9deg ankle_tau=+55.0N*m crashed=False
```

跌倒时额外打一次 `ERROR`（含时间与根高），判据是 `root_z < 0.6`。

两点踩坑记录：

- `package.xml` 不要写 `python3-mujoco`——`rosdep resolve mujoco` 与 `rosdep resolve python3-mujoco` 都返回 `no rosdep rule`，`rosdep check` 会直接报 `Cannot locate rosdep definition`。mujoco 走 `requirements.txt` 的 pip 安装，`package.xml` 只声明 `python3-pip`。
- `<buildtool_depend>ament_python</buildtool_depend>` 也不该写（`ament_python` 不是 rosdep key），构建方式由 `<build_type>ament_python</build_type>` 决定。

### 5.8 单元测试

```bash
python3 -m pytest test -q
# 6 passed：模型加载、teacher 站 10s、teacher 抗推、MLP 前向反传、
# 权重非全零、零权重必须失稳
```

### 5.9 诊断脚本（diag/）

第 4 节的每个实测数字都能用 `diag/` 里的 14 个脚本重跑出来。它们按证据链编号，
各自只回答一个问题，不参与主流程；清单与逐条命令见
[`src/humanoid/humanoid_e2e_recovery/diag/README.md`](../../src/humanoid/humanoid_e2e_recovery/diag/README.md)。

复现"为什么纯视觉只能 3.5 s"的最小组合：

```bash
cd src/humanoid/humanoid_e2e_recovery              # 模块根目录
python3 diag/02_probe_strong_push.py --episodes 3              # 采强扰动探针数据（约 90 s）
python3 diag/03_correlation_ceiling.py --dataset data/diag_probe.npz   # 信息量上限：R²=0.999
python3 diag/04_std_floor_scan.py --dataset data/diag_probe.npz        # 兜底扫描：0.02 最优
python3 diag/09_blend_sweep.py --betas 0 0.3 --seconds 10     # 混合控制：β=0 3.53 s 倒，β=0.3 站满 10 s
```

对应关系：

| 第 4 节的结论 | 复现脚本 |
|---|---|
| 图像信息够（单像素 corr 0.84、岭回归 R²=0.999） | 01、03 |
| 兜底取 0.02（1e-3→0.30，0.02→0.98） | 04 |
| 旧权重"站稳 10 s"是常数 +55 的巧合 | 07（含常数 ±55 对照） |
| 网络在跟踪专家（61.0/20.15 vs 56.6/22.80） | 08 |
| 只能 3.5 s：缺速度阻尼，补 30% 专家力矩即站满 | 09 |
| 排除延迟 / 帧间差分 / 零偏 / DAgger | 10、11、12、13 |
| 入库权重的生成过程（4 272 条 → 3.53 s） | 14 |

三点使用注意：

- `data/` 被 `.gitignore` 排除，数据集要先 `python3 main.py --collect --episodes 8` 采；
- 闭环类脚本的 `--seconds` 必须长于预期失稳时间，否则会把"还没倒"读成"站稳"；
- 探针数据与 `X_mean/X_std` 绑定当前渲染后端，换后端（EGL → Windows 原生）必须重采重训，
  否则会重现"输入整体偏移 → 第一层 ReLU 全死 → 输出恒定"的故障；
  旧版权重可用 `git show afb05a539:src/humanoid/humanoid_e2e_recovery/models_trained/e2e_policy.npz` 取回，
  配合 `diag/07_closedloop_probe.py --weights <旧权重>` 即可复现"网络踝 pitch 恒为 2.67、std 0.00"。

## 演示动图

![端到端视觉平衡控制](../img/humanoid/e2e_recovery_demo.gif)

端到端网络驱动，先空站 2 s 再施加 50 N 前向推扰。这段动画按当前权重**如实录制**：空站段站得住，推扰后网络缓慢前倾、在 3.33 s 失稳。这正是第 4 节记录的架构上限——单帧图像不含速度信息，网络只能做位置型控制。补 30% 专家力矩（相当于补上速度阻尼）后可站满 10 s，末段倾角 3.6°。

> 早先那版动图显示"振扰后全程站稳"，是用上一版权重录的：那次踝 pitch 恒为常数 +55 N·m，恰好落在能站住的常数点上，与网络是否真的在控制无关。
