# 人形机器人深度图姿态估计与比例平衡控制

> 课程任务②：深度相机感知 + 运动控制 —— 深度图 → MLP 回归 3D 躯干姿态 → 比例平衡控制

## 功能概述

从 `front_camera` 的深度缓冲渲染 64×64 深度图，用纯 NumPy 实现的三层 MLP
（256→128→64→3）回归躯干 pitch/roll/yaw 三个连续姿态角，再由比例控制器输出
15 维关节修正目标，交给关节 PD 执行。

与 RGB 灰度图分类（5 类跌倒方向）的区别：本模块用**深度图**做**连续姿态回归**，
输出是角度而不是离散类别。

## 快速开始

```bash
pip install -r requirements.txt

python3 main.py --check                      # 自检（用临时数据，不覆盖正式数据集）
python3 main.py --collect --samples 1500     # 采集深度图 + 姿态标签
python3 main.py --train                      # 训练并回存验证集最优权重
python3 main.py --demo --force 40 --gif demo.gif   # 仿真演示，逐帧真实渲染录 gif
```

ROS 2（Ubuntu 22.04 + Humble）：

```bash
colcon build --packages-select humanoid_depth_pose_control
source install/setup.bash
ros2 launch humanoid_depth_pose_control main.launch.py
```

## 实现要点

### 站姿与平衡层

站姿目标按 `ankle ≈ -(hip + knee)` 配平后网格标定：`hip=0.16 / knee=-0.08 /
ankle=0.20`。旧版 `0.08 / -0.16 / -0.08` 不满足配平关系，无外力下 1.4 s 内自身
前倾 86.9° 倒地。

`reset()` 用贴地的 `INIT_QPOS` 起手，不直接把站姿目标写进 qpos（那样脚会悬空
3~4 cm，落地冲击直接推倒）。

纯关节 PD 是开环的，这个模型 1.5 s 起就慢漂，3 s 最大倾角 59.8°。`sim.step()`
默认叠加一层 ankle strategy 平衡反馈：

```
u = BALANCE_KP * (com_x - foot_x) + BALANCE_KD * v_x      # 0.5 / 0.2
ankle_pitch_l += u,  ankle_pitch_r += u
v = BALANCE_KP * (com_y - foot_y) + BALANCE_KD * v_y
ankle_roll_l -= v,  ankle_roll_r += v
```

加上后 3 s 无扰动最大倾角 1.73°。

### 深度图

`render_depth()` 用 `mujoco.Renderer` + `render(depth=True)` 取真实深度缓冲，按
`vis.map.znear/zfar` 归一化到 [0,1]；仅当 MuJoCo 版本不支持 `depth` 参数时才回退
到 RGB 亮度（现代版本走不到这条路）。16×16 块平均池化成 256 维输入。

### 网络

```
64×64 深度图 → 16×16 平均池化 → 256 维
  → W1(256×128) ReLU → W2(128×64) ReLU → W3(64×3)
  → (pitch, roll, yaw)
```

损失是 MSE，`d_out = 2(pred-y)/n`，动量 `m = 0.9m + 0.1g`，学习率余弦退火。训练
保存**验证集最优**那一轮，不保存最后一轮。

### 控制器

`q_target = q_stand + diag(k) θ`，腿部只做 pitch/roll 修正，yaw 只驱动颈部。

增益按实测标定：`kp_ankle=0.2 / kp_hip=0.1 / kp_neck=0.3`。旧版 `2.0 / 1.5`
把无扰动站姿从 1.7° 拖到 40°，比不加控制还差 20 倍 —— 因为它和平衡层是两层同向
踝修正，等于把增益叠到超调。

## 实测性能

环境：Ubuntu 22.04.5、MuJoCo 4.0.0、Python 3.12.11、numpy 1.26.4（Xvfb 离屏渲染）。

| 指标 | 数值 |
|---|---|
| 训练样本 | 1500 帧深度图（80% 受推 60–200 N / 0–360°，20% 纯站立；丢弃 17 帧越界样本） |
| 标签覆盖 | pitch ±44°（中位 7°、90 分位 26°）、roll ±31°、yaw ±26° |
| 训练轮数 | 80 epoch，batch 64，lr 0.005（余弦退火到 0） |
| 验证集 | 300 帧，best val_mse = 0.008455，**RMS 5.27°**（best epoch 78/80） |
| 网络参数量 | 41,347 |
| 3 s 无扰动站姿 | 最大倾角 1.73°（开环 PD 对照为 59.8° 并倒地） |
| 抗推（0° 水平推力，demo 闭环） | 40 N：第 929 步跌倒（含 250 步站定共 2.44 s，施力后 1.86 s）；80 N：第 750 步（共 2.08 s，施力后 1.50 s） |
| 自检 | `python3 main.py --check` 7/7 PASS |
| 测试 | `python3 -m pytest test/ -q` 6 passed |

两个口径必须分开说，否则数字会被读高：

- **5.27° 是在选 best 权重所用的那个 val 划分上算的**（`seed=0`、前 20% = 300 帧），
  即 select-on-val，本身偏乐观。换成任意独立划分（`seed=1/2/3`）分别是
  5.63° / 5.99° / 5.75°，全量 1500 帧是 5.70°。恒输出 0 的零基线是 7.59°，
  所以真实泛化大约 5.7~6.0°，相对零基线只改善约 25%。
  误差按轴拆：pitch 8.32°、roll 2.76°、yaw 2.54° —— 主要误差集中在 pitch。
- **抗推步数与秒数不是同一个起点**。表里的步数是 demo 闭环里 `fallen_at_step`
  （不含前面 250 步站定），括号里的秒数是包含站定的总时间。40 N 的 929 步
  可三次精确复现；80 N 在 0° 方向实测是 750 步，原先文档写的 768 步无法复现。
  抗推对方向很敏感（80 N 下 0° 750 步、45° 835 步、90° 1428 步、180° 746 步），
  所以必须连方向一起写。

覆盖率换来的精度代价是实打实的：早期只覆盖 ±23° 的版本能到 2.42° RMS，但那个模型
在演示里一倾斜就失效。现在 5.27° 是覆盖到 44° 之后的结果。

## 已知局限

- 超过 45° 的倾角没有进训练集（采集时就按这个上限丢弃），实测跌倒过程中
  （倾角 53° 以上）预测值仍停在 0.1 rad 量级，**大倾角下预测退化**，演示里能看到的
  pitch 与实际倾角不再是同一个量级。
- 抗推力弱：40 N 持续推力约 1.9 s 后倒地。只有 ankle strategy，没有髋策略和
  迈步恢复，也没有足底压力/IMU 反馈。
- yaw 在直立附近接近 0，倾倒时跨过 ±90° 会进万向锁，采集阶段已按 ±90° 丢弃，
  所以 yaw 回归只在未倾倒区间有效。
