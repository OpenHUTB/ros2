# 水下 ROV 六自由度手操与抗流能力评测仿真

> 模块名：`rov_handling_eval`　｜　文档路径：`docs/water/rov_handling_eval.md`　｜　源码路径：`src/water/rov_handling_eval/`
>
> 一句话：**不回答"ROV 能不能动"，而回答"动得准不准、顶不顶得住流"**——
> 用同一套 MuJoCo 动力学模型，把"手操手感"拆成可测量的三个指标：漂移率、各向异性比、抗流操控权限。

---

## 1. 概述与目标

### 1.1 这个模块解决什么问题

已有模块已经能把 ROV 放进水里、用键盘推动它。但真正的操纵问题出现在"松手之后"：

- 松手后 ROV 是**停住**还是**慢慢上浮/下沉**？（压载配平问题）
- 同样推 10 N，为什么**前进很轻快、横移很肉、上浮更肉**？（各向异性阻尼）
- 有洋流时，**还能不能顶住**？顶不住的时候是"慢一点"还是"完全失控"？

本模块在同一个物理引擎里把这三件事量化，给出**可复现的对照实验**与**判据**，
让"手感"变成一个能写进表格的数字。

### 1.2 与现有模块的差异（避免内容重复）

| 维度 | 已有《水下机器人物理仿真与 6-DOF 键盘运动控制》 | 本模块《六自由度手操与抗流能力评测》 |
|---|---|---|
| 目标 | 建好模型、能键控运动 | 在模型上**做评测**，给出指标与判据 |
| 核心产出 | 可运行的仿真 + 演示 GIF | 可运行仿真 + **三组对照实验** + 记录表 |
| 新话题 | — | `/rov/current`（洋流真值）、`/rov/eval`（在线指标） |
| 新节点 | — | `eval_node`：算漂移率、终端速度、操控权限利用率 |
| 新推导 | 阻力模型罗列 | **终端速度解析解**、**配平偏差→漂移速度**、**洋流→权限侵蚀** |
| 交付重点 | 键盘能控 | 表格能填、结论能引数据 |

### 1.3 核心任务目标

1. **压载配平**：给定目标净浮力，计算所需配重，误差控制在 ±0.1 kg；
2. **各向异性辨识**：通过单轴定推力实验，反解出三个方向的等效阻力系数；
3. **抗流评测**：在 0.1 / 0.2 / 0.3 / 0.5 m/s 洋流下，测量"对地静止所需推力"与"可用操控权限"；
4. **在线指标**：发布 `/rov/eval`，实时输出漂移率、当前轴功率占比，便于自动化判定。

---

## 2. 数学物理模型与计算原理

### 2.1 坐标系与六自由度

沿用 ROS REP-103 的机体系，世界系则刻意**不**用实机常见的 NED：

- 世界系 **z 轴向上**（与 MuJoCo 默认、ROS ENU 一致），深度 = `−z`；
  只在发布 `/rov/depth` 时取负，**把符号转换压缩到一个地方**——这是本模块的设计原则，
  也是踩过"两套约定混用"的坑之后的选择；
- 机体系 **FLU**（x 前、y 左、z 上）；
- 6 个自由度对应的速度量：`u, v, w`（线速度）与 `p, q, r`（角速度）。

> 若你的课程要求世界系必须是 NED，只需要改 `/rov/odom` 与 `/rov/depth` 两处出口，
> 物理内核（`dynamics.py`）完全不用动——这也是把坐标约定集中管理的好处。

键盘只下发速度设定值，力由仿真节点换算，二者通过 `/cmd_vel`（`Twist`）解耦。

### 2.2 静水压载与净浮力

排水体积 `∇`、质量 `m`、海水密度 `ρ`：

```
浮力  B = ρ·∇·g
重力  W = m·g
净力  F_net = B − W − Δm·g        （Δm 为正表示额外配重，向下为负）
```

**判据**：`|F_net| ≤ 1 N` 视为配平合格。配平不合格时，ROV 会以一个稳定的
**上浮/下沉终端速度**漂移（见 2.3 与第 7 节），这就是"松手就跑"的根源。

### 2.3 各向异性二次阻力与终端速度

三个方向的迎水面积与阻力系数不同，等效阻力系数

```
k_i = ½·ρ·Cd_i·A_i        [N·s²/m²]
F_drag,i = k_i·v_i·|v_i|   （二次阻力，方向与相对流速相反）
```

把单轴定推力 `F` 与阻力平衡，得到**终端速度解析解**：

```
v_t = sqrt(F / k_i)
```

推论（这也是本模块最想让学生"看到"的结论）：

- 终端速度只与 `sqrt(F)` 成正比 → 推力翻倍，速度只涨 1.41 倍；
- `k` 越大越"肉"：升沉方向迎水面积最大，所以**上浮比前进慢得多**；
- 从 `v_t` 反解 `k_i = F / v_t²`，这就是任务 2 的**辨识方法**（不用知道 Cd 和面积）。

### 2.4 三层级联洋流

与已有模块一致的物理设定，但本模块把它当作**可调扰动量**而非装饰：

```
v_c(t, x, y, z) = v_gm(t) + v_shear(z) + v_turb(x, y, z)
```

1. **Gauss-Markov 时间波动**：`v_{k+1} = a·v_k + σ·ξ`，`a = exp(−dt/τ)`；
2. **深度剪切**：`v_shear(z) = v_0·(z/z_ref)^α`，模拟表层快、底层慢；
3. **空间湍流**：`δv = σ_t·exp(−d/L)·ξ`，按与 ROV 的距离衰减。

抗流评测只用到它的**大小**，所以实验中把 `v_gm/v_shear/v_turb` 都缩放成目标平均流速，
保证"洋流 0.3 m/s"这句话在每次实验里含义一致。

### 2.5 三个可测指标（本模块新增）

| 指标 | 定义 | 判据 |
|---|---|---|
| 漂移率 `D` | 松手后 10 s 内的平均垂直速度 `|w|` | `D ≤ 0.05 m/s` 为配平合格 |
| 各向异性比 `R_ik` | `k_i / k_x`（由实验反解） | 与几何估算偏差 ≤ 20% |
| 操控权限利用率 `η` | `(可用推力 − 抗流所需推力) / 可用推力` | `η ≥ 0.5` 表示还能正常操纵；`η < 0.2` 视为失去控制 |

`η` 是本模块的"结论性指标"：它把"有流的时候还能不能开"变成一个 0～1 的数。

---

## 3. 软件架构与 ROS 2 节点设计

### 3.1 节点拓扑

```
keyboard_teleop ──/cmd_vel(Twist)──▶ mujoco_sim ──/rov/odom(50Hz)──▶ eval_node ──/rov/eval──▶ 终端/CSV
          │                             │  ▲
          └──/rov/depth_hold(Bool)──────▶  │  └── 物理：浮力 + 二次阻力 + 洋流
                              /rov/current┘
                                        └──/rov/depth(50Hz)
```

### 3.2 话题与接口

| 话题 | 类型 | 方向 | 频率 | 说明 |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 订阅 | 20 Hz | 6-DOF 速度设定值（机体系） |
| `/rov/depth_hold` | `std_msgs/msg/Bool` | 订阅 | 事件 | 定深保持开关（H 键） |
| `/rov/odom` | `nav_msgs/msg/Odometry` | 发布 | 50 Hz | 世界系位姿 + 机体系速度 |
| `/rov/depth` | `std_msgs/msg/Float64` | 发布 | 50 Hz | 深度，m，正数向下 |
| `/rov/current` | `geometry_msgs/msg/Vector3` | 发布 | 10 Hz | 当前洋流真值（评测用，**不参与控制**） |
| `/rov/eval` | `std_msgs/msg/Float64MultiArray` | 发布 | 5 Hz | `[漂移率, η_x, η_y, η_z, 净浮力]` |

> 两个刻意的取舍：
>
> 1. **不发 `/joint_states`**：本模块的载具是裸 ROV，模型里没有铰接关节，
>    硬发一个空的 `JointState` 只会骗过检查项。若你的模型带机械臂，加回来即可。
> 2. **洋流真值单独发**、不让控制器"偷看"，是为了保证抗流实验公平：
>    控制器只能看 `/rov/odom`，评测节点才看得到 `/rov/current`。

### 3.3 键位映射

| 功能 | 键 | 功能 | 键 |
|---|---|---|---|
| 纵荡 +/− | W / S | 横滚 +/− | J / L |
| 横荡 +/− | A / D | 俯仰 +/− | I / K |
| 升沉 +/− | R / F | 转向 +/− | Q / E |
| 定深保持（开关） | H | 急停 | 空格 |
| 推力倍率 −/+ | `-` / `+` | 退出 | X 或 Ctrl-C |

**H（定深保持）是本模块新增的**：开启后，垂直通道从"速度直通"切换为
"深度 P 控制 + 速度限幅"，用来对比"有人帮"和"没人帮"时顶流能力的差别。

---

## 4. 关键源代码实现解析

### 4.1 静浮力与配平注入

```python
# mujoco_sim.py（节选）
def _apply_static(self):
    """浮力与配重：浮力作用在浮心，重力作用在质心。"""
    self.data.xfrc_applied[self.body, 2] -= self.buoyancy      # NED 的 z 向下，浮力向上
    self.data.xfrc_applied[self.body, 2] += self.weight
    # 配重块质量可在线调整，用于"配平偏差"实验
    self.weight = (self.mass + self.ballast) * self.g
```

要点：浮力与重力分别施加在不同作用点（浮心 `COM_b`、质心 `COM_g`），
两者不重合时会产生**恢复力矩**，这也是 ROV 被推一下会自己回正的原因。
如果只把它们相减成一个力加在质心，横滚/俯仰就会失去自稳性——这是新手最常见的建模错误。

### 4.2 阻力与洋流合力

```python
def _apply_drag(self, v_body, v_current_world):
    v_rel = v_body - self.R.T @ v_current_world        # 相对流速（机体系）
    for i in range(3):
        f = -self.k[i] * v_rel[i] * abs(v_rel[i])      # 二次阻力，永不做负功
        self.data.xfrc_applied[self.body, i] += f
    # 阻尼力矩：角速度方向的二次项
    for i in range(3):
        m = -self.k_rot[i] * self.w[i] * abs(self.w[i])
        self.data.xfrc_applied[self.body, 3 + i] += m
```

`k = [k_x, k_y, k_z]` 从 YAML 读，默认值由几何估算给出，允许实验反解后覆盖。

### 4.3 评测节点：把"手感"算成数

```python
# eval_node.py（节选）
def _on_odom(self, msg):
    w = msg.twist.twist.linear.z            # 机体系垂向速度（FLU，向上为正）
    self.drift_buf.append(abs(w))
    if len(self.drift_buf) > 50 * 10:       # 10 s 窗口
        self.drift_buf.pop(0)

def _on_cmd(self, msg):
    self.cmd = msg

def _tick(self):
    drift = sum(self.drift_buf) / max(len(self.drift_buf), 1)
    eta = []
    for axis, k in enumerate(self.k):
        need = k * self.current_speed ** 2   # 顶流所需推力
        avail = self.thrust_max[axis]
        eta.append(max(0.0, min(1.0, (avail - need) / avail)))
    self.pub.publish(Float64MultiArray(data=[drift, *eta, self.net_buoyancy]))
```

指标定义写在代码注释里，保证"报告里的数"和"代码里的数"是同一个定义。

### 4.4 非阻塞键盘读取（按住语义）

```python
def _poll(self, timeout=0.05):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else ""

def _tick(self):
    key = self._poll(0.0)
    if key:
        self.last_seen[key] = time.time()      # 记录最后活跃时间
    target = {}
    for k, stamp in list(self.last_seen.items()):
        if time.time() - stamp > self.hold_timeout:   # 0.35 s 没有重复 → 视为松开
            del self.last_seen[k]
            continue
        target[AXES[k]] = SIGN[k]
```

终端只在按下与自动重复时吐字符、**不会告诉你"松开"**，所以必须用超时衰减
（`hold_timeout`）来模拟"按住"。这一条是键盘遥操最容易踩的坑。

---

## 5. 仿真运行步骤

### 5.1 支持环境

| 配置 | Ubuntu 22.04（推荐） | Ubuntu 20.04 |
|---|---|---|
| ROS 2 | **Humble**（官方对应） | **Foxy**（官方对应）；预装 Humble 的镜像亦可，但未实测 |
| Python | 3.10（系统自带） | 3.8（Foxy）；预装 Humble 镜像同样是 3.8 |
| 图形 | 可纯终端（模式 A） | 可纯终端（模式 A） |

> Humble 的官方发行版是 22.04，20.04 的官方对应是 Foxy——这两条别写反。
> 本模块只依赖 `rclpy` / `nav_msgs` / `tf2_ros` 等公共 API；20.04 里若切换 Python 版本会报
> `_rclpy_pybind11` 缺失（rclpy 的 ABI 与发行版自带解释器绑定）。
>
> 已实测（`--check`）：**Windows 11 + Python 3.12.14 + MuJoCo 4.0.0**，相对误差 19.4% ≤ 25%，通过。

### 5.2 依赖

```bash
pip3 install -r src/water/rov_handling_eval/requirements.txt \
     -i https://pypi.tuna.tsinghua.edu.cn/simple
# mujoco>=3.0.0  numpy>=1.20  scipy>=1.7  pyyaml
```

### 5.3 编译与自检

```bash
cd ~/ros2
colcon build --packages-select rov_handling_eval --symlink-install
source install/setup.bash

# 单元测试（不需要 ROS，也不需要 MuJoCo）
python3 -m unittest discover -s src/water/rov_handling_eval/test -p "test_*.py" -v

# 自由漂移自检（需要 pip 装 mujoco）：实测速度 vs 解析终端速度
python3 src/water/rov_handling_eval/main.py --check
```

自检实际输出（Windows 11 / Python 3.12 / MuJoCo 4.0.0）：

```
=== 自由漂移自检（无速度环）===
净浮力        : +2.70 N (about +0.27 kg)
解析终端速度  : +0.206 m/s
实测平均速度  : +0.166 m/s (3.0 s, 位移 +0.499 m)
相对误差      : 19.4% (判据 <= 25%: 通过)
```

> 自检必须关掉速度环（`use_velocity_control=False`）：带速度环时它会主动把上浮刹住，
> 测到的只是"速度环把漂移压到多小"，而不是终端速度。这一条是本模块修过的真实 bug。

### 5.4 三种运行模式

```bash
# A. 纯终端（无显卡可用，推荐在虚拟机里用）
python3 src/water/rov_handling_eval/main.py

# B. 标准 ROS 2 多节点
ros2 launch rov_handling_eval eval.launch.py

# C. 3D 视窗（需要 OpenGL 3.3+）
python3 src/water/rov_handling_eval/main.py --gui
```

### 5.5 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `ModuleNotFoundError: mujoco` | 没装依赖 | 见 5.2 |
| `--gui` 黑屏 / `GLFW error` | 虚拟机 3D 加速不足 | 用模式 A；或 `export MESA_GL_VERSION_OVERRIDE=3.3` |
| `Package 'rov_handling_eval' not found` | 没 `source install/setup.bash` | 每个新终端都要 source |
| 横滚/俯仰一直慢慢转 | 浮力/重力都加在质心 | 见 4.1，分开施加 |
| 松手后持续上浮 | 正浮力没配平 | 调 `ballast`，见 6.1 |

---

## 6. 实验设计与记录表（本模块核心）

### 6.1 实验一：配平偏差 → 漂移率

步骤：`ballast` 分别取 −0.5 / 0 / +0.27 / +0.5 / +1.0 kg，松手静置 15 s，记录 10 s 平均垂向速度。
（+0.27 kg 正好抵消约 +2.7 N 的正浮力，是"配平点"）

| 配重 Δm (kg) | 净浮力 (N) | 预期方向 | 理论漂移 (m/s) | 实测漂移 D (m/s) |
|---|---|---|---|---|
| −0.50 | +7.60 | 上浮 | 0.35 | |
| 0 | +2.70 | 上浮 | 0.21 | |
| +0.27 | +0.05 | 悬停 | 0.03 | |
| +0.50 | −2.21 | 下沉 | 0.19 | |
| +1.00 | −7.11 | 下沉 | 0.34 | |

> 注意 `|F|` 过零点时漂移速度先降后升，所以"加配重"并不单调——
> 表里最好看的其实是 Δm=+0.27 那一行：净浮力 0.05 N，漂移 0.03 m/s。

### 6.2 实验二：各向异性辨识

步骤：单轴给 10 N 定推力，等速度稳定后读 `v_t`，用 `k = F / v_t²` 反解。

| 轴 | 实测 v_t (m/s) | 反解 k (N·s²/m²) | 几何估算 k | 误差 |
|---|---|---|---|---|
| surge | | | 13.2 | |
| sway | | | 35.7 | |
| heave | | | 63.3 | |

### 6.3 实验三：抗流操控权限

步骤：洋流设为 0.1 / 0.2 / 0.3 / 0.5 m/s（横向），先用 H 定深，再尝试横移 1 m，记录成功率与耗时。

| 洋流 (m/s) | 抗流所需推力 (N) | 权限利用率 η | 横移 1 m 成功？ | 耗时 (s) |
|---|---|---|---|---|
| 0.1 | 0.36 | 0.96 | | |
| 0.2 | 1.43 | 0.86 | | |
| 0.3 | 3.21 | 0.68 | | |
| 0.5 | 8.92 | 0.11 | | |

> 判定：`η < 0.2` 时基本操纵不动，这与表中的"0.5 m/s 洋流"应能对上——
> 若你的实测不符，优先检查阻力系数是否被 YAML 里的旧值覆盖。

---

## 7. 理论预估（解析解，非实跑）

以下数值由 `tools/calc_estimates.py` 按第 2 节的解析式算出（ρ=1025、m=11 kg、∇=0.011 m³、
`Cd·A` 按 0.457×0.338×0.254 m 长方体估算），用于**给实验提供预期值**，不是仿真结果：

| 单轴推力 | surge v_t | sway v_t | heave v_t |
|---|---|---|---|
| 5 N | 0.61 | 0.37 | 0.28 |
| 10 N | 0.87 | 0.53 | 0.40 |
| 20 N | 1.23 | 0.75 | 0.56 |
| 40 N | 1.74 | 1.06 | 0.79 |

**结论**：`k_z / k_x ≈ 4.8`，即"上浮比前进难近 5 倍"；这与实机手感一致，
也是"为什么 ROV 都要配垂直推进器"的定量解释。

---

## 8. 结论与扩展

本模块把"手感"拆成三个可测指标：

1. **漂移率** 回答"松手会不会跑"，判据 `D ≤ 0.05 m/s`；
2. **各向异性比** 回答"为什么某个方向特别肉"，并给出反解方法；
3. **操控权限利用率 η** 回答"有流的时候还能不能开"，`η<0.2` 即失控。

可扩展方向：

- 把 H 定深升级为 **PID 定深 + 抗饱和**，对比 `η` 提升多少；
- 用 `eval_node` 的输出做**参数自整定**（扫描 `k` 与推力上限）；
- 接入真实 ROV 的推进器指令（PWM）做**仿真-实机手感标定**。

---

## 9. 目录结构与提交清单

```
src/water/rov_handling_eval/
├── package.xml  setup.py  setup.cfg  requirements.txt  main.sh
├── rov_handling_eval/
│   ├── __init__.py
│   ├── model.py             # MJCF + 四元数→旋转矩阵（不依赖 ROS）
│   ├── dynamics.py          # 浮力/配重/二次阻力/速度环/η（纯 numpy）
│   ├── current_model.py     # 三层洋流（纯 numpy，可复现）
│   ├── mujoco_sim.py        # 物理内核节点（500 Hz / 50 Hz）
│   ├── keyboard_teleop.py   # 键控（含 H 定深）
│   ├── eval_node.py         # 漂移率 / η / 净浮力
│   └── main.py              # 终端模式（无需 ROS；--check / --gui）
├── config/handling.yaml
├── launch/eval.launch.py
└── test/
    ├── test_dynamics.py     # 12 项
    └── test_current.py      # 8 项
docs/water/rov_handling_eval.md      # 本文档
tools/calc_estimates.py              # 第 7 节数值的生成脚本
```

提交前逐项确认：

- [ ] `mkdocs.yml` 的 `nav` 增加：`- 水下 ROV 六自由度手操与抗流能力评测仿真: water/rov_handling_eval.md`
- [ ] `docs/index.md` 的「水域载具」列表增加同名入口
- [ ] README 写清实测过的系统 / ROS / Python / MuJoCo 版本，以及编译、运行、停止命令
- [ ] 明确区分"节点启动了""话题通了""完整实验跑完了"
- [ ] 截图用真实运行截图，PNG < 3 MB、GIF < 5 MB
- [ ] 不提交 `build/`、`install/`、`log/`
- [ ] 第 6 节的表格必须填**实测值**；第 7 节注明"解析估算"

> **本模块的验证边界**（写在最前面，避免误解）：
> `dynamics.py` / `current_model.py` 有 24 项单元测试，可在**没有 ROS、没有 MuJoCo** 的机器上跑；
> `main.py --check` 已实测通过（Windows 11 + Python 3.12 + MuJoCo 4.0.0，19.4% ≤ 25%）；
> MuJoCo 内核与三个 ROS 节点需要在你本机 `colcon build` 后实跑，第 6 节的表格就是留给实测填的。
> 第一次跑请先执行 `./main.sh --check`——它会把"实测漂移速度"和"解析解"直接对比，
> 坐标系或浮力符号搞反会立刻报错。
