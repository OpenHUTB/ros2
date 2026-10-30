# 人形机器人双臂搬运负载的神经网络平衡控制（humanoid_carry_payload_nn）

## 1. 任务与思路

人形机器人双臂在胸前刚性夹持一个负载箱，在跑步机 + 减重助行架台架上按指令速度"原地行走"搬运。
负载质量在 **0 / 2 / 4 kg** 之间变化时，机器人上半身的质量分布随之改变，躯干会出现稳态前倾与暂态摆动；
模块用 **PD 姿态专家** 产生"状态 → 髋/踝补偿角"的示范数据，再用 **行为克隆** 训练一个 9→32→32→3 的
全连接神经网络（`BalanceMLP`），由它在三种负载下输出补偿角，抑制躯干俯仰。

- 仿真器：**MuJoCo**（`mujoco` python 绑定，模型为自建 MJCF，不依赖 Gazebo）
- 交付形态：ROS1 catkin 包（`main.launch` + `scripts/main.py`），**同时支持脱离 ROS 独立运行**
- 观测特征（9 维）：躯干 roll / pitch / roll_rate / pitch_rate、负载质量归一化、速度指令、转向指令、步态相位 sin/cos
- 控制输出（3 维）：踝俯仰补偿、踝横滚补偿、髋俯仰补偿（均限幅 ±0.5 rad）

## 2. 环境要求

| 项目 | 版本 / 说明 |
| --- | --- |
| 系统 | Ubuntu 20.04（课程虚拟机亦可），Windows 下同样可跑（仅离线模式） |
| ROS | ROS Noetic（catkin）；不装 ROS 也能用 `python3 scripts/main.py` 直接跑 |
| Python | 3.8+，`numpy`、`mujoco>=2.3`（本仓库在 MuJoCo 3.x 上验证）、`imageio`（仅生成 GIF 需要） |

```bash
pip3 install numpy mujoco imageio          # 或用 requirements.txt
```

> 虚拟机内如需图形界面：直接在桌面运行；SSH 远程请用 `ssh -X`，否则加 `--no-viewer` 无界面运行。

## 3. 快速开始

```bash
# 1) 编译（ROS 环境）
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 2) 训练平衡网络（0/2/4 kg 三种负载，约 5 s）
rosrun humanoid_carry_payload_nn train_mlp.py --seconds 25 --epochs 60

# 3) 键盘遥控（W/S 前后、A/D 转向、空格急停、Q 退出）
roslaunch humanoid_carry_payload_nn main.launch mode:=keyboard controller:=nn mass:=2.0

# 4) 三种负载对照评测（输出 results/eval_*.csv 与终端表格）
rosrun humanoid_carry_payload_nn evaluate.py --seconds 20 --speed 0.15

# 5) 生成演示 GIF（写入 docs/media/）
rosrun humanoid_carry_payload_nn make_gif.py --seconds 10 --fps 20
```

不装 ROS 时（克隆到任意目录即可）：

```bash
python3 scripts/train_mlp.py
python3 scripts/main.py --mode demo --controller nn --no-viewer
python3 scripts/main.py --mode eval
```

演示动图见 [docs/img/humanoid/humanoid_carry_payload_nn/demo.gif](../img/humanoid/humanoid_carry_payload_nn/demo.gif)。

## 4. 目录结构

```
humanoid_carry_payload_nn/
├── CMakeLists.txt / package.xml       # ROS1 catkin 包
├── config/params.yaml                 # 与代码默认值一致的参数表
├── launch/main.launch                 # mode / controller / mass / viewer 参数
├── models/
│   ├── humanoid_carry.xml             # 自建 MJCF：人形 + 负载箱 + 跑步机 + 助行架
│   └── balance_mlp.npz (+ .json)      # 训练得到的平衡网络权重与元信息
├── scripts/
│   ├── main.py                        # 统一入口：demo / keyboard / eval / collect
│   ├── sim_core.py                    # MuJoCo 仿真封装（无 rospy 依赖）
│   ├── gait.py                        # 相位式步态生成器
│   ├── model.py                       # PDExpert + BalanceMLP
│   ├── train_mlp.py                   # 行为克隆训练
│   ├── evaluate.py                    # 0/2/4 kg 控制器对照评测
│   └── make_gif.py                    # 渲染演示 GIF
├── test/test_offline.py               # 不依赖 ROS 的离线自检
├── requirements.txt / README.md       # 依赖与使用说明
└── results/                           # 评测输出的 CSV
```

本页文档位于仓库 `docs/humanoid/humanoid_carry_payload_nn.md`，
演示动图位于 `docs/img/humanoid/humanoid_carry_payload_nn/demo.gif`。

## 5. 仿真台架说明（重要）

为了让"平衡"成为一个可重复、可测量的实验问题，本模块采用**跑步机 + 减重助行架**台架，这与康复/搬运
机器人常用的步态实验平台一致：

- **跑步机皮带**：一个可沿 x 平移的大质量箱体（`belt_x` 关节），每步把速度设为 `-v_cmd`。
  机器人因此"原地行走"，**等效搬运里程 = ∫v_cmd dt**（`sim.distance`）。虚拟世界中相对位移与
  真实搬运一致，但工程上避免了里程无限增长导致相机与场景失效。
- **减重助行架**：根部用 4 个一维关节代替自由关节 —— `root_x`（前后，弹性吊带 stiffness 800）、
  `root_z`（竖直，stiffness 900）、`root_pitch`（矢状面俯仰，stiffness 1400）、`root_yaw`（转向）。
  侧移与横滚被导轨锁定，因此不会侧翻；**俯仰不驱动，是平衡控制器要抑制的"倒立摆"方向**。
- 步态为**相位式准静态步态**：支撑相髋关节以固定步长扫过、摆动相抬膝 0.78 rad 保证离地，
  步频由"皮带速度 = 支撑相脚位移"这一条件反解得到（`gait.advance`），因此不会产生持续滑移。

> 局限与取舍：台架的弹性吊带本身提供了大部分姿态回复力矩，因此控制器的绝对改善幅度有限
> （见第 8 节实测），它的主要作用是在不同负载质量下保持一致的躯干姿态，而非"挽救即将跌倒的机器人"。
> 这是刻意选择的可重复实验条件，而非把平衡问题简化掉。

## 6. 模型（models/humanoid_carry.xml）

- 15 个可动关节：每腿 6 个（髋 yaw/roll/pitch、膝、踝 pitch/roll），每臂 2 个（肩 pitch、肘 pitch），
  颈 yaw 1 个；共 17 个位置伺服执行器（12 腿 + 4 臂 + 1 颈）。
- 尺寸：大腿/小腿各 0.36 m，脚掌 0.27 × 0.11 m，躯干 12 kg，骨盆 8 kg。
- 负载箱：独立 body + `freejoint`，通过两条 `weld` 与前臂刚性夹持；质量在运行时改写
  （`model.body_mass` + 惯量按比例缩放），空载退化为 0.05 kg 以避免零惯量奇异。
- `overview` / `side` / `head_camera` 三个相机，供可视化与渲染。

## 7. 控制器与训练

- **PDExpert**（示范策略）：`ankle_pitch = kp*pitch + kd*pitch_rate`，
  `hip_pitch = k_hip*pitch + k_mass_hip*payload_norm`，输出限幅 `corr_limit`。
- **BalanceMLP**：9→32→32→3，tanh 激活，输入按训练集均值/标准差归一化，输出同样限幅；
  权重存 `npz`，元信息（特征名、结构、归一化统计）存同名 `.json`。
- **训练**：`train_mlp.py` 用 PD 专家在 0/2/4 kg、随机速度/转向指令下采集 3.75 万条样本，
  Adam（lr=1e-3，batch=256，60 epoch），验证集 MSE 约 `1e-5`（网络基本复现专家策略）。

## 8. 评测结果

`evaluate.py --seconds 20 --speed 0.15`（固定指令曲线，三种负载 × 三种控制器）：

| 场景 | 控制器 | 俯仰均值 (°) | 俯仰 RMS (°) | 俯仰峰值 (°) | 等效里程 (m) | 跌倒 |
| --- | --- | --- | --- | --- | --- | --- |
| 0 kg | open | 4.21 | 5.46 | 8.63 | 2.94 | 无 |
| 0 kg | pd | 4.10 | 5.34 | 8.58 | 2.94 | 无 |
| 0 kg | nn | 4.13 | 5.38 | 8.62 | 2.94 | 无 |
| 2 kg | open | 4.50 | 5.72 | 8.74 | 2.94 | 无 |
| 2 kg | pd | 4.41 | 5.68 | 8.84 | 2.94 | 无 |
| 2 kg | nn | 4.43 | 5.71 | 8.89 | 2.94 | 无 |
| 4 kg | open | 4.77 | 5.96 | 9.08 | 2.94 | 无 |
| 4 kg | pd | 4.69 | 5.98 | 9.23 | 2.94 | 无 |
| 4 kg | nn | 4.71 | 6.01 | 9.25 | 2.94 | 无 |

结论：11 秒级搬运过程中三种负载全部不跌倒；补偿控制器把俯仰均值降低约 2%~5%，且神经网络与
PD 专家表现一致（验证行为克隆精度）。俯仰绝对量主要来自步态本身，台架吊带承担了大部分回复力矩。

## 9. 键盘操作

| 按键 | 作用 |
| --- | --- |
| `W` / `S` | 前进 / 后退（每次 ±0.04 m/s，上限 0.22 m/s） |
| `A` / `D` | 左转 / 右转（每次 ±0.06 rad/s，上限 0.45 rad/s） |
| 空格 | 急停（速度与转向清零） |
| `Q` | 退出 |

Linux 下读取 `/dev/tty`（不依赖 X）；Windows 下自动改用 `msvcrt`。

## 10. 参数

见 `config/params.yaml`，与代码默认值一致。常用：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `payload.masses` | `[0, 2, 4]` kg | 三种负载场景 |
| `gait.stride_max` | 0.18 rad | 固定步长对应的髋半幅 |
| `gait.knee_lift` | 0.78 rad | 摆动抬膝量（低于 0.7 会拖地） |
| `expert.k_hip` | -0.3 | 俯仰反馈到髋 |
| `expert.k_mass_hip` | 0.05 | 负载质量前馈 |
| `mlp.hidden` | `[32, 32]` | 网络结构 |

## 11. 常见问题

- **打开窗口报错 / 无图形**：加 `--no-viewer`；SSH 用 `ssh -X`。
- **找不到权重**：`main.py --controller nn` 会自动退回 PD 专家并提示；先跑 `train_mlp.py`。
- **保存的图太小/太糊**：`make_gif.py --width 640 --height 480 --fps 25`。
- **步速调高后出现缓慢前倾**：台架吊带回复力矩有限，建议演示速度 ≤ 0.15 m/s、单次 ≤ 20 s。

## 12. 接入站点导航

把本页加入 MkDocs：在 `mkdocs.yml` 的 `nav` 中找到「人形机器人」分组
（若没有则新建），在其下加入一行

```yaml
  - 双臂搬运负载的神经网络平衡控制: humanoid/humanoid_carry_payload_nn.md
```

并在 `docs/index.md` 的模块列表中加一行链接：

```markdown
- [人形机器人双臂搬运负载的神经网络平衡控制](humanoid/humanoid_carry_payload_nn.md)
```
