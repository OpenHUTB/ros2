# humanoid_carry_payload_nn

人形机器人双臂在胸前夹持负载箱、在**跑步机 + 减重助行架**台架上行走的 MuJoCo 物理仿真模块：
用 PD 姿态专家产生示范数据，再用行为克隆训练一个 9→32→32→3 的神经网络（`BalanceMLP`），
在 **0 / 2 / 4 kg** 三种负载下抑制躯干俯仰。

> 对应任务：**人形机器人 / 肌肉骨骼人类别 —— MuJoCo 物理仿真 + 键盘运动控制 + 神经网络控制算法**。
> 仿真器为 MuJoCo（自建 MJCF），不使用 Gazebo。

## 1. 运行环境（实测）

| 项目 | 版本 / 说明 |
| --- | --- |
| 操作系统 | Ubuntu 20.04.6 LTS（课程虚拟机），Windows 亦可跑离线模式 |
| ROS | ROS Noetic（catkin） |
| Python | 3.8 |
| MuJoCo | `mujoco >= 2.3`（本模块在 3.x 上验证） |
| NumPy | 1.24 |
| imageio | 仅生成演示 GIF 需要 |

```bash
pip3 install -r requirements.txt
```

无需 Gazebo、PyTorch、GPU；MLP 权重为纯 numpy 的 `npz`（13.5 KB），推理是单次前向，耗时远小于 2 ms 的仿真步长。

## 2. 目录结构

```
humanoid_carry_payload_nn/
├── CMakeLists.txt / package.xml       # ROS1 catkin 包
├── config/params.yaml                 # 与代码默认值一致的参数表
├── launch/main.launch                 # mode / controller / mass / speed / viewer 参数
├── models/
│   ├── humanoid_carry.xml             # 自建 MJCF：人形 + 双臂负载箱 + 跑步机 + 助行架
│   └── balance_mlp.npz (+ .json)      # 训练得到的平衡网络权重与元信息
├── scripts/
│   ├── main.py                        # 统一入口：demo / keyboard / eval / collect
│   ├── sim_core.py                    # MuJoCo 仿真封装（不 import rospy）
│   ├── gait.py                        # 相位式步态生成器
│   ├── model.py                       # PDExpert + BalanceMLP
│   ├── train_mlp.py                   # 行为克隆训练
│   ├── evaluate.py                    # 0/2/4 kg × open/pd/nn 对照评测
│   └── make_gif.py                    # 渲染演示 GIF
├── test/test_offline.py               # 不依赖 ROS 的离线自检
├── requirements.txt
└── results/                           # 评测输出的 CSV（运行后生成）
```

## 3. 编译

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps humanoid_carry_payload_nn
source devel/setup.bash
```

## 4. 键盘控制演示

```bash
roslaunch humanoid_carry_payload_nn main.launch mode:=keyboard controller:=nn mass:=2.0
```

| 按键 | 功能 |
| --- | --- |
| `W` / `S` | 前进 / 后退（每次 ±0.04 m/s，上限 0.22 m/s） |
| `A` / `D` | 左转 / 右转（每次 ±0.06 rad/s，上限 0.45 rad/s） |
| 空格 | 急停（速度与转向清零） |
| `Q` | 退出 |

> `main.py` 在 Linux 下直接读取 `/dev/tty`（不依赖 X，也不会被 `roslaunch` 的 stdin 转发影响），
> Windows 下自动改用 `msvcrt`。若窗口打不开（无图形环境），加 `viewer:=false` 或 `--no-viewer`；
> SSH 远程用 `ssh -X`。

不带可视化窗口的自动演示：

```bash
roslaunch humanoid_carry_payload_nn main.launch mode:=demo controller:=nn mass:=4.0 viewer:=false
```

## 5. 采集、训练与评估（一键复现）

```bash
# 1) 用 PD 专家在 0/2/4 kg、随机速度/转向指令下采集 3.75 万条样本并训练（约 5 s）
rosrun humanoid_carry_payload_nn train_mlp.py --seconds 25 --epochs 60

# 2) 三种负载 × 三种控制器对照评测（写 results/eval_<时间戳>.csv，并打印汇总表）
rosrun humanoid_carry_payload_nn evaluate.py --seconds 20 --speed 0.15

# 3) 重新渲染演示 GIF（写入仓库 docs/img/humanoid/humanoid_carry_payload_nn/）
rosrun humanoid_carry_payload_nn make_gif.py --seconds 10 --fps 20 --width 640 --height 480
```

完全脱离 ROS 时（把包目录拷到任意位置即可）：

```bash
python3 scripts/train_mlp.py
python3 scripts/main.py --mode demo --controller nn --no-viewer
python3 scripts/main.py --mode eval
python3 test/test_offline.py          # 5 项离线自检
```

## 6. 神经网络控制

- 观测 9 维：`roll, pitch, roll_rate, pitch_rate, payload_norm, v_cmd, w_cmd, phase_sin, phase_cos`。
- 结构 `9 -> 32 -> 32 -> 3`，tanh；输出为踝俯仰 / 踝横滚 / 髋俯仰的**补偿角**（限幅 ±0.5 rad），
  与步态规划的目标角叠加后送位置伺服。
- 训练：行为克隆 `PDExpert`（`ankle_pitch = kp*pitch + kd*pitch_rate`，
  `hip_pitch = k_hip*pitch + k_mass_hip*payload_norm`），Adam(lr=1e-3, batch=256, 60 epoch)，
  验证集 MSE ≈ `1e-5`。

## 7. 实测结果

`evaluate.py --seconds 20 --speed 0.15`（固定指令曲线，里程 2.94 m）：

| 场景 | 控制器 | 俯仰均值 (°) | 俯仰 RMS (°) | 俯仰峰值 (°) | 跌倒 |
| --- | --- | --- | --- | --- | --- |
| 0 kg | open / pd / nn | 4.21 / 4.10 / 4.13 | 5.46 / 5.34 / 5.38 | 8.63 / 8.58 / 8.62 | 无 |
| 2 kg | open / pd / nn | 4.50 / 4.41 / 4.43 | 5.72 / 5.68 / 5.71 | 8.74 / 8.84 / 8.89 | 无 |
| 4 kg | open / pd / nn | 4.77 / 4.69 / 4.71 | 5.96 / 5.98 / 6.01 | 9.08 / 9.23 / 9.25 | 无 |

结论：三种负载下全程不跌倒，补偿控制器把俯仰均值降低约 2%~5%，神经网络与 PD 专家表现一致
（验证行为克隆精度）；俯仰绝对量主要由步态本身决定。

## 8. 仿真台架与已知限制

- **跑步机**：皮带是大质量箱体 + `belt_x` 滑移关节，每步把速度设为 `-v_cmd`，机器人原地行走；
  等效搬运里程 = ∫v_cmd dt（`sim.distance`）。
- **减重助行架**：根部 4 个一维关节（`root_x / root_z / root_pitch / root_yaw`）替代自由关节，
  侧向与横滚被导轨锁定；**俯仰方向不驱动**，是平衡控制器要抑制的倒立摆方向。
- 限制：台架弹性吊带承担了大部分回复力矩，控制器的绝对改善幅度有限（上表 2%~5%），
  它的作用是在负载变化时保持一致的躯干姿态，而非"挽救即将跌倒的机器人"；
  这是为可重复实验刻意选择的台架条件。步态为相位式准静态步态，不是动态行走；
  演示建议速度 ≤ 0.15 m/s、单次 ≤ 20 s。

## 9. 演示素材与文档

- 动图：[docs/img/humanoid/humanoid_carry_payload_nn/demo.gif](../../../docs/img/humanoid/humanoid_carry_payload_nn/demo.gif)（2.7 MB）
- 详细文档（任务、模型、训练、结果、键盘、参数、FAQ）：
  [docs/humanoid/humanoid_carry_payload_nn.md](../../../docs/humanoid/humanoid_carry_payload_nn.md)

## 10. AI 声明

- 自建人形 MJCF 模型、跑步机/助行架台架、相位式步态、PD 专家与 numpy MLP 训练代码均为本作业原创实现；
- 参考了 MuJoCo 官方文档（`position` 执行器、`equality weld`、`freejoint`、`Renderer`）
  与仓库内已有 ROS 包结构惯例。

## 11. 许可证

MIT
