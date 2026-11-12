# mujoco_hopper_slip_control

MuJoCo 里的**单足弹跳机器人**，用 SLIP（弹簧-质量倒立摆）模型 + Raibert 落脚点 +
起飞速度能量伺服，做定高弹跳、抗扰恢复和弹簧刚度参数扫描。ROS 2（Humble）功能包，
同时提供**不依赖 ROS** 的独立入口，方便单独验证控制律。

| 定高弹跳 | 支撑相 / 飞行相 |
|---|---|
| ![](../../docs/img/humanoid/mujoco_hopper_slip_control/demo.gif) | 见 `python3 main.py --headless` 输出的 `phase_cycle.png` |

## 1. 和已有 `mujoco_hopper_demo` 的区别

仓库里已有一个同题目的 `mujoco_hopper_demo`（ROS 1 / Noetic）。本模块是**独立重新实现**，
不是修改它。两者差异：

| 项 | `mujoco_hopper_demo` | 本模块 |
|---|---|---|
| ROS | ROS 1 Noetic（catkin + rospy） | **ROS 2 Humble**（ament_python + rclpy） |
| MuJoCo | 2.1.1 | **>= 3.3** |
| 模型自由度 | 1 个滑动腿 | 躯干 x/z + 髋 + 伸缩腿，带真实接触 |
| 控制 | **开环正弦力** `80*sin(4t)` | **SLIP 弹簧储能 + Raibert 落脚点 + 起飞速度闭环** |
| 是否离地 | 说明书原文「红色小脚一直接触地面」 | **有真实飞行相**，占空比 0.28（支撑 170 ms / 飞行 431 ms） |
| 指标 | 无 | 顶点高度、误差、支撑/飞行时长、占空比、峰值腿力、恢复时间、刚度扫描 |
| 测试 | 无 | 单元 28 项 + 集成 21 项 + 自检 12 项，CI 直接跑 |
| 文档 | 无效果图、无量化结果 | 动图 + 4 张曲线 + 实测表 + 能力边界 |

## 2. 运行环境

| 项 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04（WSL2 也可）/ Windows 10+ |
| ROS | Humble（只在 `ros2 launch` 时需要；独立入口不需要） |
| Python | 3.10+ |
| MuJoCo | **>= 3.3**（3.2.x 的接触几何返回值不可靠） |
| 依赖 | `mujoco`、`numpy`，出图另需 `matplotlib`、`pillow` |
| GPU | 不需要（离屏渲染走 OpenGL，Linux 无显卡时 `MUJOCO_GL=osmesa`） |
| 磁盘 | < 1 MB（无网格资源，纯 XML 建模） |

## 3. 安装与运行

```bash
cd src/humanoid/mujoco_hopper_slip_control
python3 -m pip install -r requirements.txt

python3 main.py --version      # 版本与关键参数
python3 main.py --test         # 12 项自检（含一次 8 s 真实弹跳）
python3 main.py --headless     # 三项实验 + CSV + 4 张图
python3 main.py --gif          # 演示动图
python3 main.py --gui          # 交互窗口（需图形环境）
```

常用参数：`--target-height 0.62`、`--duration 20`、
`--headless --export-docs`（顺带把图写进 `docs/img/`）。

ROS 2：

```bash
colcon build --packages-select mujoco_hopper_slip_control
source install/setup.bash
ros2 launch mujoco_hopper_slip_control main.launch.py
ros2 launch mujoco_hopper_slip_control main.launch.py target_height:=0.62 use_viewer:=true

# 另开终端看状态 / 改目标（[目标高度, 目标速度]）
ros2 topic echo /hopper_slip_node/status
ros2 topic pub --once /hopper_slip_node/target std_msgs/Float64MultiArray "{data: [0.60, 0.0]}"
```

## 4. 实测结果

Ubuntu 22.04 + Python 3.10 + MuJoCo 3.17，物理步长 2 ms，控制 500 Hz。
目标顶点 **0.55 m**，12 s。

### 4.1 定高弹跳（实验 A）

| 指标 | 值 |
|---|---|
| 总跳数 | 21 |
| 稳态顶点 | **0.566 ± 0.006 m**（误差 +0.016 m） |
| 支撑相 / 飞行相 | **170 ms / 431 ms** |
| 占空比 | 0.28 |
| 起飞竖直速度 | 2.28 m/s |
| 峰值腿力 | 98 N（约 1.7 倍体重） |

### 4.2 抗扰恢复（实验 B）

t = 5 s 施加 **+1.0 m/s** 横向速度冲量：

| 指标 | 值 |
|---|---|
| 扰动后 \|vx\| 峰值 | 1.81 m/s |
| 扰动后最低躯干高度 | 0.176 m（未摔倒） |
| 顶点恢复时间 | **6.12 s**（连续 3 跳回到 0.55 ± 0.03 m） |
| 横向漂移 | 0.10 m，末速 1.30 m/s（**未收敛，见 §5**） |

### 4.3 弹簧刚度扫描（实验 C）

| 刚度 k (N/m) | 顶点 (m) | 支撑相 (ms) | 飞行相 (ms) | 峰值腿力 (N) |
|---|---|---|---|---|
| 1100 | 0.622 | 230 | 482 | 91 |
| 1600 | 0.593 | 194 | 457 | 94 |
| 2200 | 0.566 | 170 | 431 | 98 |
| 2800 | 0.566 | 152 | 432 | 105 |
| 3400 | 0.562 | 140 | 428 | 110 |

三条单调趋势：**刚度↑ → 支撑相↓（230→140 ms）、峰值腿力↑（91→110 N）、飞行相略↓**。
这与 SLIP 理论一致：更硬的腿储能快、离地早，代价是冲击力更大。

### 4.4 ROS 2 实测（Ubuntu 22.04 + ROS 2 Humble）

在 WSL2 的 Ubuntu 22.04（ROS 2 Humble ros-base + colcon）里实测：

```text
$ colcon build --packages-select mujoco_hopper_slip_control --symlink-install
Finished <<< mujoco_hopper_slip_control [0.61s]

$ ros2 pkg executables mujoco_hopper_slip_control
mujoco_hopper_slip_control hopper_slip_node
mujoco_hopper_slip_control hopper_viewer

$ ros2 launch mujoco_hopper_slip_control main.launch.py target_height:=0.62
[hopper_slip_node]: 单足弹跳已启动：目标高度 0.62 m，目标速度 0.00 m/s，控制 100 Hz（每周期 5 个物理步）

$ ros2 param get /hopper_slip_node target_height
Double value is: 0.62

$ ros2 topic list | grep hopper
/hopper_slip_node/state
/hopper_slip_node/status
/hopper_slip_node/target

$ ros2 topic echo /hopper_slip_node/status --once
data: [0.3149, -2.0226, 0.0, 0.0023, 106.69, 0.0, 0.8222]

$ ros2 topic pub --once /hopper_slip_node/target \
      std_msgs/Float64MultiArray "{data: [0.60, 0.0]}"
[hopper_slip_node]: 新目标：高度 0.60 m，速度 0.00 m/s
```

节点连续运行 8 s 以上不退出，收到目标指令后仍持续发布状态。
`models/hopper_scene.xml` 与 `launch/main.launch.py` 均正确安装到 `share/`。
CI 里也有等价的 `ros:humble` 容器作业（见 `.github/workflows/mujoco_hopper_slip_control_ros2_build.yml`）。

## 5. 能力边界（已知限制）

1. **横向控制权限弱**。扰动后横向速度衰减极慢（11 s 仍未回到 0.15 m/s 以内），
   横向漂移持续存在。原因是支撑相髋关节自由（这是 SLIP 被动弹跳的前提），
   点足 + 自由髋的落脚点对横向动量几乎没有修正能力。要做真正的横向定速，
   需要加髋关节力矩控制或侧向自由度。
2. **1.5 m/s 冲量会摔倒**。可稳定恢复的量级在 1.0 m/s 附近，1.5 m/s 时躯干最低
   掉到 0.059 m。文中只报了 1.0 m/s。
3. **前进速度上限约 0.2 m/s**。Raibert 落脚点能产生方向性漂移，但受腿长（0.28 m）
   与支撑时长（0.17 s）限制，做不到 0.6 m/s 量级的定速前进。
4. **纯平面模型**：只有 x/z 平动，没有偏航/翻滚，不会侧倒。
5. **腿的质量不可忽略**（1.4 kg / 6 kg），落脚角大幅摆动会对躯干产生反作用。
6. 顶点存在**恒定的 +0.01~0.02 m 正偏差**，来自能量效率在线辨识的一跳滞后。

## 6. 参数

| 参数 | 位置 | 默认 | 说明 |
|---|---|---|---|
| `LEG_STIFFNESS` | `config.py` | 2200 N/m | 被动弹簧刚度（同时是实验 C 的扫描对象） |
| `LEG_DAMPING` | `config.py` | 12 N·s/m | 腿阻尼 |
| `ENERGY_GAIN` | `config.py` | 0.04 | 起飞速度伺服增益（>0.08 会出极限环） |
| `FORCE_LIMIT` | `config.py` | 800 N | 注能力上限 |
| `RAIBERT_KV` | `config.py` | 0.10 | 落脚点速度增益 |
| `RAIBERT_ANGLE_LIMIT` | `config.py` | 0.60 rad | 落脚角限幅 |
| `TARGET_HEIGHT` | `config.py` / launch | 0.55 m | 顶点目标 |
| `TIMESTEP` | `config.py` | 0.002 s | 物理步长 |

## 7. 测试

```bash
python3 src/humanoid/mujoco_hopper_slip_control/test/test_standalone.py   # 28 项，无需 MuJoCo
python3 tests/test_mujoco_hopper_slip_control_local.py                    # 21 项，需 MuJoCo
python3 src/humanoid/mujoco_hopper_slip_control/main.py --test            # 12 项自检
```

CI：`.github/workflows/mujoco_hopper_slip_control_tests.yml`（ubuntu-22.04 + Python 3.10）。

## 8. 第三方与许可

模型、控制律、代码、图、动图均为本模块原创，没有引用外部模型或权重。
SLIP 模型与 Raibert 落脚点规则属公开理论（Raibert, *Legged Robots That Balance*, 1986），
代码为本模块独立实现，不涉及第三方代码复制。许可：MIT。
