# humanoid_push_recovery —— 人形机器人平衡控制与外力抗扰恢复

基于 MuJoCo 的人形机器人 **站立平衡与外力扰动恢复** 模块：自建双足模型底盘
freejoint 完全自由（不做速度锁定），机器人依靠双脚与地面的真实接触以及关节力矩
站立；模块实现 **ZMP / 捕获点稳定性判据** 与 **踝策略 + 髋策略** 的恢复控制律，
并定量给出「扰动力 → 最大质心偏移 / 是否跌倒」的完整恢复包线。

与仓库内其他人形模块的区别：本模块关注的是**外力扰动下的稳定性与恢复能力**
（外力 → 稳定性判据 → 恢复控制），不是运动指令执行，也不是步态生成或视觉感知。

## 功能概述

- **自建双足人形 MJCF**（15 关节）：底盘 freejoint 自由，双脚为带摩擦的箱体，
  可真实地「被推 → 摇晃 → 恢复 / 跌倒」；
- **站姿 PD 伺服**：15 关节力矩 PD 保持直立位，髋/踝 roll 关节加强增益抗侧倾；
- **稳定性判据**：
  - 支撑多边形：由双脚底面 8 个角点求凸包；
  - ZMP：LIPM 近似 `ZMP = CoM_xy − (z/g)·CoM_acc_xy`；
  - 捕获点：`ξ = CoM_xy + v_com/√(g/h)`，并给出到支撑多边形边界的稳定裕度；
- **恢复控制律**（代码基础为开源项目
  [PrageethM1702/Bipedal-Walking-and-Push-Recovery](https://github.com/PrageethM1702/Bipedal-Walking-and-Push-Recovery)，
  Apache-2.0，改写适配见 `THIRD_PARTY_NOTICES.md`）：
  - **踝策略**：躯干倾角 P 项（上游原律）+ 质心偏移/速度反馈（消除稳态偏差），
    输出踝 pitch / roll 关节**力矩** (N·m)，本站已按力矩量纲重新整定（主力）；
  - **髋策略**：踝策略饱和后，用质心偏差与速度反馈输出髋关节力矩继续把质心拉回
    （可开关，用于对照实验）；
- **扰动注入**：对 torso 施加 6 维外力/力矩（`data.xfrc_applied`），支持任意方向与时长；
- **性能评价**：单次扰动实验 + 二分法测定临界冲量 + 策略对照（仅踝 vs 踝+髋）。

## 环境要求

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS（已在 VMware 虚拟机 + ROS2 Humble 实测）；Windows 11 同样可运行 standalone |
| ROS2 | Humble（launch 启动）/ standalone 模式不需要 ROS |
| Python | 3.10（Ubuntu 实测）/ 3.13（Windows 实测） |
| 主要依赖 | mujoco ≥ 3.0、numpy、pyyaml |
| 文档素材（可选） | imageio（关键帧/动图；原始时序数据导出为 CSV，无需 matplotlib） |

```bash
pip install -r requirements.txt
# 可视化窗口（Linux 桌面）如提示缺少 GLFW：
sudo apt install -y libglfw3
```

## 运行步骤

### 1. standalone（无需 ROS，最快验证）

```bash
cd src/humanoid/humanoid_push_recovery     # 或工作空间的 ~/ros2_ws/src/humanoid_push_recovery

python3 main.py --check                    # 模型/判据/控制器 自检
python3 main.py                            # 演示：一次推扰 + 平衡恢复（打开可视化窗口）
python3 main.py --force 180                # 指定扰动力 (N)
python3 main.py --direction 90             # 扰动方向（0=前向, 90=侧向, 180=后向）
python3 main.py --no-hip                   # 关闭髋策略（仅踝策略，对照用）
python3 main.py --sweep                    # 测定各方向临界冲量并与策略对照
python3 main.py --capture assets_out       # 采集关键帧/动图/时序CSV/metrics.json
python3 main.py --test                     # 自动化测试（38 项）
```

无图形环境（SSH / 容器）下渲染需要 EGL：

```bash
export MUJOCO_GL=egl
```

### 2. ROS2 Humble launch

```bash
mkdir -p ~/ros2_ws/src && cp -r <本包路径> ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install --packages-select humanoid_push_recovery
source install/setup.bash
ros2 launch humanoid_push_recovery main.launch.py
```

焦点保持在启动终端，用键盘施加扰动：

| 按键 | 动作 |
|---|---|
| `W` / `S` | 向前 / 向后推 |
| `A` / `D` | 向左 / 向右推 |
| `Q` / `E` | 左前 / 右后斜向推 |
| `↑` / `↓` | 同 `W` / `S`（方向键） |
| `←` / `→` | 同 `A` / `D`（方向键） |
| `+` / `-` | 推扰力度 ±20 N |
| `空格` | 复位 |
| `ESC` | 退出 |

另开终端观察状态：

```bash
ros2 topic echo /balance/status            # stable / recovering / fallen
ros2 topic echo /balance/stability_margin  # 捕获点到支撑边界的裕度 (m)
ros2 topic echo /balance/zmp               # 零矩点
ros2 topic echo /balance/capture_point     # 捕获点
ros2 run tf2_ros tf2_echo world humanoid/base_link   # 验证 TF 可读取
```

### 3. 可视化窗口（看到人形机器人界面）

交互演示默认打开 MuJoCo 可视化窗口，可实时看到机器人被推后的位移与姿态变化，
相机自动跟踪躯干。

| 运行环境 | 能否看到窗口 | 说明 |
|---|---|---|
| 本机 Ubuntu 桌面 / 虚拟机桌面终端 | ✅ | 直接运行即可 |
| SSH 远程登录 | 需 X11 转发 | 用 `ssh -X user@host`，或先 `export DISPLAY=:0` |
| 无显示服务器（纯命令行 / CI） | ❌ | standalone 加 `--no-viewer`；ROS2 用 `visualize:=false` |

无图形界面时程序会打印提示并自动退化为无窗口模式，**物理仿真、话题发布与自动化测试均不受影响**。

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/balance/push` | geometry_msgs/Vector3 | 订阅：扰动力 (N)，方向为分量方向 |
| `/balance/reset` | std_msgs/Empty | 订阅：复位仿真并更新平衡参考 |
| `/balance/com` | geometry_msgs/PointStamped | 发布：全身质心位置 |
| `/balance/zmp` | geometry_msgs/PointStamped | 发布：零矩点（LIPM 近似） |
| `/balance/capture_point` | geometry_msgs/PointStamped | 发布：捕获点 |
| `/balance/stability_margin` | std_msgs/Float64 | 发布：捕获点到支撑多边形边界的裕度 (m) |
| `/balance/status` | std_msgs/String | 发布：stable / recovering / fallen |
| `/joint_states` | sensor_msgs/JointState | 发布：15 个关节位置/速度/力矩 |
| `/tf` | tf2_msgs/TFMessage | 发布：`world → humanoid/base_link`（tf2_ros.TransformBroadcaster） |

参数集中在 `config/params.yaml`，standalone 与 ROS2 共用同一套参数。

## 文件结构

```
humanoid_push_recovery/
├── humanoid_push_recovery/
│   ├── sim_core.py            # MuJoCo 核心：站姿 PD / 扰动注入 / ZMP / 躯干倾角
│   ├── balance/               # 稳定性判据与恢复策略子包（基础代码来自上游开源项目）
│   │   ├── capture_point.py   #   捕获点 ξ = x + v/√(g/h)（改写）
│   │   ├── support_polygon.py #   凸包 / 点在多边形内 / 稳定裕度（改写）
│   │   ├── ankle_strategy.py  #   踝策略：倾角 P + 质心反馈 → 踝力矩（改写）
│   │   ├── hip_strategy.py    #   髋策略：质心偏差/速度 → 髋力矩（改写）
│   │   ├── controller.py      #   BalanceController：组合两策略输出 15 维力矩
│   │   └── __init__.py        #   对外接口
│   ├── evaluate.py            # 单次扰动实验、临界冲量二分测定、策略对照
│   ├── mujoco_sim_node.py     # ROS2 平衡仿真节点
│   └── push_teleop_node.py    # ROS2 扰动遥控节点（键盘）
├── config/params.yaml         # 仿真/PD/平衡/扰动参数
├── models/humanoid_balance.xml # 自建双足模型（底盘自由 + 可施加外力）
├── launch/main.launch.py      # launch 启动入口
├── test/test_standalone.py    # 自动化测试（38 项）
├── THIRD_PARTY_NOTICES.md     # 上游开源代码声明（Apache-2.0）
├── main.py / main.sh          # 模块主入口
└── README.md
```

## 实测指标

由 `python3 main.py --sweep` 与 `python3 main.py --test` 实测
（Windows 11 / MuJoCo 3.16.0 / Python 3.13；物理量为确定性仿真结果，与机器无关）：

| 指标 | 结果 |
|---|---|
| 无扰动自由站立（8 s） | 不跌倒，质心漂移 **0.55 mm**，根高 1.084 m |
| 静立稳定裕度（捕获点到支撑边界） | **93.1 mm** |
| 前向 120 N×(0.08 s) 推扰 | 最大偏移 **36.3 mm**，恢复 **0.84 s**，残差 0.49 mm |
| 侧向 150 N×(0.08 s) 推扰 | 最大偏移 **34.5 mm**，恢复 **0.74 s** |
| 斜向 45° 120 N×(0.08 s) 推扰 | 最大偏移 29.9 mm（停在 8.2 mm 稳态偏移，见文档 5.2） |
| 临界冲量（前向） | 仅踝 **147.3 N / 11.79 N·s**；踝+髋 **163.7 N / 13.10 N·s**（+11.1%） |
| 临界冲量（侧向） | 仅踝 **226.6 N / 18.13 N·s**；踝+髋 **284.6 N / 22.77 N·s**（+25.6%） |
| 自动化测试 | `main.py --test` **38 项全部通过** |
| 仿真吞吐 | 9,047 ~ 14,691 step/s（实时因子 18.1× ~ 29.4×，随机器负载波动） |

**结论**：踝策略是站立抗扰的主力；髋策略在不改变硬件的前提下把可恢复冲量提升
11.1%（前向）/ 25.6%（侧向）。侧向抗扰能力强于前向，原因是双足站立时侧向支撑
宽度（460 mm）大于前后支撑长度（220 mm）。

## 能力边界（重要）

- 本模块的恢复能力**完全来自踝/髋关节力矩**，未实现**跨步恢复（stepping）**：
  当扰动超过临界冲量时机器人会跌倒，这是**由脚掌几何决定的物理上限**，不是控制器参数没调好
  （分析见文档 5.3 节）；
- 模型为自建简化双足，参数（连杆长度、质量分布、摩擦）与真实人形机器人不同，
  临界冲量的**绝对值**不可直接外推到实机，但「扰动力—偏移—跌倒」的**定性规律与
  相对提升**具有参考意义；
- 评价指标（稳定裕度、临界冲量）只描述**本仿真模型**在上述参数下的行为，
  不代表硬件实验结论。

## AI 声明

本模块在 AI 助手协助下完成：稳定性判据与踝/髋策略的**代码基础来自 Apache-2.0
开源项目**（见文件结构与 `THIRD_PARTY_NOTICES.md`，已改写适配 MuJoCo 力矩控制），
MJCF 模型、ROS2 节点、参数整定与评测脚本为在其基础上开发；代码与文档由本人负责。
表中所有实测数字均由本机运行 `main.py --sweep` / `--test` 得到
（原始时序数据可用 `--capture` 导出为 CSV），未做任何未运行的声明。
