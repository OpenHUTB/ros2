# mujoco_car_avoidance

MuJoCo 地面两轮小车：**射线感知 → 单目测距语义（横向偏移判定）→ 分级减速 → 刹停**。

模块用 MuJoCo 原生 `mj_ray` 构建一个前视射线扇面（相当于小型二维激光雷达），
只用射线测距做决策，不使用任何场景真值；真值仅用于事后性能评价。
不含神经网络，作为后续"神经网络感知/控制"课题的规则基线与对照。

## 与仓库已有模块的区别

| 模块 | 引擎 | 感知方式 | 是否有神经网络 | 依赖 |
|---|---|---|---|---|
| `src/ground/carla_perception_control` | CARLA 0.9.16 | RGB + 深度 + 雷达 → MLP | 是 | CARLA 服务端 + GPU + ROS |
| `src/ground/carla_yolo_avoidance`（在审 PR #184） | CARLA 0.9.16 | YOLOv8 + 单目测距 | 是 | CARLA + torch + ROS |
| `src/water/rov_mujoco` | MuJoCo | 声呐 / 相机 / IMU | 部分任务有 | MuJoCo + ROS 2（水下） |
| **本模块** | **MuJoCo** | **`mj_ray` 射线扇面（无渲染、无 GPU 依赖）** | **否（规则基线）** | **mujoco + numpy** |

关键差别：本模块**不需要图形界面即可完成全部仿真、测试与性能评价**，
只有 `--demo` / `--interactive` 需要 OpenGL 窗口；因此克隆下来就能在无 GPU 的虚拟机里跑通。

## 目录结构

```
src/ground/mujoco_car_avoidance/
├── main.py                     统一入口（--demo/--interactive/--headless/--test/--launch）
├── main.sh / main.bat          脚本入口（无参数时默认 --headless）
├── mujoco_car_avoidance/
│   ├── config.py               路径解析与全部可调参数
│   ├── decision.py             纯函数：最近距离 → 状态与目标速度
│   ├── controller.py           纯函数：加减速限幅 + 差速运动学
│   ├── metrics.py              指标记录、汇总、CSV/曲线导出
│   ├── sensors.py              mj_ray 射线扇面与"本车道判定"
│   ├── car_env.py              场景加载、步进、车辆状态、真值间距
│   ├── runner.py               闭环：起步静置 + 感知/决策/控制/指标 + 直行航向保持
│   ├── teleop.py               键盘遥控（Linux termios / Windows msvcrt）
│   └── ros_node.py             ROS 2 节点（~/scan ~/odom ~/decision，订阅 ~/cmd_vel）
├── models/car_road_scene.xml   自建 MJCF 场景（纯几何体，无网格资产）
├── config/params.yaml          ROS 参数
├── launch/main.launch.py       ROS 2 一键启动
├── test/                       单元测试（纯逻辑 + 桩环境 + 场景/传感器）
├── tools/check_scene.py        场景静态检查（不需要 MuJoCo）
└── requirements.txt
```

## 运行环境

| 组件 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04（虚拟机，已实测通过）；Ubuntu 20.04 亦可（见下） |
| Python | 3.10（22.04 自带） |
| MuJoCo | 3.12.0（课程作业要求版本；`requirements.txt` 用 `>=3.0.0` 下限约束） |
| 依赖 | `mujoco`、`numpy`；绘图可选 `matplotlib` |
| ROS | ROS 2 Humble（仅 `--launch` 与 ROS 节点需要） |
| 显卡 | 不需要（射线感知不涉及渲染） |

> Ubuntu 20.04 说明：ROS 2 Humble 在 20.04 上会强制使用 Python 3.8，
> 若 `mujoco` 3.12.0 无 cp38 wheel，可单独用 3.10 的解释器跑仿真部分
> （本模块的仿真与 ROS 节点之间没有 Python 进程内耦合，`main.py` 独立可用）。

## 安装

```bash
# 系统依赖（无图形界面也需要）
sudo apt update && sudo apt install -y python3-pip patchelf libgl1 libglfw3

# Python 依赖
python3 -m pip install -r src/ground/mujoco_car_avoidance/requirements.txt
python3 -m pip install matplotlib          # 可选，导出曲线用

# ROS 2 功能包（只跑 main.py 时不需要）
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ln -s <本仓库路径>/src/ground/mujoco_car_avoidance .
cd ~/ros2_ws && colcon build --packages-select mujoco_car_avoidance --symlink-install
source install/setup.bash
```

## 运行步骤

```bash
cd src/ground/mujoco_car_avoidance

python3 main.py --version         # 先确认版本（当前 1.11.0），避免对着旧代码排查
python3 tools/check_scene.py      # 场景静态检查：名称引用、geom group、接触、支撑质心、出力限幅
python3 tools/check_traction.py   # 接触与牵引力诊断：接触数、轮速、滑移率、车身高度与姿态（需要 MuJoCo）
python3 main.py --test            # 自检：纯逻辑单测 + 场景冒烟测试（当前 62 项）
python3 main.py --headless        # 无窗口跑完整闭环，导出 results/run_log.csv 与 run_curves.png
python3 main.py --demo            # 打开视窗自动演示（需要 OpenGL）
python3 main.py --interactive     # 视窗 + WASD 手动驾驶，避障限速仍然生效
python3 main.py --launch          # ros2 launch 一键启动（需先 source 工作空间）
bash main.sh                      # 等价于 --headless
```

先跑 `--test`，再跑 `--headless`，最后录 `--demo` / `--interactive` 的效果图。

> Windows 环境请确保仓库路径不含中文，否则 MuJoCo 加载模型会报 `ParseXML: Error opening file`。

## 参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `cruise_speed` | 2.0 m/s | 巡航速度 |
| `slow_start_m` | 6.0 m | 开始减速的距离 |
| `stop_target_m` | 1.8 m | 期望停车位置（障碍物前） |
| `emergency_m` | 0.8 m | 急刹距离 |
| `stale_timeout_s` | 0.5 s | 感知失效超时 → 兜底刹停 |
| `creep_speed` | 0.12 m/s | 低于该目标速度即主动刹停，避免蠕动 |
| `acc_max` / `brake_max` | 2.0 / 3.5 m/s² | 加速度 / 减速度限幅 |
| `settle` | 0.6 s | 起步静置：先零轮速推进，让轮胎接触建立后再进入闭环 |
| `heading_gain` | 0.8 | 直行航向保持增益（0 = 关闭），抑制左右牵引不对称造成的偏航 |
| `LIDAR_ANGLES_DEG` | −60°…60°，5° 间隔（25 束） | 射线扇面 |
| `lane_half_width_m` | 0.8 m | 本车道半宽，超出即视为邻车道/路侧目标 |
| `MAX_WHEEL_FORCE_N` | 8.0 N | 单个驱动轮出力上限（对应 XML 的 `forcerange`），防止阶跃指令把车弹飞 |

## 版本记录

每次修复都来自一次真机（虚拟机 + MuJoCo 3.15）复现，问题与修法一一对应：

| 版本 | 修的问题 | 现象 |
|---|---|---|
| 1.0.0 | 初版 | — |
| 1.1.0 | `mj_ray` 返回值量纲：必须传单位向量，返回值才是米 | 静止时"最近距离 0.23 m"，实际是 4.69 m |
| 1.2.0 | 射线改为水平面内按航向解算（免疫车身俯仰）；地面/路面移出射线检测组；感知单帧丢失沿用上次有效值 | 俯仰 3.7° 时测距变 `None`；下俯 2.3° 时打到路面报出 2.68 m 假目标；单帧丢失直接急刹 |
| 1.3.0 | 地面接触加 `margin`、起步静置 0.6 s、直行航向保持；新增直行回归测试 | `ncon` 在 0/3/6 之间跳、车轮滑移 55%、车偏航约 10° |
| 1.4.0 | **修正直行航向保持的符号**（原实现是正反馈）；出生高度改 0.048 保证接触从第 0 步起稳定 | 偏航被推到 29°，路侧干扰物被误判为本车道障碍物而提前刹停 |
| 1.5.0 | 一致性整理：`setup.py` 版本与包版本同步（原先各自写死）、ROS 节点补齐静置/航向参数并调用静置、静态检查新增接触与出生高度校验、文档补齐静置与航向保持说明 | 无行为变化 |
| 1.6.0 | 修正驱动轮法向力不足：车体质心前移 0.04 m、后支撑轮抬离地面 6 mm，形成静定三点支撑；路侧干扰物外移到 `y=1.8 m`、车道半宽收紧到 0.8 m | 车速只有指令的 25%；擦着路侧物开过时出现横向偏移 **0.898 m** 的擦线命中（与当时 0.9 m 门限只差 0.002）而误触发刹停 |
| 1.7.0 | 修正"被驱动指令弹飞/掀翻"：删除地面 `margin`（让车身悬浮在弹性接触上）、给驱动轮加 `forcerange=±8 N` | 阶跃给 40 rad/s 时 0.05 s 内 z 冲到 0.13 m、车速 2.77 m/s，0.15 s 到 z=0.26 m 并翻车（`upright=-1`） |
| 1.8.0 | 修正停车指标判定：必须"先真正起步过"再停住才算停稳；最大减速度只在减速/刹停状态统计 | 起步瞬间 `t=0.62 s`、`v=0.005 m/s` 被误判成已停车 → `c_停车` 记成初始间距 7.48 m、`t_停车` 记成 0.62 s |
| 1.9.0 | 停车判定再加"连续低速 ≥0.3 s"；回归测试校验 `c_停车 ≈ c_最小` | 低速接近段的接触抖动让 `v` 瞬间掉到 0.01 → `c_停车` 记成 2.86 m，而车实际停在 2.05 m |
| 1.10.0 | 最大减速度改用 0.1 s 滑动窗口估计 | 单帧差分被接触抖动放大 → `a_最大制动` 报成 15.14 m/s²，而指令限幅只有 3.5 |
| 1.11.0 | `--demo/--interactive` 修正视窗：`mujoco.viewer` 需显式 `import mujoco.viewer` | 报 `module 'mujoco' has no attribute 'viewer'`，演示模式完全打不开 |

`python3 main.py --version` 可直接确认当前代码版本；`python3 main.py --test` 会先打印版本再跑测试（当前 62 项）。

## 常见问题

**Q1：`ModuleNotFoundError: No module named 'mujoco'`**
按上面的"安装"一节装依赖，并确认 `pip` 与运行脚本用的是同一个解释器（`python3 -m pip`）。

**Q2：`--demo` 打不开窗口**
无图形界面或未开 3D 加速。先用 `--headless` 完成全部验证；
虚拟机里可在 `--demo` 前加 `MESA_GL_VERSION_OVERRIDE=3.3`。

**Q3：车在障碍物前反复加减速？**
检查 `emergency_m < stop_target_m < slow_start_m`（构造参数时会校验），
以及是否有邻车道目标落进了车道半宽内（调小 `LANE_HALF_WIDTH_M`）。

**Q4：`--test` 里场景测试被跳过？**
说明当前环境没有 MuJoCo，只有纯逻辑测试生效；装好 MuJoCo 后会全部执行。

## 已知限制

- 障碍物是静态的；动态目标需要把真值间距的统计口径改为时间对齐。
- 车辆按直线行驶设计，转弯时真值间距的 AABB 近似会偏保守。
- 射线扇面是二维水平面，对悬空/低矮目标不敏感（可通过增加俯仰射线扩展）。
