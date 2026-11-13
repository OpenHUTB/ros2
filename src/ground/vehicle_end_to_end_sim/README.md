# vehicle_end_to_end_sim —— 规则式 AEB 自动紧急制动演示系统

> 机器人操作系统及应用 · 课程作业
>
> **规则式（rule-based）AEB 演示**：系统通过前向雷达（Raycasting）测距获取障碍物
> 距离，由固定距离阈值三级状态机给出期望速度与转向，经自行车运动学模型推进车辆，
> 实现自动巡航、人机接管转向、预警提示与 AEB（自动紧急制动）功能。
>
> 本模块**不是**图像端到端的神经网络模型：无相机图像输入、无神经网络、无训练与测试
> 脚本；包名 `vehicle_end_to_end_sim` 沿用课程作业的既有命名。
>
> 纯 Pygame 实现，仅依赖 `pygame`，无需 CARLA / ROS 节点。

## 1. 功能

$$
\underbrace{\text{前向雷达扫描}}_{\text{感知}}
\ \xrightarrow{\ \text{视野锥 ±40°}\ }\ 
\underbrace{\text{最近障碍物距离 }D}_{\text{决策输入}}
\ \xrightarrow{\ \text{三级状态机}\ }\ 
\underbrace{(\text{目标车速},\ \text{转向角})}_{\text{控制}}
\ \xrightarrow{\ \text{自行车模型}\ }\ 
(\text{坐标},\ \text{航向角})
$$

| 距离区间（像素） | 状态 | 行为 |
|---|---|---|
| `D >= 300` | 安全区 | 正常加速至目标车速 |
| `180 <= D < 300` | 预警区 | 橙色提示"请转弯"，速度保持不变 |
| `D < 180` | 刹车区 | 红色提示 + AEB 恒减速，平滑刹停 |

## 2. 目录结构

```
src/ground/vehicle_end_to_end_sim/
├── main.py                                   # 课程约定入口（薄壳，转发到包内实现）
├── main.sh / main.bat                        # 一键运行脚本
├── vehicle_end_to_end_sim/
│   ├── __init__.py
│   ├── config.py                             # 全局常量与单位换算（km/h ↔ 像素/帧）
│   ├── kinematics.py                         # 自行车运动学模型（Car 类）
│   ├── perception.py                         # 前方视野锥感知（最近障碍物距离）
│   ├── decision.py                           # 三级状态机（安全/预警/刹车 + AEB）
│   ├── control.py                            # 键盘转向 + 平滑回正
│   └── main.py                               # Pygame 主程序（界面 + 主循环）
├── launch/main.launch.py                     # ROS 2 launch（ExecuteProcess 拉起）
├── config/sim_params.yaml                    # 仿真参数
├── resource/vehicle_end_to_end_sim
├── setup.py / setup.cfg / package.xml
├── requirements.txt
└── test/test_end_to_end_logic.py             # 单元测试（16 项，无需 pygame）
```

> **为什么 `main.py` 有两份？** 课程要求入口叫 `main.*`，所以包根必须有 `main.py`；
> 但 `setup.py` 是 `packages=[package_name]`，`colcon build` 只会把
> `vehicle_end_to_end_sim/` 目录装进 site-packages，包根那份不会被安装。
> 入口按 `vehicle_end_to_end_sim.main` 导入，实现必须放在包内，包根只做转发。

## 3. 快速开始

```bash
# 安装依赖
pip install pygame

# 直接运行（推荐）
python3 main.py                # 或 Windows: main.bat

# 流程：↑/↓ 调节目标车速 → 回车启动仿真
# 驾驶：← / → 控制转向，松开自动回正
```

ROS 2 工作空间内统一拉起：

```bash
colcon build --packages-select vehicle_end_to_end_sim
source install/setup.bash
ros2 launch vehicle_end_to_end_sim main.launch.py
```

## 4. 测试

```bash
python3 test/test_end_to_end_logic.py     # 16 项全部通过，无需 pygame
```

覆盖四层纯逻辑：运动学数值正确性、视野锥测距与侧/后方过滤、三级状态机速度行为、
转向回正与限幅、单位换算往返一致性。

## 5. 与已有示例的关系

仓库中 [`carla_end_to_end_nn`](./carla_end_to_end_nn/) 是学习型端到端示例：用可训练的
卷积网络把相机像素映射为转向。本模块与之主题相邻但方法不同，用**射线测距 + 固定距离
阈值规则 + 自行车运动学模型**在纯 Python + Pygame 环境下演示 AEB 与巡航逻辑：不含图像
输入，也不含任何学习/训练环节。

包名中的 `end_to_end` 仅为课程作业沿用命名，指"感知到执行的完整链路"，不表示本模块是
学习型端到端模型；本模块的可复现行为以 `test/test_end_to_end_logic.py` 的 16 项纯逻辑
单元测试为准。完整教程见 [模块文档](https://openhutb.github.io/ros2/ground/vehicle_end_to_end_sim/)。
