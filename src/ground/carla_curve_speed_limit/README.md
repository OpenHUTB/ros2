# CARLA 弯道自适应限速（CSL）

在 CARLA 0.9.16 中实现**弯道自适应限速**（Curve Speed Limit）完整闭环：
地图路点曲率预瞄 → `v=√(a·R)` 安全过弯速度 → 滞回状态机 + 加减速斜坡 →
油门/制动平顺控车。入弯前提前减速、出弯恢复巡航。

与同仓库 AEB / BSD（LiDAR 感知）不同，本模块**零传感器依赖**——
数据源是地图拓扑路点，是首个纯地图决策的地面载具模块。

## 模块结构

```
carla_curve_speed_limit/
├── carla_curve_speed_limit/
│   ├── curve_speed_core.py   # 决策核心：曲率/限速/滞回/斜坡（纯离线）
│   ├── bridge_node.py        # CARLA 桥接：路点 → /csl/target_speed 等
│   ├── csl_node.py           # 速度控制节点：滞回+斜坡 → /csl/control
│   ├── carla_common.py       # CARLA 封装（复用 AEB 已验证实现）
│   └── demo.py               # 离线 / --live 真实 CARLA 一键演示
├── launch/main.launch.py     # 桥接+控制两节点一键启动
├── config/csl_params.yaml    # 全部决策参数
├── tests/                    # 49 个离线单元测试
└── main.py                   # doctor / test / build / demo 入口
```

## 快速开始

```bash
# 离线（无需 ROS 与 CARLA，目录：本模块根目录）
python3 main.py doctor
python3 main.py test        # 49/49 单元测试
python3 main.py demo        # 离线弯道场景，artifacts/ 出曲线 PNG + CSV

# CARLA 实测（先启动 CARLA 0.9.16，--host 填 CARLA 所在机器 IP）
python3 main.py demo --live --host 192.168.1.100 --seconds 45

# ROS 2 launch（目录：ros2 工作空间根目录）
colcon build --packages-select carla_curve_speed_limit && source install/setup.bash
ros2 launch carla_curve_speed_limit main.launch.py host:=192.168.1.100
# 另开终端：ros2 topic echo /csl/state
```

## 核心参数（config/csl_params.yaml）

| 参数 | 默认 | 含义 |
|---|---|---|
| `cruise_speed` | 8.0 | 巡航速度 m/s |
| `a_lat_max` | 2.5 | 横向加速度上限 m/s²（弯道限速 v=√(a·R)） |
| `look_ahead` | 30.0 | 预瞄距离 m（越大减速越提前） |
| `v_min` | 1.5 | 弯道最低速度 m/s |
| `decel_max` / `accel_max` | 2.0 / 1.5 | 加减速斜坡限幅 m/s² |

## 决策语义

目标速度 = **min(巡航速度, 预瞄弯道限速)**——限速是上限不是设定值；
cruise/slowdown 滞回状态机（滞回带 0.2~0.5 m/s）消除临界抖动，
跃迁事件 slowdown_warn / cruise_resume 只发一次。

教程与实测结果见 `docs/ground/carla_curve_speed_limit.md`。
