# CARLA 车道偏离预警（LDW）

在 CARLA 0.9.16 中实现**车道偏离预警**（Lane Departure Warning）完整闭环：
地图路点投影 → 带符号横向偏移 + 跳变剔除 → 三级滞回状态机
（normal → deviation → departure）→ 带方向跃迁事件。

与同仓库 AEB / BSD（LiDAR 感知）不同，本模块**零传感器依赖**——
数据源是地图拓扑路点；与 CSL（纵向曲率限速）互补，LDW 看**横向偏移**。

## 模块结构

```
carla_lane_departure_warning/
├── carla_lane_departure_warning/
│   ├── lane_departure_core.py  # 决策核心：偏移几何/跳变剔除/三级滞回（纯离线）
│   ├── bridge_node.py          # CARLA 桥接：路点投影 → /ldw/lateral_offset 等
│   ├── ldw_node.py             # 预警节点：状态机 → /ldw/state + /ldw/event
│   ├── carla_common.py         # CARLA 封装（复用 AEB/BSD/CSL 已验证实现）
│   └── demo.py                 # 离线 / --live 真实 CARLA 一键演示
├── launch/main.launch.py       # 桥接+预警两节点一键启动
├── config/ldw_params.yaml      # 全部决策参数
├── tests/                      # 40 个离线单元测试
└── main.py                     # doctor / test / build / demo 入口
```

## 快速开始

```bash
# 离线（无需 ROS 与 CARLA，目录：本模块根目录）
python3 main.py doctor
python3 main.py test        # 40/40 单元测试
python3 main.py demo        # 离线正弦偏离场景，artifacts/ 出曲线 PNG + CSV

# CARLA 实测（先启动 CARLA 0.9.16，--host 填 CARLA 所在机器 IP）
python3 main.py demo --live --host 192.168.1.100 --seconds 48

# ROS 2 launch（目录：ros2 工作空间根目录）
colcon build --packages-select carla_lane_departure_warning && source install/setup.bash
ros2 launch carla_lane_departure_warning main.launch.py host:=192.168.1.100
# 另开终端：ros2 topic echo /ldw/state
```

## 核心参数（config/ldw_params.yaml）

| 参数 | 默认 | 含义 |
|---|---|---|
| `warn_enter` / `warn_exit` | 0.5 / 0.3 | deviation 进/出横向偏移（m，滞回） |
| `depart_enter` / `depart_exit` | 0.9 / 0.65 | departure 进/出横向偏移（m，接近压线） |
| `lane_width` | 3.5 | 车道宽（m），bridge 按路点动态发布 |

## 决策语义

横向偏移（左偏正）双阈值滞回：normal → deviation（>0.5m）→
departure（>0.9m）；回中时 0.65m/0.3m 双阈值恢复。
事件带方向且仅跃迁拍发布一次：deviation_start_left/right、
departure_start_left/right、departure_end、deviation_clear。

教程与实测结果见 `docs/ground/carla_lane_departure_warning.md`。
