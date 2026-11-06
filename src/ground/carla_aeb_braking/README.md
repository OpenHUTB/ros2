# carla_aeb_braking

CARLA 0.9.16 **自动紧急制动（AEB）** 模块：自车以巡航速度行驶，前向激光雷达实时测距，
AEB 状态机（cruise → warning → braking → recover，带滞回阈值）在障碍物逼近时自动
预警并全力制动，障碍消除后缓释恢复。

## 组成

| 文件 | 职责 |
|---|---|
| `carla_aeb_braking/aeb_core.py` | 决策核心（纯离线逻辑）：LiDAR 点云测距、深度图工具、AEB 滞回状态机 |
| `carla_aeb_braking/carla_common.py` | CARLA API 封装：连接 / 生成 / 传感器 / 控制 / 清理（提交 2） |
| `carla_aeb_braking/bridge_node.py` | ROS↔CARLA 桥接节点：生成车辆+前向 LiDAR，发布距离/速度/里程计，应用控制指令（提交 2） |
| `carla_aeb_braking/aeb_node.py` | AEB 控制节点：订阅距离/速度 → 状态机决策 → 发布制动/油门、arm/abort 服务（提交 3） |
| `carla_aeb_braking/demo.py` | 一键演示：离线制动曲线 + `--live` 连 CARLA 真实场景（提交 4） |
| `main.py` | 统一入口：`doctor` / `test` / `build` / `demo` / `live` |
| `config/aeb_params.yaml` | 决策参数（进入/退出距离、滞回、制动强度） |
| `launch/main.launch.py` | 一键启动两个 ROS 节点（提交 2） |

## 快速开始

```bash
# 1. 依赖体检（python / numpy / pyyaml / carla / ROS）
python3 main.py doctor

# 2. 离线单元测试（状态机 + LiDAR 测距，不需要 ROS 与 CARLA）
python3 main.py test

# 3. colcon 构建
python3 main.py build

# 4. 离线制动曲线演示（无需 CARLA，导出 PNG + CSV）
python3 main.py demo

# 5. 连 CARLA 真实场景演示（--host 填 CARLA 所在机器 IP）
python3 main.py demo --live --host 192.168.1.100

# 6. ROS 2 节点（桥接 + AEB 控制）
ros2 launch carla_aeb_braking main.launch.py host:=192.168.1.100
```

## 工作原理

```
  巡航 ──d<warn_enter──▶ 预警 ──d<brake_enter 且接近──▶ 紧急制动
    ▲                      │                              │
    └──d>warn_exit─────────┴────d>brake_exit──▶ 恢复 ────┘
```

- **测距**：前向 LiDAR 点云 → 车道带滤波 + 车身高度带 + 距离直方图聚类，
  稳健区分障碍车与地面/路侧设施杂波（深度相机工具保留但不作主测距）
- **滞回**：每个状态的进入/退出阈值不同（如 warn_enter=18 m、warn_exit=24 m），
  距离在滞回带内波动时状态不变，避免临界抖动
- **接近判定**：只有相对接近速度 `> close_min` 才允许触发紧急制动，
  静止/远离的近距离障碍只会预警

## 测试

离线单元测试覆盖：LiDAR 测距（聚类/地面/横向/近距过滤）、深度解码往返/已知值/边界、
前向区域提取、状态机进入/退出/滞回/非法状态/自定义参数、远→近→清除全周期。
`python3 main.py test` 运行，全部不依赖 ROS 与 CARLA。
