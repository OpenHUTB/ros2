# carla_bsd_blind_spot

CARLA 0.9.16 **盲区监测（BSD，Blind Spot Detection）** 模块：顶置 LiDAR 按
侧别窗口监测相邻车道来车，接近速度差分估计 + 滞回状态机输出分级预警
（normal → approach → warn），覆盖邻车从斜后方逼近、穿越盲区、超越离开全过程。
只预警不干预驾驶，与前向 AEB 模块（`carla_aeb_braking`）互补。

## 组成

| 文件 | 职责 |
|---|---|
| `carla_bsd_blind_spot/bsd_core.py` | 决策核心（纯离线逻辑）：侧向窗口+车身高度带+径向直方图聚类测距、接近速度估计、normal/warn 滞回状态机 + approach 提前预警 |
| `carla_bsd_blind_spot/carla_common.py` | CARLA API 封装（复用 AEB 模块已验证实现）：连接/清理/生成/雷达/控制/恢复异步 |
| `carla_bsd_blind_spot/bridge_node.py` | ROS↔CARLA 桥接节点：自车+前顶 LiDAR，8 帧融合，发布左右盲区距离与自车速度 |
| `carla_bsd_blind_spot/bsd_node.py` | BSD 预警决策节点：定时器统一推进左右状态机，发布分级预警与跃迁事件 |
| `carla_bsd_blind_spot/demo.py` | 一键演示：离线邻车逼近场景 + `--live` 真实 CARLA 邻车超越场景（报警瞬间自动截图） |
| `main.py` | 统一入口：`doctor` / `test` / `build` / `demo` |
| `config/bsd_params.yaml` | 决策参数（预警/滞回阈值、盲区窗口） |
| `launch/main.launch.py` | 一键启动桥接 + 预警两节点 |

## 快速开始

```bash
# 1. 依赖体检
python3 main.py doctor

# 2. 离线单元测试（不依赖 ROS 与 CARLA）
python3 main.py test

# 3. 离线演示：邻车从左后方 35m 逼近穿越盲区（导出 PNG + CSV）
python3 main.py demo

# 4. 真实 CARLA 演示：邻车加速超越自车盲区（--host 填 CARLA 所在机器 IP）
python3 main.py demo --live --host 192.168.1.100

# 5. ROS 2 节点（桥接 + 预警）
ros2 launch carla_bsd_blind_spot main.launch.py host:=192.168.1.100
```

## 工作原理

```
盲区窗口（传感器系，LiDAR 装高 2.0m）：
  左：x ∈ [-25, 2]，y ∈ [2.9, 4.2]     右：y 取负对称
  z ∈ [-1.7, -0.4]（车身高度带，滤地面与路侧 2m 标志）

  normal ──d<12m──▶ warn               （滞回：warn 中 d>15m 才恢复）
    ▲
    └── approach：d<25m 且接近速度>0.5m/s（盲区外提前预警，与状态解耦）
```

- **测距**：侧向窗口过滤 → 径向距离直方图聚类（≥6 点最近箱），
  区分连续车身与孤立杂波；8 帧点云时间融合抗稀疏闪烁
- **分级预警**：normal（无目标）/ approach（远处快速接近）/ warn（已入盲区）
- **不控车**：BSD 只监测预警，自车不施加任何控制，变道决策留给驾驶员/上层

## 测试

离线单元测试覆盖：盲区测距（侧别/窗口/高度带/聚类/对称性）、接近速度估计、
状态机滞回与事件去重、桥接几何逻辑、预警节点分级语义、demo 导出结构。
`python3 main.py test` 运行，宿主机无 ROS 环境自动打桩，全部可离线跑。
