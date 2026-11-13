# lidar_intensity_traversability

用激光雷达**回波强度**辅助判断可通行性的差速小车避障示例。

## 为什么不用"只看距离"的做法

镜面玻璃、黑色泡棉这类表面回波很弱：几何上它们是障碍，但 `/scan` 的
`ranges` 看起来像"远处/空旷"。只用 `min(ranges)` 的避障逻辑会在这两种
物体前减速过晚甚至直接撞上去。本模块把 `intensities` 折算成回波置信度，
再按不确定度把低置信度的回波视为更近的危险源。

## 目录

```
lidar_intensity_traversability/
├── main.py                 # 直接运行入口
├── launch/main.launch.py   # 同时启动仿真与控制器
├── lidar_intensity_traversability/
│   ├── core.py             # 纯算法（无 ROS 依赖，可离线测试）
│   ├── node.py             # /scan -> /cmd_vel
│   └── scan_sim.py         # 带强度的激光雷达仿真（玻璃 / 泡棉场景）
├── scripts/make_figure.py  # 由 CSV 生成轨迹与有效距离图
└── test/test_core.py       # 单元测试
```

## 环境

- Ubuntu 20.04 + ROS 2 Foxy（实测）
- Python 3.8
- 无 Gazebo 依赖：仿真由 `scan_sim` 节点在进程内完成

## 构建与运行

```bash
cd ~/ros2_ws
colcon build --packages-select lidar_intensity_traversability
source install/setup.bash
ros2 launch lidar_intensity_traversability main.launch.py
```

单独运行控制器（需要外部提供 `/scan`）：

```bash
ros2 run lidar_intensity_traversability main --ros-args -p max_speed:=0.3
```

生成配图：

```bash
python3 src/chap9/lidar_intensity_traversability/scripts/make_figure.py \
  --csv /tmp/lidar_intensity_demo.csv --out /tmp/lidar_intensity_demo.png
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `max_speed` | 0.35 | 最大线速度 (m/s) |
| `fov_deg` | 120 | 前向视场角 |
| `stop_distance` | 0.45 | 有效距离小于该值停车 (m) |
| `slow_distance` | 1.20 | 有效距离小于该值按比例减速 (m) |
| `ref_intensity` | 1200 | 满置信度参考强度 |
| `uncertainty` | 0.60 | 低置信度回波的距离折算系数 |

## 验证范围（如实说明）

- ✅ 单元测试覆盖：强度置信度、有效距离折算、扇区最小距离、选向逻辑
- ✅ 在 Ubuntu 20.04 + ROS 2 Foxy 上 `colcon build` 通过
- ✅ `ros2 launch` 实际运行，`/scan`、`/cmd_vel`、`/traversability/decision` 话题有数据，终端日志与截图见文档页
- ⚠️ 仿真由 `scan_sim` 的几何世界提供，**不是 Gazebo/TurtleBot3 真机或真实雷达**
- ⚠️ 未在 Humble 上复测；未做真实玻璃/泡棉标定，`ref_intensity` 需按实际雷达调整
