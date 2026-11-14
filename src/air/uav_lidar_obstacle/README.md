# 无人机激光雷达避障（uav_lidar_obstacle）

`src/air/uav_lidar_obstacle/`

用机载激光雷达的点云求前方最近障碍物距离：小于阈值就悬停，否则向前飞。
本包同时提供两条互不依赖的运行路径：

| 路径 | 入口 | 依赖 | 适用场景 |
|---|---|---|---|
| 纯 CARLA 脚本 | `drone_control.py` | `carla` Python 包、OpenHUTB / CARLA 服务端 | 按实验报告直接演示避障 |
| ROS 2 节点 | `uav_lidar_obstacle_node` | `rclpy` + `sensor_msgs` / `geometry_msgs` | 与 `carla_ros_bridge` 等 ROS 2 组件联调 |

两者是**同一套控制律的两个入口**，不是把非 ROS 脚本塞进 `console_scripts`：
`drone_control.py` 里没有 `rclpy`、没有节点类，它直接调 CARLA 的
`drone.set_velocity()`；ROS 2 路径由 `uav_lidar_obstacle/node.py` 里真正的
`rclpy` 节点实现，输出 `geometry_msgs/Twist` 而不是 CARLA 客户端调用。

## 1. 目录结构

```
uav_lidar_obstacle/
├── drone_control.py                 纯 CARLA 脚本（不依赖 ROS，可直接 python 运行）
├── uav_lidar_obstacle/
│   ├── __init__.py
│   └── node.py                      ROS 2 节点 UavLidarObstacleNode
├── launch/
│   └── uav_lidar_obstacle.launch.py 可组合启动文件（参数全部可在命令行覆盖）
├── resource/uav_lidar_obstacle      ament 资源标记
├── package.xml                      format 3，ament_python
├── setup.cfg
└── setup.py                         entry_points: uav_lidar_obstacle_node
```

## 2. 纯 CARLA 脚本路径

前提：已启动 OpenHUTB / CARLA 服务端（默认 `127.0.0.1:2000`），场景里已有带
lidar 传感器的无人机 Actor，且 `carla` Python 包可导入（把仿真器自带的
`PythonAPI/carla/dist/*.egg` 加入 `PYTHONPATH`，或用 `CARLA_PY_PATH` 指向它）。

```bash
cd src/air/uav_lidar_obstacle
python drone_control.py
```

脚本会打印连接状态、无人机与雷达检索结果，并在终端持续输出"前方安全 /
停止飞行"；`Ctrl+C` 退出时停止雷达监听并把速度置零，避免残留活动回调。

## 3. ROS 2 节点路径

```bash
# 编译（工作空间根目录）
colcon build --packages-select uav_lidar_obstacle
source install/setup.bash

# 启动节点（参数全部有默认值）
ros2 launch uav_lidar_obstacle uav_lidar_obstacle.launch.py

# 或直接跑可执行文件
ros2 run uav_lidar_obstacle uav_lidar_obstacle_node
```

节点启动前需要有点云发布者。用本仓库的 `carla_ros_bridge` 时，无人机上挂的
lidar 传感器会发布到 `<role_name>/lidar/point_cloud`，默认参数按
`/carla/lidar/point_cloud` 取；话题名不一致时用 `lidar_topic:=...` 覆盖。

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `lidar_topic`（默认 `/carla/lidar/point_cloud`） | `sensor_msgs/PointCloud2` | 订阅 | 机载激光雷达点云 |
| `cmd_vel_topic`（默认 `/uav/cmd_vel`） | `geometry_msgs/Twist` | 发布 | `linear.x` = 机体系前向速度，其余分量为 0 |
| `~/min_range` | `std_msgs/Float32` | 发布 | 实时最近障碍距离（米），无有效点时 NaN |

| 参数 | 默认值 | 含义 |
|---|---|---|
| `lidar_topic` | `/carla/lidar/point_cloud` | 点云话题 |
| `cmd_vel_topic` | `/uav/cmd_vel` | 速度指令话题 |
| `stop_distance` | `4.0` | 停车阈值（米） |
| `forward_speed` | `2.0` | 前飞速度（米/秒） |
| `control_rate_hz` | `10.0` | 控制频率 |
| `data_timeout_s` | `1.0` | 点云超时（秒） |
| `publish_min_range` | `true` | 是否发布 `~/min_range` |

控制律：`min_range < stop_distance` → 发布零速度；否则发布 `forward_speed`。
点云超过 `data_timeout_s` 未更新时按**故障安全**停车（看不见就停），不沿用
上一帧判断继续前飞。

点云解析用 `struct` 逐点读取 `x/y/z` 字段，不依赖 numpy 或 `sensor_msgs_py`；
CARLA 对未命中光束给出的 `(0, 0, 0)` 与 NaN / inf 都按"无回波"丢弃。

## 4. 与实验报告的关系

实验报告（`docs/air/uav_lidar_obstacle/index.md`）描述的是纯 CARLA 脚本路径：
场景搭建、低分辨率启动、终端运行与现象观察。ROS 2 路径是同一控制律在话题
层面的复刻，报告第 5 节给出 `ros2 launch` 的启动方式与话题核对方法。

## 5. 已知局限

- 只取全向最小距离做阈值判断，不区分障碍物方位，因此只会"停"不会"绕"。
- 阈值固定，没有随速度 / 场景自适应。
- ROS 2 路径假设上游已把点云发布出来；本包不做传感器创建与仿真器启动。

## 6. 自检

```bash
# 语法检查（含 launch 文件）
python -m py_compile drone_control.py uav_lidar_obstacle/__init__.py \
    uav_lidar_obstacle/node.py launch/uav_lidar_obstacle.launch.py

# package.xml 可被 XML 解析
python -c "import xml.etree.ElementTree as ET; ET.parse('package.xml')"
```

没有 ROS 2 环境时 `py_compile` 与 XML 解析即可完成静态自检；`colcon build` 需要
本机安装 ROS 2（Humble 及以上）。
