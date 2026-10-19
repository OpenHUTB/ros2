# waypoint_publisher

路径点发布与**动态目标点切换**：发布整条路线的路径点，同时把「当前目标点」单独发布
出来，并支持运行期间按序号切换、顺序推进、循环、手动「下一个」和就近吸附。

不依赖 `carla_ros_bridge`，也不依赖 CARLA —— 自带一台可选的模拟车辆（`mock/enabled`），
所以**没有仿真器也能跑通全链路并看到目标点依次切换**。

## 功能

- 发布 `/waypoints`（`geometry_msgs/PoseArray`，全部路径点，latched）与
  `/waypoint/target`（`geometry_msgs/PoseStamped`，当前目标点）
- 三种推进模式：`sequential`（顺序，走完结束）/ `loop`（循环）/ `manual`（只响应指令）
- 动态切换目标点：`/waypoint/set_index`（按序号跳转）、`/waypoint/next`（下一个）、
  `/waypoint/snap_nearest`（吸附到离当前位置最近的点）
- 到达判定：订阅 `/odom`，进入 `control/target_tolerance` 半径即视为到达并推进；
  阈值设为 `0` 可关闭自动推进
- 路线用纯文本配置（`x y yaw` 每行一个点），不依赖 PyYAML
- 内置模拟车辆（`mock/enabled: true`）自己发 `/odom`，不需要 CARLA
- 核心逻辑与 ROS 解耦，可跑离线回归测试（不需要 ROS）

## 1. 放入工作空间并构建

```bash
cp -rf waypoint_publisher ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

只依赖 `rospy` / `geometry_msgs` / `nav_msgs` / `std_msgs`，都是 ROS 自带，
不需要额外装库。

## 2. 运行

默认使用内置示例路线（10 m 见方环路、8 个点）：

```bash
roslaunch waypoint_publisher main.launch
```

想看到「目标点自动依次切换」，加 `mock_enabled:=true` 让本节点自带一台模拟车辆：

```bash
roslaunch waypoint_publisher main.launch mock_enabled:=true
```

也可以只跑节点（前台）：

```bash
rosrun waypoint_publisher main.py
```

## 3. 动态切换目标点

运行期间在**任意终端**执行：

```bash
# 跳到第 3 个路径点（1 起算）
rostopic pub -1 /waypoint/set_index std_msgs/Int32 "data: 3"

# 手动前进一个
rostopic pub -1 /waypoint/next std_msgs/Empty "{}"

# 吸附到离车辆当前位置最近的路径点
rostopic pub -1 /waypoint/snap_nearest std_msgs/Empty "{}"
```

## 话题接口

| 方向 | 话题 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| 发布 | `/waypoints` | `geometry_msgs/PoseArray` | 全部路径点（latched，晚连上的节点也能收到） |
| 发布 | `/waypoint/target` | `geometry_msgs/PoseStamped` | 当前目标点，按 `rate/publish_hz` 发布 |
| 发布 | `/waypoint/current_index` | `std_msgs/Int32` | 当前目标点下标（0 起算），变化时才发 |
| 发布 | `/odom` | `nav_msgs/Odometry` | 仅 `mock/enabled: true` 时由本节点模拟发布 |
| 订阅 | `/waypoint/set_index` | `std_msgs/Int32` | 切换到第 N 个路径点（1 起算，越界忽略） |
| 订阅 | `/waypoint/next` | `std_msgs/Empty` | 手动前进一个（`manual` 模式也生效） |
| 订阅 | `/waypoint/snap_nearest` | `std_msgs/Empty` | 吸附到最近路径点 |
| 订阅 | `/odom` | `nav_msgs/Odometry` | `mock/enabled: false` 时的位姿来源 |

## 参数

默认值见 `config/waypoint_publisher.yaml`，由 `main.launch` 加载，并由 launch `<arg>` 覆盖。

| 参数名 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- |
| `rate/publish_hz` | `10.0` | Hz | 目标点 / 状态的发布频率 |
| `frame/id` | `map` | — | 路径点所在坐标系 |
| `control/mode` | `sequential` | — | 推进模式：`sequential` / `loop` / `manual` |
| `control/target_tolerance` | `1.0` | m | 到达判定阈值；`<= 0` 关闭自动推进 |
| `control/start_index` | `0` | — | 初始目标点下标（0 起算，超范围自动夹住） |
| `control/autostart` | `true` | — | 启动后立即开始；`false` 时等切换指令 |
| `route/file` | `""` | — | 路线文件绝对路径；留空用内置示例路线 |
| `mock/enabled` | `false` | — | `true` 时自带模拟车辆并发布 `/odom` |
| `mock/speed` | `2.0` | m/s | 模拟车辆速度 |
| `topic/waypoints` | `/waypoints` | — | 全部路径点话题 |
| `topic/target` | `/waypoint/target` | — | 当前目标点话题 |
| `topic/current_index` | `/waypoint/current_index` | — | 当前下标话题 |
| `topic/set_index` | `/waypoint/set_index` | — | 切换指令话题 |
| `topic/odom` | `/odom` | — | 位姿话题 |

命令行覆盖示例（用 launch 参数名，不带 `rate/`、`control/` 等前缀）：

```bash
# 循环跑，每 0.5 m 就认为到达
roslaunch waypoint_publisher main.launch mode:=loop target_tolerance:=0.5

# 从第 4 个点开始，用随包的示例路线文件
roslaunch waypoint_publisher main.launch start_index:=3 \
    route_file:=$(rospack find waypoint_publisher)/config/demo_route.txt
```

## 路线文件格式

纯文本，每行一个路径点，写成 `x y yaw`（`yaw` 可省略，默认 0）：

```
# x y yaw —— # 之后是注释，空行忽略
0.0   0.0   0.0
5.0   0.0   0.0
10.0  10.0  0.0
```

分隔符可以用空格、制表符或逗号；坐标单位 m，`yaw` 单位 rad（逆时针为正）。
随包提供了 `config/demo_route.txt`（与内置示例路线完全一致，离线测试会校验两者相同）。

## 目录结构

```
waypoint_publisher/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── waypoint_publisher.yaml   # 全部可调参数（默认值）
│   └── demo_route.txt            # 示例路线（纯文本，格式样例）
├── launch/
│   └── main.launch               # 加载 yaml、暴露 <arg>、启动 main.py
└── scripts/
    ├── main.py                   # 入口（main.* 约定）
    ├── waypoint_core.py          # 纯逻辑：解析、游标、到达判定、步进（不依赖 ROS）
    └── waypoint_node.py          # 节点：发布 / 订阅 / 日志
```

## 离线测试

`waypoint_core.py` 不导入 rospy、也不依赖 PyYAML，因此核心逻辑可以在没有 ROS、
没有 CARLA、没有图形界面的机器上直接跑离线回归：

```bash
cd <仓库根目录>
python3 tests/test_waypoint_publisher_local.py   # Noetic / 任意 Python 3
python2 tests/test_waypoint_publisher_local.py   # Melodic / Python 2.7
```

覆盖参数默认值与覆盖、路线文本解析（含各类报错）、内置路线与随包示例文件一致性、
三种推进模式与动态切换、到达判定、最近点吸附、模拟车辆步进、四元数互转，
以及工程文件约定（入口为 `main.py`、launch 加载 yaml 并暴露参数、文档页与本模块一致等）。
全部通过时打印 `全部通过` 并以 0 退出。

## 常见问题

- **目标点不自动切换**：确认 `/odom` 有数据（`rostopic hz /odom`），或加
  `mock_enabled:=true` 用内置模拟车辆；另外 `control/target_tolerance` 必须大于 0。
- **切换指令没反应**：`/waypoint/set_index` 的下标 **1 起算**，越界会被忽略并在终端打印警告。
- **`roslaunch` 报找不到 `main.py`**：先 `catkin_make` 并 `source devel/setup.bash`。
- **Melodic 上启动即报 `No module named 'rospy'`**：脚本 shebang 按 Noetic 约定为 `python3`，
  Melodic 的 rospy 在 `python2` 里，需要让脚本由 `python2` 启动
  （`catkin_install_python` 会按发行版重写 shebang），或直接
  `python2 $(rospack find waypoint_publisher)/scripts/main.py`。
