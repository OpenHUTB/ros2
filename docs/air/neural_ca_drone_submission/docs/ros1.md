# ROS 1（Noetic）部署

## 1. 包结构

```
ros1_ws/
└── src/
    └── neural_ca_drone_ros/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   ├── main.launch          # 总入口
        │   ├── full.launch          # 四模块串联
        │   ├── perception.launch
        │   ├── planning.launch
        │   ├── control.launch
        │   └── e2e.launch
        └── scripts/
            ├── main.py              # 推理节点
            └── sim_bridge.py        # CA-Sim → ROS 桥
```

`main.py` 以 `main.` 开头，满足“入口统一命名”的要求；
`main.launch` 是模块级总入口，四个子 launch 分别对应四个算法模块。

## 2. 依赖

| 依赖 | 版本 | 说明 |
| --- | --- | --- |
| Ubuntu | 20.04 | 已验证 |
| ROS | Noetic | `ros-noetic-desktop-full` |
| Python | 3.8 | 系统自带 |
| numpy | ≥1.20 | `python3-numpy` |
| Pillow | ≥7.0 | `python3-pil` |
| 项目本体 | — | `pip3 install -e .`（仓库根目录） |

不需要 PyTorch、CUDA 或任何 GPU 驱动。

## 3. 构建

```bash
# 1) 安装项目本体（提供 ca_drone 包与权重）
cd neural_ca_drone
pip3 install -e .

# 2) 构建 ROS 包
cd ros1_ws
catkin_make
source devel/setup.bash
```

实测输出（Ubuntu 20.04 虚拟机）：

```
-- Using CATKIN_DEVEL_PREFIX: .../devel
-- Configuring done
-- Generating done
-- Build files have been written to: .../build
[100%] Built target neural_ca_drone_ros
```

## 4. 运行

### 4.1 一键启动（仿真桥 + 推理节点 + RViz 可选）

```bash
roslaunch neural_ca_drone_ros main.launch scene:=sparse module:=full
```

| 参数 | 默认 | 取值 |
| --- | --- | --- |
| `scene` | `sparse` | sparse / dense / dynamic / corridor |
| `module` | `full` | full / perception / planning / control / e2e |
| `rate` | `10` | 控制频率 [Hz] |
| `rviz` | `false` | 是否启动 RViz |
| `goal_x`, `goal_y` | 49, 0 | 目标点 |

### 4.2 只启动单个模块

```bash
roslaunch neural_ca_drone_ros perception.launch
roslaunch neural_ca_drone_ros planning.launch
roslaunch neural_ca_drone_ros control.launch
roslaunch neural_ca_drone_ros e2e.launch
```

### 4.3 手动验证

```bash
# 终端 A
roslaunch neural_ca_drone_ros main.launch scene:=sparse module:=full

# 终端 B
rostopic hz /camera/depth/image_raw      # 期望 ~10 Hz
rostopic echo -n1 /cmd_vel
rostopic echo -n1 /ca/status
```

实测（虚拟机，`module:=full`）：

```
$ rostopic hz /camera/depth/image_raw
average rate: 10.033
	min: 0.090s max: 0.109s std dev: 0.00683s window: 10

$ rostopic echo -n1 /cmd_vel
linear:
  x: 1.1999
  y: 0.0153
  z: 0.0
angular:
  z: 1.1634

$ rostopic echo -n1 /ca/status
module: full
cmd: [1.20, -0.05]
yaw_rate: 0.84
speed: 1.20
```

## 5. 话题

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 发布 | 32FC1 深度，64×48，米 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 世界系位姿与速度 |
| `/tf` | `tf2_msgs/TFMessage` | 发布 | `map → base_link` |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 订阅 | 目标点（可选） |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 机体系速度指令 |
| `/ca/occupancy` | `nav_msgs/OccupancyGrid` | 发布 | 在线栅格（每 10 帧） |
| `/ca/path` | `nav_msgs/Path` | 发布 | 当前 A\* 路径 |
| `/ca/status` | `std_msgs/String` | 发布 | 模块状态摘要 |

`/ca/status` 的 `data` 是类 JSON 的 `key=value` 串，便于 `grep` 与日志记录：

```
module=full cmd=(1.20,-0.05) yaw_rate=0.84 speed=1.20
```

## 6. 与 Gazebo 的接口

`sim_bridge.py` 只是“发布深度 + 订阅速度”的仿真器；把这一节点换成
Gazebo 的深度相机与位姿真值即可，推理节点无需改动：

```
Gazebo 深度相机 ──/camera/depth/image_raw──▶ main.py ──/cmd_vel──▶ PX4/控制器
```

深度图分辨率与内参通过 `scripts/main.py` 中的 `decode_depth(msg, 64, 48, 12.0)`
适配；若相机为 640×480，改这三处常量即可。

## 7. 常见问题

**Q: `ImportError: No module named ca_drone`**
A: 未执行 `pip3 install -e .`，或在错误的 Python 环境里。

**Q: `roslaunch` 报找不到包**
A: 忘记 `source devel/setup.bash`。

**Q: 深度图是空的**
A: `sim_bridge` 未启动，或 `scene` 参数拼写错误（只接受五个场景名）。

**Q: 控制频率不稳**
A: 虚拟机 CPU 争抢导致；把 `rate:=5` 可稳定，但轨迹会变粗糙。
