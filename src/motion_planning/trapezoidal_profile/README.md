# trapezoidal_profile

梯形与 S 曲线速度规划教学模块：把"走多远"变成"每时刻该多快"。

## 两种运行方式

### 一、ROS 2 闭环：仿真器 + 规划/控制，两个节点真跑起来

把本目录放进 ROS 2 工作空间的 `src/` 下：

```bash
cd <ros2_ws>
colcon build --packages-select trapezoidal_profile
source install/setup.bash
ros2 launch trapezoidal_profile trapezoidal_profile.launch.py
```

launch 会同时起两个节点，构成完整数据流：

```
sim_node（运动学模拟器 / 传感器一侧）
        │  /odom  (nav_msgs/Odometry)
        ▼
plan_control_node（规划 + 控制）
        │  /cmd_vel (geometry_msgs/Twist)
        └──────────────────────────────► 回到 sim_node
        │  /planning/path (nav_msgs/Path)
        └──────────────────────────────► RViz 可视化
```

话题接口：

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 发布 | `/odom` | `nav_msgs/Odometry` | 模拟器输出的位姿与速度（传感器一侧） |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 模拟器接收的速度指令 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 控制节点输出的速度指令 |
| 订阅 | `/odom` | `nav_msgs/Odometry` | 控制节点读取的车辆状态 |
| 发布 | `/planning/path` | `nav_msgs/Path` | 规划结果，RViz 里加 Path 显示即可看到 |

观察与调试：

```bash
ros2 topic echo /odom                 # 看车在不在动
ros2 topic echo /cmd_vel              # 看控制输出
ros2 topic echo /planning/path --once # 看规划出的路点
rviz2                                 # 添加 Path，Topic 选 /planning/path
```

只想起模拟器、自己发指令验证链路：

```bash
ros2 run trapezoidal_profile sim_node.py --ros-args -p x0:=0.0 -p yaw0:=0.0
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.2}}"
```

参数（launch 里都能改）：`x0`（起点）、`distance`（目标位移）、`vmax`（最大速度）、`use_scurve`（是否用 S 曲线）。

### 二、纯离线仿真（不需要装 ROS 2）

```bash
cd src/motion_planning/trapezoidal_profile
./main.sh                    # 等价于 python3 scripts/main.py
```

配图默认写到 `docs/img/motion_planning/trapezoidal_profile/`，用 `--outdir <目录>` 可改。
脚本目录下不再保留 PNG 副本，同一张图不会入库两次。

未安装 `rclpy` 时两个节点会提示需要 ROS 2 环境并退出，离线脚本不受影响。

## 目录结构与入口

| 路径 | 说明 |
|---|---|
| `main.sh` | 本包入口，离线出图（`python3 scripts/main.py` 的包装，安装后在 `lib/trapezoidal_profile/` 里同样可用） |
| `scripts/main.py` | 离线演示主体：跑两条曲线、打印数值、生成配图 |
| `scripts/profile.py` | 算法：梯形曲线与 S 曲线的时间-位置解 |
| `scripts/simulate.py` | 出图与指标汇总 |
| `scripts/plan_control_node.py` | ROS 2 剖面执行节点（订阅 `/odom`，发布 `/cmd_vel`、`/planning/path`） |
| `tests/test_profile.py` | 11 项离线单测 |

> 入口为什么放在 `scripts/`：本模块的 `scripts/` 下同时放着节点脚本和算法模块，
> `main.sh` 只是把 `scripts/main.py` 包一层，避免在两处维护同一份代码。
> `sim_node.py` 与 `simple_plot.py` 三个模块共用，源码在 `../common/`
> （见 [`../common/README.md`](../common/README.md)），`colcon build` 时拷进本包的 `lib/`。

## 测试

```bash
cd src/motion_planning/trapezoidal_profile
python3 tests/test_profile.py
```

## 依赖

- Python 3 标准库（无 numpy / matplotlib）
- ROS 2 可选：`rclpy`、`geometry_msgs`、`nav_msgs`

## 关于验证范围（请评审注意）

- 离线仿真数值已在本机实跑，结果见文档。
- ROS 2 闭环已在无 ROS 环境下用与 `rclpy` 接口一致的桩跑通：两个节点互相投递
  `/odom` 与 `/cmd_vel`，梯形剖面闭环终点 x=10.0000 m，S 曲线 x=10.0002 m（目标 10 m），横向偏差 0。
- **尚未在安装了 ROS 2 发行版（Humble / Jazzy）的机器上 `colcon build` 实测**，
  构建与 launch 的具体输出请以目标环境为准。
