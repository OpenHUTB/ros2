# bug2_navigation

Bug2 反应式导航教学模块：不建图的"撞墙—沿墙—回线"完备导航，含确定性
不可达判据（沿墙撞到相邻墙体即宣告无解，这是 A*/RRT 给不了的答案）。

## 两种运行方式

### 一、ROS 2 闭环：仿真器 + 导航/控制，两个节点真跑起来

把本目录放进 ROS 2 工作空间的 `src/` 下：

```bash
cd <ros2_ws>
colcon build --packages-select bug2_navigation
source install/setup.bash
ros2 launch bug2_navigation bug2_navigation.launch.py
```

launch 会同时起两个节点，构成完整数据流：

```
sim_node（运动学模拟器 / 传感器一侧）
        │  /odom  (nav_msgs/Odometry)
        ▼
plan_control_node（Bug2 状态机 + 跟踪控制）
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
| 发布 | `/planning/path` | `nav_msgs/Path` | Bug2 参考轨迹，RViz 里加 Path 显示即可看到 |

观察与调试：

```bash
ros2 topic echo /odom                  # 看车在不在动
ros2 topic echo /cmd_vel               # 看控制输出
ros2 topic echo /planning/path --once  # 看参考轨迹路点
rviz2                                  # 添加 Path，Topic 选 /planning/path
```

窄门腔不可达演示（门宽 2.0 m，机器人安全间隙需求 2.4 m）：

```bash
# launch 换场景（注意把模拟器起点放到腔外）
ros2 launch bug2_navigation bug2_navigation.launch.py scenario:=walls x0:=29.0 y0:=5.0 yaw0:=1.5708
# 节点日志会逐条打印 hit / leave / blocked 事件，宣告不可达后清空参考
# 轨迹、不发布 cmd_vel，机器人保持静止（有 odom 反馈也不盲动）
```

参数（launch 里都能改）：`x0`/`y0`/`yaw0`（模拟器起点与朝向）、`v`（巡航速度）、
`scenario`（standard / walls）。

### 二、纯离线仿真（不需要装 ROS 2）

```bash
cd src/reactive_navigation/bug2_navigation
./main.sh                    # 等价于 python3 scripts/main.py
```

配图默认写到 `docs/img/reactive_navigation/bug2_navigation/`，用
`--outdir <目录>` 可改。脚本目录下不再保留 PNG 副本，同一张图不会入库两次。

未安装 `rclpy` 时两个节点会提示需要 ROS 2 环境并退出，离线脚本不受影响。

## 目录结构与入口

| 路径 | 说明 |
|---|---|
| `main.sh` | 本包入口，离线出图（`python3 scripts/main.py` 的包装，安装后在 `lib/bug2_navigation/` 里同样可用） |
| `scripts/main.py` | 离线演示主体：跑标准/窄门/间隙扫描三组实验、打印数值、生成配图 |
| `scripts/bug2.py` | 算法：m-line 直行、矩形边界弧长参数化、沿墙、hit/leave、相邻墙不可达判定 |
| `scripts/simulate.py` | 出图与指标汇总 |
| `scripts/plan_control_node.py` | ROS 2 导航/控制节点（订阅 `/odom`，发布 `/cmd_vel`、`/planning/path`） |
| `tests/test_bug2.py` | 12 项离线单测（含跟踪回放、相邻障碍保守判据已知限制回归） |
| `tests/test_ros_params.py` | 4 项 rclpy 接口桩测试（参数声明/读回、cmd_vel 与 planning/path 发布） |

> `sim_node.py` 与 `simple_plot.py` 三个模块共用，源码在 `../common/`
> （见 [`../common/README.md`](../common/README.md)），`colcon build` 时拷进本包的 `lib/`。

## 测试

```bash
cd src/reactive_navigation/bug2_navigation
python3 -m unittest discover -s tests -t .
```

覆盖：标准场景必达且路径无碰撞、窄门腔确定性不可达（`blocked` 事件）、
m-line 几何、边界弧长参数化往返闭环、跟踪回放 90 s 内到达且全程距障碍 > 0.3 m。

## 依赖

- Python 3 标准库（无 numpy / matplotlib）
- ROS 2 可选：`rclpy`、`geometry_msgs`、`nav_msgs`

## 关于验证范围（请评审注意）

- 离线仿真数值已在本机实跑，结果见文档。
- 跟踪闭环已在无 ROS 环境下按 20 Hz 积分回放验证：全程不进入任何障碍，
  抵达目标 0.8 m 半径内；已写成 `tests/test_bug2.py` 回归测试。
- **尚未在安装了 ROS 2 发行版（Humble / Jazzy）的机器上 `colcon build` 实测**，
  构建与 launch 的具体输出请以目标环境为准。
