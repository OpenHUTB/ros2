# potential_field_avoidance

人工势场（Artificial Potential Field）反应式避障教学模块，含局部极小的
切向滑行逃逸。

## 两种运行方式

### 一、ROS 2 闭环：仿真器 + 导航/控制，两个节点真跑起来

把本目录放进 ROS 2 工作空间的 `src/` 下：

```bash
cd <ros2_ws>
colcon build --packages-select potential_field_avoidance
source install/setup.bash
ros2 launch potential_field_avoidance potential_field_avoidance.launch.py
```

launch 会同时起两个节点，构成完整数据流：

```
sim_node（运动学模拟器 / 传感器一侧）
        │  /odom  (nav_msgs/Odometry)
        ▼
plan_control_node（势场导航 + 跟踪控制）
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
| 发布 | `/planning/path` | `nav_msgs/Path` | 势场下降得到的参考轨迹，RViz 里加 Path 显示即可看到 |

观察与调试：

```bash
ros2 topic echo /odom                  # 看车在不在动
ros2 topic echo /cmd_vel               # 看控制输出
ros2 topic echo /planning/path --once  # 看参考轨迹路点
rviz2                                  # 添加 Path，Topic 选 /planning/path
```

凹形陷阱演示（局部极小 + 切向滑行逃逸）：

```bash
ros2 run potential_field_avoidance sim_node.py --ros-args -p x0:=30.0 -p y0:=5.0 -p yaw0:=1.5708
ros2 run potential_field_avoidance plan_control_node.py --ros-args -p trap:=true
# 关掉逃逸看"卡死"：-p escape:=false —— 节点提示局部极小后清空参考
# 轨迹、不发布 cmd_vel，机器人保持静止（有 odom 反馈也不盲动）
```

参数（launch 里都能改）：`x0`/`y0`/`yaw0`（模拟器起点与朝向）、`v`（巡航速度）。
节点另支持 `trap`（切换陷阱场景）、`escape`（切向滑行开关，默认开）。

### 二、纯离线仿真（不需要装 ROS 2）

```bash
cd src/reactive_navigation/potential_field_avoidance
./main.sh                    # 等价于 python3 scripts/main.py
```

配图默认写到 `docs/img/reactive_navigation/potential_field_avoidance/`，用
`--outdir <目录>` 可改。脚本目录下不再保留 PNG 副本，同一张图不会入库两次。

未安装 `rclpy` 时两个节点会提示需要 ROS 2 环境并退出，离线脚本不受影响。

## 目录结构与入口

| 路径 | 说明 |
|---|---|
| `main.sh` | 本包入口，离线出图（`python3 scripts/main.py` 的包装，安装后在 `lib/potential_field_avoidance/` 里同样可用） |
| `scripts/main.py` | 离线演示主体：跑标准/陷阱/参数扫描三组实验、打印数值、生成配图 |
| `scripts/apf.py` | 算法：引力/斥力势与梯度、自适应步长下降、震荡检测、切向滑行逃逸 |
| `scripts/simulate.py` | 出图与指标汇总 |
| `scripts/plan_control_node.py` | ROS 2 导航/控制节点（订阅 `/odom`，发布 `/cmd_vel`、`/planning/path`） |
| `tests/test_apf.py` | 18 项离线单测（含逃逸不出地图与跟踪回放回归） |
| `tests/test_ros_params.py` | 6 项 rclpy 接口桩测试（参数声明/读回、cmd_vel 与 planning/path 发布、卡死时有 odom 也不盲动） |

> 入口为什么放在 `scripts/`：本模块的 `scripts/` 下同时放着节点脚本和算法模块，
> `main.sh` 只是把 `scripts/main.py` 包一层，避免在两处维护同一份代码。
> `sim_node.py` 与 `simple_plot.py` 三个模块共用，源码在 `../common/`
> （见 [`../common/README.md`](../common/README.md)），`colcon build` 时拷进本包的 `lib/`。

## 测试

```bash
cd src/reactive_navigation/potential_field_avoidance
python3 -m unittest discover -s tests -t .
```

覆盖：梯度解析式与数值差分逐点对照（曾抓出斥力符号写反的缺陷）、
d0 边界归零、陷阱场景关逃逸必卡死/开逃逸必到达、跟踪回放 90 s 内到达且
全程距障碍 > 0.3 m。

## 依赖

- Python 3 标准库（无 numpy / matplotlib）
- ROS 2 可选：`rclpy`、`geometry_msgs`、`nav_msgs`

## 关于验证范围（请评审注意）

- 离线仿真数值已在本机实跑，结果见文档。
- 跟踪闭环已在无 ROS 环境下按 20 Hz 积分回放验证：全程不进入任何障碍，
  抵达目标 0.8 m 半径内；已写成 `tests/test_apf.py::TestTracking` 回归测试。
- **尚未在安装了 ROS 2 发行版（Humble / Jazzy）的机器上 `colcon build` 实测**，
  构建与 launch 的具体输出请以目标环境为准。
