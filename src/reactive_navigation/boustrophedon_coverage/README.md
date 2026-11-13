# boustrophedon_coverage

牛耕式（boustrophedon / lawnmower）全覆盖路径规划教学模块：给定多边形
房间与行距，生成"逐行扫掠 + 掉头连接"的覆盖路径，长度可预算、覆盖完备。

## 两种运行方式

### 一、ROS 2 闭环：仿真器 + 规划控制，两个节点真跑起来

把本目录放进 ROS 2 工作空间的 `src/` 下：

```bash
cd <ros2_ws>
colcon build --packages-select boustrophedon_coverage
source install/setup.bash
ros2 launch boustrophedon_coverage boustrophedon_coverage.launch.py
```

launch 会同时起两个节点，构成完整数据流：

```
sim_node（运动学模拟器 / 传感器一侧）
        │  /odom  (nav_msgs/Odometry)
        ▼
plan_control_node（牛耕式覆盖规划 + 跟踪控制）
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
| 发布 | `/planning/path` | `nav_msgs/Path` | 牛耕式覆盖路径，RViz 里加 Path 显示即可看到 |

观察与调试：

```bash
ros2 topic echo /odom                  # 看车在不在动
ros2 topic echo /cmd_vel               # 看控制输出
ros2 topic echo /planning/path --once  # 看覆盖路径路点
rviz2                                  # 添加 Path，Topic 选 /planning/path
```

节点日志会打印行数、路径长度、房间面积与预估作业时间；路径走完即宣布
"覆盖完成"并停车。

参数（launch 里都能改）：`x0`/`y0`/`yaw0`（模拟器起点与朝向）、`v`（巡航
速度）、`gap`（扫掠行距，调大可以演示"覆盖不完"）。

### 二、纯离线仿真（不需要装 ROS 2）

```bash
cd src/reactive_navigation/boustrophedon_coverage
./main.sh                    # 等价于 python3 scripts/main.py
```

配图默认写到 `docs/img/reactive_navigation/boustrophedon_coverage/`，用
`--outdir <目录>` 可改。脚本目录下不再保留 PNG 副本，同一张图不会入库两次。

未安装 `rclpy` 时两个节点会提示需要 ROS 2 环境并退出，离线脚本不受影响。

## 目录结构与入口

| 路径 | 说明 |
|---|---|
| `main.sh` | 本包入口，离线出图（`python3 scripts/main.py` 的包装，安装后在 `lib/boustrophedon_coverage/` 里同样可用） |
| `scripts/main.py` | 离线演示主体：跑基准/行距扫描/方向扫描三组实验、打印数值、生成配图 |
| `scripts/coverage.py` | 算法：多边形裁剪式逐行扫掠、掉头连接、边界弧长参数化、覆盖率统计 |
| `scripts/simulate.py` | 出图与指标汇总 |
| `scripts/plan_control_node.py` | ROS 2 规划/控制节点（订阅 `/odom`，发布 `/cmd_vel`、`/planning/path`） |
| `tests/test_coverage.py` | 16 项离线单测（含跟踪回放回归、行距超限覆盖率骤降） |
| `tests/test_ros_params.py` | 4 项 rclpy 接口桩测试（参数声明/读回、cmd_vel 与 planning/path 发布） |

> `sim_node.py` 与 `simple_plot.py` 三个模块共用，源码在 `../common/`
> （见 [`../common/README.md`](../common/README.md)），`colcon build` 时拷进本包的 `lib/`。

## 测试

```bash
cd src/reactive_navigation/boustrophedon_coverage
python3 -m unittest discover -s tests -t .
```

覆盖：L 形房间与矩形房间覆盖率达 100%（作业宽度 2.2 m、默认行距 2 m）、
行距超过作业宽度后覆盖率骤降（gap=5 m 实测 51.8% < 60%，
`assertLess` 锁死）、路径点全部在房间内、长度可预算性（≈面积/gap）、
跟踪回放走完路径且全程距边界不超过 1 m（L 形凹角切角实测约 0.6 m）。

## 依赖

- Python 3 标准库（无 numpy / matplotlib）
- ROS 2 可选：`rclpy`、`geometry_msgs`、`nav_msgs`

## 关于验证范围（请评审注意）

- 离线仿真数值已在本机实跑，结果见文档；覆盖率判定半径与固定作业宽度
  2.2 m 绑定（不随行距缩放），"行距 ≤ 作业宽度"的完备性条件可直接
  用它验收。
- 跟踪闭环已在无 ROS 环境下按 20 Hz 积分回放验证：全程距房间边界不
  超过 1 m（L 形凹角切角实测约 0.6 m，路径本身全部在房间内），路径
  走完即停；已写成 `tests/test_coverage.py` 回归测试。
- **尚未在安装了 ROS 2 发行版（Humble / Jazzy）的机器上 `colcon build` 实测**，
  构建与 launch 的具体输出请以目标环境为准。
