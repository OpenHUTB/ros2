# ROS Scenario Runner

## 参考

* [Carla ROS Scenario Runner](https://openhutb.github.io/doc/carla_ros_scenario_runner/)

## 这一页解决什么问题

上面的参考链接指向 CARLA 官方的 scenario_runner：它很强，但要求先起 CARLA 服务端、
再配 PythonAPI、再写一份 XML 的 OpenSCENARIO。对只想验证「我这段控制逻辑在某个工况下
表现是否正常」的人来说，这套链路太重了——虚拟机里光开 CARLA 就要吃掉几个 G 内存，
一次回归跑下来十几分钟，还很难在 CI 或同学的笔记本上复现。

本页介绍仓库里 `src/ground/ros_scenario_runner` 这个包：把**一次驾驶写成一份 YAML 场景脚本**
（什么时刻发什么速度指令、结束后应当满足哪些指标），运行器按时序把指令发到 `/cmd_vel`，
把车辆状态记成轨迹，跑完逐条判定断言，打印一份带 `PASS` / `FAIL` 的报告。

它和官方 scenario_runner 不是替代关系，而是分工：

| | CARLA scenario_runner | 本模块 |
|---|---|---|
| 场景描述 | OpenSCENARIO（XML） | 一份 YAML，人眼可读 |
| 依赖 | CARLA 服务端 + PythonAPI | 只要 ROS，内置车辆模型时连模拟器都不用 |
| 适合的粒度 | 交通参与者、天气、行人的完整剧情 | 模块级的工况回归：加速、刹车、转弯、超速保护 |
| 单次耗时 | 分钟级 | 秒级 |

## 本地场景脚本长什么样

`config/scenarios/demo_brake_stop.yaml` 是一份完整例子：

```yaml
scenario:
  name: 直线加速与刹停
  duration: 14.0
  initial: {x: 0.0, y: 0.0, yaw: 0.0, speed: 0.0}
  steps:
    - at: 0.5
      name: 起步
      command: {linear: 5.0, angular: 0.0}
    - at: 8.0
      name: 刹停
      command: {linear: 0.0, angular: 0.0}
  expects:
    - type: max_speed
      value: 5.0
      tolerance: 0.1
    - type: reach_speed
      value: 5.0
      tolerance: 0.05
    - type: final_speed
      value: 0.0
      tolerance: 0.05
    - type: distance
      min: 30.0
      max: 35.0
    - type: position
      x: 32.3
      y: 0.0
      tolerance: 0.5
    - type: heading
      value: 0.0
      tolerance: 0.01
```

三个字段的含义：

* `steps`：时间轴。`at` 是相对开始运行的秒数，到点就把 `command` 发出去，一直保持到下一条。
* `expects`：跑完之后要满足的条件。任何一条不满足，结论就是 `FAILED`。
* `duration`：跑多久。必须不小于最后一条 `at`。

## 六种断言

| type | 需要给的字段 | 判定内容 |
|---|---|---|
| `max_speed` | `value`、`tolerance` | 全程峰值速度不超过 `value + tolerance` |
| `reach_speed` | `value`、`tolerance` | 峰值速度至少到 `value - tolerance`（"确实加到了"） |
| `final_speed` | `value`、`tolerance` | 结束那一刻的速度 |
| `distance` | `min`、`max`（至少一个） | 累计里程落在区间内 |
| `position` | `x`、`y`、`tolerance` | 结束位置与目标点的距离 |
| `heading` | `value`、`tolerance` | 结束航向（弧度，自动做 ±π 折算） |

`max_speed` 和 `reach_speed` 是两种方向相反的检查，建议成对写：
一个防"油门给大了"，一个防"根本没加上去"。只写一个的话，控制器完全不动作也会通过。

## 内置车辆模型与外部车辆

`control/use_mock_odom` 决定轨迹从哪里来：

* `true`（默认）：用内置的二维运动学模型。它只做三件事——速度按加速度上限逼近指令、
  角速度按上限截断、位置按 `v·cos/sin(yaw)` 积分。好处是不需要任何模拟器就能复现，
  离线回归和真机运行走的是同一个 `advance()` 函数，所以数值可以对得上。
* `false`：不自己算，改为订阅 `topic/odom`，从真实车辆（CARLA、实车、或者其他节点）的里程计
  记录轨迹。这时场景脚本一行都不用改。

## 参数与话题

`config/scenario_runner.yaml` 里是默认值，`launch/main.launch` 把每一项都做成了一个 `<arg>`：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scenario/file` | `config/scenarios/demo_brake_stop.yaml` | 场景脚本路径 |
| `rate/publish_hz` | `20.0` | 指令发布频率，同时也是内置模型的积分频率 |
| `control/use_mock_odom` | `true` | 用内置模型还是订阅外部里程计 |
| `control/sim_dt` | `0.05` | 内置模型的积分步长 |
| `vehicle/max_speed` | `5.0` | 内置车辆最高速度 m/s |
| `vehicle/max_accel` | `1.5` | 内置车辆加速度上限 m/s² |
| `vehicle/max_decel` | `4.0` | 内置车辆减速度上限 m/s² |
| `vehicle/max_yaw_rate` | `1.0` | 内置车辆角速度上限 rad/s |
| `topic/cmd_vel` | `/cmd_vel` | 指令发布话题 |
| `topic/odom` | `/odom` | 车辆状态话题 |
| `topic/report` | `/scenario/report` | 判定报告（latched） |

## 运行

```bash
cd ~/catkin_ws
catkin_make           # 纯 Python 包，编译阶段没有输出是正常的
source devel/setup.bash

# 默认场景：加速到 5 m/s 再刹停
roslaunch ros_scenario_runner main.launch

# 换成转弯场景

roslaunch ros_scenario_runner main.launch \
    scenario_file:=$(find ros_scenario_runner)/config/scenarios/demo_turn_arc.yaml

# 换成"断言写错了"的反例，用来确认失败真的会红
roslaunch ros_scenario_runner main.launch \
    scenario_file:=$(find ros_scenario_runner)/config/scenarios/demo_expect_fail.yaml

# 改内置车辆参数，观察断言跟着变红
roslaunch ros_scenario_runner main.launch max_accel:=0.5
```

跑完在终端里会看到这样一段：

```
[INFO] 里程 32.29 m，峰值 5.00 m/s，采样 281 个
[INFO] 场景: 直线加速与刹停
[INFO] 断言: 6 通过 / 0 失败（共 6 条）
[INFO]   [PASS] max_speed    峰值 5.00 m/s，上限 5.10 m/s
[INFO]   [PASS] reach_speed  峰值 5.00 m/s，目标 5.00 m/s（容差 0.05）
...
[INFO] 结论: PASSED
```

报告同时以 latched 方式发布在 `/scenario/report`，可以用
`rostopic echo -n 1 /scenario/report` 取出来贴进 PR。

## 离线测试

`tests/test_ros_scenario_runner_local.py` 不需要 ROS、不需要 CARLA、也不需要 PyYAML：

```bash
python3 tests/test_ros_scenario_runner_local.py
```

它覆盖了场景解析（一个 120 行的 YAML 子集解析器）、六种断言的正反例、
内置车辆模型的加减速与限幅、三个示例场景的端到端结论，以及包结构约定。
场景文件里的 `expects` 也是被这份测试验证过的——如果改坏了模型参数，
`J4` 那条会立刻红。

## 为什么自己写 YAML 解析

ROS 里加载参数走的是 `<rosparam command="load">`，但场景文件比参数多一层结构
（`steps` 是对象数组、`expects` 里的字段随 `type` 变），用 rosparam 表达会很别扭，
而且离线测试一旦 `import yaml` 就失去了"不装 ROS 也能跑"的意义。
所以 `scenario_core.py` 里带了一个只支持本场景所需子集的解析器：映射、列表、
行内 `{...}` / `[...]`、引号、注释。遇到不认识的写法会直接抛错并说明是哪一个字段，
不会静默忽略。

## 与已有模块的关系

仓库里已有 `src/ground/waypoint_target_switcher`、`vehicle_pygame_teleop` 这类模块，
它们各自有离线回归测试，但都只测"函数算得对不对"。本模块补的是另一层：
**把一段带时间轴的行为跑出来，检查它整体表现得对不对**。
两者不冲突，后者可以把前者发布的指令当成输入。

## 常见问题

* **报告里里程是 0**：默认场景的 `steps` 第一条 `at` 大于 0，前面那段时间车辆是静止的；
  如果你把 `duration` 改得比最后一条 `at` 还小，脚本会在起步前就结束。
* **断言全红但车明明动了**：先看 `use_mock_odom`。设成 `false` 时轨迹来自外部 `/odom`，
  如果没人发这个话题，轨迹里就只有起始那一个采样点。
* **想加新的断言类型**：在 `scenario_core.py` 的 `EXPECT_TYPES` 里加名字、写一个
  `_check_xxx(expect, trajectory)` 函数、注册进 `_CHECKERS`，再补两条测试即可。

## 大模型声明

本模块的代码、测试与文档在编写与整理过程中使用了 AI 辅助，提交内容由作者本人负责。
