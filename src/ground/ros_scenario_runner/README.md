# ros_scenario_runner

把一次驾驶写成 YAML 场景脚本，按时序发速度指令、记录轨迹、逐条判定断言，
打印一份带 PASS / FAIL 的回归报告。不依赖 CARLA。

## 为什么需要这个包

仓库里已经有若干控制类模块（比如 `vehicle_pygame_teleop` 发布的 `/cmd_vel`、
`waypoint_target_switcher` 算出的目标点），它们各自的离线测试只能验证
"函数算得对不对"，验证不了"车跑起来这段时间整体表现对不对"——
加速是不是到位了、刹车有没有超调、转弯是不是画出了预期的圆弧、超速保护有没有生效。
这些都得跑一遍才知道。上 CARLA 跑一次要几分钟起，虚拟机里还容易卡死。

所以这里做一个轻量的替代：场景用 YAML 描述，车辆模型内置（也可以换成订阅真实 `/odom`），
一次回归几秒钟出结果。

## 目录结构

```
ros_scenario_runner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── scenario_runner.yaml          # 参数默认值
│   └── scenarios/
│       ├── demo_brake_stop.yaml      # 加速到 5 m/s 再刹停
│       ├── demo_turn_arc.yaml        # 3 m/s + 0.5 rad/s 画圆弧
│       └── demo_expect_fail.yaml     # 故意写错断言的反例
├── launch/
│   └── main.launch
└── scripts/
    ├── main.py                       # 入口
    ├── scenario_core.py              # 解析 / 积分 / 断言 / 报告（不 import rospy）
    └── scenario_runner_node.py       # ROS 收发
```

## 场景文件格式

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
    - type: final_speed
      value: 0.0
      tolerance: 0.05
    - type: distance
      min: 30.0
      max: 35.0
```

* `steps`：时间轴，`at` 单位是秒，到点发 `command`，保持到下一条。
* `expects`：跑完要满足的条件，任何一条不满足结论就是 `FAILED`。
* `duration`：跑多久，不小于最后一条 `at`。

六种断言：`max_speed`、`reach_speed`、`final_speed`、`distance`、`position`、`heading`。
其中 `max_speed` 和 `reach_speed` 建议成对写——一个防油门过大，一个防根本没加速，
只写一个的话控制器完全不动作也能通过。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scenario/file` | `config/scenarios/demo_brake_stop.yaml` | 场景脚本 |
| `rate/publish_hz` | `20.0` | 指令发布频率 |
| `control/use_mock_odom` | `true` | `true` 用内置模型，`false` 订阅外部 `/odom` |
| `control/sim_dt` | `0.05` | 内置模型积分步长 |
| `vehicle/max_speed` | `5.0` | 内置车辆最高速度 m/s |
| `vehicle/max_accel` | `1.5` | 加速度上限 m/s² |
| `vehicle/max_decel` | `4.0` | 减速度上限 m/s² |
| `vehicle/max_yaw_rate` | `1.0` | 角速度上限 rad/s |
| `topic/cmd_vel` | `/cmd_vel` | 指令发布 |
| `topic/odom` | `/odom` | 车辆状态 |
| `topic/report` | `/scenario/report` | 判定报告（latched） |

## 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 场景里的速度指令 |
| `/odom` | `nav_msgs/Odometry` | 发布或订阅 | 内置模型发；`use_mock_odom:=false` 时改为订阅 |
| `/scenario/report` | `std_msgs/String` | 发布（latched） | 判定报告全文 |

## 运行

```bash
cd ~/catkin_ws
catkin_make                    # 纯 Python 包，编译阶段没有输出是正常的
source devel/setup.bash

roslaunch ros_scenario_runner main.launch
```

换场景、换车辆参数：

```bash
roslaunch ros_scenario_runner main.launch \
    scenario_file:=$(find ros_scenario_runner)/config/scenarios/demo_turn_arc.yaml

roslaunch ros_scenario_runner main.launch max_accel:=0.5
```

看报告：

```bash
rostopic echo -n 1 /scenario/report/data
```

## 离线测试

不需要 ROS、不需要 CARLA、不需要 PyYAML：

```bash
cd ~/catkin_ws
python3 tests/test_ros_scenario_runner_local.py
# 或者指定任意一份副本
python3 tests/test_ros_scenario_runner_local.py --package /path/to/src/ground/ros_scenario_runner
```

它覆盖场景解析、六种断言的正反例、内置模型的加减速与限幅、三个示例场景的端到端结论，
以及包结构约定（launch 用 `main.py`、参数 11 个、核心模块不 import rospy 等）。

## 常见问题

* **里程是 0**：第一个 `at` 之前车辆静止；若 `duration` 小于最后一条 `at`，场景会在起步前结束。
* **断言全红但车动了**：`use_mock_odom:=false` 时轨迹来自外部 `/odom`，没人发这个话题就只有起点一个采样。
* **想加断言类型**：在 `scenario_core.py` 的 `EXPECT_TYPES` 加名字、写 `_check_xxx(expect, trajectory)`、
  注册进 `_CHECKERS`，再补两条测试。
