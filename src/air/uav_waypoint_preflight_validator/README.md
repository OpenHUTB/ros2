# 无人机航点任务飞行前校验 / UAV Waypoint Preflight Validator

该模块在飞行前检查一份相对起飞点的 ENU 航点任务，不向无人机发布控制指令。
它用于在航点轨迹跟踪之前发现重复航点、高度越界、过陡爬升、急转弯、超长航程等问题。

This module validates a waypoint mission in a takeoff-relative ENU frame. It does
not command the vehicle. It catches duplicate points, altitude violations,
over-steep climbs, sharp turns and over-long routes before tracking starts.

## 环境 / Environment

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- `python3-yaml`；仅可视化脚本需要 `python3-matplotlib`

## 任务格式 / Mission Format

`config/missions/demo_valid.yaml` 给出了完整示例：

```yaml
frame: enu_relative
limits:
  min_altitude_m: 0.5
  max_altitude_m: 20.0
  max_waypoints: 100
  min_segment_length_m: 0.25
  max_climb_angle_deg: 60.0
  warning_turn_angle_deg: 90.0
  max_turn_angle_deg: 150.0
  max_horizontal_speed_mps: 1.5
  max_vertical_speed_mps: 0.8
  max_total_distance_m: 500.0
waypoints:
  - [0.0, 0.0, 2.0]
  - [3.0, 0.0, 2.0]
  - [6.0, 2.0, 2.5]
  - [6.0, 5.0, 2.5]
```

坐标单位为米。`x` 为东、`y` 为北、`z` 为上，原点为起飞点。
不写 `frame` 或使用其他坐标系会被拒绝，避免把 World/ENU 绝对坐标与相对坐标混用。

## 离线校验 / Offline Validation

```bash
cd src/air/uav_waypoint_preflight_validator
python3 -m pip install -r requirements.txt

python3 scripts/preflight.py \
  --mission config/missions/demo_valid.yaml \
  --json-out validation/demo_valid.json

python3 scripts/preflight.py \
  --mission config/missions/demo_invalid.yaml \
  --json-out validation/demo_invalid.json
```

退出码为 0 表示任务合法，2 表示存在阻塞错误。

本次离线结果：

| 任务 | 状态 | 航点数 | 总航程 | 保守 ETA | 错误 / 警告 |
|---|---|---:|---:|---:|---:|
| `demo_valid.yaml` | VALID | 4 | 11.640 m | 8.904 s | 0 / 0 |
| `demo_invalid.yaml` | INVALID | 5 | 45.636 m | 52.700 s | 5 / 1 |

## ROS 启动 / ROS Launch

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

roslaunch uav_waypoint_preflight_validator main.launch \
  mission_file:=$(rospack find uav_waypoint_preflight_validator)/config/missions/demo_valid.yaml \
  output_json:=/tmp/uav_preflight.json
```

节点发布：

| 话题 / Topic | 类型 / Type | 说明 / Description |
|---|---|---|
| `/uav_waypoint_preflight/status` | `std_msgs/String` | 完整 JSON 校验报告 |
| `/uav_waypoint_preflight/valid` | `std_msgs/Bool` | 是否允许继续任务 |
| `/uav_waypoint_preflight/markers` | `visualization_msgs/MarkerArray` | RViz 绿色合法航段、红色错误航段和航点编号 |

节点默认在发布一次结果后退出：合法任务退出码为 `0`，存在阻塞错误时退出码为 `2`。
设置 `exit_after_publish:=false` 可保持节点运行并缓存结果。

Marker 坐标是相对起飞点的 ENU 偏移。默认 `marker_frame=world` 只适用于 RViz 的
`world` 原点与起飞点重合、坐标轴也同向的场景；如果真实地图原点不同，应传入一个
以起飞点为原点且轴向为 ENU 的 TF frame，或关闭 Marker 预览。

已在 Ubuntu 20.04 + ROS Noetic 的干净工作空间验证：`catkin_make` 通过，
`catkin_make run_tests_uav_waypoint_preflight_validator` 通过，`roslaunch` 成功
启动并发布结果。合法示例返回 `VALID`、4 个航点、11.640 m、ETA 8.904 s，
`errors=[]`、`warnings=[]`。

## 检查规则 / Checks

- 帧必须显式为 `enu_relative`；
- `x/y/z` 必须是有限数值，拒绝字符串和布尔值；
- 至少两个航点，且不超过 `max_waypoints`；
- 高度必须位于允许区间；
- 相邻航段不得重复或短于最小长度；
- 非竖直航段的爬升角不得超过阈值；
- 每个中间航点的水平转弯角不能超过最大阈值，超过建议值会给出警告；
- 统计总航程与基于最大水平/垂直速度的保守 ETA；
- 航程超过上限会产生错误，超过上限 80% 会产生警告。

## 测试 / Tests

```bash
python3 -m unittest discover -s test -v
```

18 项测试覆盖合法任务、缺失文件、YAML 错误、坐标系、非法数值、高度、重复点、
过陡爬升、急转弯、普通警告、总航程、非法限制参数、路径展开和 JSON 往返。

## 边界 / Limitations

- 不检查障碍物、禁飞区、地形或动力学可行性；
- 不执行轨迹跟踪，不发布 `/uav/cmd_vel`；
- 速度可行性只按最大水平/垂直速度做保守估计；
- 急转弯是否真正可执行仍应由飞控和现场条件确认。

## AI 使用声明 / AI Assistance Disclosure

Codex（OpenAI）辅助完成代码、测试和文档；作者负责在 ROS/OpenHUTB 环境中运行验证并对提交内容负责。
