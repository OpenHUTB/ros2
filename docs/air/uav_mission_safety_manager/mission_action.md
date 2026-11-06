# ROS 2 Action 任务接口

`/uav_mission_safety_manager/execute_mission` 使用
`uav_mission_interfaces/action/ExecuteMission`，将已有多航点队列接入标准
ROS 2 Action。目标携带路线，服务端持续反馈进度，并返回成功、取消或中止结果。
原有 JSON 命令与 Trigger 服务继续可用。

## Humble 构建与验证

环境为 Ubuntu 22.04 / ROS 2 Humble / Python 3.10。接口定义位于同级
`src/air/uav_mission_interfaces`，因此必须同时安装依赖并按依赖顺序构建：

```bash
# 仓库根目录
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/air/uav_mission_interfaces src/air/uav_mission_safety_manager --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to uav_mission_safety_manager
source install/setup.bash
ros2 interface show uav_mission_interfaces/action/ExecuteMission
ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 python3 src/air/uav_mission_safety_manager/tests/ros_action_smoke.py
```

冒烟脚本在一个 `SingleThreadedExecutor` 中运行真实的 ActionClient、服务端、
话题和 Trigger 服务，使用 demo 运动适配器。覆盖起飞前拒绝、非法目标、忙碌拒绝、
暂停/继续反馈、三航点成功、执行中与暂停时客户端取消、旧队列互斥、外部取消、
注入任务超时以及返航后落地。超时是测试主动缩短控制器时限，不能算作模拟器断连实测。
脚本会关闭它创建的 ROS 节点；运行时不要在同域启动另一个同名节点。

## 发送目标与观察反馈

每个终端先加载 Humble 和 `install/setup.bash`。终端一启动本地演示：

```bash
ros2 launch uav_mission_safety_manager main.launch.py use_demo_adapter:=true
```

完成 `connect → armed → airborne`，
等到 `EXECUTING` 且 `command_pending=false`。无人机还必须没有单航点或队列目标。
演示起点是 `[0,0,0]`，然后在另一终端发送：

```bash
ros2 action send_goal /uav_mission_safety_manager/execute_mission uav_mission_interfaces/action/ExecuteMission \
  '{waypoints: [{x: 4.0, y: 0.0, z: -2.0}, {x: 8.0, y: 0.0, z: -2.0}, {x: 12.0, y: 0.0, z: -2.0}], speeds: [2.0, 2.0, 2.0]}' --feedback
```

`waypoints` 是 1–100 个绝对 NED 坐标，`speeds` 必须提供每点对应的正数速度，
单位 m/s，无默认值。所有数值必须有限，高度必须满足
`home_z - max_altitude <= z < home_z`。整队校验失败直接拒绝 Goal，不替换已有队列。
100 点上限也由生成的 ROS 消息类型约束。发送本地 ENU 坐标前需自行转换。

| Feedback 字段 | 含义 |
| --- | --- |
| `flight_state` | 控制器状态，如 EXECUTING、PAUSED、FAILSAFE |
| `queue_state` | RUNNING、PAUSED 或终止状态 |
| `total` / `completed` | 总航点数 / 已到达数 |
| `current_index` | 从 0 开始的当前索引；终止时为 -1 |
| `remaining` | 包含当前点；终止时为 0 |

反馈随 0.2 秒控制定时器发布。Action 执行回调等待 Future，定时器继续驱动控制器，
无需阻塞等待飞行。节点使用默认的单线程 executor，Action 回调组与控制定时器分开。

## 暂停、取消与结果

暂停与恢复沿用既有服务，暂停不会结束 Action：

```bash
ros2 service call /uav_mission_safety_manager/pause std_srvs/srv/Trigger '{}'
ros2 service call /uav_mission_safety_manager/resume std_srvs/srv/Trigger '{}'
```

客户端使用其 GoalHandle 的 `cancel_goal_async()` 取消本次 Action。
以下示例在空闲的 EXECUTING demo 控制器上提交慢速任务，随后取消并读取结果：

```python
import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from uav_mission_interfaces.action import ExecuteMission

rclpy.init()
node = rclpy.create_node('mission_cancel_example')
client = ActionClient(node, ExecuteMission, '/uav_mission_safety_manager/execute_mission')
assert client.wait_for_server(timeout_sec=5.0)
sent = client.send_goal_async(
    ExecuteMission.Goal(waypoints=[Point(x=30.0, y=0.0, z=-2.0)], speeds=[0.5]))
rclpy.spin_until_future_complete(node, sent)
handle = sent.result()
if handle.accepted:
    canceled = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, canceled)
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result)
    print(result.result())
client.destroy()
node.destroy_node()
rclpy.shutdown()
```

取消响应表示已接收取消请求，最终以 `get_result_async()` 为准；派发未结束时，
定时器重试取消。取消清空目标并请求悬停，暂停时取消仍保留飞行状态 PAUSED。
取消后若要新建任务，需先调用 resume 并等待 EXECUTING。

| 触发条件 | Action 最终状态 | Result.queue_state |
| --- | --- | --- |
| 所有点到达 | SUCCEEDED | COMPLETED |
| 通过该 GoalHandle 取消 | CANCELED | CANCELED |
| 外部 cancel、return_home、land 抢占 | ABORTED | CANCELED |
| 控制器 FAILSAFE / 任务超时 | ABORTED | ABORTED |

Result 还保留 `total`、`completed` 和说明 `message`。已终止任务不接受迟到取消，
取消与到达同时发生时以控制器已确定的终态为准。Action 结果表示队列终止；
外部返航或降落仍继续执行，应查看状态话题等待 LANDED。

同一时刻只接受一个 Action。旧话题正在执行队列或单航点时拒绝 Action；
Action 结果返回前拒绝话题的 `mission`、`goto` 和 `reset`，避免覆盖结果。
暂停、恢复、取消、返航和降落仍可从旧服务或命令话题发起。

## 验证范围与限制

本接口已在 Humble 的 demo 适配器下自动验证，并在 WSL2 Ubuntu 22.04.5 / ROS 2 Humble
与 OpenHUTB Air 模拟器中完成手动复测：三航点任务以 `SUCCEEDED / COMPLETED` 结束，
Action 客户端取消以 `CANCELED` 结束，返航服务抢占以 `ABORTED` 结束，随后遥测确认
`RETURNING → LANDING → LANDED`。起飞 RPC 返回 `False`，但遥测确认已进入 `EXECUTING`；该既有异常保留记录。
该接口不自动连接、解锁、起飞、避障或落地。COMPLETED 仍采用原有 1 米到达容差，
表示已请求悬停，不保证停稳；网络中断下的取消和 FAILSAFE 不保证物理停机或自动降落。
已有起飞 RPC 返回 False 的异常仍未修复。
