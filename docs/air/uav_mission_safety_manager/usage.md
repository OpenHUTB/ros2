# 构建与运行

## 环境与构建

环境：Ubuntu 20.04、ROS2 Foxy、Python 3.8。实际飞行测试使用 Windows OpenHUTB Air 场景与 WSL2。

在仓库根目录执行：

```bash
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src/air/uav_mission_safety_manager --ignore-src --rosdistro foxy -r -y
colcon build --symlink-install --packages-select uav_mission_safety_manager
source install/setup.bash
```

AirSim 模式另需安装包内 `requirements.txt` 中的 Python 依赖。AirSim 的安装脚本会提前导入 NumPy 和 RPC 客户端，首次安装需先准备构建工具和这两项，并关闭隔离构建：

```bash
python3 -m pip install --user "setuptools>=45.2.0,<76" "wheel<0.46" numpy==1.17.4 msgpack-rpc-python==0.4.1
python3 -m pip install --user --no-build-isolation -r src/air/uav_mission_safety_manager/requirements.txt
```

依赖版本面向 Python 3.8；OpenCV 使用兼容该环境的 4.5.5.64。`rclpy`、消息类型和 launch 等 ROS 依赖由 `package.xml` 声明，通过上述 `rosdep` 安装，不从 pip 安装。仅运行离线状态机时无需安装 AirSim 依赖。

## 启动节点

先启动 OpenHUTB 并选择 Air 模式，等待无人机落地停稳。确认 Windows 的 AirSim TCP 端口 `41451` 对 WSL 可达。

```bash
# WSL2 NAT 模式下获取 Windows 主机地址；其他网络配置请填写实际地址。
AIRSIM_HOST=$(ip route | awk '/default/ {print $3; exit}')
ros2 launch uav_mission_safety_manager main.launch.py \
  use_airsim:=true airsim_host:="$AIRSIM_HOST" airsim_port:=41451
```

只运行一个飞行控制节点。每个新终端均需加载 ROS 和工作空间环境：

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

在独立终端查看状态：

```bash
ros2 topic echo /uav_mission_safety_manager/status
```

状态包含 `state`、`reason`、`position_ned`、`target_ned`、`target_speed`、`takeoff_rpc_result`、`command_pending` 和 `last_error`。

## 连接与起飞

在命令终端逐条执行，确认状态后再进行下一步。启动节点本身不会连接、解锁或起飞。

```bash
# 连接并记录起点，等待 ARMING。
ros2 topic pub --once /uav_mission_safety_manager/command std_msgs/msg/String '{data: "{\"event\":\"connect\"}"}'

# 解锁，等待 TAKEOFF；此时尚未请求起飞。
ros2 topic pub --once /uav_mission_safety_manager/command std_msgs/msg/String '{data: "{\"event\":\"armed\"}"}'

# 请求起飞，遥测确认离地后进入 EXECUTING。
ros2 topic pub --once /uav_mission_safety_manager/command std_msgs/msg/String '{data: "{\"event\":\"airborne\"}"}'
```

`takeoff_rpc_result` 为 `null` 表示暂无结果。实测存在返回 `false` 但无人机已升空的情况，进入 `EXECUTING` 不代表起飞 RPC 返回成功。

## 航点与服务

航点使用 AirSim 本地 NED 绝对坐标，单位为米：x 向北、y 向东、z 向下。它不是相对位移，也不会自动转换为 ROS ENU。请根据起点坐标及场景障碍物修改以下示例：

```bash
ros2 topic pub --once /uav_mission_safety_manager/command std_msgs/msg/String '{data: "{\"event\":\"goto\",\"x\":3,\"y\":0,\"z\":-5,\"speed\":0.5}"}'
```

| 服务 | 功能 |
| --- | --- |
| `/uav_mission_safety_manager/pause` | 取消当前飞行任务并请求悬停，保留目标和速度 |
| `/uav_mission_safety_manager/resume` | 恢复原目标及指令速度 |
| `/uav_mission_safety_manager/return_home` | 返回连接时记录的起点上方并自动降落 |
| `/uav_mission_safety_manager/land` | 显式请求降落 |

调用格式：

```bash
ros2 service call /uav_mission_safety_manager/pause std_srvs/srv/Trigger '{}'
ros2 service call /uav_mission_safety_manager/resume std_srvs/srv/Trigger '{}'
ros2 service call /uav_mission_safety_manager/return_home std_srvs/srv/Trigger '{}'
# 需要显式降落时单独调用：
ros2 service call /uav_mission_safety_manager/land std_srvs/srv/Trigger '{}'
```

服务成功表示本地接受请求。若 `command_pending=true` 导致拒绝，等待派发完成再重试。返航完成应观察到 `RETURNING → LANDING → LANDED`。降落服务可在 `TAKEOFF`、`EXECUTING`、`PAUSED`、`RETURNING`、`LANDING` 或 `FAILSAFE` 请求。

落地后可发送 `reset` 回到 `IDLE`，下一轮重新连接记录起点。AirSim 模式禁止手动注入 `connected`、`at_home` 和 `landed` 事件。

## 控制参数与安全边界

| 参数或条件 | 默认值 |
| --- | --- |
| `max_altitude` | 相对起点最大高度 50 m |
| `telemetry_timeout` | 遥测超时 2 s |
| `operation_timeout` | 飞行阶段超时 120 s |
| 普通航点到达容差 | 1 m |
| 返航速度 | 0.5 m/s |
| 返航转降落条件 | 位置误差 ≤ 0.08 m、合速度 ≤ 0.05 m/s，持续 1 s |

前三项在 `config/mission.yaml` 中配置，其余为当前控制实现中的固定值。返航保持当前高度或至少起点上方 5 m，受高度上限约束；末段使用低速修正，无避障能力。

屋顶落地兼容逻辑仅在降落 RPC 成功、距起点 ≤ 0.15 m、合速度 ≤ 0.1 m/s 持续 3 s 后尝试解除解锁，之后仍需新遥测确认落地。

`FAILSAFE` 不保证自动悬停或降落；停止节点不会中止模拟器中的飞行任务。关闭节点前应确认无人机已落地。

## 测试命令

### 离线检查

```bash
python3 -m unittest discover -s src/air/uav_mission_safety_manager/tests -v
ROS_DOMAIN_ID=87 python3 src/air/uav_mission_safety_manager/tests/ros_smoke.py
```

离线节点使用 `use_airsim:=false`，不计为实际模拟器验证。

### 实际 AirSim 复测

以下脚本会重置并起飞模拟无人机，自行启动 ROS 节点。先停止其他飞行控制节点，每条命令完成并确认落地后再运行下一条：

```bash
ROS_DOMAIN_ID=93 python3 src/air/uav_mission_safety_manager/tests/ros_airsim_validation.py \
  --host "$AIRSIM_HOST" --case mission --log /tmp/uav_mission.jsonl --fly

ROS_DOMAIN_ID=93 python3 src/air/uav_mission_safety_manager/tests/ros_airsim_validation.py \
  --host "$AIRSIM_HOST" --case return_land --log /tmp/uav_return_land.jsonl --fly

ROS_DOMAIN_ID=93 python3 src/air/uav_mission_safety_manager/tests/ros_airsim_validation.py \
  --host "$AIRSIM_HOST" --case disconnect --log /tmp/uav_disconnect.jsonl --fly
```

`mission` 验证航点、暂停/继续及返航自动降落；`return_land` 验证起点上方返航中显式降落；`disconnect` 切断节点专用 TCP 转发，验证遥测超时。测试输出留在 `/tmp`，不提交仓库。断线后的独立恢复降落不计入自动故障恢复能力。

已有结果见[测试结果与已知限制](airsim_validation.md)。
