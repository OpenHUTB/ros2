# UAV GNSS 定位完整性监测与位置跳变失效保护

无人机依赖 GNSS 做定点悬停和航点飞行。多路径、遮挡后重新捕获、定位降级或数据流中断，
都会让定位结果在相邻两帧之间突跳几米甚至几十米；如果飞控继续采信这帧定位，
就会朝错误方向做大幅修正，表现为「飞走」。本模块串在定位输入与下游控制器之间，
先判断这一帧是否可信，不可信时冻结最后可信位姿、置 `gnss_valid=False` 请求悬停，
直到定位连续多帧回到保持点附近才恢复。

## 边界与分工

| 已有模块 | 做什么 | 本模块不做什么 |
| --- | --- | --- |
| `uav_altitude_command_guard` | 约束上层发来的**目标高度指令**（限幅、变化率、超时） | 不约束目标指令 |
| `telemetry_diagnostics` | 基于 GRU 的遥测**异常评分与在线诊断** | 不做学习式异常评分、不输出诊断看板 |
| `uav_mission_safety_manager` | **任务层**状态机（航点队列、暂停、返航、降落） | 不做任务调度、不发起返航 |
| `uav_battery_rth_failsafe` | 电量不足时**返航迫降** | 不判断电量、不生成返航航点 |
| 本模块 | **定位输入侧**的运动学合理性门控，输出可用位姿与有效性标志 | 不输出推力/速度指令，不代替飞控保护 |

`gnss_valid=False` 只是给下游的信号：下游应保持悬停并停止用定位做闭环修正。
本模块**不产生任何控制指令**，也不能替代飞控自身的失效保护。

## 目录结构

```
src/air/uav_gnss_integrity_monitor/
├── main.py                        # 离线回放入口（零第三方依赖，main.* 入口）
├── test_gnss_integrity_monitor.py # 22 项单元测试（unittest / pytest 均可）
├── package.xml / setup.py / setup.cfg / resource/
├── launch/uav_gnss_integrity_monitor.launch.py
└── uav_gnss_integrity_monitor/
    ├── integrity_math.py          # 判据 + 状态机，纯标准库，不依赖 ROS
    └── gnss_integrity_node.py     # ROS 2 话题封装（需要 rclpy）
```

教程在 [`docs/air/uav_gnss_integrity_monitor.md`](../../../docs/air/uav_gnss_integrity_monitor.md)，
离线回放数据在 `docs/evidence/uav_gnss_integrity_monitor/offline_replay.csv`。

## 已验证环境

实际运行过命令的环境：

- Windows 11，Python 3.13.14，**未安装** ROS 2 / NumPy / pytest / 仿真器。
- 在本机实际执行：`python main.py`、`python main.py --plot <目录>`、
  `python -m unittest -v test_gnss_integrity_monitor.py`（22/22 通过）。

目标部署环境（**本次未在本机验证，需在部署机上复测**）：

- Ubuntu 22.04 + ROS 2 Humble + Python 3.10；`rclpy`、`sensor_msgs`、
  `geometry_msgs`、`std_msgs` 来自系统 ROS 安装，不能用 pip 代替。

## 离线运行（不需要 ROS）

从仓库根目录进入模块目录（下列命令的执行目录都是模块目录）：

```bash
cd src/air/uav_gnss_integrity_monitor

# 打印回放报告（合成定位流，注入 5 类异常）
python main.py

# 额外输出逐帧 CSV 和带标注的 SVG 曲线
python main.py --plot ./outputs

# 覆盖判据，例如把最大合理速度改成 8 m/s
python main.py --config '{"max_speed": 8.0}'

# 打印默认配置
python main.py --show-config

# 单元测试
python -m unittest -v test_gnss_integrity_monitor.py
# 若已装 pytest：python -m pytest -v test_gnss_integrity_monitor.py
```

`--plot` 生成的文件是本地产物，不属于提交内容；仓库中保留的那份
`offline_replay.svg` / `offline_replay.csv` 是用上述命令实际生成后作为证据提交的。

## ROS 2 构建与运行

在仓库根目录执行（`colcon` 会递归发现 `src/` 下的功能包）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_gnss_integrity_monitor --symlink-install
source install/setup.bash
ros2 launch uav_gnss_integrity_monitor uav_gnss_integrity_monitor.launch.py
```

或者直接以 main 脚本启动节点（等价于上面的 launch）：

```bash
cd src/air/uav_gnss_integrity_monitor
source /opt/ros/humble/setup.bash && source ../../../install/setup.bash
python main.py --ros
```

不接真机时，可在另一个终端手动发一帧定位做话题连通性检查：

```bash
ros2 topic pub --once /drone/gnss sensor_msgs/msg/NavSatFix \
  "{header: {stamp: {sec: 1, nanosec: 0}, frame_id: map}, \
    status: {status: 0, service: 0}, latitude: 0.0, longitude: 0.0, altitude: 5.0, \
    position_covariance: [0.25, 0, 0, 0, 0.25, 0, 0, 0, 0.25], \
    position_covariance_type: 2}"
```

观察输出：

```bash
ros2 topic echo /drone/pose_validated
ros2 topic echo /drone/gnss_valid
ros2 topic echo /drone/gnss_status
```

`Ctrl+C` 停止节点。节点内另有 10 Hz 看门狗，定位流完全中断超过 `stale_timeout`
时也会主动进入 HOLD，不依赖是否有新消息到达。

## 话题与参数

订阅：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/drone/gnss` | `sensor_msgs/NavSatFix` | 必选；经纬高按 `origin_*` 换成局部米制坐标 |
| `/drone/gnss_satellites` | `std_msgs/Int32` | 可选；`NavSatFix` 不含卫星数，未提供时按 `min_sats` 处理 |

发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/drone/pose_validated` | `geometry_msgs/PoseStamped` | 门控后的位姿；HOLD 期间为最后可信位姿 |
| `/drone/gnss_valid` | `std_msgs/Bool` | `False` 时下游应保持悬停 |
| `/drone/gnss_status` | `std_msgs/String` | JSON：状态、原因、动作、连续好/坏帧计数 |

launch 中的参数与 `Config` 一一对应，详见
[教程的参数表](../../../docs/air/uav_gnss_integrity_monitor.md#4-参数)。
`origin_lat` / `origin_lon` / `origin_alt` 是局部坐标原点，
需要填成飞行区域附近的真实值，否则经纬度换算会带来固定偏移。

## 验证范围与已知限制

- 已验证：22 项单元测试、离线回放中的判据/状态机/输出冻结/恢复、SVG 与 CSV 生成。
- **未验证**：ROS 2 话题闭环、AirSim / MuJoCo 仿真飞行、真实 GNSS 硬件故障、
  实机悬停效果。本机无 ROS 2 与仿真器，不能把离线回放解释成飞行效果。
- 合成轨迹是 0.8 m/s 匀速直线，注入的是**软件层**的定位异常，
  不是真实多路径或欺骗干扰。
- 经纬高换算用等距近似，只适用于小范围；大范围或高精度场景应换成正式投影。
- 若飞行器在 HOLD 期间被风吹离保持点超过 `reacquisition_radius`，
  模块会保持 HOLD 等待人工介入，不会自动把远处的新位置当成恢复。
  这是有意的保守设计，不是漏检。
- 阈值默认值只适用于本次低速示例，实机需按机型最大速度和 GNSS 精度重新标定。

## 大模型使用声明

本模块的代码框架、注释、文档、单元测试与 PR 描述使用 WorkBuddy（均衡模型）辅助生成；
作者已逐行审阅、实际运行全部命令并核对输出，对提交内容负责。

## 许可证

MIT。
