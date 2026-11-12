# AirSim ROS 2 Humble 气压计桥接

本模块把 AirSim `getBarometerData()` 返回的压力、气压高度和 QNH 接入 ROS 2
Humble，为高度估计、定高控制和气压计噪声分析提供数据源。实现位于
`src/air/airsim_ros2_barometer`，入口统一为 `main`，启动文件为
`launch/bringup.launch.py`。

## 1. 接口

| 类型 | 名称 | 消息 | 单位 |
| --- | --- | --- | --- |
| 发布 | `/uav/barometer/pressure` | `sensor_msgs/msg/FluidPressure` | Pa |
| 发布 | `/uav/barometer/altitude` | `std_msgs/msg/Float64` | m |
| 发布 | `/uav/barometer/qnh` | `std_msgs/msg/Float64` | Pa |

压力话题默认使用传感器数据 QoS；高度和 QNH 使用普通深度 10 的发布队列。
默认 `frame_id=base_link`、`publish_rate=20.0 Hz`、`rpc_timeout=1.0 s`，
并保持 `use_sim_time=false`，因为当前 AirSim 场景通常不发布 `/clock`。

### 1.1 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `host` | `127.0.0.1` | AirSim RPC 地址 |
| `port` | `41451` | AirSim RPC 端口 |
| `vehicle_name` | `""` | 多旋翼名称 |
| `barometer_name` | `""` | AirSim 气压计名称 |
| `pressure_topic` | `/uav/barometer/pressure` | 压力输出话题 |
| `altitude_topic` | `/uav/barometer/altitude` | 气压高度输出话题 |
| `qnh_topic` | `/uav/barometer/qnh` | QNH 输出话题 |
| `frame_id` | `base_link` | 压力消息坐标系 |
| `publish_rate` | `20.0` | 最大读取调度和发布频率 |
| `reconnect_period` | `2.0` | 连接失败后的等待时间 |
| `rpc_timeout` | `1.0` | AirSim RPC 连接和读取超时 |
| `pressure_variance` | `1.0` | 写入 `FluidPressure.variance` 的假定方差（Pa²） |

## 2. 计算原理与公式

### 2.1 数据转发与单位

AirSim 的压力和气压高度按 Pa、m 原样转发；QNH 在 AirSim 1.8.1 中按
hPa 返回，而本模块对外统一发布 Pa。因此转换关系为

$$
h_{\mathrm{ROS}}\,[\mathrm{m}]
=h_{\mathrm{AirSim}}\,[\mathrm{m}],
\qquad
p_{\mathrm{ROS}}\,[\mathrm{Pa}]
=p_{\mathrm{AirSim}}\,[\mathrm{Pa}],
$$

$$
q_{\mathrm{ROS}}\,[\mathrm{Pa}]
=100\,q_{\mathrm{AirSim}}\,[\mathrm{hPa}].
$$

ROS `FluidPressure.fluid_pressure` 的约定单位是 Pa；高度和 QNH 的
`Float64` 分别是本节接口表声明的 m 和 Pa。例如 AirSim 默认 QNH
`1013.25 hPa` 会发布为 `101325 Pa`。转换在
`airsim_ros2_barometer/barometer.py` 中统一完成，调用方不需要再次乘 100。
AirSim 1.8.1 的 hPa 约定见
[`BarometerSimpleParams.hpp`](https://github.com/microsoft/AirSim/blob/v1.8.1/AirLib/include/sensors/barometer/BarometerSimpleParams.hpp)。

### 2.2 样本合法性

一次样本必须同时满足

$$
h\in\mathbb{R},\qquad p\in(0,+\infty),\qquad q\in(0,+\infty),
$$

其中 NaN 和正负无穷都不属于 $\mathbb{R}$。压力或 QNH 等于零表示物理上
无效的绝对基准，也会被拒绝。直接依赖 `float()` 会造成类型问题的字段，
统一转为 Python 浮点数后检查。

### 2.3 方差语义

`FluidPressure.variance` 表示压力测量方差 $\sigma_p^2$，单位为 Pa²。本模块
没有从样本中估计真实噪声，而是把参数值原样写入：

$$
\operatorname{variance}_{\mathrm{ROS}}
=\sigma_{p,\mathrm{configured}}^2.
$$

默认值 `1.0 Pa²` 只是可配置的假定值，不代表 AirSim 气压计的实测噪声。
未知精度时应按实验数据调整；高度和 QNH 使用 `Float64`，没有协方差字段。

### 2.4 时间戳与频率

发布周期为

$$
T_p=\frac{1}{f_p},
$$

其中 $f_p$ 是 `publish_rate`。定时器只触发调度；当单次 AirSim RPC 长于
$T_p$ 时，不会堆积多个读取任务，实际发布频率会低于参数值。消息时间戳取
ROS 当前时间，表示“桥接节点发布该样本的时刻”，当前版本没有映射 AirSim
样本时间，也没有实现重复样本时间戳检测。

### 2.5 当前不做的计算

模块不使用国际标准大气公式重新计算高度，不执行温度补偿、零偏标定、低通
滤波、QNH 动态估计或 GPS/IMU/气压计融合。它只作为原始气压计数据入口，
不能替代飞控 EKF 或状态估计器。

## 3. 算法流程

```text
ROS 定时器触发
     │
     ▼
client 是否存在？──否──▶ 后台线程执行限时连接
     │是                         │
     ▼                           ▼
上一次读取 Future 完成？     连接成功后在下个周期使用
     │是
     ▼
校验 height / pressure / QNH
     │
     ▼
构造 FluidPressure、Float64、Float64
     │
     ▼
发布三条 ROS 话题
```

连接和读取流程：

1. `_ensure_connected()` 在到达重连时刻后向单线程线程池提交连接任务；
2. `MultirotorClient` 使用 `rpc_timeout`，随后执行 `confirmConnection()`；
3. 连接 Future 未完成时，不重复创建连接，ROS timer 立即返回；
4. `_schedule_read()` 最多保留一个读取 Future，因此 RPC 变慢时不会堆积请求；
5. Future 完成后，ROS timer 取出结果、校验并发布三条消息。

故障流程：

1. 读取失败 1 至 4 次时保留 client，但停止发布失败样本；
2. 连续失败达到 5 次时丢弃 client，并按 `reconnect_period` 等待重连；
3. 读取或消息转换失败都不会继续发布缓存旧值；
4. 连接失败和 RPC 超时由后台线程处理，不阻塞单线程 ROS executor。

## 4. 源码解析（带文件与行号）

### 4.1 `airsim_ros2_barometer/barometer.py`

| 位置 | 内容 |
| --- | --- |
| `barometer.py:10-16` | 不可变数据类 `BarometerSample` |
| `barometer.py:13-37` | 定义样本模型，校验高度、压力、QNH，并把 QNH 从 hPa 转为 Pa |
| `barometer.py:27-32` | 有限性、正压力和正 QNH 检查 |

### 4.2 `airsim_ros2_barometer/main.py`

| 位置 | 内容 |
| --- | --- |
| `main.py:23-92` | 参数声明、空字符串回退、有限性/正数检查及端口范围检查 |
| `main.py:94-121` | 连接状态、单线程 RPC executor、三个发布者和定时器 |
| `main.py:123-174` | 非阻塞连接调度、RPC 超时、`confirmConnection()` 与重连 |
| `main.py:180-231` | timer 调度、Future 消费、连续失败和断线处理 |
| `main.py:233-261` | 三个 ROS 消息的字段映射和发布 |
| `main.py:264-276` | `main()`、构造失败清理和幂等 shutdown |

`client_factory` 用于测试注入：不启动 AirSim 时也能覆盖阻塞连接、阻塞读取、
失败重连和消息发布。生产环境动态导入 `airsim`，缺失时给出 requirements 路径。

### 4.3 `launch/bringup.launch.py`

| 位置 | 内容 |
| --- | --- |
| `bringup.launch.py:13-18` | 从安装后的 share 目录解析 YAML |
| `bringup.launch.py:31-55` | 声明 host、话题、坐标系、频率和超时参数 |
| `bringup.launch.py:56-99` | 启动 `main` 并显式声明 int、float、str、bool 类型 |

`use_sim_time` 使用 `ParameterValue(..., value_type=bool)`，避免字符串
`"false"` 与 rclpy 自动声明的 bool 参数冲突。

### 4.4 测试与 CI

| 测试文件 | 用例数 | 覆盖内容 |
| --- | --- | --- |
| `test/test_barometer.py:12-58` | 11 | 有效样本及压力、高度、QNH 的 NaN/Inf/非正数拒绝 |
| `test/test_node_startup.py:54-91` | 1 | AirSim 离线时 launch、节点和三条话题仍可发现 |
| `test/test_reconnect.py:73-265` | 8 | 端口校验、1–4 次失败恢复、第 5 次断线、重连、阻塞连接/读取、安装提示和三消息字段 |

`.github/workflows/air_bridge_tests.yml` 的 AirSim sensor bridge 作业同时构建
`airsim_ros2_magnetometer` 与 `airsim_ros2_barometer`，再用 `colcon test`
执行两个包的测试。该 CI 使用官方 `ros:humble-ros-base` 容器，不依赖本地
ROS 环境或运行中的 AirSim。

## 5. 构建

以下命令统一以 `~/hutb_ws` 为工作区，软链接后的包目录为
`~/hutb_ws/src/airsim_ros2_barometer`：

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/hutb_ws/src
ln -sfn ~/ros2/src/air/airsim_ros2_barometer \
  ~/hutb_ws/src/airsim_ros2_barometer
python3 -m pip install -r \
  ~/hutb_ws/src/airsim_ros2_barometer/requirements.txt
cd ~/hutb_ws
colcon build --symlink-install --packages-select airsim_ros2_barometer
source install/setup.bash
```

## 6. 运行

先启动 AirSim/OpenHUTB 场景，再运行：

```bash
ros2 launch airsim_ros2_barometer bringup.launch.py \
  host:=<Windows_IP> port:=41451 vehicle_name:=""
```

检查数据：

```bash
ros2 topic echo /uav/barometer/pressure --once
ros2 topic echo /uav/barometer/altitude --once
ros2 topic echo /uav/barometer/qnh --once
ros2 topic hz /uav/barometer/pressure
```

真实运行至少要记录压力/高度/QNH 数值、单位、静态噪声和发布频率，并验证
高度变化时气压方向是否符合场景定义。现有离线测试不能替代这些数据。

## 7. 测试

从同一个 `~/hutb_ws` 工作区执行：

```bash
source /opt/ros/humble/setup.bash
source ~/hutb_ws/install/setup.bash
python3 -m pytest ~/hutb_ws/src/airsim_ros2_barometer/test -q
colcon test --packages-select airsim_ros2_barometer
colcon test-result --verbose
```

本地 Humble 环境实际收集 20 个用例。测试只证明解析、消息映射、后台连接
状态机和离线 launch 行为，不证明真实 AirSim 气压计数值正确。

## 8. 范围与限制

- 除 QNH 的 hPa→Pa 单位转换外只转发 AirSim 数据，不执行温度补偿、偏置标定或滤波；
- 不使用气压公式重新计算高度，不替代飞控 EKF 或高度融合模块；
- `pressure_variance` 是配置值，不是从仿真样本估计的实测噪声；
- QNH 来自仿真场景配置，不能假设与真实气象基准一致；
- 使用 ROS 当前时间戳，不映射 AirSim 样本时间；
- 真实压力、高度、QNH、噪声和频率仍需在 AirSim/OpenHUTB 场景中验证。

## 9. ROS 2 与仓库贡献规范

本模块遵循 [ROS 2 Humble 文档](https://docs.ros.org/en/humble/) 的节点、参数、
QoS、launch 和消息接口约定。模块入口为 `main`，并提供 ROS 2 launch 文件。

按照 [OpenHUTB/ros2 提交注意事项](https://github.com/OpenHUTB/ros2#注意事项) 和
[OpenHUTB 贡献指南](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md)：

- 源码位于 `src/air/airsim_ros2_barometer`；
- 文档位于 `docs/air/airsim_ros2_barometer.md`；
- 首页和 `mkdocs.yml` 已加入链接；
- 提交前运行 `mkdocs serve --livereload`；
- PR 需附真实运行截图或 GIF，并请另一名贡献者复核；
- 合并前解决与主分支的冲突并检查 Files changed。

## AI 辅助声明

本模块、测试和文档由大语言模型辅助整理；提交者应连接实际 AirSim 场景，
检查压力、高度、QNH、噪声和频率，并对提交内容负责。
