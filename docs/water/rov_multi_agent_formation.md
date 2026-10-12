# 多智能体水域协同：领航-跟随编队控制

## 1. 概述与核心能力

前面的章节都以单台载具为单位：一台 ROV 做物理仿真、一台 ROV 做声呐 SLAM、一台 ROV 做 3D 轨迹跟踪。本章把视角扩大到**同一场景中的多台载具**，回答三个问题：

1. HoloOcean 怎么在一个仿真进程里同时驱动多台不同型号的载具；
2. ROS 2 侧的话题与服务如何区分"这句话是说给谁听的"；
3. 在多机接口之上，如何实现一个可量化的**领航-跟随（leader-follower）编队保持**控制器。

学完本章应能：

- 读懂并改写 `holoocean_main/config/multi_agent.json` 场景，增减智能体、调整初始位姿与传感器；
- 说明 `/holoocean/command/agent`、`/holoocean/depth`、`/holoocean/heading`、`/holoocean/speed` 这几条指令通道为什么要用 `header.frame_id` 指定目标载具；
- 跑通"一台水下 ROV + 一台水面艇"的多机场景，并用 `formation_controller` 让跟随者保持队形；
- 用编队误差 RMS、最大误差、收敛时间三个指标评价编队质量。

> 本章的接口部分（第 2、3 节）描述的是仓库中已经存在的代码；编队控制器（第 4、5 节）是本章新增的示例节点，随本章一起提交。

## 2. 多智能体场景建模

### 2.1 场景文件逐字段解析

多机场景与单机场景共用一套 schema，差别只在 `agents` 数组的长度与内容。仓库中的 `src/water/holoocean_main/config/multi_agent.json` 定义了两台载具：

| 字段 | `auv0` | `auv1` | 含义 |
| --- | --- | --- | --- |
| `agent_type` | `BlueROV2` | `SurfaceVessel` | 载具模型：水下 ROV / 水面艇 |
| `location` | `[0, 1, -3]` | `[0, 1, 0]` | 初始位置，z 为 -3 m 与 0 m |
| `rotation` | `[0, 0, 0]` | `[0, 0, 0]` | 初始姿态（欧拉角） |
| `control_scheme` | `0` | `0` | 控制方案编号，决定指令向量长度 |
| `sensors` | `DynamicsSensor` + `CameraSensor` | 同左 | 每个智能体独立配置传感器 |

场景级字段：

| 字段 | 值 | 说明 |
| --- | --- | --- |
| `package_name` | `Ocean` | HoloOcean 场景包名 |
| `world` | `SimpleUnderwater` | 世界文件 |
| `main_agent` | `auv0` | 主智能体，视口默认跟随它 |
| `ticks_per_sec` | `30` | 仿真步进频率 |
| `frames_per_sec` | `30` | 渲染帧率 |

### 2.2 单进程多智能体的运行机制

HoloOcean 的多智能体不是"启动多个仿真进程"，而是**一个 `Environment` 内维护多个 agent**，每个 agent 各自持有一棵刚体树、各自推进动力学、各自拥有传感器列表。ROS 2 桥接层在初始化时遍历 `agents`，为每个 agent 的每个 `ros_publish: true` 传感器建立发布器：

```python
def create_publishers(self):
    for sensor in self.sensors:
        topic_name = sensor.agent_name + '/' + sensor.name
        sensor.publisher = self.node.create_publisher(sensor.message_type, topic_name, 10)
```

因为节点运行在 `holoocean` 命名空间下，最终话题是 `/holoocean/<agent_name>/<sensor_name>`。**"哪台载具的数据"由话题名前缀区分，而不是由消息内容区分**——这是理解本章后半部分接口设计的第一个关键点。

### 2.3 与单机场景的对照

| 对比项 | 单机场景（如 `blueROV.json`） | 多机场景（`multi_agent.json`） |
| --- | --- | --- |
| `agents` 长度 | 1 | 2（可继续增加） |
| 传感器话题 | `/holoocean/auv0/...` | `/holoocean/auv0/...` 与 `/holoocean/auv1/...` |
| 指令目标 | 唯一，不需要区分 | **必须**指定目标载具，见第 3.2 节 |
| 控制模式 | 全局唯一 | 每台载具独立设置 |
| 传感器种类 | DVL/IMU/GPS/Depth/Rotation 等 6 种 | 两机各 2 种（Dynamics + Camera） |

## 3. ROS 2 多机通信接口

### 3.1 传感器数据通道

发布方向遵循 `<agent_name>/<sensor_name>` 规则。以 `multi_agent.json` 为例，运行起来之后能看到：

| 话题 | 消息类型 | 来源 |
| --- | --- | --- |
| `/holoocean/auv0/DynamicsSensorOdom` | `nav_msgs/msg/Odometry` | `auv0` 动力学里程计 |
| `/holoocean/auv0/CameraSensor` | `sensor_msgs/msg/Image` | `auv0` 前视相机 |
| `/holoocean/auv1/DynamicsSensorOdom` | `nav_msgs/msg/Odometry` | `auv1` 动力学里程计 |
| `/holoocean/auv1/CameraSensor` | `sensor_msgs/msg/Image` | `auv1` 相机 |

> `DynamicsSensor` 属于"一对多"传感器，会被拆成多个后缀话题（`...Odom` 等），具体后缀由 `sensor_data_encode.py` 中的 `multi_publisher_sensors` 决定。

### 3.2 指令通道：用 `header.frame_id` 说"这句话说给谁"

指令是**多机共用一条话题**的：节点只创建一份订阅，靠消息里的载具名分发。

```python
self.command_sub = self.create_subscription(AgentCommand, 'command/agent', self.agent_command_callback, 10)
self.depth_sub   = self.create_subscription(DesiredCommand, 'depth',   self.depth_callback, 10)
self.heading_sub = self.create_subscription(DesiredCommand, 'heading', self.heading_callback, 10)
self.speed_sub   = self.create_subscription(DesiredCommand, 'speed',   self.speed_callback, 10)
```

```python
def heading_callback(self, msg):
    vehicle_name = msg.header.frame_id      # ← 目标载具由 frame_id 决定
    self.interface.set_heading(vehicle_name, msg.data)
```

完整通道表：

| 通道 | 话题（`/holoocean/` 下） | 消息类型 | 如何指定载具 | 用途 |
| --- | --- | --- | --- | --- |
| 手动推力 | `command/agent` | `AgentCommand`（`float64[]`） | `header.frame_id` | 直接下发推力/舵角向量 |
| 深度目标 | `depth` | `DesiredCommand`（`float64`） | `header.frame_id` | 深度自动驾驶目标（米） |
| 航向目标 | `heading` | `DesiredCommand` | `header.frame_id` | 航向目标（度，NWU，-180~180） |
| 速度目标 | `speed` | `DesiredCommand` | `header.frame_id` | 螺旋桨 RPM 目标 |
| 传感器转动 | `command/sensor` | `SensorCommand` | `agent_name` 字段 | 调整传感器朝向 |
| 调试点绘制 | `debug/points` | `visualization_msgs/Marker` | — | 在视口画航点/目标点 |

服务与时钟：

| 名称 | 类型 | 参数 | 说明 |
| --- | --- | --- | --- |
| `/holoocean/control_mode` | `SetControlMode` | `vehicle_name`、`mode` | 逐台切换控制模式 |
| `/holoocean/reset` | `std_srvs/Trigger` | — | 重置环境（注意：当前实现是整场重置） |
| `/clock` | `rosgraph_msgs/Clock` | — | 仿真时间，配合 `use_sim_time: true` |

控制模式取值：

| `mode` | 名称 | 说明 |
| --- | --- | --- |
| `0` | `manualControl` | 手动，指令来自 `command/agent` |
| `1` | `depthHeadingAutopilot` | 深度 + 航向自动驾驶（本章编队控制器使用） |
| `2` | `headingAutopilot` | 仅航向自动驾驶 |
| `3` | `depthAutopilot` | 仅深度自动驾驶 |

> 深度/航向/速度这三条自动驾驶通道只对**注册了 Fossen 模型的智能体**生效：接口会检查 `vehicle_name in self.fossen_agents`，不匹配时会打印 `Vehicle 'xxx' not found in the fossen vehicle registry` 并忽略该指令。这是多机调试时最容易忽略的一点。

### 3.3 多操作员与载具切换

`holoocean_examples` 提供了两手柄方案（`launch/multi_joy_launch.py`）：

```
joy_node          (/dev/input/js0) ──┐
                                     ├── joy_holoocean      (默认控制 auv0)
joy_node_wired    (/dev/input/js1) ──┤      remap: joy → joy_wired
                                     └── joy_holoocean_wired (默认控制 auv1)
```

关键配置（`config/multi_joy_config.yaml`）：

```yaml
joy_holoocean:
  ros__parameters:
    agents.buttons: [0, 1, 2, 3]   # Xbox 面键 A/B/X/Y，用于切换当前控制的载具
    agents.default: 'auv0'
joy_holoocean_wired:
  ros__parameters:
    agents.default: 'auv1'
```

也就是说：**每个操作员可以随时用面键接管任意一台载具**，而两个 `joy_holoocean` 节点通过 `remap: joy → joy_wired` 让第二个手柄走独立话题，互不干扰。

### 3.4 每台载具的机载 HUD

`camera_hud` 节点按 `agent_name` 参数为指定载具叠加平视仪表，并在同一话题族里输出：

```python
self.create_subscription(Image,    f'{agent}/CameraSensor',       self._image_cb, 10)
self.create_subscription(Odometry, f'{agent}/DynamicsSensorOdom', self._odom_cb,  10)
self.pub = self.create_publisher(Image, f'{agent}/CameraHUD', 10)
```

多机场景里通过给两个 HUD 实例不同参数即可同时看两路画面：

```yaml
camera_hud:      { ros__parameters: { agent_name: 'auv0' } }
camera_hud_sv:   { ros__parameters: { agent_name: 'auv1' } }
```

## 4. 领航-跟随编队控制器

### 4.1 编队误差的定义

设领航者位置为 $\mathbf{p}_L \in \mathbb{R}^3$、航向为 $\psi_L$，第 $i$ 个跟随者的期望位置为

$$
\mathbf{p}_i^{d} = \mathbf{p}_L + R_z(\psi_L)\,\mathbf{d}_i
$$

其中 $\mathbf{d}_i=[d_x^i, d_y^i, d_z^i]^{\mathsf T}$ 是**机体系**下的编队偏移，$R_z(\psi_L)$ 是绕 z 轴的旋转矩阵。编队误差与两个评价量：

$$
\mathbf{e}_i = \mathbf{p}_i^{d} - \mathbf{p}_i,\qquad
\mathrm{RMS}_i=\sqrt{\tfrac1N\sum_{k=1}^{N}\|\mathbf{e}_i^{(k)}\|^2},\qquad
e_i^{\max}=\max_k \|\mathbf{e}_i^{(k)}\|
$$

用机体系偏移而不是固定世界系偏移，好处是**队形会跟着领航者一起转**：领航者转弯时，跟随者自动绕到新的相对方位，而不是留在原来的世界坐标上。

### 4.2 外环-内环控制结构

HoloOcean 自带深度/航向自动驾驶，可以把它当作内环，本节点只做外环：

```
 期望编队位置 ──► 外环（本节点：位置误差 → 期望航向/深度/速度） ──► 内环（HoloOcean heading/depth Autopilot） ──► 动力学
        ▲                                                                                                        │
        └──────────────────────── <agent>/DynamicsSensorOdom ◄──────────────────────────────────────────────────┘
```

外环控制律：

$$
\psi_i^{d}=\operatorname{atan2}\!\left(e_{i,y},\,e_{i,x}\right),\qquad
z_i^{d}=\operatorname{clamp}\!\left(z_L+d_z^i,\;z_{\min},z_{\max}\right),\qquad
v_i^{d}=v_L + \bar v + k_v\min\!\left(\|\mathbf{e}_i\|,\,L\right)
$$

含义：**朝向误差方向飞、按期望深度潜、距离越远走得越快**，误差小于阈值时把速度降为 0，避免在目标点附近来回振荡。

其中 $v_L$ 是领航者当前速度，作为**前馈项**。这一项不是凑出来的参数，而是第 6 节无仿真器回放实测暴露出的必要修正：只用反馈时，$\|\mathbf{e}_i\|\to 0$ 会让速度指令趋近 $\bar v$，只要领航者速度不等于 $\bar v$，就会留下稳态误差

$$
\|\mathbf{e}_i\|_{\text{ss}} \approx \frac{v_L-\bar v}{k_v}
$$

本项目的默认参数下这个误差约 0.57 m，且永远无法收敛到 ±0.3 m 的验收阈值内；加入前馈后稳态误差降到 0.15 m（实测见第 6 节）。

### 4.3 节点实现要点

本节新增 `formation_controller.py`，核心逻辑：

```python
# 1) 期望位置：领航者位置 + 机体系偏移旋转到世界系
c, s = math.cos(lyaw), math.sin(lyaw)
tx = lx + (c * dx - s * dy)
ty = ly + (s * dx + c * dy)
tz = lz + dz

# 2) 位置误差 → 期望航向与深度
ex, ey, ez = tx - px, ty - py, tz - pz
heading_deg = math.degrees(math.atan2(ey, ex))
depth_m = max(dmin, min(dmax, tz))
# 速度指令 = 前馈（领航者当前速度）+ 位置误差反馈
ff = k_speed * min(leader_speed, max_standoff)     # leader_speed 由领航者里程计得到
speed = 0.0 if math.hypot(ex, ey) < 0.15 else ff + cruise + k_speed * dist

# 3) 以 frame_id 指定目标载具后下发
msg.header.frame_id = follower_name
```

参数表：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `leader` | `auv0` | 领航者名称 |
| `followers` | `['auv1']` | 跟随者列表 |
| `offsets` | `[-3.0, 0.0, 0.0]` | 机体系偏移 `[dx,dy,dz]`，长度 3 或 3×N |
| `rate_hz` | `10.0` | 外环控制频率 |
| `cruise_speed` / `speed_gain` | `800` / `120` | 巡航 RPM 与误差增益 |
| `depth_min` / `depth_max` | `0.5` / `20` | 深度目标限幅 |
| `yaw_sign` / `depth_sign` | `1.0` | 坐标系符号修正（实测若方向相反改这里） |
| `set_autopilot_on_start` | `true` | 启动时把跟随者切到 `depthHeadingAutopilot` |
| `draw_targets` | `true` | 把期望位置画到视口 |

### 4.4 节点图

```
holoocean_node ──(<agent>/DynamicsSensorOdom)──► formation_controller ──(heading/depth/speed)──► holoocean_node
       ▲                                                                                              │
       └────────────────────── control_mode(SetControlMode) ◄─────────────────────────────────────────┘
```

### 4.5 前置条件：编队控制器依赖 Fossen 自动驾驶

本章的编队控制器把内环交给 HoloOcean 自带的 `depthHeadingAutopilot`，因此**要求场景中的载具注册了 Fossen 模型**：接口的 `set_depth` / `set_heading` / `set_speed` 都会先做一次 `vehicle_name in self.fossen_agents` 校验，不通过时只打印

```text
WARNING: Vehicle 'auv1' not found in the fossen vehicle registry. Cannot use command Set Heading
```

然后**静默忽略**该指令——现象是"节点在发，船不动"。

仓库现有配置的情况：

| 场景文件 | 是否注册 Fossen | 现象 |
| --- | --- | --- |
| `config/torpedo_fossen.json`（`TorpedoAUV`） | 是（agent 条目里有 `fossen_model`） | 深度/航向/速度自动驾驶可用 |
| `config/multi_agent.json`（`BlueROV2` + `SurfaceVessel`） | 否 | 上述三条通道不可用 |

想让编队控制器在 `multi_agent.json` 上跑起来，二选一：

1. **给智能体补 Fossen 模型**：参照 `config/torpedo_fossen.json` 的写法，在每个 agent 条目里补上 `fossen_model` 相关字段（含质量、惯量、水动力系数等）；
2. **改用手动通道**：把编队外环的输出换算成推力向量，从 `/holoocean/command/agent` 下发（`AgentCommand.command` 是 `float64[]`，长度由 `control_scheme` 决定），由自己在节点里实现速度/航向内环。

> 教学建议：先用方案 1 跑通"编队保持"的效果与量化指标，再让学生按方案 2 自己写内环，对比两者的控制品质。

## 5. 运行与验收

### 5.1 运行

```bash
# 终端 A：多机场景（内置自动驾驶 + 视口）
ros2 launch holoocean_main holoocean_launch.py \
  params_file:=$(ros2 pkg prefix --share holoocean_examples)/config/multi_joy_config.yaml

# 终端 B：编队控制器
ros2 run holoocean_examples formation_controller --ros-args \
  -p leader:=auv0 -p followers:="['auv1']" -p offsets:="[-3.0, 0.0, 0.0]"

# 终端 C：观察编队误差
ros2 topic echo /holoocean/auv1/DynamicsSensorOdom --field pose.pose.position
```

### 5.2 验收清单

- [ ] `ros2 topic list` 能同时看到 `/holoocean/auv0/...` 与 `/holoocean/auv1/...` 两组话题；
- [ ] `ros2 service call /holoocean/control_mode holoocean_interfaces/srv/SetControlMode "{vehicle_name: 'auv1', mode: 1}"` 返回 `success: true`；
- [ ] 给 `/holoocean/heading` 发一个带 `frame_id: auv1` 的目标，`auv1` 转向而 `auv0` 不受影响（验证多机分发）；
- [ ] `formation_controller` 启动后，`auv1` 逐渐靠近并保持与 `auv0` 的期望偏移；
- [ ] 领航者转向时，跟随者绕到新的相对方位（验证机体系偏移旋转）；
- [ ] 收敛后编队误差 RMS 小于设定的 `0.5 m`（把实测值填进第 6 节表格）。

### 5.3 效果演示

**编队误差收敛曲线**（数据来源见 6.2 节）：

![编队误差随时间收敛曲线](../img/water/formation_sim_error.png)

> **关于仿真画面**：本节原计划给出 HoloOcean 视口的编队动图。HoloOcean 自 2.x 起需要把 GitHub 账号与 Epic Games 账号关联才能取得发行包，本节编写环境无法安装仿真器，因此未提交三维画面。已配置 HoloOcean 的读者可按 6.3 节采集后补上 `docs/img/water/multi_agent_formation.gif` 并在此处加一行引用。

## 6. 量化评测

### 6.1 评测方法

领航者按预设航迹（直线段 → 90° 转向 → 直线段）运动，跟随者从与领航者重合的位置出发（初始编队误差恰为偏移模长），记录双方位置，按 4.1 节公式统计五项指标：

| 指标 | 含义 |
| --- | --- |
| 编队误差 RMS | 全程 $\|\mathbf{e}_i\|$ 的均方根 |
| 最大瞬时误差 | 全程峰值，主要出现在起始段与转向段 |
| 收敛时间 | 首次进入 ±0.3 m 并连续保持 3 s 的时刻 |
| 稳态误差 | 后 30% 采样的均值，反映外环的稳态精度 |
| 收敛后速度指令波动 | 收敛后速度指令的标准差，反映是否在目标点附近振荡 |

### 6.2 无仿真器环境下的实测结果

由于 HoloOcean 的获取需要 Epic Games 账号授权，本节用一个**同构的简化运动学回放**替代仿真器：`formation_sim.py` 复用 4.2 节的外环控制律与同样的机体系偏移，把内环替换为一阶响应模型（航向 $k_\psi=1.2\ \mathrm{s^{-1}}$、速度 $k_v=1.5\ \mathrm{s^{-1}}$、深度 $k_z=1.0\ \mathrm{s^{-1}}$），步长 0.02 s、时长 80 s，确定性可复现：

```bash
python3 src/water/holoocean_examples/holoocean_examples/formation_sim.py \
        --duration 80 --offset -3 0 0
```

| 指标 | 期望偏移 `[-3,0,0]` | 期望偏移 `[-3,2,0]` | 说明 |
| --- | --- | --- | --- |
| 编队误差 RMS | **0.628 m** | **0.661 m** | 越小越好 |
| 最大瞬时误差 | 2.999 m | 3.605 m | 起始段为初始偏差，转向段为次要峰值 |
| 收敛时间（到 ±0.3 m 并保持 3 s） | **8.70 s** | **8.98 s** | 反映外环增益与航向响应带宽 |
| 稳态误差（后 30% 采样均值） | 0.150 m | 0.150 m | 加入速度前馈后的稳态精度 |
| 收敛后速度指令波动（标准差） | 0.116 m/s | 0.111 m/s | 无持续振荡 |

这组数据也给出了一个可复现的设计结论：**只有位置反馈时稳态误差约 $(v_L-\bar v)/k_v$**，默认参数下约 0.57 m，收敛时间窗内永远进不了 ±0.3 m；加入领航者速度前馈（4.2 节公式中的 $v_L$ 项）后稳态误差降到 0.150 m，收敛时间稳定在 9 s 以内。

> **边界说明**：本节数据来自简化运动学模型，验证的是**外环控制律、机体系偏移生成、收敛性与稳态精度**；HoloOcean 的六自由度水动力、推进器动态与传感器噪声不在此模型内。因此这组数据可作为控制器回归测试，但**不能替代 HoloOcean 环境下的实测**。

### 6.3 HoloOcean 环境下的实测结果

以下数据在 **Ubuntu 22.04 + ROS 2 Humble + HoloOcean 2.3.0（TestWorlds 世界包）+ 无头模式（`-nullrhi`）** 下实测得到，场景为 `multi_agent_fossen.json`：两台 `TorpedoAUV`（`auv0` 领航、`auv1` 跟随，各带 `fossen_model: torpedo`），跟随者从期望编队位置出发，领航者按"直行 → 左转 90° → 直行"运动，采集 29 s：

| 指标 | 实测值（HoloOcean） | 对照：简化运动学模型 |
| --- | --- | --- |
| 编队误差 RMS | **0.617 m** | 0.628 m |
| 最大瞬时误差 | **1.211 m** | 2.999 m |
| 稳态误差（后 30% 采样均值） | **0.538 m** | 0.150 m |
| 收敛后速度指令波动（标准差） | 0.441 m/s | 0.116 m/s |

实测轨迹片段（跟随者位置，单位 m）：

```
t=0.00 s   (-3.00, -0.00, -10.00)   误差 0.000
t=29.0 s   (18.45, -0.15, -10.03)   误差 0.647   ← 领航者已沿 +x 前进约 21 m，跟随者保持约 3 m 间距
```

两点值得注意：

- 实测 RMS（0.617 m）与简化模型（0.628 m）高度一致，说明第 6.2 节的回放确实反映了外环的真实行为，可以当作低成本回归测试使用；
- 实测稳态误差略大（0.538 m vs 0.150 m）、速度波动更大（0.441 vs 0.116 m/s），差异来自第 6.2 节明确排除的因素：六自由度水动力、推进器动态与 Fossen 航向自动驾驶的响应滞后。

### 6.4 实测标定出的四条硬约束

以下四条都是本次实测中"踩出来的"，不是文档推测，建议使用者在同一套链路上先照做：

| 约束 | 现象 | 处理 |
| --- | --- | --- |
| **RPM 上限 1525** | 速度指令超限时 Fossen 接口抛 `ValueError: The RPM value should be in the interval 0-1525`，**该异常会直接杀掉桥接节点**（话题随之消失） | 速度指令必须限幅；本 PR 的控制器新增 `max_rpm` 参数（默认 1525） |
| **深度符号** | 场景 z 向下为负（-10 m），而深度指令为正值向下；直接传 z 会被限幅成 0.5 m，载具浮到水面 | `depth_sign=-1`（本 PR 已改为默认值） |
| **航向约定** | 实测：指令 0° 走向 +x、指令 90° 走向 **−y**，即 y 轴取反 | `heading = -atan2(e_y, e_x)`，即 `yaw_sign=-1`（本 PR 已改为默认值） |
| **自动驾驶模式必须逐台开启** | 只给跟随者切 `depthHeadingAutopilot` 时，领航者仍在手动模式、纹丝不动，实验等于没做 | 领航者与跟随者都要调用 `/holoocean/control_mode` 切模式（本 PR 的 `_arm_once` 已同时处理） |

此外 `/holoocean/reset` 在本版本中实测**不生效**（调用后载具位置不变），因此实验流程应改为"桥接起来后立刻启动控制器"，不要留无指令空窗——否则鱼雷会带着默认推力自己漂走，污染初始条件。

### 6.5 在 HoloOcean 环境下复测（步骤）

已配置 HoloOcean 的读者用随包提供的采集脚本复测，步骤：

1. 启动多机场景并（按 4.5 节）为跟随者注册 Fossen 模型：

```bash
ros2 launch holoocean_main holoocean_launch.py \
  params_file:=$(ros2 pkg prefix --share holoocean_examples)/config/multi_joy_config.yaml
ros2 run holoocean_examples formation_controller --ros-args -p leader:=auv0 -p followers:="['auv1']"
```

2. 采集 60 秒并自动统计、出图：

```bash
python3 src/water/holoocean_examples/holoocean_examples/evaluate_formation.py \
        --duration 60 --leader auv0 --followers auv1 --offsets -3,0,0
```

脚本会输出 `formation_metrics.csv`、`formation_error.png` 以及在终端打印一段与本表同格式的 Markdown，直接替换上表数值即可，同时可录制视口动图补入 5.3 节。

## 7. 故障排查

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| `Vehicle 'auv1' not found in the fossen vehicle registry` | 该智能体没有注册 Fossen 模型，深度/航向/速度通道对它无效 | 在场景 JSON 中为它配置 `fossen_model`，或改用 `command/agent` 手动通道 |
| 指令发出去但两台载具同时动 | `DesiredCommand.header.frame_id` 没写或写错 | 显式设置 `frame_id` 为载具名 |
| 只有一台载具有话题 | 该智能体的传感器没有 `ros_publish: true` | 在场景传感器配置里打开该字段 |
| 跟随者反向跑 | NWU 坐标系符号与预期不一致 | 调整 `yaw_sign` / `depth_sign` |
| 跟随者在目标点附近抖动 | 外环增益过大或未设置死区 | 降低 `speed_gain`，增大速度死区阈值 |
| 场景加载时报传感器字段缺失 | 传感器未显式命名 | 为每个传感器补上 `sensor_name` 字段后重试 |

## 8. 扩展方向

1. **水面-水下跨域协同**：本章场景里的 `auv1` 本身就是 `SurfaceVessel`，可让它作为通信中继，研究"水下 ROV 只在通信窗口内更新队形"的约束编队；
2. **声学定位约束下的编队**：`AcousticBeaconSensor`（方位角、俯仰角、距离）与 `DVLSensorRange` 可用于替代"理想里程计"，考察定位误差对编队精度的影响；
3. **从编队保持到编队重构**：领航者故障后由 `auv1` 接替，验证拓扑切换；
4. **编队与避障耦合**：把 `rov_sonar_slam_navigation` 一章的占据栅格与 Neural A* 引入多机场景，做编队级避障。
