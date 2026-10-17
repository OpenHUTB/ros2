# 车道保持控制（carla_lane_keeping）

本模块在车道线检测的基础上，把**横向偏差**与**航向偏差**闭环成车辆转向指令，
实现车辆自动保持在车道中央行驶。检测部分的原理见[车道线检测](./lane_detection.md)。

## 1. 控制结构

采用自动驾驶中最常见的**级联式横向控制**：

```text
横向偏差 e_y  ──┐
                ├─► PID ──┐
                │         ├─► 转向角 delta ──► 车辆
航向偏差 e_psi ─────────── ┘
```

控制律：

```text
delta = -(Kp·e_y + Ki·∫e_y dt + Kd·de_y/dt) - K_psi·e_psi
```

### 1.1 为什么偏差项要取负号

约定：

- `e_y > 0` 表示**车辆在车道中心右侧**，此时应该**向左**修正
- 控制输出 `steering > 0` 表示向左打方向

因此偏差项前带负号。若实测发现方向相反（不同相机的安装朝向、
或不同的控制接口符号约定），把参数 `invert_steering` 置为 `true` 即可，
**不需要改代码**。

### 1.2 纵向速度策略

```text
throttle = throttle_gain · target_speed · (1 - speed_correction_gain · |e_y|)
```

偏差越大越减速，给横向修正留出余量。这是工程上的常见做法：
高速下大偏差意味着车辆正在快速偏离车道，此时优先转向控制而不是保持车速。

## 2. 与车辆控制的对接

CARLA 的转向指令有两种传递方式，本模块**同时支持**，以便适配不同环境：

| 方式 | 话题 | 消息类型 | 适用场景 |
|---|---|---|---|
| 直接控制 | `/carla/ego_vehicle/vehicle_control_cmd` | `carla_msgs/CarlaEgoVehicleControl` | 已装 `carla_msgs`（推荐） |
| 间接控制 | `/carla/ego_vehicle/twist` | `geometry_msgs/Twist` | 缺少 `carla_msgs` 时，配合 `carla_twist_to_control` 节点 |

节点启动时会检测 `carla_msgs` 是否可用并自动选择，无需手工配置。

## 3. 安全设计

横向控制直接操纵车辆转向，因此做了三层保护：

### 3.1 感知失效保护

控制器以固定频率（默认 20 Hz）输出指令，但只有当感知数据**新鲜**
（最近 `perception_timeout` 秒内更新过）且**置信度达标**时才输出转向。

感知失效时：

- 停止转向输出（避免用过期数据持续打方向）
- **同时复位 PID 积分项**——否则偏差累积会在感知恢复瞬间造成转向突变
- 若同时判定为偏离车道，可选择主动刹车（`brake_on_departure`）

### 3.2 PID 积分限幅

积分项被限制在 `integral_limit` 之内，防止长时间同向偏差导致积分饱和
（积分饱和的典型表现是：偏差反向时车辆需要很长时间才回正）。

### 3.3 输出限幅

转向输出被限制在 `steering_limit` 之内，避免极端指令。

此外，检测节点在未稳定检出时**不发布横向偏差**，从源头避免下游把
误检当成有效测量。这两层是配合关系，缺一不可。

## 4. 运行

### 4.1 一键启动检测 + 控制

```shell
roslaunch carla_lane_keeping main.launch mode:=both
```

### 4.2 单独启动控制器

```shell
rosrun carla_lane_keeping lane_keeping_controller.py
```

### 4.3 观察控制效果

```shell
# 转向量（调试用）
rostopic echo /lane/steering

# 横向偏差，应逐渐收敛到 0 附近
rostopic echo /lane/lateral_offset

# 看实际下发的控制指令
rostopic echo /carla/ego_vehicle/vehicle_control_cmd
```

## 5. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `lane/lateral_offset` | `std_msgs/Float32` | 横向偏差（米） |
| 订阅 | `lane/heading_error` | `std_msgs/Float32` | 航向偏差（弧度） |
| 订阅 | `lane/confidence` | `std_msgs/Float32` | 置信度 |
| 订阅 | `lane/departure` | `std_msgs/Bool` | 是否偏离车道 |
| 发布 | `/carla/ego_vehicle/vehicle_control_cmd` | `carla_msgs/CarlaEgoVehicleControl` | 车辆控制指令 |
| 发布 | `/carla/ego_vehicle/twist` | `geometry_msgs/Twist` | 备用控制通道 |
| 发布 | `lane/steering` | `std_msgs/Float32` | 转向量（调试） |

## 6. 参数整定

参数见 `config/lane_params.yaml` 的控制段。整定顺序建议：

| 步骤 | 操作 | 观察 |
|---|---|---|
| 1 | 先只开检测（`mode:=detect`），把车开到车道中央 | `lateral_offset` 应接近 0 |
| 2 | 只调 `kp`，从小到大，先让车能回到中心 | 车辆出现缓慢来回摆动说明 `kp` 已够大 |
| 3 | 加入 `kd` 抑制摆动 | 摆动幅度减小、收敛加快 |
| 4 | 最后加很小的 `ki`（约 kp 的 1/10）消除稳态误差 | 不再有固定偏差 |
| 5 | 调 `k_psi` 让车头与车道方向对齐 | 车辆不再斜着走 |

**整定注意**：

- `ki` 过大是横向控制最常见的问题，会造成低频摆动，建议从 0 开始
- 若车辆持续来回大幅摆动，先降低 `kp` 而不是加 `kd`
- 转向方向相反时，先检查 `invert_steering`，不要用负的 `kp`

## 7. 控制安全提示

- 首次测试请把 `target_speed` 设小（如 2.0 m/s），确认方向正确后再提高
- 建议在 `brake_on_departure:=true` 下先跑一轮，确认偏离判定有效
- 控制器的输出频率（`control_rate`）不要低于车桥接收指令的期望频率，
  否则桥接层可能因超时而自动接管

## 8. 后续改进方向

- **前馈控制**：利用车道曲率直接给出前馈转角，减少弯道上的跟踪误差
- **模型预测控制（MPC）**：显式处理转向速率约束与预测时域
- **速度自适应**：按曲率自动限速，弯道减速
- **驾驶员接管检测**：检测到人工输入时自动让出控制权

## 参考文献

- [CARLA 官方文档](https://carla.readthedocs.io/)
- 检测部分见[车道线检测](./lane_detection.md)
- [设置并连接到 Carla 模拟器](../../set_up_and_connect_to_carla.md)
