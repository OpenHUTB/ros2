# uav_openhutb_lidar_nn_nav

> 基于 OpenHUTB 模拟器的无人机感知、建图导航与神经网络控制
> 环境：Ubuntu 20.04 + ROS Noetic（虚拟机）｜OpenHUTB（Windows 宿主机，RPC 41451）

## 1 模块定位

本模块围绕课程要求的四项任务，在**学校自研的 OpenHUTB 模拟器**上完成：

| 任务 | 内容 | 实现文件 |
| --- | --- | --- |
| 任务 1 | 物理仿真 + 键盘运动控制 | `scripts/keyboard_control.py` |
| 任务 2 | 传感器感知 + 运动控制 | `scripts/lidar_perception.py` |
| 任务 3 | 建图 + 导航（栅格 + A*） | `scripts/grid_nav.py` |
| 任务 4 | 端到端模型（扫描 → 控制） | `scripts/nn_policy.py`、`nn_train.py` |

模块遵循"**单一接触面**"设计：只有 `airsim_bridge.py` 调用 OpenHUTB 的 Python 客户端，
其余节点仅依赖 ROS 话题，因此算法可以脱离模拟器单独测试与复现。

## 2 坐标系与单位

OpenHUTB（继承 AirSim）使用 **NED**（N 前、E 右、D 下），ROS 惯例为 **ENU**（x 前、y 左、z 上）。
转换只发生在桥接层，规则为

```
位置： x_enu = y_ned,  y_enu = x_ned,  z_enu = −z_ned
速度： vx_enu = vy_ned, vy_enu = vx_ned, vz_enu = −vz_ned
偏航： yaw 由四元数解算，两套坐标下数值相同
```

因此 `/uav/cmd_vel` 中 `linear.z > 0` 表示**上升**，与 ROS 习惯一致。

## 3 任务 1：键盘遥控

键盘节点把按键映射为速度指令，经桥接层调用 `moveByVelocityZAsync`：

| 按键 | 动作 | 按键 | 动作 |
| --- | --- | --- | --- |
| w / s | 前进 / 后退 | r / f | 上升 / 下降 |
| a / d | 左移 / 右移 | q / e | 左偏航 / 右偏航 |
| t | 起飞 | g | 降落 |
| 空格 | 悬停 | | |

采用**速度控制**而非位置控制：给定水平速度与偏航角速度，高度单独作为设定值（`moveByVelocityZAsync`
的第三个参数），这样松开按键后飞行器在当前高度悬停，符合手动操控直觉。

## 4 任务 2：激光感知与反应式避障

### 4.1 扫描数据的生成

桥接层把 OpenHUTB 返回的点云投影成二维扫描：取机体系高度带

```
|z_ned + h_lidar| < 0.6 m
```

内的点，按方位角

```
θ_i = atan2(y, x) ∈ [−π/2, π/2]
```

分箱到 180 个角度格，每格取最近距离，未命中填 `range_max`。

### 4.2 避障控制律

设正前方扇区（±0.6 rad）最近距离为 `d_front`，左右扇区最近距离为 `d_left`、`d_right`：

```
d_front < d_stop        →  v = 0,      ω = ±ω_max（转向空隙较大一侧）
d_stop ≤ d_front < d_warn →  v = v_slow, ω = k · (d_left − d_right)/(d_left + d_right)
d_front ≥ d_warn        →  v = v_norm, ω = 0
```

默认 `d_stop = 1.2 m`、`d_warn = 2.5 m`。该策略计算量小、响应及时，适合作为行为克隆的专家。

## 5 任务 3：栅格建图与 A* 导航

### 5.1 占据栅格

栅格边长 0.25 m，覆盖 20 m × 20 m。对每束激光在端点处累加占据计数：

```
occ(i, j) += 1,  (i, j) = cell(x_robot + r·cos(θ), y_robot + r·sin(θ))
```

`occ > 0` 即视为障碍。为抑制噪声，每帧只使用一半激光束。

### 5.2 A* 规划

八邻域搜索，评价函数

```
f(n) = g(n) + h(n),  h(n) = ‖p_n − p_goal‖₂
```

直行代价 1、斜行 √2。为避免路径贴墙，规划前对障碍做 1 格膨胀。

### 5.3 纯追踪跟踪

取最近航点为目标，方位误差 `e = atan2(Δy, Δx) − yaw`，控制律

```
ω = clip(1.6 e, ±1.0)
v = v_cruise · max(0, 1 − |e|/1.6)
```

距当前航点小于 `goal_tolerance` 时切换下一航点。

## 6 任务 4：端到端神经网络

### 6.1 数据采集

运行专家策略（任务 2 的避障节点）的同时，`dataset_recorder.py` 旁听
`/uav/lidar/scan` 与 `/uav/cmd_vel`，把 90 维降采样扫描与对应动作写成 CSV。

### 6.2 网络与训练

```
输入 90 → 隐层 96（ReLU）→ 输出 2（v, ω）
损失 MSE + L2(1e-5)，SGD + 动量 0.9，学习率 2e-3，300 轮
```

**按时间顺序切分**训练/验证集（前 85% / 后 15%），避免相邻帧泄漏导致验证损失虚低。

### 6.3 推理

`nn_policy.py` 加载 `models/scan_policy.npz`，仅凭扫描输出速度指令，不使用任何真值位姿。

### 6.4 已知限制

行为克隆只在专家访问过的状态分布内可靠。跨场景（障碍更密）时策略会趋于保守，
表现为速度显著下降。改进方向：多场景采集数据、DAgger 式在线纠正、按轨迹段划分验证集。

## 7 源码解析要点

| 文件 | 关键点 |
| --- | --- |
| `airsim_bridge.py` | 连接与重连、NED↔ENU 转换、点云→二维扫描、20 Hz 主循环 |
| `keyboard_control.py` | `termios` 非阻塞读键、按 `rate` 持续发布 Twist |
| `lidar_perception.py` | 扇区最近距离、三段式控制律 |
| `grid_nav.py` | ray-casting 建图、八邻域 A*、纯追踪 |
| `dataset_recorder.py` | 序列化扫描特征与动作标签 |
| `nn_train.py` | 手写反向传播（NumPy），时间序切分 |
| `nn_policy.py` | 归一化推理、动作限幅 |

## 8 运行

```bash
python3 main.py --list
python3 main.py --mode keyboard --airsim-ip 10.0.2.2
roslaunch uav_openhutb_lidar_nn_nav main.launch mode:=nav goal_x:=8 goal_y:=0
```

各模式与参数说明见模块 `README.md` 与 `config/params.yaml`。
