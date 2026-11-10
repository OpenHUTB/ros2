# uav_moving_platform_landing —— 移动平台（车 / 船 / 移动甲板）精准降落

无人机落在一块**会动**的平台上（行驶中的车顶、航行中的甲板、会起伏的船）。和落在
静止屋顶不同，这里有两个绕不过去的麻烦：

1. **量测是旧的**：平台位置从感知到执行总要过一段时间（视觉推理 + RPC + 控制周期，
   实测 0.1~0.45 s）。直接拿旧数据当"现在的位置"去追，落点就必然带着
   `v × 时延` 的偏差；
2. **平台会从脚下溜走**：从 2.5 m 高度以 0.4 m/s 下降要 6 s，这 6 s 里 0.6 m/s 的平台
   已经走了 3.6 m —— 比一整块车顶都大。

本模块的做法：**只吃一路平台中心位置**（不管它来自视觉、另一个载具还是仿真器转发），
用加权最小二乘在线估出平台速度，据此判断"还要多久接地、那一刻平台在哪里"，再按
速度匹配或相遇点等待的方式制导，最后以相对高度判接地。全程只发 `/uav/cmd_vel`，
不碰飞控内部。

---

## 1. 架构位置

```
Windows 宿主                                    Ubuntu 客户机
┌──────────────────┐                        ┌──────────────────────────────────┐
│ AirSim / CarlaAir│◀── TCP 41451 ─────────▶│ carlair_ros_bridge               │
│  无人机 / 载具   │                        │  /uav/odom      /uav/cmd_vel     │
└──────────────────┘                        └───────────┬──────────────────────┘
                                                        │
                                    ┌───────────────────▼───────────────────┐
                                    │ 感知/转发节点（任选其一）              │
                                    │  视觉、另一个载具位姿、AirSim 位姿转发 │
                                    │  -> /uav/platform_pose                │
                                    └───────────────────┬───────────────────┘
                                                        │
                                    ┌───────────────────▼───────────────────┐
                                    │ uav_moving_platform_landing（本模块） │
                                    │  估计速度 → 预测落点 → 制导 → 接地     │
                                    └───────────────────────────────────────┘
```

连仿真器、NED↔ENU 换算、速度限幅都在桥接层完成，所以本模块与键盘控制、屋顶降落等
模块**共用同一条 `/uav/cmd_vel` 通道**。

---

## 2. 运行

### 2.1 前置

```bash
# 客机 1：桥接层（host 填宿主 IP）
rosparam set /control/cmd_duration 0.5      # 关键！默认 0.1 s 会让速度指令互相顶掉
roslaunch carlair_ros_bridge main.launch host:=192.168.49.1
```

> `cmd_duration=0.1`（默认）时，10 Hz 下发的下一条指令会在上一条生效前把它顶掉，
> 飞机几乎不动（实测：同样是 0.5 m/s 下降跑 4 s，`0.1 s` 只掉 0.11 m，`0.5 s` 掉 2.27 m）。

### 2.2 平台位置

`/uav/platform_pose`（`geometry_msgs/PoseStamped`）由你自己接：

* 视觉（Apriltag / 色块 / 神经网络）→ 平台中心的 ENU 位置；
* 仿真器里另一台载具的位姿转发；
* 离线回放。

**只要位置、不用给速度**——速度由本模块自己估，这也是它相对"直接订阅平台速度"的好处：
视觉方案拿不到可靠的平台速度。

### 2.3 启动

```bash
roslaunch uav_moving_platform_landing main.launch
# 或
rosrun uav_moving_platform_landing main.py
```

常用覆盖参数：

```bash
roslaunch uav_moving_platform_landing main.launch lead_mode:=now        # 强制速度匹配
roslaunch uav_moving_platform_landing main.launch descend_speed:=0.3    # 降慢一点
roslaunch uav_moving_platform_landing main.launch track_altitude:=1.5   # 对准高度低一点
roslaunch uav_moving_platform_landing main.launch z_up:=false           # 高度下为正
```

### 2.4 看什么

```bash
rostopic echo /uav/moving_landing/state     # 2 Hz 的 JSON：状态、剩余接地时间、平台速度估计
rostopic echo /uav/moving_landing/touchdown # 接地那一刻发一次
rostopic echo /uav/moving_landing/lead_point # 当前认定的落点（可加进 RViz）
```

节点日志会逐条打印状态切换，接地时打印横向误差，结束时打印状态轨迹。

---

## 3. 话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/uav/odom` | `nav_msgs/Odometry` | 无人机位姿/速度（桥接层提供，ENU） |
| 订阅 | `/uav/platform_pose` | `geometry_msgs/PoseStamped` | 平台中心位置；**时间戳要填观测时刻**，节点靠它补偿延迟 |
| 发布 | `/uav/cmd_vel` | `geometry_msgs/Twist` | 世界系速度指令，10 Hz |
| 发布 | `/uav/moving_landing/state` | `std_msgs/String` | 2 Hz JSON 状态 |
| 发布 | `/uav/moving_landing/lead_point` | `geometry_msgs/PointStamped` | 当前落点 |
| 发布 | `/uav/moving_landing/touchdown` | `geometry_msgs/Point` | 接地事件 |

状态机：`WAIT_DECK → ALIGN → DESCEND → TOUCHDOWN → LANDED`，
过程中可能 `DESCEND → ALIGN`（复飞），任何异常都收敛到 `ABORT`（原地悬停）。

---

## 4. 三种制导口径（`guide/lead_mode`）

| 值 | 朝哪一刻的平台位置飞 | 特点 |
| --- | --- | --- |
| `measured` | 最近一条**实测**样本 | 朴素基线，不补偿延迟。仅用于对照 |
| `now` | 估计器给出的**当前**平滑位置 + 平台速度前馈 | 贴着平台飞，精度最好；要求无人机横向速度 ≥ 平台速度 |
| `touchdown` | `deck(t+T)`，T = 还要多久接地（**相遇点**） | 由于 `dT/dt ≈ -1`，相遇点在世界上几乎静止 —— 无人机飞到那儿悬停，平台自己滑到下面 |
| `auto`（默认） | 跟上就跟、跟不上就等，转弯/升沉自动退回 `now` | 见下 |

`auto` 的判定：

* 平台速度 ≤ `keep_up_ratio × max_xy_speed` → `now`；
* 平台更快（跟不住）→ 锁存成 `touchdown`，改"飞到相遇点等它"；
* 平台在转弯（横向加速度显著）或升沉（垂向速度 > 0.1 m/s）→ 相遇点外推不可信，
  退回 `now`。

---

## 5. 实测数据（离线闭环，可复现）

`tools/evaluate.py` 用**同一套运动学模型**（一阶速度滞后 0.35 s、10 Hz 带延迟带噪声观测）
跑四种制导策略，落点误差用仿真真值算（不是控制器自己的估计）：

| 场景 | 追实测位置 | 追当前位置+前馈 | 相遇点+前馈 | **自动（默认）** |
| --- | --- | --- | --- | --- |
| 直线 0.4 m/s | 0.24 m / 100% | 0.03 m / 100% | 0.10 m / 100% | **0.03 m / 100%** |
| 直线 0.8 m/s | 0.48 m / 0% | 0.03 m / 100% | 0.22 m / 100% | **0.03 m / 100%** |
| 直线 1.2 m/s | 落不准 / 0% | 0.03 m / 100% | 0.34 m / 20% | **0.03 m / 100%** |
| 圆周 0.6 m/s (R=4) | 0.36 m / 0% | 0.08 m / 100% | 0.09 m / 100% | **0.08 m / 100%** |
| 圆周 1.0 m/s (R=4) | 落不准 / 0% | 0.11 m / 100% | 0.11 m / 100% | **0.11 m / 100%** |
| 升沉 0.5 m/s ±0.15 m | 0.30 m / 40% | 0.02 m / 100% | 0.02 m / 100% | **0.02 m / 100%** |

（误差 = 接地瞬间无人机与平台真实中心的水平距离，5 个噪声种子均值；成功率 = 误差 ≤ 0.30 m 的比例。）

**平台比无人机快时**（横向上限压到 0.6 m/s，无人机起点在平台前方 10 m）：

| 平台速度 | 追当前位置+前馈 | 相遇点+前馈 | 自动 |
| --- | --- | --- | --- |
| 0.4 m/s | 0.02 m / 100% | 0.14 m / 100% | **0.02 m / 100%** |
| 0.8 m/s | 2.66 m / 0% | 0.21 m / 100% | **0.21 m / 100%** |
| 1.0 m/s | 5.66 m / 0% | 0.29 m / 60% | **0.11 m / 100%** |
| 1.2 m/s | 8.90 m / 0% | 3.14 m / 0% | 4.30 m / 0% |

最后一行是**物理上做不到**：从 8 m 高度以 0.4 m/s 下降要 20 s，这 20 s 里相遇点在
24 m 外，无人机需要 0.7 m/s 的水平速度，超过 0.6 m/s 的上限 —— 谁落不准都是应该的，
这条数据反而说明模型没有作弊。

重跑：

```bash
python3 src/air/uav_moving_platform_landing/tools/evaluate.py
# 产出 docs/img/air/uav_moving_platform_landing/*.png 与 tools/results/scenarios.csv
```

ROS 闭环（假桥接层，`tests/test_uav_moving_platform_landing_ros_sim.py`）两轮实跑：
`WAIT_DECK → ALIGN → DESCEND → TOUCHDOWN → LANDED`，桥接层用真值量到的落点误差
**0.034 / 0.069 / 0.145 m**（三轮）（平台 0.6 m/s、观测延迟 0.15 s、噪声 0.03 m）。

---

## 6. 参数（`config/moving_platform_landing.yaml`）

最常动的几个：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `descend/speed` | 0.4 | 下降段垂直速度（m/s），越大越快也越容易落偏 |
| `guide/track_altitude` | 2.5 | 对准段保持高度（m）。降低它会缩短 `T`，适合快速平台 |
| `guide/max_xy_speed` | 1.6 | 横向速度上限，必须与飞控实际能力一致 |
| `guide/align_tol` | 0.35 | 对准容差（m） |
| `guide/abort_xy_tol` | 0.9 | 预计落偏超过它复飞（m） |
| `guide/lead_mode` | auto | 见上表 |
| `tracker/window_s` | 2.5 | 速度估计滑窗（s）。平台机动多就调小，噪声大就调大 |
| `mission/max_go_around` | 3 | 最多复飞几次，超过就 `ABORT` |

调参顺序建议：先看 `state` 话题里的 `v_deck` 是否接近平台真实速度，再动制导增益。

---

## 7. 故障排查

| 现象 | 原因 / 处理 |
| --- | --- |
| 一直停在 `WAIT_DECK` | 没收到 `/uav/platform_pose`，或样本不足 `tracker/min_samples`；`rostopic hz` 看 10 Hz 左右 |
| 状态在 `ALIGN` 反复超时 | 看 `state` 里的 `t_touchdown` 是否剧烈跳动：多半是平台位置噪声太大 / 时间戳没填观测时刻 |
| 飞机几乎不动但日志正常 | 桥接层 `control/cmd_duration` 还是 0.1 s，按 §2.1 调到 0.5 s |
| 下降中突然复飞 | `abort_xy_tol` 太小，或平台确实在加速；看日志里"预计落偏 x.xx m" |
| 落点总是偏后 | 平台位置时间戳填成了"收到消息的时刻"而非"观测时刻"，延迟补偿反了 |
| 高度越降越高 / 方向反了 | `frame/z_up` 与桥接层约定不符（桥接层实测 ENU，`true`） |

---

## 8. 测试

```bash
# 1) 离线回归：无 ROS、无仿真器、无 GPU
python3 tests/test_uav_moving_platform_landing_local.py

# 2) ROS 闭环：假桥接层（自建 /uav/odom 与 /uav/platform_pose，积分 cmd_vel）
roscore &
python3 tests/test_uav_moving_platform_landing_ros_sim.py
```

第 1 个测试覆盖：速度估计（含噪声/延迟/跳点/断流）、曲率判定、下降剖面与接地时间
（含升沉甲板的不动点解）、状态机（正常/超时/穿甲板/复飞/限幅）、四种策略的定量对照、
包结构约定（含"必需文件确实被 git 跟踪"、"CMakeLists 里 install 的脚本都存在"、
"package.xml 能被 catkin_pkg 解析"——这三条是补第一次提交漏提交 `CMakeLists.txt` 的教训）。

包级检查（不需要编译器）：

```bash
rospack find uav_moving_platform_landing                    # 包可被发现
roslaunch --files uav_moving_platform_landing main.launch    # launch XML 解析
roslaunch uav_moving_platform_landing main.launch             # 60 个参数从 YAML 载入并起节点
```

---

## 9. 已知限制

* **不做避障**：对准段与下降段都假设当前高度到平台之间没有障碍物（桥接层的
  `moveToPositionAsync` 也不避障）；
* **平台转弯很急时**相遇点外推会偏，`auto` 会自动退回速度匹配；平台速度超过无人机
  横向速度上限 **且** 几何上赶不上相遇点时无解（见 §5 最后一行）；
* **平台倾角**未建模：假定甲板水平，落在倾斜甲板上需要另外估计法向；
* **接地判据靠相对高度**：真机建议把 `/uav/status` 或电流/触地开关一起纳入判决。

## 10. 大模型使用声明

本模块的代码、测试与文档在编写过程中使用了大模型辅助（算法讨论、代码与文档起草、
绘图脚本整理）。所有结论均来自仓库内可复现的运行结果（离线回归 + ROS 闭环 + 定量对照
实验），作者对提交内容的正确性与完整性负全部责任。
