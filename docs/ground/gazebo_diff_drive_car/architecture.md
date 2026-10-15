# 系统架构

---

## 4.1 整体设计

本项目采用**统一入口 + 模式调度**的架构，满足任务书
"每个模块必须支持 launch 启动，入口为 `main.xxx`" 的硬性要求。

```
                    roslaunch openhutb_car main.launch
                                  │
                    ┌─────────────┴─────────────┐
                    │   mode := teleop/laser/   │
                    │          camera/idle      │
                    └─────────────┬─────────────┘
                                  ▼
                    ┌───────────────────────────┐
                    │      main.launch          │
                    │  ┌─────────────────────┐  │
                    │  │ 1. Gazebo + world   │  │
                    │  │ 2. spawn_car        │  │
                    │  │ 3. robot_state_pub  │  │
                    │  │ 4. joint_state_pub  │  │
                    │  │ 5. main.py ★        │  │
                    │  │ 6. odom_recorder    │  │
                    │  │ 7. rviz (optional)  │  │
                    │  └─────────────────────┘  │
                    └───────────┬───────────────┘
                                ▼
                    ┌───────────────────────────┐
                    │        main.py            │
                    │   ModeDispatcher          │
                    │  ┌─────────────────────┐  │
                    │  │ VelocityWatchdog    │  │
                    │  │ (安全看门狗)         │  │
                    │  └──────────┬──────────┘  │
                    │             │ feed()      │
                    │  ┌──────────┴──────────┐  │
                    │  │  按 mode 动态加载    │  │
                    │  └──────────┬──────────┘  │
                    └─────────────┼─────────────┘
                                  ▼
          ┌───────────────┬───────┴───────┬───────────────┐
          ▼               ▼               ▼               ▼
   ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐
   │teleop_     │ │laser_      │ │depth_      │ │  (idle)    │
   │keyboard.py │ │avoid.py    │ │camera_     │ │  不加载     │
   │            │ │            │ │avoid.py    │ │            │
   └────────────┘ └────────────┘ └────────────┘ └────────────┘
          │               │               │
          └───────────────┴───────────────┘
                          ▼
                  /cmd_vel (Twist)
                          ▼
              ┌───────────────────────┐
              │ gazebo_ros_diff_drive │
              │      插件              │
              └───────────┬───────────┘
                          ▼
                    Gazebo 物理引擎
```

---

## 4.2 为什么采用统一入口

### 4.2.1 备选方案对比

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| A. 每功能独立 launch | `teleop.launch`、`laser.launch`… | 结构直观 | 违反任务书"统一入口"要求；重复配置 |
| B. 每功能独立入口脚本 | `main_teleop.py`、`main_laser.py`… | — | 不符合"入口为 main.xxx" |
| **C. 统一入口 + 模式参数** | **单 `main.launch` + `mode` 参数** | **符合要求；共享安全层；易扩展** | 需设计动态加载机制 |

本项目采用**方案 C**。

### 4.2.2 带来的好处

**① 共享安全策略**

所有模式的速度指令都经过同一个 `VelocityWatchdog`，保证：

- 失联自动停车
- 算法异常自动停车
- 退出时强制停车

若每个功能独立实现，安全逻辑需重复 3 次，容易不一致。

**② 切换零成本**

```bash
roslaunch openhutb_car main.launch mode:=teleop   # 切换键盘
roslaunch openhutb_car main.launch mode:=laser    # 切换雷达
```

**③ 便于对比**

三种模式共享同一套底盘接口与参数，实验条件可控，
性能对比才有意义。

---

## 4.3 安全看门狗设计

### 4.3.1 为什么需要看门狗

无人车在真实与仿真环境中都必须具备**失联停车**能力。可能的中断场景：

| 场景 | 后果（无看门狗） |
|---|---|
| 算法节点崩溃 | 车辆以最后速度持续行驶，可能撞墙 |
| 算法死循环卡住 | 同上 |
| 通信中断 | 底盘收不到新指令，保持旧速度 |
| 程序异常退出 | 车辆不停止 |

### 4.3.2 工作机制

```
        算法节点                          看门狗
    （teleop/laser/camera）
            │
            │  feed(cmd)                 ┌─────────────────┐
            ├───────────────────────────▶│ 刷新计时         │
            │                            │ _last_cmd_time  │
            │                            │ _stopped = False│
            │                            └────────┬────────┘
            │                                     │
            │                                     ▼ publish(cmd)
            │                            ┌─────────────────┐
            │                            │  /cmd_vel       │
            │                            └─────────────────┘
            │
            │  （主循环每周期）              ┌─────────────────┐
            │                            │  check()        │
            ├───────────────────────────▶│ elapsed > 1.0s? │
            │                            └────────┬────────┘
            │                                是   │   否
            │                          ┌──────────┘   └──────┐
            │                          ▼                     ▼
            │                 ┌─────────────────┐   ┌─────────────┐
            │                 │ publish(Twist())│   │   正常继续   │
            │                 │ 全零 = 停车      │   └─────────────┘
            │                 │ _stopped = True │
            │                 └─────────────────┘
```

### 4.3.3 实现要点

```python
class VelocityWatchdog(object):
    def __init__(self, publisher, timeout=COMMAND_TIMEOUT):
        self._pub = publisher
        self._timeout = timeout
        self._last_cmd_time = rospy.Time.now()
        self._stopped = False
        self._lock = threading.Lock()

    def feed(self, twist_msg):
        """喂入新指令：刷新计时并立即发布。"""
        with self._lock:
            self._last_cmd = twist_msg
            self._last_cmd_time = rospy.Time.now()
            self._stopped = False
        self._pub.publish(twist_msg)      # ★ 必须真正发布

    def check(self):
        """周期性检查，超时则强制停车。"""
        with self._lock:
            if self._stopped:
                return False
            elapsed = (rospy.Time.now() - self._last_cmd_time).to_sec()
            if elapsed > self._timeout:
                self._pub.publish(Twist())   # 全零 Twist
                self._stopped = True
                rospy.logwarn("速度指令超时 %.2fs，已触发安全停车", elapsed)
                return False
            return True
```

> **踩坑记录**：早期实现中 `feed()` 只缓存指令，由 `check()` 在超时后
> 才发布。结果正常指令**永远发不出去** —— `/cmd_vel` 完全无消息，
> 车辆静止。该缺陷已修复，详见
> [工程问题排查记录](troubleshooting.md#101-cmd_vel-完全无消息)。

---

## 4.4 主循环实现

### 4.4.1 为什么不用 rospy.Rate

`rospy.Rate` 以 `rospy.Time.now()` 为时间基准。当 `use_sim_time = True`
且 `/clock` 尚未发布时，`rospy.Time.now()` 返回 0 或停滞，
导致 `Rate.sleep()` **长时间阻塞**，主循环无法推进。

实测现象：节点启动后打印完初始化日志就再无输出，
`/cmd_vel` 完全无消息。

### 4.4.2 采用墙钟时间自行控制

```python
def spin(self):
    period = 1.0 / CONTROL_RATE        # 20 Hz → 50 ms

    if self.node is not None:
        self.node.start()

    ticks = 0
    while not rospy.is_shutdown():
        t_begin = time.time()

        if self.node is not None:
            try:
                self.node.update()
            except Exception as exc:
                # 算法异常不应导致整车失控
                rospy.logerr("算法更新异常：%s", exc)
                self.cmd_pub.publish(Twist())
                time.sleep(0.5)
                continue

        self.watchdog.check()
        ticks += 1

        # 每 5 秒打印心跳
        if ticks % int(CONTROL_RATE * 5) == 0:
            rospy.loginfo("控制回路运行中：已执行 %d 次循环", ticks)

        # 补偿本次循环耗时，保持稳定频率
        elapsed = time.time() - t_begin
        sleep_time = period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
```

### 4.4.3 实测频率稳定性

```
average rate: 20.021
	min: 0.045s max: 0.058s std dev: 0.00178s window: 19

average rate: 20.106
	min: 0.048s max: 0.067s std dev: 0.00411s window: 19
```

标准差仅 0.002~0.004 s，频率稳定性良好。

---

## 4.5 模块接口约定

所有算法节点遵循统一接口，便于 `main.py` 动态调度：

| 方法 | 签名 | 说明 |
|---|---|---|
| `__init__` | `(cmd_pub, watchdog)` 或 `(watchdog)` | 构造时注入依赖 |
| `start()` | `()` | 启动时初始化（如设置终端模式） |
| `update()` | `()` | **主循环每周期调用**，执行一次感知→决策→控制 |
| `stop()` | `()` | 退出时清理（恢复终端、输出统计） |

**设计优势**：`main.py` 无需了解各节点的内部实现，
只需按约定调用即可。新增功能模式时，只需：

1. 实现符合接口的节点类
2. 在 `ModeDispatcher._create_node()` 中注册
3. 在 `SUPPORTED_MODES` 中登记

---

## 4.6 TF 坐标变换树

```
odom                          ← 里程计坐标系（Gazebo 插件发布）
 └── base_footprint           ← 车体地面投影点（TF 根节点）
      └── base_link           ← 车体中心
           ├── left_wheel_link        ← 左驱动轮
           ├── right_wheel_link       ← 右驱动轮
           ├── front_caster_link      ← 前万向轮（仅视觉）
           ├── rear_caster_link       ← 后万向轮（仅视觉）
           ├── laser_link             ← 激光雷达
           └── camera_link            ← 深度相机
                └── camera_optical_link  ← 相机光学坐标系
```

**坐标系约定**（遵循 [REP-103](https://www.ros.org/reps/rep-0103.html)）：

| 轴 | 方向 |
|---|---|
| X | 车辆正前方 |
| Y | 车辆左侧 |
| Z | 垂直向上 |

**检查 TF 树**：

```bash
rosrun tf view_frames        # 生成 frames.pdf
rosrun tf tf_echo odom base_link   # 查看实时变换
```

---

## 4.7 包文件组织

```
openhutb_car/
├── package.xml                      包定义与依赖
├── CMakeLists.txt                   编译配置（含 devel 脚本复制规则）
├── README.md                        运行说明
│
├── launch/
│   └── main.launch                  ★ 统一启动入口
│
├── worlds/
│   └── car_world.world              仿真场景定义
│
├── urdf/
│   └── car.xacro                    车辆模型（几何 + 传感器 + 插件）
│
├── scripts/
│   ├── main.py                      ★ 统一入口节点
│   ├── teleop_keyboard.py           键盘控制
│   ├── laser_avoid.py               激光雷达避障
│   ├── depth_camera_avoid.py        深度相机避障
│   ├── odom_recorder.py             轨迹记录与评价
│   ├── plot_trajectory.py           轨迹绘图
│   └── analyze_trajectory.py        轨迹分析与对比
│
├── config/
│   └── params.yaml                  参数配置参考
│
├── rviz/
│   └── car.rviz.launch              RViz 配置
│
└── docs/                            文档（mdBook 规范）
```

---

## 4.8 数据流总览

```
  ┌─────────────────────────────────────────────────────────────┐
  │                      Gazebo 仿真环境                         │
  │                                                             │
  │   ┌──────────────┐     ┌──────────────┐   ┌─────────────┐  │
  │   │ 激光雷达      │     │ 深度相机      │   │ 物理引擎     │  │
  │   │ ray sensor   │     │ depth sensor │   │    ODE      │  │
  │   └──────┬───────┘     └──────┬───────┘   └──────┬──────┘  │
  │          │                    │                   │         │
  │          │ libgazebo_ros_laser│  libgazebo_ros_  │         │
  │          │ .so                │  depth_camera.so │         │
  └──────────┼────────────────────┼───────────────────┼─────────┘
             │                    │                   │
             ▼                    ▼                   │
        /scan               /camera/depth/            │
    (LaserScan)             image_raw (Image)         │
             │                    │                   │
             ▼                    ▼                   │
   ┌──────────────────┐ ┌──────────────────┐         │
   │  laser_avoid.py  │ │depth_camera_     │         │
   │                  │ │  avoid.py        │         │
   │ 感知→决策→控制    │ │ 感知→决策→控制    │         │
   └────────┬─────────┘ └────────┬─────────┘         │
            │                    │                   │
            └────────┬───────────┘                   │
                     ▼                               │
              ┌─────────────┐                        │
              │  watchdog   │                        │
              │   .feed()   │                        │
              └──────┬──────┘                        │
                     ▼                               │
                 /cmd_vel (Twist)                    │
                     │                               │
                     ▼                               │
        ┌────────────────────────────┐               │
        │ libgazebo_ros_diff_drive.so│               │
        │  差速运动学解算 → 轮速       │               │
        └────────────┬───────────────┘               │
                     │                               │
                     └───────────────────────────────┘
                              物理仿真
                                  │
                                  ▼
                            /odom (Odometry)
                                  │
                                  ▼
                        ┌──────────────────┐
                        │ odom_recorder.py │
                        │  轨迹记录+评价    │
                        └────────┬─────────┘
                                 ▼
                    ~/openhutb_car_results/
                       trajectory_*.csv
```
