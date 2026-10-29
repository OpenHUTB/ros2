# gesture_control_drone

**手势控制无人机**：摄像头 + MediaPipe 识别手部手势，转换成指令通过 ROS 话题发布，
由控制节点调用 **AirSim 多旋翼 API** 完成 起飞 / 降落 / 悬停 / 前进 / 上升 / 下降 /
紧急停止，并把位姿以 `nav_msgs/Odometry`（ENU）回传。

对应文档页面：[手势控制 AirSim 无人机](../../../docs/air/gesture_control_drone.md)。

---

## 1. 目录结构

```
gesture_control_drone/
├── CMakeLists.txt             catkin_install_python 安装全部脚本
├── package.xml                依赖 rospy/std_msgs/nav_msgs + python3-opencv 等
├── requirements.txt           mediapipe(<1.0) / airsim / pygame / Pillow
├── README.md
├── config/gesture_control.yaml    参数（launch 载入各节点私有命名空间）
├── launch/main.launch         统一启动入口
└── scripts/
    ├── main.py                模块入口（转发到手势感知节点）
    ├── gesture_ros_node.py    感知节点：摄像头 -> 手势 -> 指令话题
    ├── drone_control_node.py  控制节点：订阅指令 -> 工作线程执行 -> 状态回传
    ├── gesture_map.py         ★ 手势 / 指令唯一词表（含模块级自检）
    ├── gesture_gate.py        去抖 + 边沿触发 + 冷却
    ├── gesture_detection.py   手势识别（唯一实现，纯函数可离线单测）
    ├── camera_utils.py        跨平台摄像头
    ├── drone_client.py        ★ AirSim 多旋翼封装（连接/动作/限幅/超时/状态）
    └── gesture_demo.py        无 ROS 单机演示（pygame 虚拟无人机）
```

## 2. 安装依赖

```bash
# 优先用 apt（避免 pip 版 OpenCV 与系统 ROS 冲突）
sudo apt install -y python3-numpy python3-opencv python3-pil fonts-wqy-zenhei
# mediapipe / airsim / pygame 用 pip
pip3 install -r requirements.txt

> 官方 `airsim` 在 PyPI 只有 sdist，构建期要 `numpy` 与 `msgpackrpc`，Linux 上按这个顺序装：
> ```bash
> sudo apt install -y gcc python3-dev
> pip3 install --user numpy msgpack-rpc-python
> pip3 install --user --no-build-isolation airsim
> ```
> Windows 上编不了 `msgpack-python`，可改用 `pip install cosysairsim`（注意：Cosys 客户端是协议 v4，只对接新版本服务端）。

> `mediapipe`、`airsim`、`pygame` 没有 rosdep 键，`rosdep install` 装不上，**必须**用上面的 `pip3` 命令装。
> 其中 **mediapipe 必须 0.10.x**（`requirements.txt` 已钉 `<1.0`）：1.x 移除了 `mp.solutions`，装最新版会直接 `AttributeError: module 'mediapipe' has no attribute 'solutions'`。
```

## 3. 编译与运行

```bash
cd ~/uav_ws && catkin_make && source devel/setup.bash

# 宿主机先启动 AirSim 仿真器并放行 41451，然后：
roslaunch gesture_control_drone main.launch airsim_ip:=192.168.80.1
```

跨机部署时 **`airsim_ip` 必须填宿主机的 IP**（不是 `127.0.0.1`）。

只测识别、不动飞机：

```bash
roslaunch gesture_control_drone main.launch start_drone_control:=false show_window:=true
```

没有 ROS / AirSim 时用单机演示验证手势与词表：

```bash
rosrun gesture_control_drone gesture_demo.py
```

## 4. 手势与指令（7 种）

| 手势 | 手势 token | 指令 token | 动作 |
|---|---|---|---|
| 张开手掌 | `PALM_OPEN` | `takeoff` | 解锁 → 起飞 → 爬到 `takeoff_altitude` → 悬停 |
| 握拳 | `FIST` | `land` | 降落 |
| 食指指向 | `POINT` | `forward` | 机体系向前一段 |
| 胜利手势 | `VICTORY` | `up` | 上升一档（限 `max_altitude`） |
| 拇指向下 | `THUMB_DOWN` | `down` | 下降一档（限 `min_altitude`） |
| 拇指向上 | `THUMB_UP` | `hover` | 原地悬停 |
| OK 手势 | `OK` | `estop` | 紧急停止（默认悬停） |

另有 `yaw_left` / `yaw_right` / `home` 三个指令不占手指，可直接用话题触发：

```bash
rostopic pub -1 /gesture_command std_msgs/String "data: 'home'"
```

要改映射只改 `scripts/gesture_map.py` 的 `GESTURE_TO_COMMAND`，两端自动同步。

## 5. AirSim 侧做了什么（`scripts/drone_client.py`）

| 能力 | 实现 |
|---|---|
| 连接 | `confirmConnection` + `enableApiControl` 只做一次并复用；失败可重试 |
| 起飞 | `armDisarm(True)` → `takeoffAsync` → `moveToZAsync(-takeoff_altitude)` → `hoverAsync`（定高仅在 `*Async` 路径生效，同步回退由仿真器默认高度决定） |
| 降落 / 悬停 | `landAsync` / `hoverAsync` |
| 位移 | `moveByVelocityBodyFrameAsync`（机体系；NED 下 `-z` 为上） |
| 偏航 | `rotateByYawRateAsync`（NED 顺时针为正） |
| 返航 | `goHomeAsync(timeout)`，缺该方法时回退到 `goHome` / `moveToPositionAsync` |
| 急停 | `hover` / `land` / `disarm` 三选一，默认 hover |
| 状态 | `getMultirotorState` + `simGetCollisionInfo` + `isApiControlEnabled`，转 ENU |
| 安全 | 限速 `max_speed`、限高 `max_altitude`、限低 `min_altitude` |
| 超时 | 每个 RPC 在工作线程执行并加 `command_timeout`，卡住即失败 |
| 退出 | 先悬停再 `enableApiControl(False)`，不锁桨 |
| **API 兼容** | `*Async` 只是客户端对 `call_async('takeoff', ...)` 的包装（服务端所有版本都只有 `takeoff/land/hover/goHome`）。连上后探测客户端是否提供 `*Async`：有则走异步，没有则回退到同名同步方法，并用**独立连接发完即弃**、状态靠轮询 |
| **客户端** | 优先 `import airsim`；装不上时回退 `cosysairsim`（Windows 无需编译器，RPC 协议一致） |
| **结果语义** | 有 `*Async` 时 `join()` 能拿到 AirSim 的异常；同步回退模式下同步命令是"发完即弃"，只能确认"已下发"，结果看 `/drone/odom` 与 `/drone_state` |
| **摄像头** | Linux 下默认用 V4L2 + **MJPG 640×480**：VMware/VirtualBox 透传的摄像头在默认 YUYV 720p 下会 `select() timeout`；可用 `GESTURE_CAMERA_FOURCC=YUYV` 覆盖 |
| **暂停检查** | 连接时查 `simIsPaused`，若被暂停自动 `simPause(False)`（暂停时物理不推进，`takeoff` 会永远等不到高度） |
| **线程安全** | `msgpack-rpc` 客户端不可并发使用（状态定时器 + 指令线程），所有 RPC 由 `RLock` 串行化 |
| **动作超时** | `command_timeout`(10 s) 用于一般调用；`takeoff_timeout` / `land_timeout`(60 s) 单独给起降（3 m 降落约 14 s） |

> 细节见 `docs/air/gesture_control_drone.md` §10：同步 `takeoff`/`land` 会阻塞到动作
> 完成（3 m 降落约 14 s）；`landed_state` 官方语义是 `Landed = 0`、`Flying = 1`；
> 起降判定以**高度**为准。

## 6. 状态回传

| 话题 | 类型 | 说明 |
|---|---|---|
| `/drone/odom` | `nav_msgs/Odometry` | 位姿 + 速度，**ENU**，`frame_id=world`；`state_rate` Hz |
| `/drone_state` | `std_msgs/String` | 每次指令的执行结果（`takeoff OK` / `down FAIL …`） |
| `/gesture_state` | `std_msgs/String` | 当前识别到的手势 token |

RViz 里把 Fixed Frame 设为 `world`，追加 `Odometry`（话题 `/drone/odom`）即可看到轨迹。

## 7. 参数

见 `config/gesture_control.yaml`。常用：`camera_index`、`stable_frames`、
`command_cooldown`、`airsim/ip`、`estop_action`、`takeoff_altitude`、`speed`、
`max_speed`、`max_altitude`、`min_altitude`、`command_timeout`、`state_rate`。

## 8. 测试

```bash
python3 tests/test_gesture_control_local.py
```

不需要 ROS、摄像头、仿真器或显示器；10 组断言包含 **7 种手势 → 指令 → AirSim 调用**
的端到端链路、安全限幅、RPC 超时、NED→ENU 转换、Odometry 回传与工程文件约定。

## 附：大模型使用声明

本模块代码与文档在编写过程中使用了 AI 大模型辅助（需求分析、问题排查、代码与
文档起草）。感知与控制的纯逻辑部分由 `tests/test_gesture_control_local.py`
离线回归覆盖；真机（AirSim + 摄像头）的动作序列以现场日志为准。
作者对提交内容的正确性与完整性负全部责任。
