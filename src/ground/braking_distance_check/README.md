# braking_distance_check

制动距离与跟车间距的物理核算：由车速、反应时间、路面附着系数与坡度，算出反应距离、制动距离、停车距离与两秒法则要求的车距，并用四条断言判断这次跟车/接近是否安全。

纯物理计算，**不依赖 CARLA**，不需要 GPU 与显示器，一条命令在终端出结果。

## 为什么需要它

跟车太近、雨天不减速、下坡不提前刹车，本质都是同一个问题：**没人算过「以这个速度，从看见到停住要多少米」**。

教科书上这个公式一直存在，但在工程里它通常是散落在设计文档里的几张表。本包把它做成可执行、可断言、可回归的东西：

* 三个可加和的量：`反应距离 + 制动距离 = 停车距离`
* 一个可对照的量：两秒法则要求的跟车距离
* 四条互相独立的断言：停车距离、跟车间距、路面附着、限速

## 目录结构

```
braking_distance_check/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/braking_distance_check.yaml   # 10 个参数默认值
├── launch/main.launch                   # 10 个带 doc 的 arg
└── scripts/
    ├── main.py                          # roslaunch 入口
    ├── braking_core.py                  # 纯物理计算（不 import rospy）
    └── braking_node.py                  # ROS 收发放这里
```

## 公式

| 量 | 公式 |
|---|---|
| 反应距离 | `v0 * t_r` |
| 制动减速度 | `a = μ g cosθ + g sinθ`（θ > 0 上坡，帮着刹车） |
| 制动距离 | `v0^2 / (2a)` |
| 停车距离 | `反应距离 + 制动距离` |
| 两秒法则车距 | `v0 * 2 s + 余量` |

`v0` 单位 m/s，`t_r` 秒，`μ` 附着系数，`g = 9.81 m/s²`。

## 四条断言

| 断言 | 判据 | 不过时的含义 |
|---|---|---|
| 停车距离 | 停车距离 ≤ 前方可用距离 | 到前车/障碍物前停不住 |
| 跟车间距 | 车距 ≥ 两秒法则要求 | 跟得太紧 |
| 路面附着 | `μ ≥ 0.30` | 路面太滑（雪/冰/积水） |
| 限速 | 车速 ≤ 路段限速 | 超速 |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `scene/speed_kmh` | 50.0 | 当前车速 |
| `scene/available_distance` | 40.0 | 前方可用距离 |
| `scene/gap` | 32.0 | 当前跟车间距 |
| `vehicle/reaction_time` | 1.00 | 反应时间 |
| `vehicle/speed_limit_kmh` | 60.0 | 限速 |
| `road/friction` | 0.70 | 附着系数（干 0.7~0.9、湿 0.4、雪 0.2、冰 0.1） |
| `road/slope_deg` | 0.0 | 坡度（正为上坡） |
| `rate/publish_hz` | 1.0 | 报告重发频率 |
| `topic/report` | `/braking/report` | 报告话题 |
| `topic/bars` | `/braking/bars` | 距离字符条话题 |

## 运行

```bash
cd ~/catkin_ws8
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
roslaunch braking_distance_check main.launch
```

改参数：

```bash
roslaunch braking_distance_check main.launch friction:=0.35
roslaunch braking_distance_check main.launch gap:=20
roslaunch braking_distance_check main.launch speed_kmh:=70 speed_limit_kmh:=60
```

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/braking/report` | `std_msgs/String`（latched） | 完整报告：距离分解 + 四条断言 + 车速对照表 |
| `/braking/bars` | `std_msgs/String`（latched） | 一条字符条：`=` 反应距离、`#` 制动距离、`-` 前方尚可用 |

## 离线测试

```bash
cd <仓库根目录>
python3 tests/test_braking_distance_check_local.py
```

只用标准库，不需要 ROS / CARLA / 显示器；同时兼容 Python 2.7 与 3.x。

也可以交给 pytest（每个分组被包装成真正会失败的 `test_*`）：

```bash
python3 -m pytest tests/test_braking_distance_check_local.py -q
```

## 已知问题

**Melodic 上中文日志报 `UnicodeEncodeError`？**
Python 2.7 里 rosgraph 的日志格式化会对非 ASCII unicode 调 `str()`。本包所有日志都先编码成 byte str（`braking_node.py` 的 `_log()`），所以安全。

**`roslaunch` 报 `[main.launch] is neither a launch file in package`？**
那个终端没 source：先 `source /opt/ros/melodic/setup.bash`，再 `source devel/setup.bash`。
