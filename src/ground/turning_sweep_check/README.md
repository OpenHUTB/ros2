# turning_sweep_check

车辆转弯扫掠包络校验器：由轴距、轮距、前后悬与转弯半径，算出内轮差、外摆与所需通道宽度，并用四条断言判断这个弯能不能过。

纯几何计算，**不依赖 CARLA**，不需要 GPU 与显示器，一条命令在终端出结果。

## 为什么需要它

车身转弯时扫过的是一个圆环，不是一个圆：

* 内侧后轮走小圆，外侧前角走大圆，两者之差就是「这个弯至少要多宽的通道」
* 轴距越长、前悬越长，环越宽；窄路掉头、地下车库、限宽门都卡在这个量上
* 平时只能靠「打满方向盘试一下」，本题把它变成可计算、可断言、可回归的三个数字：**所需通道宽度、内轮差、外摆**

## 目录结构

```
turning_sweep_check/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/turning_sweep_check.yaml    # 13 个参数默认值
├── launch/main.launch                 # 13 个带 doc 的 arg
└── scripts/
    ├── main.py                        # roslaunch 入口
    ├── turning_core.py                # 纯几何计算（不 import rospy）
    └── turning_node.py                # ROS 收发放这里
```

## 公式

以后轴中心到转弯中心的距离 `R` 为基准（米）。`L` 轴距，`b` 轮距，`fo` 前悬，`ro` 后悬：

| 量 | 公式 |
|---|---|
| 内后轮半径 | `R - b/2` |
| 外后轮半径 | `R + b/2` |
| 内前轮半径 | `hypot(R - b/2, L)` |
| 外前轮半径 | `hypot(R + b/2, L)` |
| 外前角半径 | `hypot(R + b/2, L + fo)` |
| 内后角半径 | `hypot(R - b/2, ro)` |
| 扫掠宽度 | `max(所有半径) - min(所有半径)` |
| 内轮差 | `内前轮半径 - 内后轮半径` |
| 外摆 | `外前角半径 - 外前轮半径` |
| 最小可行半径 | `L / tan(max_steer)` |

## 四条断言

| 断言 | 判据 | 不过时的含义 |
|---|---|---|
| 转弯可行性 | `R >= L / tan(max_steer)` | 方向盘打满也拐不过 |
| 通道宽度 | `扫掠宽度 <= channel_width` | 路/库位不够宽 |
| 内轮差 | `内轮差 <= max_inner_diff` | 轴距太大，内后轮切内角 |
| 内侧余量 | `内圈半径 >= inner_clearance` | 内侧贴墙，没有余量 |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `vehicle/wheelbase` | 2.70 | 轴距 |
| `vehicle/track` | 1.54 | 轮距 |
| `vehicle/front_overhang` | 0.90 | 前悬 |
| `vehicle/rear_overhang` | 0.85 | 后悬 |
| `vehicle/width` | 1.80 | 车宽 |
| `vehicle/max_steer` | 0.55 | 最大前轮转角（rad） |
| `turning/radius` | 8.0 | 后轴中心转弯半径 |
| `check/channel_width` | 6.0 | 通道可用宽度 |
| `check/inner_clearance` | 0.30 | 内侧最小余量 |
| `check/max_inner_diff` | 1.00 | 内轮差上限 |
| `rate/publish_hz` | 1.0 | 报告重发频率 |
| `topic/report` | `/turning/sweep` | 报告话题 |
| `topic/ring` | `/turning/ring` | 字符图话题 |

## 运行

```bash
cd ~/catkin_ws7
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
roslaunch turning_sweep_check main.launch
```

改参数：

```bash
roslaunch turning_sweep_check main.launch channel_width:=2.0
roslaunch turning_sweep_check main.launch turn_radius:=4.0
```

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/turning/sweep` | `std_msgs/String`（latched） | 完整报告，含四条断言与结论 |
| `/turning/ring` | `std_msgs/String`（latched） | 扫掠圆环字符图 |

## 离线测试

```bash
cd <仓库根目录>
python3 tests/test_turning_sweep_check_local.py
```

只用标准库，不需要 ROS / CARLA / 显示器；同时兼容 Python 2.7 与 3.x。

也可以交给 pytest（每个分组被包装成真正会失败的 `test_*`）：

```bash
python3 -m pytest tests/test_turning_sweep_check_local.py -q
```

## 已知问题

**Melodic 上中文日志报 `UnicodeEncodeError`？**
Python 2.7 里 rosgraph 的日志格式化会对非 ASCII unicode 调 `str()`。本模块所有日志都先编码成 byte str（`turning_node.py` 的 `_log()`），所以安全。

**`roslaunch` 报 `[main.launch] is neither a launch file in package`？**
那个终端没 source：先 `source /opt/ros/melodic/setup.bash`，再 `source devel/setup.bash`，`catkin_make` 之后再跑一次。
