# 里程计轨迹记录器（odom_trace_recorder）

把「车开过的这一段」记录下来：订阅 `/odom` 边收边记，到点写成 CSV，再回放成
`nav_msgs/Path` 并与一条参考圆弧逐点比对，最后打印误差直方图与四条断言。

不依赖 CARLA、不需要显示器、不需要任何 pip 包，只要有 rospy 就能跑。

## 为什么不是「算一次就打印」

仓库里已经有不少「读参数 → 算一次 → 打印报告」的校验模块，它们的输入是参数，
输出是结论。这个包不一样：

| | 一次算完的校验模块 | 本模块 |
|---|---|---|
| 输入 | 参数 | `/odom` 话题（时间序列） |
| 状态 | 无 | 边收边累积采样点 |
| 磁盘 | 不写文件 | 轨迹落盘成 CSV，可读回来复算 |
| 输出 | 报告文本 | 报告 + `nav_msgs/Path` 回放 + CSV |
| 判定依据 | 参数之间的自洽 | 记录下来的轨迹 vs 参考轨迹 |

所以它验证的是「记录这件事本身有没有丢掉信息」：点数够不够、里程对不对、
插值往返是否稳定、与参考轨迹差多少。

## 目录结构

```
odom_trace_recorder/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── trace_recorder.yaml      # 13 个默认参数
├── launch/
│   └── main.launch              # 13 个带 doc 的 arg
└── scripts/
    ├── main.py                  # 入口
    ├── trace_core.py            # 纯逻辑：采样点/轨迹/重采样/比对/直方图
    └── trace_node.py            # ROS 收发：订阅 /odom，发布 /trace/replay 与 /trace/report
```

## 运行

```bash
# 默认：记录 8 s，内置模拟车辆按半径 6 m、速度 2 m/s 画一段圆弧
roslaunch odom_trace_recorder main.launch

# 改参考半径，让「误差在限」失败（验证比对环节真的在工作）
roslaunch odom_trace_recorder main.launch reference_radius:=4.0

# 要求 500 个采样点，让「采样完整」失败
roslaunch odom_trace_recorder main.launch min_samples:=500

# 只订阅外部 /odom（例如接 carla_ros_bridge 或自己 rostopic pub）
roslaunch odom_trace_recorder main.launch use_mock:=false
```

节点会自己结算并退出，不需要手动 `Ctrl+C`。

## 真实输出（默认参数）

```
记录：161 个点，时长 8.00 s，里程 16.000 m
起点：(0.000, 0.000)，终点：(2.649, 11.359)
包围盒：x ∈ [0.000, 5.950]，y ∈ [0.000, 11.359]
落盘：/tmp/trace.csv（161 行数据）
参考：mock_r6.00（161 个点，16.000 m）
比对：最大误差 0.0972 m，平均误差 0.0573 m，最差点第 161 个
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 采样完整 记录 161 个点，要求 >= 100
  [PASS] 里程合理 里程 16.000 m，参考 16.000 m（容差 0.500 m）
  [PASS] 回放一致 加密 2 倍再回采样，往返偏差 0.0002 m（阈值 0.200 m）
  [PASS] 误差在限 最大误差 0.0972 m（第 161 点，2.65, 11.36），阈值 0.200 m
结论：PASSED

误差直方图（单位 mm）：
[ 0.000, 12.149) ################                         15
[12.149, 24.299) ################                         15
[24.299, 36.448) ################                         15
[36.448, 48.597) #################                        16
[48.597, 60.746) ###################                      18
[60.746, 72.896) #####################                    20
[72.896, 85.045) #########################                24
[85.045, 97.194) ######################################## 38
```

误差不是 0 是刻意的：内置车辆用**定步长积分**开圆弧（`integration_trace`），
参考轨迹用**解析公式**（`mock_trace`），两者的差就是离散化误差，随走过的弧长
单调增长——直方图的右端偏重正是这个原因。

## 四条断言

| 断言 | 判什么 | 失败时的常见原因 |
|---|---|---|
| 采样完整 | 采样点数 ≥ `check/min_samples` | 记录时长太短、采样率太低、外部 `/odom` 频率不稳 |
| 里程合理 | 记录里程与参考里程之差 ≤ `check/length_tolerance` | `/odom` 数据被丢弃、时间戳乱序 |
| 回放一致 | 加密 2 倍再回采样的往返偏差 ≤ `check/max_error` | 重采样插值实现有误 |
| 误差在限 | 与参考轨迹逐点比对的最大误差 ≤ `check/max_error` | 参考半径与实际行驶半径不符 |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `record/seconds` | 8.0 | 记录时长（秒），到点自动结算 |
| `record/rate` | 20.0 | 采样率（Hz），也是内置车辆发 `/odom` 的频率 |
| `mock/enabled` | true | true 时节点自己发 `/odom`，false 时只订阅 |
| `mock/radius` | 6.0 | 内置车辆转弯半径（m） |
| `mock/speed` | 2.0 | 内置车辆线速度（m/s） |
| `check/min_samples` | 100 | 采样点下限 |
| `check/max_error` | 0.20 | 逐点误差上限（m） |
| `check/length_tolerance` | 0.50 | 里程相对偏差（m） |
| `check/reference_radius` | 6.0 | 参考圆弧半径（m） |
| `trace/csv_path` | /tmp/trace.csv | CSV 落盘路径，目录不存在会自动创建 |
| `topic/odom` | /odom | 里程计话题 |
| `topic/replay` | /trace/replay | 回放用的 `nav_msgs/Path` |
| `topic/report` | /trace/report | 报告文本（latched） |

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/odom` | `nav_msgs/Odometry` | 只用 `pose.pose` 的位置与朝向 |
| 发布 | `/trace/replay` | `nav_msgs/Path` | 记录的轨迹，latched，结算后发布一次 |
| 发布 | `/trace/report` | `std_msgs/String` | 报告全文，latched |
| 发布 | `/odom` | `nav_msgs/Odometry` | 仅 `mock/enabled:=true` 时，由内置车辆发出 |

## 离线回归测试

```bash
python3 tests/test_odom_trace_recorder_local.py
```

覆盖采样点校验、CSV 往返、解析真值与积分真值的差异、重采样、比对、四条断言、
直方图、参数读取与工程文件约定；不需要 ROS、不需要安装任何包。

## 与仓库里其他模块的分工

| 模块 | 职责 |
|---|---|
| `twist_to_control` | 把 `/cmd_vel` 换成转角与轮速（控制量换算） |
| `turning_sweep_check` | 转弯时车身占多宽（横向包络） |
| `braking_distance_check` | 从看见到停住要几米（纵向距离） |
| `speed_profile_check` | 这一段路该怎么开（时间-速度曲线） |
| **`odom_trace_recorder`** | **车实际开过的轨迹记下来、存下来、放回去、比出来** |

## 常见问题

**Q：为什么 CSV 写在 `/tmp`？**
`/tmp` 所有用户可写，避免仓库里出现运行产物；要换位置就传 `csv_path:=/home/xxx/trace.csv`。

**Q：`use_mock:=false` 时没有任何输出？**
说明 `/odom` 没数据。先 `rostopic hz /odom` 确认频率，再检查 `topic/odom` 是否改过名。

**Q：Melodic 上打印中文日志报 `UnicodeEncodeError`？**
节点里的 `_log()` 已经把消息统一转成 byte str，这是 Melodic（Python 2.7）上给
rosgraph 打非 ASCII 日志的唯一安全写法。自己改代码时不要直接 `rospy.loginfo(unicode)`。

**Q：能和真正的 CARLA 一起用吗？**
可以，`use_mock:=false` 后把 `topic/odom` 指向 `carla_ros_bridge` 发布的里程计即可；
本包只要求 `/odom` 是 `nav_msgs/Odometry`。

## 参考

* [Carla ROS 里程计与导航](https://openhutb.github.io/doc/carla_ros_navigation/)

## 大模型声明

本模块的代码、测试与文档在编写与整理过程中使用了大模型辅助，提交内容由本人负责。
