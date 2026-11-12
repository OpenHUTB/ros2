# ultrasonic_rear_alert —— 倒车雷达分级告警

倒车时最怕两件事：**该响的时候不响**，以及**旁边有根草就一路狂叫**。这个包把「四路后向测距 → 五级告警」做成一个可回归的判定器，把两条最容易出问题的地方显式建模：

1. **迟滞**：进入阈值和退出阈值分开（退出 = 进入 + 余量），障碍物停在 0.5 m 边界上来回晃时不会反复播报；
2. **帧数确认**：升级要连续 `confirm_frames` 帧都到线，降级要连续 `release_frames` 帧都退出去，一帧的毛刺（水面、草丛、坡道）不会触发告警。

判定全部用**纯标准库的几何与状态机**实现，不依赖 ROS、不依赖 CARLA、不需要显卡，一条命令就能跑完 23 帧脚本读数并打印报告。

## 目录结构

```
ultrasonic_rear_alert/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── ultrasonic_rear_alert.yaml     # 16 项参数默认值
├── launch/
│   └── main.launch                    # 16 个 <arg>，逐个带 doc
└── scripts/
    ├── main.py                        # 入口（仓库约定：入口以 main. 开头）
    ├── alert_node.py                  # ROS 收发放这里
    └── alert_core.py                  # 判定器、报告、字符图（不 import rospy）
```

## 级别与阈值

| 级别 | 名称 | 进入阈值（默认） | 退出阈值（默认） | 典型含义 |
|---|---|---|---|---|
| 0 | 无 | > 2.00 m | — | 后方空旷 |
| 1 | 提示 | ≤ 2.00 m | > 2.20 m | 有东西，注意一下 |
| 2 | 警告 | ≤ 1.50 m | > 1.70 m | 正在靠近 |
| 3 | 危险 | ≤ 1.00 m | > 1.20 m | 快贴上了 |
| 4 | 停止 | ≤ 0.50 m | > 0.70 m | 不要再倒了 |

四路通道（左后 / 左中 / 右中 / 右后）中取**最近**的那一路作为整车判定，并记录是哪一路触发的。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `alert/enter_info_m` | 2.00 | 提示级进入阈值 |
| `alert/enter_warn_m` | 1.50 | 警告级进入阈值 |
| `alert/enter_danger_m` | 1.00 | 危险级进入阈值 |
| `alert/enter_stop_m` | 0.50 | 停止级进入阈值（必须从大到小） |
| `alert/exit_margin_m` | 0.20 | 退出阈值相对进入阈值的余量 |
| `alert/confirm_frames` | 3 | 升级需要的连续帧数（10 Hz 下 0.3 s） |
| `alert/release_frames` | 3 | 降级需要的连续帧数 |
| `alert/min_range_m` | 0.15 | 量程下限，更小视为贴住传感器（按无回波处理） |
| `alert/max_range_m` | 3.00 | 量程上限，更大视为无回波 |
| `alert/max_switches` | 6 | 一次演示允许的切换次数上限 |
| `rate/read_hz` | 10.0 | 读数节拍 |
| `rate/publish_hz` | 10.0 | 发布节拍 |
| `topic/level` | `/rear_alert/level` | 当前级别（Int32） |
| `topic/report` | `/rear_alert/report` | 文字报告（String，latched） |
| `topic/levels` | `/rear_alert/levels` | 级别条带（String，latched） |
| `topic/radar` | `/rear_alert/radar` | 四通道雷达条（String，latched） |

## 运行

```bash
catkin_make
source devel/setup.bash
roslaunch ultrasonic_rear_alert main.launch
```

默认参数下的输出：

```
倒车雷达分级告警（四个后向通道，带迟滞）
进入阈值：2.00 / 1.50 / 1.00 / 0.50 m，退出余量 0.20 m，确认 3 帧 / 释放 3 帧
------------------------------------------------------------
帧数：23，时长：2.20 s
最高级别：4（停止），末帧级别：0（无）
级别切换：3 次
  t=1.10 s -> 停止（通道 左后，0.36 m）
  t=1.80 s -> 提示（通道 左后，1.90 m）
  t=2.10 s -> 无（通道 左后，3.00 m）
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 分级准确 最高级别 4（期望 4），末帧级别 0（期望 0）
  [PASS] 无抖动 切换 3 次（上限 6），3 帧内反复切换 0 次
  [PASS] 响应及时 最高级别 4 在第 11 帧判定（原始第 9 帧），延迟 2 帧（上限 3）
  [PASS] 覆盖全部通道 四个通道里进入过告警区间的有 4 个
结论：PASSED
```

改参数看差别：

```bash
# 关掉帧数确认：毛刺会直接叫起来，切换次数从 3 涨到 8
roslaunch ultrasonic_rear_alert main.launch confirm_frames:=1 release_frames:=1

# 更迟钝：确认 5 帧，判定要等到第 13 帧（延迟 4 帧）
roslaunch ultrasonic_rear_alert main.launch confirm_frames:=5

# 把余量清零：退出立刻生效，级别会一路 4→2→0 地掉
roslaunch ultrasonic_rear_alert main.launch exit_margin_m:=0.0
```

## 四条断言

| 断言 | 判定方式 |
|---|---|
| 分级准确 | 轨迹里的最高级别、末帧级别，都要等于「无迟滞时的即时级别」 |
| 无抖动 | 级别切换次数 ≤ `max_switches`，且不存在方向相反、间隔小于 `confirm_frames` 的切换对 |
| 响应及时 | 最高级别从「原始读数到线」到「判定生效」的延迟 ≤ `confirm_frames` 帧 |
| 覆盖全部通道 | 四路读数里，进入过告警区间（≤ 2.00 m）的通道数 = 4 |

## 离线测试

```bash
python3 tests/test_ultrasonic_rear_alert_local.py
```

不需要 ROS、不需要 CARLA、不需要显示器。

## 与其他模块的分工

| 模块 | 回答的问题 |
|---|---|
| `twist_to_control` | 速度指令 → 转角与轮速 |
| `reverse_parking_planner` | 倒车入库该走什么轨迹 |
| `turning_sweep_check` | 转弯时车身扫过多宽 |
| `braking_distance_check` | 从看见到停住要几米 |
| `scene_object_layouter` | 场景里的物体该摆在哪 |
| `odom_trace_recorder` | 走过的轨迹记下来怎么比对 |
| **`ultrasonic_rear_alert`** | **距离在告警边界上来回时，该响哪一级、什么时候响** |

## 常见问题

**为什么障碍物退到 1.9 m 了还显示「提示」？**
这是迟滞的设计：提示级的退出阈值是 2.20 m，1.9 m 还没退出去，所以保持「提示」而不是立刻清空。要想更灵敏就把 `exit_margin_m` 调小。

**为什么一帧读到 2.90 m 也没把级别清掉？**
降级要连续 `release_frames` 帧都退出去，单帧跳变会被吃掉。这正是防止草丛、井盖反复触发告警的关键。

**为什么「覆盖全部通道」会失败？**
这条断言检查的是**场景覆盖**：如果脚本读数里只有一路通道进过告警区间（比如一直只有左后最近），它就判失败，提醒你换一个更有代表性的场景。

**Melodic 上日志乱码或崩掉？**
节点里的日志统一走 `_log()` 把消息转成 byte str，因为 Python 2.7 下 rospy 的日志格式化对非 ASCII 的 unicode 会报 `UnicodeEncodeError`。

## 服务调用：`rosservice call /rear_alert/simulate`

除了四路 latched 话题，节点还把「重算一次并给出报告」暴露成一个
`std_srvs/Trigger` 服务。它不依赖话题的历史值，调用一次就重跑整个判定过程，
把「重算后的四条断言是否全过」放进 `success`、把一行 **ASCII 摘要**放进 `message`：

```text
$ rosservice call /rear_alert/simulate
success: True
message: max_level=4 final_level=0 switches=3 asserts=4/4 passed=True
```

`message` 只放 ASCII 摘要（`max_level` 最高级别、`final_level` 末帧级别、
`switches` 切换次数、`asserts` 通过条数、`passed` 总结论）。原因很实际：
`rosservice call` 会把中文转义成 `\u5012\u8F66` 这种形式，截图里根本没法读；
完整的中文报告仍然在终端日志与 `/rear_alert/report` 话题里。

参数改动后不必重启节点，改完 `rosparam set` 再调一次服务即可拿到新报告；
CI 或其它脚本也能以 `success` 为准做断言，不用解析文本。
把 `alert/confirm_frames` 设成 1 再调用，可以现场看到 `success: False` 与
`[FAIL] 无抖动`。
