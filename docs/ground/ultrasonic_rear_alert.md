# 倒车雷达分级告警

## 这一页解决什么问题

倒车雷达的难点不在「测距」，而在「怎么叫」：

* 障碍物停在 0.5 m 的边界上来回晃，提示音会不会变成一串断续的尖叫？
* 草叶、井盖、坡道造成一帧假读数，会不会直接触发「停止」？
* 真靠近了，从读数到线到真正叫出来，晚了几帧？
* 四路通道里只有一路近，另外三路要不要也参与判定？

这一页给出一套**可回归**的答案：把四路测距的时序喂给一个带迟滞与帧数确认的状态机，输出五级告警，并用四条断言把上面四个问题变成可判定的检查。

## 级别与阈值

| 级别 | 名称 | 进入阈值（默认） | 退出阈值（默认） |
|---|---|---|---|
| 0 | 无 | > 2.00 m | — |
| 1 | 提示 | ≤ 2.00 m | > 2.20 m |
| 2 | 警告 | ≤ 1.50 m | > 1.70 m |
| 3 | 危险 | ≤ 1.00 m | > 1.20 m |
| 4 | 停止 | ≤ 0.50 m | > 0.70 m |

两个关键机制：

1. **迟滞**：退出阈值 = 进入阈值 + `exit_margin_m`。级别越高区间越窄，所以在 0.70~1.20 m 之间晃动时级别会稳定在 3，而不是在 3 和 4 之间跳。
2. **帧数确认**：升级要连续 `confirm_frames` 帧都到线；降级要连续 `release_frames` 帧都退出当前级别的退出阈值。单帧毛刺既不能触发升级，也不能抹掉正在生效的告警。

判定取四路通道里**最近**的一路，并记录是哪一路触发的。

## 四条断言

| 断言 | 判定方式 | 默认结果 |
|---|---|---|
| 分级准确 | 最高级别、末帧级别都等于「无迟滞时的即时级别」 | 4 / 0，通过 |
| 无抖动 | 切换次数 ≤ `max_switches`，且没有方向相反、间隔 < `confirm_frames` 的切换对 | 3 次 / 0 次振荡，通过 |
| 响应及时 | 最高级别的判定延迟 ≤ `confirm_frames` 帧 | 延迟 2 帧（上限 3），通过 |
| 覆盖全部通道 | 四路里进入过告警区间（≤ 2.00 m）的通道数 = 4 | 4 个，通过 |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `alert/enter_info_m` | 2.00 | 提示级进入阈值 |
| `alert/enter_warn_m` | 1.50 | 警告级进入阈值 |
| `alert/enter_danger_m` | 1.00 | 危险级进入阈值 |
| `alert/enter_stop_m` | 0.50 | 停止级进入阈值 |
| `alert/exit_margin_m` | 0.20 | 退出余量 |
| `alert/confirm_frames` | 3 | 升级确认帧数 |
| `alert/release_frames` | 3 | 降级确认帧数 |
| `alert/min_range_m` | 0.15 | 量程下限 |
| `alert/max_range_m` | 3.00 | 量程上限 |
| `alert/max_switches` | 6 | 切换次数上限 |
| `rate/read_hz` | 10.0 | 读数节拍 |
| `rate/publish_hz` | 10.0 | 发布节拍 |
| `topic/level` | `/rear_alert/level` | 当前级别（Int32） |
| `topic/report` | `/rear_alert/report` | 文字报告（String） |
| `topic/levels` | `/rear_alert/levels` | 级别条带（String） |
| `topic/radar` | `/rear_alert/radar` | 四通道雷达条（String） |

## 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/rear_alert/level` | `std_msgs/Int32` | 发布（latched） | 当前级别 0~4 |
| `/rear_alert/report` | `std_msgs/String` | 发布（latched） | 完整文字报告 |
| `/rear_alert/levels` | `std_msgs/String` | 发布（latched） | 级别随时间的字符条带 |
| `/rear_alert/radar` | `std_msgs/String` | 发布（latched） | 四通道距离条 |

## 运行

```bash
roslaunch ultrasonic_rear_alert main.launch
```

默认参数下的真实输出（节点跑完 23 帧后自己退出）：

```text
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

## 两个反例

**关掉帧数确认**（`confirm_frames:=1 release_frames:=1`）：切换次数从 3 涨到 8，超过上限，`[FAIL] 无抖动`：

```text
帧数=23 时长=2.200 最高级别=4 末帧级别=0 切换=8 抖动=0
[FAIL] 无抖动 切换 8 次（上限 6），1 帧内反复切换 0 次
结论=FAILED
```

**更迟钝**（`confirm_frames:=5`）：判定推迟到第 13 帧，延迟 4 帧（上限 5）仍然通过，但能明显看出「慢半拍」：

```text
帧数=23 时长=2.200 最高级别=4 末帧级别=0 切换=3 抖动=0
[PASS] 响应及时 最高级别 4 在第 13 帧判定（原始第 9 帧），延迟 4 帧（上限 5）
结论=PASSED
```

## 离线测试

```bash
python3 tests/test_ultrasonic_rear_alert_local.py
```

只用标准库，不需要 ROS、不需要 CARLA、不需要显示器；同时兼容 Python 2.7 与 Python 3.x。

## 与已有模块的关系

| 模块 | 回答的问题 | 与本页的边界 |
|---|---|---|
| `reverse_parking_planner` | 倒车入库该走什么轨迹 | 只管几何，不管告警 |
| `braking_distance_check` | 从看见到停住要几米 | 只管纵向距离 |
| `turning_sweep_check` | 转弯时车身扫过多宽 | 只管横向包络 |
| `twist_to_control` | 速度指令 → 转角 | 只管执行量换算 |
| **`ultrasonic_rear_alert`** | **距离在告警边界上来回时该怎么叫** | 只管分级、迟滞、帧数确认 |

## 常见问题

**为什么障碍物退到 1.9 m 还显示「提示」？**
提示级的退出阈值是 2.20 m，1.9 m 还没退出去。这是迟滞的设计，避免边界抖动。

**一帧读到 2.90 m 为什么没把级别清掉？**
降级要连续 `release_frames` 帧都退出去，单帧跳变会被吃掉。

**为什么「覆盖全部通道」会失败？**
它检查的是场景覆盖：读数里只有一路进过告警区间时判失败，提示换一个有代表性的场景。

**Melodic 上日志报 `UnicodeEncodeError`？**
Python 2.7 下 rospy 的日志格式化对非 ASCII unicode 会炸，节点里的日志统一走 `_log()` 转成 byte str。

## 本页内容由大模型辅助整理

本页的代码、测试与说明由作者与大模型协作完成，提交内容由作者负责。

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
