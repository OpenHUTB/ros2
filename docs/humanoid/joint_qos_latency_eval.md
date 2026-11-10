# 关节指令的 QoS 与时延影响评估

## 修改概述 / Overview

- 中文：新增 `joint_qos_latency_eval` 模块，用 ROS 2 话题以固定频率发布 6 个关节的
  目标角，在同一台机器上切换可靠性策略与队列深度，量化单程时延、抖动、丢包与乱序，
  输出可直接对比的表格。
- English: A QoS benchmark for high-frequency joint command topics. It publishes
  6-joint targets at a fixed rate, swaps the reliability policy and history depth,
  and measures one-way latency, jitter, packet loss and reordering.

安装、启动、参数与话题接口见
[模块 README](../../src/humanoid/joint_qos_latency_eval/README.md)。

## 为什么要做这个评估

关节指令是典型的高频、小载荷话题：200 Hz 意味着每 5 ms 就有一条消息，单条载荷
只有 72 字节。这类话题的瓶颈通常不在带宽，而在**接收队列能否及时取走**。QoS 配置
一旦选错，表现为偶发丢包和抖动，而且很难在日常调试中发现。

## 测量方法

1. `protocol.py` 把一条指令编码成 `Float64MultiArray`（长度 9）：
   `[seq, stamp_sec, stamp_nanosec, q1..q6]`。ROS 2 消息自身没有序号，丢包与乱序
   只能靠自带序号判断；时间戳拆成秒 + 纳秒两段，避免浮点秒在 1e9 量级丢精度。
2. `joint_pub.py` 用定时器按 `rate_hz` 发布，同时记录发送时刻（同一 ROS 时钟）。
   启动时先等订阅者匹配（`start_timeout`，默认 5 s）再开定时器，否则 launch 同时起
   两端时，DDS 发现期的几十毫秒会被如实计入丢包，结果与 `run_sweep.sh` 不可比。
3. `joint_sub.py` 在回调里取到达时刻，算出单程时延、相邻到达间隔、序号缺口，
   写 CSV：`seq,stamp_ns,recv_ns,latency_ms,gap`。其中 `gap` 是与上一条消息的序号差，
   仅供排查；统计层按 `seq` 去重计算丢包，不依赖这一列。
4. `stats.py` 提供纯函数统计（分位数、抖动标准差、丢包率），与 ROS 解耦，可单测。
   丢包按 `seq` 去重计算，乱序到达不会重复计数；分母为应发总数
   `expected_msgs = rate_hz × duration`（由 `run_sweep.sh` / launch 传入），
   因此末尾丢包与发布端欠触发都会被计入，而不只是序号区间内的缺口。
   发布端退出时打印 `published N (actual X Hz)`，可与 `count` 对照区分
   “定时器欠触发”与“传输丢失”。
5. `run_sweep.sh` 逐个场景起进程（不共享内存，避免交叉干扰），每个场景重复
   `REPEATS` 次（默认 3），最后 `qos_summarize --median` 按场景取中位数汇总成一张
   对比表。单次结果对系统负载敏感，中位数比单次值可靠。

## 受测 QoS 组合

| 名称 | reliability | history | depth | durability |
|------|-------------|---------|-------|------------|
| `reliable_depth10` | RELIABLE | KEEP_LAST | 10 | VOLATILE |
| `reliable_depth1` | RELIABLE | KEEP_LAST | 1 | VOLATILE |
| `best_effort_depth10` | BEST_EFFORT | KEEP_LAST | 10 | VOLATILE |
| `best_effort_depth1` | BEST_EFFORT | KEEP_LAST | 1 | VOLATILE |
| `reliable_transient_local` | RELIABLE | KEEP_LAST | 1 | TRANSIENT_LOCAL |

注意兼容性：`BEST_EFFORT` 发布端 + `RELIABLE` 订阅端属于不兼容组合，收不到数据；
反向组合是兼容的，订阅端会降级。

## 运行环境与命令

Ubuntu 20.04 + ROS 2 Foxy，Python 3.8，无需第三方依赖。脚本需在包目录下执行，
否则 `results/` 会落到当前工作目录：

```
colcon build --packages-select joint_qos_latency_eval
source install/setup.bash
cd src/humanoid/joint_qos_latency_eval
DURATION=10 RATE=200 REPEATS=3 bash run_sweep.sh
```

单场景也可以用 launch 一次起两端（CSV 默认写到 `results/launch.csv`）：

```
ros2 launch joint_qos_latency_eval main.launch.py \
  pub_profile:=best_effort_depth1 sub_profile:=best_effort_depth1 duration:=10
```

数值参数 `rate_hz` / `duration` 传整数或小数都可以（如 `duration:=10` 与
`duration:=10.0` 等价），节点内部会统一转成 float。

## 运行效果

![编译与测试](../img/humanoid/joint_qos_latency_eval/compile_test.png)

![启动冒烟](../img/humanoid/joint_qos_latency_eval/launch_demo.png)

![五场景对比表](../img/humanoid/joint_qos_latency_eval/effect.png)

## 结果结论

下表为 `REPEATS=3` 的中位数（Ubuntu 20.04 / ROS 2 Foxy 单机回环，200 Hz × 10 s）：

| 场景 | count | missing | drop% | p50 (ms) | p95 (ms) | p99 (ms) | max (ms) | jitter (ms) |
|------|-------|---------|-------|----------|----------|----------|----------|-------------|
| best_effort_depth10 | 1993 | 7 | 0.4% | 0.756 | 1.162 | 1.862 | 6.355 | 0.676 |
| best_effort_depth1 | 1990 | 10 | 0.5% | 0.809 | 1.202 | 1.792 | 3.880 | 0.585 |
| reliable_depth10 | 1993 | 7 | 0.4% | 0.782 | 1.185 | 1.665 | 4.948 | 0.525 |
| reliable_depth1 | 1993 | 7 | 0.4% | 0.837 | 1.307 | 1.866 | 3.748 | 0.594 |
| reliable_transient_local | 1991 | 9 | 0.4% | 0.816 | 1.291 | 1.760 | 3.890 | 0.595 |

1. **BEST_EFFORT 的时延略低于 RELIABLE。** 同 depth 成对看，p50 低 0.026 ms
   （depth=10）与 0.028 ms（depth=1），符合可靠传输多出的确认开销。
2. **队列深度的影响与可靠性差异同量级**：`depth=1` 比 `depth=10` 的 p50 高
   0.053~0.055 ms，两个维度的差异都在 0.05 ms 的噪声边界内。
3. **`reliable_transient_local` 的 p50（0.816 ms）介于 `reliable_depth10` 与
   `reliable_depth1` 之间**，本实验分辨不出持久性本身的独立影响；该组合主要
   用于验证 TRANSIENT_LOCAL 能被正常收发。
4. **missing 主要是发布定时器欠触发，不是传输丢失**：15 次运行里 11 次
   `published == received`（真实丢失 0 条），其余 4 次各差 1~3 条；`expected=2000`
   与 `count≈1992` 的差来自定时器实际只跑到 198.9~199.4 Hz（发布端日志的
   `actual Hz` 可核对）。
5. **单次结果差异很大，必须重复测量。** 本轮 15 次全部低丢包，但前一轮
   `best_effort_depth10` 曾单次真实丢 301 条（15.3%）而相邻两次为 0；p99
   中位数在 1.7~1.9 ms。因此 `run_sweep.sh` 默认 `REPEATS=3` 并按场景取中位数。

## 测试

```
colcon test --packages-select joint_qos_latency_eval
colcon test-result --verbose
```

14 个用例（载荷编解码 4 个 + 统计函数 10 个），不依赖 ROS，`0 failures`。
