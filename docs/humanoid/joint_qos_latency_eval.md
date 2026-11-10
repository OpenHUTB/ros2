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
3. `joint_sub.py` 在回调里取到达时刻，算出单程时延、相邻到达间隔、序号缺口，
   写 CSV：`seq,latency_ms,gap,received_ns`。
4. `stats.py` 提供纯函数统计（分位数、抖动标准差、丢包率），与 ROS 解耦，可单测。
5. `run_sweep.sh` 逐个场景起进程（不共享内存，避免交叉干扰），最后 `qos_summarize`
   汇总成一张对比表。

## 受测 QoS 组合

| 名称 | reliability | history | depth |
|------|-------------|---------|-------|
| `reliable_depth10` | RELIABLE | KEEP_LAST | 10 |
| `reliable_depth1` | RELIABLE | KEEP_LAST | 1 |
| `best_effort_depth10` | BEST_EFFORT | KEEP_LAST | 10 |
| `best_effort_depth1` | BEST_EFFORT | KEEP_LAST | 1 |

## 运行环境与命令

Ubuntu 20.04 + ROS 2 Foxy，Python 3.8，无需第三方依赖。

```bash
colcon build --packages-select joint_qos_latency_eval
source install/setup.bash
DURATION=10 RATE=200 bash run_sweep.sh
```

单场景也可以用 launch 一次起两端：

```bash
ros2 launch joint_qos_latency_eval main.launch.py \
  pub_profile:=best_effort_depth1 sub_profile:=best_effort_depth1 duration:=10
```

## 运行效果

![编译与测试](../img/humanoid/joint_qos_latency_eval/compile_test.png)

![四场景对比表](../img/humanoid/joint_qos_latency_eval/effect.png)

## 结果结论

| 场景 | count | missing | drop% | p50 (ms) | p99 (ms) |
|------|-------|---------|-------|----------|----------|
| best_effort_depth10 | 1993 | 0 | 0.0% | 0.693 | 1.434 |
| best_effort_depth1 | 1927 | 67 | 3.4% | 0.793 | 1.452 |
| reliable_depth10 | 1992 | 0 | 0.0% | 0.865 | 1.761 |
| reliable_depth1 | 1931 | 63 | 3.2% | 0.777 | 1.572 |

1. **丢包的主因是队列深度，不是可靠性策略。** `depth=10` 两组零丢包，`depth=1`
   两组稳定丢 3.2%~3.4%，且 RELIABLE 与 BEST_EFFORT 的丢包率几乎相同。
2. **BEST_EFFORT 的时延略低**，p50 相差约 0.1 ms，量级符合可靠传输多出的确认开销。
3. 结果对系统负载敏感：负载轻时曾出现四组全部零丢包，冷启动未预热时 `depth=10`
   甚至丢过 24%。因此这类测量必须重复多次，并丢弃第一次的数据。

## 测试

```bash
colcon test --packages-select joint_qos_latency_eval
colcon test-result --verbose
```

11 个用例（载荷编解码 + 统计函数），不依赖 ROS，`0 failures`。
