# 关节指令的 QoS 与时延影响评估

用 ROS 2 话题把六个关节的目标角按固定频率发布出去，在同一台机器上更换不同的
QoS 组合（可靠性 / 队列深度 / 持久性），记录单程时延、抖动、丢包与乱序，
最终输出可直接对比的表格。目标是回答一个问题：**在关节指令这种高频、小载荷的
话题上，QoS 选择到底带来多少时延与丢失的差异。**

## 目录结构

```
src/humanoid/joint_qos_latency_eval/     # 模块目录（仓库内位置）
├── joint_qos_latency_eval/              # ROS 2 包源码
│   ├── protocol.py                      # 载荷编解码（序号 + 时间戳 + 6 关节角）
│   ├── qos_profiles.py                  # 受测 QoS 组合
│   ├── stats.py                         # 分位数、抖动、丢包率等纯函数统计
│   ├── joint_pub.py                     # 发布端节点
│   ├── joint_sub.py                     # 订阅端节点（落 CSV）
│   └── summarize.py                     # 汇总 CSV 输出对比表
├── launch/main.launch.py                # 一次起发布端 + 订阅端
├── test/                                # 不依赖 ROS 的纯单元测试
├── resource/joint_qos_latency_eval
├── package.xml  setup.py  setup.cfg
├── run_sweep.sh                         # 批量跑多个 QoS 场景
└── README.md
```

| 内容 | 说明 |
|------|------|
| 入口 | `launch/main.launch.py`；另有 `joint_qos_pub` / `joint_qos_sub` / `qos_summarize` 三个可执行 |
| 启动方式 | 必须支持 launch：`ros2 launch joint_qos_latency_eval main.launch.py` |
| 效果图 | `docs/img/humanoid/joint_qos_latency_eval/effect.png`（png < 1 MB） |


## 载荷契约

`std_msgs/Float64MultiArray.data`，长度 9：

```
[seq, stamp_sec, stamp_nanosec, q1, q2, q3, q4, q5, q6]
```

- `seq`：递增序号。ROS 2 消息本体没有序号字段，丢包/乱序必须靠它自己带。
- `stamp_*`：发送时刻，取自同一 ROS 时钟，拆成秒 + 纳秒两段以免浮点丢精度。
- `q1..q6`：左髋、左膝、左踝、右髋、右膝、右踝（rad）。

## QoS 场景

| 名称 | reliability | history | depth | durability |
|------|-------------|---------|-------|------------|
| `reliable_depth10` | RELIABLE | KEEP_LAST | 10 | VOLATILE |
| `reliable_depth1` | RELIABLE | KEEP_LAST | 1 | VOLATILE |
| `best_effort_depth10` | BEST_EFFORT | KEEP_LAST | 10 | VOLATILE |
| `best_effort_depth1` | BEST_EFFORT | KEEP_LAST | 1 | VOLATILE |
| `reliable_transient_local` | RELIABLE | KEEP_LAST | 1 | TRANSIENT_LOCAL |

注意兼容性：`BEST_EFFORT` 发布端 + `RELIABLE` 订阅端属于不兼容组合，收不到数据；
反向组合（`RELIABLE` 发布 + `BEST_EFFORT` 订阅）是兼容的，订阅端会降级。

## 运行

```bash
colcon build --packages-select joint_qos_latency_eval
source install/setup.bash

# 单个场景（两端分进程，时延才可信）
ros2 run joint_qos_latency_eval joint_qos_sub --ros-args -p qos_profile:=best_effort_depth1 -p csv_path:=results/be1.csv &
ros2 run joint_qos_latency_eval joint_qos_pub --ros-args -p qos_profile:=best_effort_depth1 -p rate_hz:=200 -p duration:=10

# 批量场景
DURATION=10 RATE=200 bash run_sweep.sh

# 或者用 launch 一次起两端
ros2 launch joint_qos_latency_eval main.launch.py \
  pub_profile:=best_effort_depth1 sub_profile:=best_effort_depth1 duration:=10 \
  csv_path:=results/be1.csv
```

运行效果图见 `docs/img/humanoid/joint_qos_latency_eval/effect.png`，展示 `run_sweep.sh` 跑完打印的对比表。

## 指标定义

| 指标 | 含义 |
|------|------|
| `count` | 收到条数 |
| `missing` | 由序号间隔推算的丢失条数 |
| `drop%` | `missing / (count + missing)` |
| `p50 / p95 / p99 / max` | 单程时延分位数与最大值（ms） |
| `jitter` | 相邻到达间隔的样本标准差（ms），衡量周期稳定性 |
| `ooo` | 到达顺序与序号顺序相反的次数 |

## 实测结果（Ubuntu 20.04 / ROS 2 Foxy 单机回环，200 Hz × 10 s）

| 场景 | count | missing | drop% | p50 (ms) | p95 (ms) | p99 (ms) | max (ms) | jitter (ms) |
|------|-------|---------|-------|----------|----------|----------|----------|-------------|
| best_effort_depth10 | 1993 | 0 | 0.0% | 0.693 | 1.012 | 1.434 | 3.978 | 0.526 |
| best_effort_depth1 | 1927 | 67 | 3.4% | 0.793 | 1.124 | 1.452 | 2.570 | 0.593 |
| reliable_depth10 | 1992 | 0 | 0.0% | 0.865 | 1.269 | 1.761 | 3.135 | 0.605 |
| reliable_depth1 | 1931 | 63 | 3.2% | 0.777 | 1.218 | 1.572 | 3.135 | 0.549 |

结论：

- **队列深度是丢包的主因，可靠性策略不是。** `depth=10` 的两个场景 missing 为 0，
  `depth=1` 的两个场景分别丢 67 条（3.4%）和 63 条（3.2%），且 RELIABLE 与
  BEST_EFFORT 的丢包率几乎一致。200 Hz 下接收队列深度 1 没有吸收调度抖动的余量，
  来不及取走的消息直接被覆盖。
- **BEST_EFFORT 的时延略低于 RELIABLE**（p50 0.693~0.793 vs 0.777~0.865 ms），
  差异约 0.1 ms，量级符合可靠传输多出的确认开销。
- 四种组合的 p99 都在 1.8 ms 以内、jitter 在 0.5~0.6 ms，实时性整体可控。
- **结果对系统负载敏感，必须重复测量。** 同一套代码在负载较轻时曾出现四组全部
  missing=0，负载升高后 `depth=1` 稳定丢 3% 左右；另有一次冷启动未预热时
  `depth=10` 两个场景各丢约 470~490 条（24%）。所以结论应取多次运行的中位数，
  第一次运行的数据建议丢弃。

## 已知限制

- 时延是同机、同 ROS 时钟下的**单程**测量；跨机需要额外的时钟同步，结论不通用。
- 所有节点跑在同一台虚拟机上，宿主机负载、虚拟网卡、DDS 实现的默认配置都会影响结果，
  `reliable_depth1` 与 `best_effort_depth1` 的差异在本地回环上可能不明显。
- 未施加 CPU 干扰或丢包注入；要做压力对比需要额外的负载注入手段。

## 测试

```bash
colcon test --packages-select joint_qos_latency_eval
colcon test-result --verbose
```

单元测试只覆盖 `protocol` 与 `stats`，不启动 ROS；话题链路的验证属于运行期测试。
