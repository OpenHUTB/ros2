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

`run_sweep.sh` 用相对路径写 `results/`，**需要在包目录下执行**，否则结果会落到
当前工作目录：

```bash
colcon build --packages-select joint_qos_latency_eval
source install/setup.bash
cd src/humanoid/joint_qos_latency_eval

# 单个场景（两端分进程，时延才可信）
ros2 run joint_qos_latency_eval joint_qos_sub --ros-args -p qos_profile:=best_effort_depth1 -p csv_path:=results/be1.csv &
ros2 run joint_qos_latency_eval joint_qos_pub --ros-args -p qos_profile:=best_effort_depth1 -p rate_hz:=200 -p duration:=10

# 批量场景：每个场景重复 REPEATS 次，按场景取中位数
DURATION=10 RATE=200 REPEATS=3 bash run_sweep.sh

# 也可以只跑指定场景
PROFILES="reliable_depth10 best_effort_depth1" bash run_sweep.sh
```

单次结果对系统负载很敏感（冷启动未预热时曾丢 24%），因此脚本默认 `REPEATS=3`，
CSV 命名 `results/<profile>.r<N>.csv`，最后用 `qos_summarize --median` 汇总中位数。
也可以手动汇总：

```bash
ros2 run joint_qos_latency_eval qos_summarize results/be1.csv          # 每次运行一行
ros2 run joint_qos_latency_eval qos_summarize --median results/*.csv   # 按 profile 取中位数
# 追加 --expected N 用应发总数作分母（run_sweep.sh 已自动带上）
```

用 launch 一次起两端也可以（发布端会先等订阅者匹配，最多 `start_timeout` 秒，
避免把 DDS 发现的几十毫秒算成丢包；`csv_path` 可省略，默认 `results/launch.csv`）：

```bash
ros2 launch joint_qos_latency_eval main.launch.py \
  pub_profile:=best_effort_depth1 sub_profile:=best_effort_depth1 duration:=10
```

数值参数 `rate_hz` / `duration` 传整数或小数均可（`duration:=10` 与
`duration:=10.0` 等价），节点内部统一转成 float。

运行效果图见 `docs/img/humanoid/joint_qos_latency_eval/effect.png`，展示 `run_sweep.sh`
跑完打印的对比表。

## 指标定义

| 指标 | 含义 |
|------|------|
| `count` | 收到条数 |
| `missing` | 未到达的条数：`expected_msgs - 去重后的序号数`，乱序重发不会重复计数 |
| `drop%` | `missing / expected_msgs`。`expected_msgs = rate_hz × duration` 为应发总数（`run_sweep.sh` / launch 自动传入），因此末尾丢包与发布端欠触发都会计入；未传 `expected_msgs` 时分母退化为 `max(seq) + 1` |
| `p50 / p95 / p99 / max` | 单程时延分位数与最大值（ms） |
| `jitter` | 相邻到达间隔的样本标准差（ms），衡量周期稳定性 |
| `ooo` | 到达顺序与序号顺序相反的次数 |
| `gap`（CSV 列） | 与上一条消息的序号差，仅供排查；统计层按 `seq` 去重，不依赖该列 |

`count < expected_msgs` 时分两种原因：发布定时器欠触发（发布端退出日志里的
`published N (target X Hz, actual Y Hz)` 可核对）与传输丢失；前者是调度问题，
后者才是 QoS 问题。

## 实测结果（Ubuntu 20.04 / ROS 2 Foxy 单机回环，200 Hz × 10 s，3 次取中位数）

| 场景 | count | missing | drop% | p50 (ms) | p95 (ms) | p99 (ms) | max (ms) | jitter (ms) |
|------|-------|---------|-------|----------|----------|----------|----------|-------------|
| best_effort_depth10 | 1993 | 7 | 0.4% | 0.756 | 1.162 | 1.862 | 6.355 | 0.676 |
| best_effort_depth1 | 1990 | 10 | 0.5% | 0.809 | 1.202 | 1.792 | 3.880 | 0.585 |
| reliable_depth10 | 1993 | 7 | 0.4% | 0.782 | 1.185 | 1.665 | 4.948 | 0.525 |
| reliable_depth1 | 1993 | 7 | 0.4% | 0.837 | 1.307 | 1.866 | 3.748 | 0.594 |
| reliable_transient_local | 1991 | 9 | 0.4% | 0.816 | 1.291 | 1.760 | 3.890 | 0.595 |

结论：

- **观测到 BEST_EFFORT 的 p50 略低于 RELIABLE**：同 depth 成对看，分别低
  0.026 ms（depth=10）与 0.028 ms（depth=1），但差异与噪声同量级，
  不足以归因于可靠传输的确认开销。
- **队列深度的影响与可靠性差异同量级**：`depth=1` 的 p50 比 `depth=10` 高
  0.053 ms（BEST_EFFORT）与 0.055 ms（RELIABLE），都在 0.05 ms 的噪声边界内。
- **`reliable_transient_local` 的 p50（0.816 ms）介于 `reliable_depth10` 与
  `reliable_depth1` 之间**，本实验分辨不出持久性本身的独立影响。
- **missing 主要是发布定时器欠触发，不是传输丢失。** 15 次运行里 11 次
  `published == received`（真实丢失 0 条），其余 4 次各差 1~3 条；`expected` 分母
  （2000）与 `count`（约 1992）的差来自定时器实际只跑到 198.9~199.4 Hz
  （发布端日志的 `actual Hz` 可核对）。注意此前一轮 `best_effort_depth10`
  曾单次真实丢 301 条（15.3%）而相邻两次为 0——丢包率必须重复测量取中位数。
- p99 中位数 1.7~1.9 ms，max 中位数 3.7~6.4 ms；单机回环下 200 Hz
  有充足余量。

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

14 个用例（`protocol` 4 个 + `stats` 10 个），不启动 ROS；话题链路的验证属于运行期测试。
