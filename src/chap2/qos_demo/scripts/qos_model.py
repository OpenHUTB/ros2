"""ROS 2 QoS 服务质量策略的最小可运行模型（仅依赖标准库）。

真实 ROS 2 的 QoS 由 DDS 实现（Fast DDS / Cyclone DDS）负责，涉及可靠传输、
历史缓存、持久化等机制。本模块用一个离散时间队列模型把这三件事的**行为**
复现出来，便于在没有 ROS 环境时观察策略差异：

- Reliability（可靠性）
  * ``best_effort``：尽力而为。队列（History 缓存）满了之后新样本直接丢弃，
    这也是默认传感器话题（如 ``/scan``、``/image``）用 best_effort 的原因。
  * ``reliable``：可靠传输。队列满了不是丢样本，而是**发布端阻塞/重传**，
    表现为积压持续增长、端到端延迟线性上升（这就是"可靠但不实时"）。

- History / Depth（历史深度）
  * ``keep_last`` + ``depth=N``：只保留最近 N 个样本，更老的被逐出（满时
    逐出最旧、保留最新，即"旧不如新"的传感器语义）。
  * 注意：``reliable`` 分支里的发布端等待队列是本模型的简化（无界 FIFO），
    真实 DDS 的发送端 History 同样受 depth 限制并伴随资源限制与阻塞行为，
    见文档中的模型边界说明。

- Durability（持久性）
  * ``volatile``：晚加入的订阅者拿不到历史消息。
  * ``transient_local``：发布端保留"最后一条"（latched），晚加入者立刻收到。

模型参数都是可解释的物理量：发布频率、订阅端单条回调耗时、缓存深度。
"""

import math
from collections import deque

RELIABLE = "reliable"
BEST_EFFORT = "best_effort"
VOLATILE = "volatile"
TRANSIENT_LOCAL = "transient_local"


class QoSProfile:
    """对应 rclcpp / rclpy 的 QoSProfile。"""

    def __init__(self, reliability=BEST_EFFORT, depth=1,
                 durability=VOLATILE, name=None):
        assert reliability in (RELIABLE, BEST_EFFORT), reliability
        assert durability in (VOLATILE, TRANSIENT_LOCAL), durability
        assert depth >= 1, depth
        self.reliability = reliability
        self.depth = int(depth)
        self.durability = durability
        self.name = name or "%s/depth=%d" % (reliability, depth)

    def __repr__(self):
        return ("QoSProfile(reliability=%s, depth=%d, durability=%s)"
                % (self.reliability, self.depth, self.durability))


class LinkStats:
    """一次仿真的统计结果。"""

    def __init__(self, ts, backlog, drops_cum, latencies, published, delivered,
                 delivered_seqs=None):
        self.ts = ts
        self.backlog = backlog          # 未处理样本数（队列 + 发布端等待队列）
        self.drops_cum = drops_cum      # 累计丢弃样本数
        self.latencies = latencies      # 每条成功处理的样本的端到端延迟 (s)
        self.published = published
        self.delivered = delivered
        self.delivered_seqs = delivered_seqs or []   # 实际送达回调的样本序号

    @property
    def drop_rate(self):
        return self.drops_cum[-1] / float(self.published) if self.published else 0.0

    @property
    def mean_latency(self):
        return (sum(self.latencies) / len(self.latencies)) if self.latencies else 0.0

    @property
    def max_latency(self):
        return max(self.latencies) if self.latencies else 0.0


def simulate(profile, pub_rate=100.0, proc_time=0.030, duration=1.0, dt=0.001):
    """模拟"高频发布 + 慢速回调"这条链路。

    参数
    ----
    profile   : QoSProfile
    pub_rate  : 发布频率 (Hz)。例如 100 Hz 对应 10 ms 一帧。
    proc_time : 订阅端处理**单条**消息所需时间 (s)，即回调耗时。
                容量上限 = 1/proc_time（本例 33 Hz），小于 pub_rate 就会积压。
    duration  : 仿真总时长 (s)
    dt        : 仿真步长 (s)

    返回 LinkStats。
    """
    reliable = (profile.reliability == RELIABLE)
    depth = profile.depth

    seq = 0
    pub_time = {}                 # seq -> 发布时刻
    queue = deque()               # 已进入订阅端 History 缓存、等待回调的样本
    waiting = deque()             # reliable 时的发布端等待队列（背压）
    ts, backlog, drops_cum = [], [], []
    latencies = []
    delivered_seqs = []
    dropped = 0
    delivered = 0
    busy_until = 0.0
    next_pub = 0.0
    period = 1.0 / pub_rate

    n_steps = int(round(duration / dt))
    for k in range(n_steps):
        t = k * dt

        # 1) 发布端：到点就发
        while t + 1e-12 >= next_pub:
            pub_time[seq] = next_pub
            seq += 1
            next_pub += period
            if len(queue) < depth:
                queue.append(seq - 1)
            elif reliable:
                waiting.append(seq - 1)      # 不丢，交给背压队列
            else:
                # best_effort 且缓存已满：keep_last 的语义是**保留最新**、
                # 逐出最旧（而不是丢新保旧）——传感器数据旧不如新。
                queue.popleft()
                dropped += 1
                queue.append(seq - 1)

        # 2) 订阅端：空闲且有样本就取一条，占用 proc_time
        if t + 1e-12 >= busy_until and queue:
            s = queue.popleft()
            delivered += 1
            delivered_seqs.append(s)
            latencies.append((t + proc_time) - pub_time[s])
            busy_until = t + proc_time
            # 缓存空出位置后，背压队列中的样本补进来（FIFO）
            while waiting and len(queue) < depth:
                queue.append(waiting.popleft())

        ts.append(t)
        backlog.append(len(queue) + len(waiting))
        drops_cum.append(dropped)

    return LinkStats(ts, backlog, drops_cum, latencies, seq, delivered,
                     delivered_seqs)


def sweep_depth(pub_rate=100.0, proc_time=0.030, duration=1.0, dt=0.001,
                depths=(1, 2, 5, 10, 20, 50)):
    """扫描 keep_last 深度，返回 [(depth, drop_rate), ...]（best_effort）。"""
    out = []
    for d in depths:
        st = simulate(QoSProfile(BEST_EFFORT, depth=d), pub_rate, proc_time,
                      duration, dt)
        out.append((d, st.drop_rate))
    return out


def latch_demo(pub_rate=10.0, join_time=0.5, duration=1.5):
    """演示 durability（持久性）：发布端持续发布，订阅者在 join_time 才加入。

    返回 dict：
      history_sent    —— 订阅者加入**之前**已发布的样本数
      volatile_got    —— volatile 订阅者从历史里收到的条数（恒为 0）
      transient_got   —— transient_local 订阅者从历史里收到的条数（1，即 latched）

    语义依据：volatile 不保留历史，后加入者只能收到加入之后的新样本；
    transient_local 会保留最后一个样本（depth=1 的 latched），后加入者立刻收到。
    """
    period = 1.0 / pub_rate
    history_sent = int(math.floor(join_time / period))   # t=0 起每 period 发一条
    volatile_got = 0
    transient_got = 1 if history_sent > 0 else 0
    return {
        "history_sent": history_sent,
        "volatile_got": volatile_got,
        "transient_got": transient_got,
        "duration": duration,
    }
