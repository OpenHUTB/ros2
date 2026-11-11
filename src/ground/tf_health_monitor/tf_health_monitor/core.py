#!/usr/bin/env python3
"""TF 链路健康判据的纯逻辑实现。

本模块不导入 rclpy，只处理「(parent, child, stamp) 观测序列 + 当前时刻」，
因此可以在没有 ROS 的机器上用 pytest 直接回归。节点只做两件事：把
TFMessage 拆成观测喂进来，把结论发出去。

单条边的状态：
- ok：新鲜、频率达标，且未发现结构性错误；
- stale：超过 stale_timeout 没有新观测（发布者掉线、回调被阻塞、话题被取消）；
- backwards：时间戳相对上一次不增反减（时间源被重置、收发两端时钟不一致）；
- old_data：消息在到达，但时间戳落后现实超过 stamp_lag，对应 tf2 的
  ``TF_OLD_DATA``（发布者时钟漂移、恢复时补发旧帧、仿真时间没对齐）；
- multi_parent：同一个 child 在窗口内有多个存活 parent，TF 树被破坏；
- low_rate：观测已稳定，但频率低于 min_rate（发布循环被拖慢、计算量过大）；
- unknown：样本数量不足，暂不下结论；
- missing：watched 中显式声明、但从未收到过。
"""

import math
from collections import deque
from dataclasses import dataclass

OK = 'ok'
STALE = 'stale'
BACKWARDS = 'backwards'
OLD_DATA = 'old_data'
MULTI_PARENT = 'multi_parent'
LOW_RATE = 'low_rate'
MISSING = 'missing'
UNKNOWN = 'unknown'
NO_INPUT = 'no_input'

# 总体状态取最严重的一条；元组顺序即严重程度，越靠前越严重。
SEVERITY = (MISSING, MULTI_PARENT, BACKWARDS, OLD_DATA, STALE, LOW_RATE,
            UNKNOWN, OK)


def normalize_frame(name):
    """tf2 要求 frame_id 不带前导斜杠。

    带斜杠的 ``/base_link`` 和不带的 ``base_link`` 在 tf2 里是两条不同的边，
    TF 树看起来「断了」，实际只是名字写法不一致。这里统一去掉一个前导斜杠，
    避免把命名差异误判成断链。
    """
    text = str(name)
    if len(text) > 1 and text.startswith('/'):
        return text[1:]
    return text


def parse_watched_frames(spec):
    """把 ``['map->odom', 'odom -> base_footprint']`` 解析成边列表。

    返回 ``[('map', 'odom'), ...]``。空列表表示不预先声明，动态发现所有边。
    格式不合法（缺少 ``->``、某一端为空、parent 与 child 相同）时抛 ValueError。
    """
    frames = []
    for item in spec or []:
        text = str(item).replace(' ', '')
        if not text:
            continue
        if '->' not in text:
            raise ValueError(
                '监视项必须是 parent->child 形式，当前为 %r' % (item,))
        parent, child = text.split('->', 1)
        if not parent or not child:
            raise ValueError(
                '监视项两端都不能为空，当前为 %r' % (item,))
        key = (normalize_frame(parent), normalize_frame(child))
        if key[0] == key[1]:
            raise ValueError(
                'parent 与 child 不能相同，当前为 %r' % (item,))
        if key not in frames:
            frames.append(key)
    return frames


def validate_parameters(stale_timeout, rate_window, min_rate, min_samples,
                        backwards_hold, stamp_lag, publish_rate):
    """检查参数合法性，非法时抛 ValueError（节点启动时调用）。"""
    if not math.isfinite(stale_timeout) or stale_timeout <= 0.0:
        raise ValueError(
            'stale_timeout 必须大于 0，当前为 %r' % (stale_timeout,))
    if not math.isfinite(rate_window) or rate_window <= 0.0:
        raise ValueError(
            'rate_window 必须大于 0，当前为 %r' % (rate_window,))
    if not math.isfinite(min_rate) or min_rate <= 0.0:
        raise ValueError('min_rate 必须大于 0，当前为 %r' % (min_rate,))
    if min_samples < 2:
        raise ValueError(
            'min_samples 必须大于等于 2，当前为 %r' % (min_samples,))
    if not math.isfinite(backwards_hold) or backwards_hold <= 0.0:
        raise ValueError(
            'backwards_hold 必须大于 0，当前为 %r' % (backwards_hold,))
    if not math.isfinite(stamp_lag) or stamp_lag <= 0.0:
        raise ValueError('stamp_lag 必须大于 0，当前为 %r' % (stamp_lag,))
    if not math.isfinite(publish_rate) or publish_rate <= 0.0:
        raise ValueError(
            'publish_rate 必须大于 0，当前为 %r' % (publish_rate,))


@dataclass
class EdgeReport:
    """一条边的判定结果。"""

    parent: str
    child: str
    status: str
    stamp: object = None
    rate: float = 0.0
    backwards_count: int = 0
    age: float = float('inf')
    static: bool = False
    skew: float = 0.0

    def label(self):
        """返回 ``parent->child``。"""
        return '%s->%s' % (self.parent, self.child)


class _Edge:
    """内部状态：一条边的观测聚合。"""

    __slots__ = ('parent', 'child', 'last_stamp', 'last_seen', 'arrivals',
                 'backwards_at', 'backwards_count', 'static')

    def __init__(self, parent, child):
        self.parent = parent
        self.child = child
        self.last_stamp = None
        self.last_seen = None
        self.arrivals = deque()
        self.backwards_at = None
        self.backwards_count = 0
        self.static = False


class TfHealthTracker:
    """把 TF 观测流按边聚合，并给出每条边的健康判定。"""

    def __init__(self, stale_timeout=0.5, rate_window=2.0, min_rate=1.0,
                 min_samples=5, backwards_hold=2.0, stamp_lag=1.0,
                 watched=None):
        self.stale_timeout = float(stale_timeout)
        self.rate_window = float(rate_window)
        self.min_rate = float(min_rate)
        self.min_samples = int(min_samples)
        self.backwards_hold = float(backwards_hold)
        self.stamp_lag = float(stamp_lag)
        self.watched = [tuple(key) for key in (watched or [])]
        self._edges = {}
        self._parents = {}
        self.total_samples = 0

    def observe(self, parent, child, stamp_ns, now_s, static=False):
        """记录一次观测：``parent`` 在 ``stamp_ns`` 时刻发布了到 ``child`` 的变换。"""
        key = (normalize_frame(parent), normalize_frame(child))
        edge = self._edges.get(key)
        if edge is None:
            edge = _Edge(key[0], key[1])
            self._edges[key] = edge
        if edge.last_stamp is not None and stamp_ns < edge.last_stamp:
            edge.backwards_count += 1
            edge.backwards_at = now_s
        edge.last_stamp = stamp_ns
        edge.last_seen = now_s
        edge.static = edge.static or bool(static)
        edge.arrivals.append(now_s)
        self._prune(edge, now_s)
        self._parents.setdefault(key[1], {})[key[0]] = now_s
        self.total_samples += 1
        return key

    def _prune(self, edge, now_s):
        horizon = now_s - self.rate_window
        while edge.arrivals and edge.arrivals[0] < horizon:
            edge.arrivals.popleft()

    def _rate(self, edge):
        """窗口内的平均到达频率，Hz；样本不足时返回 0。"""
        count = len(edge.arrivals)
        if count < 2:
            return 0.0
        span = edge.arrivals[-1] - edge.arrivals[0]
        if span <= 0.0:
            return 0.0
        return (count - 1) / span

    def _active_parents(self, child, now_s):
        """同一 child 在 stale_timeout 内仍然存活的 parent。"""
        parents = self._parents.get(child, {})
        alive = [name for name, seen in parents.items()
                 if now_s - seen <= self.stale_timeout]
        return sorted(alive)

    def _classify(self, edge, now_s):
        age = (now_s - edge.last_seen
               if edge.last_seen is not None else float('inf'))
        rate = self._rate(edge)
        # 时间戳为 0 表示发布者没有填 header.stamp，此时不判「消息陈旧」。
        skew = (now_s - edge.last_stamp / 1e9) if edge.last_stamp else 0.0
        report = EdgeReport(
            edge.parent, edge.child, OK,
            stamp=edge.last_stamp, rate=rate,
            backwards_count=edge.backwards_count, age=age,
            static=edge.static, skew=skew)

        # 判定顺序：树结构错误 → 时间戳回退 → 断链 → 消息陈旧 → 频率 → 样本。
        # 结构性错误先于时效问题，因为多 parent 会让 tf2 的查询直接失败。
        if len(self._active_parents(edge.child, now_s)) > 1:
            report.status = MULTI_PARENT
            return report
        if (edge.backwards_at is not None
                and now_s - edge.backwards_at <= self.backwards_hold):
            report.status = BACKWARDS
            return report

        # 静态变换只发布一次，按「时效」「频率」「时间戳新旧」判它必然误报。
        if edge.static:
            return report

        if age > self.stale_timeout:
            report.status = STALE
            return report
        if skew > self.stamp_lag:
            report.status = OLD_DATA
            return report
        if len(edge.arrivals) < self.min_samples:
            report.status = UNKNOWN
            return report
        if rate < self.min_rate:
            report.status = LOW_RATE
            return report
        return report

    def evaluate(self, now_s):
        """返回全部边的判定结果，按严重程度排序。"""
        reports = []
        for key in self.watched:
            if key not in self._edges:
                reports.append(EdgeReport(key[0], key[1], MISSING))
        for edge in self._edges.values():
            reports.append(self._classify(edge, now_s))
        order = {status: index for index, status in enumerate(SEVERITY)}
        reports.sort(key=lambda item: (order.get(item.status, len(SEVERITY)),
                                       item.label()))
        return reports


def overall_status(reports):
    """返回 ``(总体状态, 异常边列表)``；一条边都没有时状态为 ``no_input``。"""
    if not reports:
        return NO_INPUT, []
    problems = [item for item in reports
                if item.status not in (OK, UNKNOWN)]
    unknown = [item for item in reports if item.status == UNKNOWN]
    for status in SEVERITY:
        for item in problems:
            if item.status == status:
                return status, problems
    if unknown:
        return UNKNOWN, []
    return OK, []


def format_edge(report):
    """单条边的紧凑表示：``parent->child:status:rate``。"""
    return '%s:%s:%.2f' % (report.label(), report.status, report.rate)


def format_report(reports):
    """整表，分号分隔；无数据时返回空串。"""
    return ';'.join(format_edge(item) for item in reports)


def format_problems(problems):
    """异常边清单，``parent->child(status)`` 逗号分隔；无异常时返回空串。"""
    abnormal = [item for item in problems
                if item.status not in (OK, UNKNOWN)]
    return ','.join('%s(%s)' % (item.label(), item.status)
                    for item in abnormal)
