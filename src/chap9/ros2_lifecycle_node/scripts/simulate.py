"""时间轴演示：按脚本依次下发状态转移，记录每次「发布」是否被接受。

场景模拟一次真实的部署：节点起来后先 configure 读参数、activate 开始发数据，
中途因为换了参数 deactivate 再 activate，最后 cleanup 退出。所有时刻都以
0.1 s 为单位推进，每个 tick 都尝试发布一条心跳，据此统计「被接受 / 被丢弃」
的条数——未激活时发布会被丢弃，这是生命周期节点最直观的收益。
"""

from lifecycle import (LifecycleStateMachine, ManagedPublisher,
                       STATE_ACTIVE, STATE_INACTIVE, STATE_UNCONFIGURED,
                       STATE_FINALIZED, InvalidTransition, CallbackFailed)

TICK = 0.1           # 时间步长 (s)
DURATION = 14.0      # 总时长 (s)

# (时刻, 动作, 说明)
SCRIPT = [
    (0.5, 'configure', '读取参数、建 publisher'),
    (2.0, 'activate', '开始发布心跳'),
    (5.5, 'deactivate', '暂停发布'),
    (6.5, 'configure', None),      # 非法：inactive 下不能 configure
    (7.0, 'activate', '恢复发布'),
    (10.5, 'deactivate', '准备退出'),
    (11.0, 'cleanup', '释放资源'),
    (12.0, 'activate', None),      # 非法：unconfigured 下不能 activate
    (13.0, 'shutdown', '节点终止'),
]


def run(duration=DURATION, tick=TICK, script=None):
    """跑一遍时间轴，返回轨迹与统计。"""
    script = SCRIPT if script is None else script
    machine = LifecycleStateMachine()
    pub = ManagedPublisher(machine)

    events = {round(t / tick): (act, note) for t, act, note in script}
    trace = []
    applied = []
    rejected = []

    n = int(round(duration / tick))
    for i in range(n + 1):
        t = round(i * tick, 6)
        if i in events:
            act, note = events[i]
            try:
                machine.trigger(act)
                applied.append((t, act, machine.state))
            except (InvalidTransition, CallbackFailed) as exc:
                rejected.append((t, act, str(exc)))
        value = round(t, 3)
        accepted = pub.publish(value)
        trace.append({
            't': t,
            'state': machine.state,
            'state_idx': machine.state_index(),
            'accepted': accepted,
            'published': pub.published,
            'dropped': pub.dropped,
            'value': value,
        })

    return {
        'trace': trace,
        'machine': machine,
        'pub': pub,
        'applied': applied,
        'rejected': rejected,
    }


def summary(result):
    trace = result['trace']
    pub = result['pub']
    total_ticks = len(trace)
    return {
        'ticks': total_ticks,
        'published': pub.published,
        'dropped': pub.dropped,
        'active_ratio': pub.published / float(total_ticks),
        'active_count': result['machine'].active_count,
        'applied': len(result['applied']),
        'rejected': len(result['rejected']),
        'final_state': result['machine'].state,
    }


def transitions_demo():
    """完整跑一遍合法动作序列，返回每一步后的状态。"""
    m = LifecycleStateMachine()
    steps = []
    for act in ('configure', 'activate', 'deactivate', 'activate',
                'deactivate', 'cleanup', 'configure', 'activate', 'shutdown'):
        m.trigger(act)
        steps.append((act, m.state))
    return steps
