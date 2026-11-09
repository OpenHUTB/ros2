"""ROS 2 生命周期状态机（不含 rclpy 依赖的纯逻辑部分）。

ROS 2 的托管节点（managed node）把「构造」和「干活」拆成两步：节点先被创建
但什么都不做，必须依次收到 configure、activate 才能开始发布；要停就
deactivate，要重新配置就 cleanup。这样启动顺序、资源释放和故障恢复都是
确定的——这正是生命周期节点相对普通节点的价值。

四个稳定状态（本模块只实现这四个，过渡态在 trigger 内部瞬时完成）：

    unconfigured --configure--> inactive --activate--> active
         ^                          |                     |
         |______cleanup_____________|____deactivate_______|
                        shutdown -> finalized

状态机严格校验转移：在 unconfigured 上调 activate 直接抛 InvalidTransition，
不会「悄悄什么都不做」。
"""

STATE_UNCONFIGURED = 'unconfigured'
STATE_INACTIVE = 'inactive'
STATE_ACTIVE = 'active'
STATE_FINALIZED = 'finalized'

STATES = (STATE_UNCONFIGURED, STATE_INACTIVE, STATE_ACTIVE, STATE_FINALIZED)

# 动作 -> (源状态, 目标状态)；源状态为 None 表示「任意状态」
TRANSITIONS = {
    'configure': (STATE_UNCONFIGURED, STATE_INACTIVE),
    'activate': (STATE_INACTIVE, STATE_ACTIVE),
    'deactivate': (STATE_ACTIVE, STATE_INACTIVE),
    'cleanup': (STATE_INACTIVE, STATE_UNCONFIGURED),
    'shutdown': (None, STATE_FINALIZED),
}

# 每个动作触发的回调名
CALLBACKS = {
    'configure': 'on_configure',
    'activate': 'on_activate',
    'deactivate': 'on_deactivate',
    'cleanup': 'on_cleanup',
    'shutdown': 'on_shutdown',
}


class InvalidTransition(Exception):
    """在当前状态下不允许该动作。"""


class CallbackFailed(Exception):
    """回调返回失败或抛异常。"""


class LifecycleStateMachine(object):
    """四状态生命周期机，回调返回 False 视为失败。"""

    def __init__(self, callbacks=None, verbose=False):
        self.state = STATE_UNCONFIGURED
        self.callbacks = dict(callbacks or {})
        self.verbose = verbose
        self.history = [(STATE_UNCONFIGURED, 'init')]
        self.errors = []
        self.active_count = 0      # 进入 active 的次数，便于观察反复启停

    # -- 查询 -------------------------------------------------------------
    def can(self, action):
        if action not in TRANSITIONS:
            return False
        src, _dst = TRANSITIONS[action]
        if self.state in (STATE_FINALIZED,):
            return False
        return src is None or self.state == src

    def actions(self):
        return [a for a in TRANSITIONS if self.can(a)]

    # -- 转移 -------------------------------------------------------------
    def trigger(self, action):
        if action not in TRANSITIONS:
            raise InvalidTransition('未知动作：%s' % action)
        if not self.can(action):
            raise InvalidTransition(
                '状态 %s 下不能执行 %s（合法动作：%s）'
                % (self.state, action, ', '.join(self.actions()) or '无'))
        src, dst = TRANSITIONS[action]
        name = CALLBACKS[action]
        cb = self.callbacks.get(name)
        if cb is not None:
            try:
                ok = cb()
            except Exception as exc:              # 回调抛异常等价于失败
                self.errors.append((action, repr(exc)))
                ok = False
            if ok is False:
                self.errors.append((action, 'callback returned False'))
                raise CallbackFailed('%s 失败，状态保持 %s' % (name, self.state))
        self.state = dst
        if dst == STATE_ACTIVE:
            self.active_count += 1
        self.history.append((dst, action))
        if self.verbose:
            print('  [%s] %s -> %s' % (action, src or '*', dst))
        return dst

    def configure(self):
        return self.trigger('configure')

    def activate(self):
        return self.trigger('activate')

    def deactivate(self):
        return self.trigger('deactivate')

    def cleanup(self):
        return self.trigger('cleanup')

    def shutdown(self):
        return self.trigger('shutdown')

    # -- 工具 -------------------------------------------------------------
    @property
    def is_active(self):
        return self.state == STATE_ACTIVE

    def state_index(self):
        return STATES.index(self.state)

    def reset(self):
        self.state = STATE_UNCONFIGURED
        self.history = [(STATE_UNCONFIGURED, 'reset')]
        self.errors = []
        self.active_count = 0


class ManagedPublisher(object):
    """模拟一个「只有 activate 之后才真正发布」的节点。

    publish() 在非 active 状态下会被丢弃并计数，用来演示生命周期节点的
    核心收益：消息不会在依赖还没就绪时漏出去。
    """

    def __init__(self, machine):
        self.machine = machine
        self.published = 0
        self.dropped = 0
        self.sent = []

    def publish(self, value):
        if not self.machine.is_active:
            self.dropped += 1
            return False
        self.published += 1
        self.sent.append(value)
        return True

    def stats(self):
        return {'published': self.published, 'dropped': self.dropped}
