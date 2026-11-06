"""红绿灯状态机。

按 [(持续时间, 颜色), ...] 的相位序列循环切换。
颜色取值：'red' / 'yellow' / 'green'。
"""
class TrafficLight:
    def __init__(self, phases, t0=0.0):
        """
        phases: [(duration_sec, color), ...]，例如
                [(4.0, 'green'), (1.0, 'yellow'), (9.0, 'red'), (12.0, 'green')]
        """
        if not phases:
            raise ValueError("phases must not be empty")
        self.phases = list(phases)
        self.total = sum(d for d, _ in self.phases)
        self.t = float(t0)
        self.cycle = 0.0

    def update(self, dt):
        """推进时间，返回当前灯色。"""
        self.t += dt
        self.cycle = self.t % self.total
        acc = 0.0
        for dur, color in self.phases:
            if self.cycle < acc + dur:
                return color
            acc += dur
        return self.phases[-1][1]

    @property
    def color(self):
        return self.update(0.0)
