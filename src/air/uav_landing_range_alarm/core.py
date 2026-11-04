import math

class Alarm:
    def __init__(self, low=1.0, high=1.3, confirm=3, timeout=.5):
        if not (math.isfinite(low) and math.isfinite(high) and 0 < low < high):
            raise ValueError('require 0 < low < high')
        if isinstance(confirm, bool) or not isinstance(confirm, int) or confirm < 1:
            raise ValueError('confirm must be a positive integer')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('positive finite timeout required')
        self.low, self.high, self.confirm, self.timeout = low, high, confirm, timeout
        self.state, self.last, self.count = 'UNKNOWN', None, 0
        self.last_now = None

    def update(self, value, now):
        if not math.isfinite(now) or (self.last_now is not None and now < self.last_now):
            self.state, self.count, self.last = 'INVALID', 0, None
            return self.state
        if self.last is not None and now-self.last > self.timeout:
            self.state, self.count = 'STALE', 0
        self.last_now = now
        if not math.isfinite(value) or value < 0:
            self.state, self.count = 'INVALID', 0
            return self.state
        self.last = now
        if value <= self.low:
            if self.state == 'ALARM': return self.state
            self.count += 1
            if self.count >= self.confirm: self.state, self.count = 'ALARM', 0
        elif value >= self.high:
            self.count = 0
            self.state = 'CLEAR'
        else:
            self.count = 0
        return self.state

    def check(self, now):
        if not math.isfinite(now) or (self.last_now is not None and now < self.last_now):
            self.state, self.count, self.last = 'INVALID', 0, None
        elif self.last is None or now-self.last > self.timeout:
            self.state, self.count = 'STALE', 0
        self.last_now = now if math.isfinite(now) else None
        return self.state
