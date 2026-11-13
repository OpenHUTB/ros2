"""带积分限幅与输出限幅的 PID 控制器。"""


class PID:
    def __init__(self, kp, ki, kd, max_total=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_total = max_total
        self.integral = 0.0
        self.prev_err = None

    def reset(self):
        self.integral = 0.0
        self.prev_err = None

    def step(self, err, dt=1.0):
        self.integral += err * dt
        d = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp * err + self.ki * self.integral + self.kd * d
        if self.max_total is not None:
            out = max(-self.max_total, min(self.max_total, out))
        return out
