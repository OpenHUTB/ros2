"""带抗积分饱和的位置式 PID。"""


class PIDController:
    def __init__(self, kp, ki, kd, dt, u_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.u_limit = u_limit
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error):
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / self.dt
        candidate = self.integral + error * self.dt
        u = self.kp * error + self.ki * candidate + self.kd * derivative

        if self.u_limit is not None and abs(u) > self.u_limit:
            # 饱和且误差仍在同向增大时不累积积分，避免积分饱和
            u = self.u_limit if u > 0 else -self.u_limit
            if not ((u > 0 and error > 0) or (u < 0 and error < 0)):
                self.integral = candidate
        else:
            self.integral = candidate

        self.prev_error = error
        return u
