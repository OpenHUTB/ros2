class PIDController:
    def __init__(self, kp, ki, kd, output_min=-5.0, output_max=5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        # 输出限幅，防止力矩过大
        output = max(self.output_min, min(self.output_max, output))

        self.prev_error = error
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
