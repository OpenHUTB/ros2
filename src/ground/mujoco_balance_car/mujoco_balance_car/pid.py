#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PID 控制器（带积分限幅与微分可选项）。

与仓库里原有的极简实现相比，这里补了两点——都是实测会出事的地方：

1. **积分限幅（anti-windup）**：输出被 `output_max` 截断时，如果积分项继续累加，
   误差会在饱和期间"喂"出一个巨大的积分量，退出饱和后要很久才退回来，表现为
   小车冲过目标位置再慢慢爬回（位置外环尤其明显）。因此积分项单独限幅。
2. **微分项基于测量值（derivative on measurement）**：默认对"误差的变化率"求导，
   一旦目标位置突变，误差瞬间跳变，微分项会给出一个尖峰（derivative kick）。
   本模块的位置外环目标是阶跃，所以默认对测量值求导，阶跃时不会踢一下。

两项都可以用参数关掉，测试里分别验证了开关行为。
"""


class PIDController:
    """标准位置式 PID。

    参数
    ----
    kp, ki, kd      : 增益
    output_min/max  : 输出限幅（同时用于积分限幅，避免饱和期间的积分累积）
    integral_limit  : 积分项限幅；None 表示按输出限幅换算
    derivative_on_measurement : True 时微分作用于测量值（避免目标阶跃引起的尖峰）
    """

    def __init__(self, kp, ki, kd, output_min=-5.0, output_max=5.0,
                 integral_limit=None, derivative_on_measurement=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_min = float(output_min)
        self.output_max = float(output_max)
        # 积分项限幅：默认让 ki*integral 不超过输出幅值，防止饱和期间疯涨
        if integral_limit is None:
            integral_limit = (max(abs(self.output_min), abs(self.output_max))
                              / max(abs(self.ki), 1e-9)) if self.ki else 0.0
        self.integral_limit = float(integral_limit)
        self.derivative_on_measurement = bool(derivative_on_measurement)

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = None
        self.last_output = 0.0

    # ---------------------------------------------------------------- 核心
    def compute(self, target, current, dt):
        """一步控制：返回限幅后的输出。"""
        if dt <= 0:
            raise ValueError(f"dt 必须为正，收到 {dt}")
        error = float(target) - float(current)

        # 积分项：先累加再限幅（限幅本身即 anti-windup）
        self.integral += error * dt
        if self.integral_limit:
            self.integral = max(-self.integral_limit,
                                min(self.integral_limit, self.integral))

        if self.derivative_on_measurement and self.prev_measurement is not None:
            derivative = -(float(current) - self.prev_measurement) / dt
        else:
            derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.output_min, min(self.output_max, output))

        self.prev_error = error
        self.prev_measurement = float(current)
        self.last_output = output
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = None
        self.last_output = 0.0
