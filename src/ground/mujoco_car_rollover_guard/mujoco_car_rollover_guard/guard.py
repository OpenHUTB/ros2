# -*- coding: utf-8 -*-
"""侧翻风险门控：把网络输出的风险概率转成对上游指令的限速/制动动作。

纯函数实现，不依赖 MuJoCo 与 ROS，便于单元测试。
三类策略（评测时对比）：
    none  ：不干预，原样执行上游指令
    rule  ：传统门限，横滚角超过阈值才减速（反应式，必然滞后）
    nn    ：按 0.5s 前瞻的翻车概率提前减速，概率很高时直接制动
"""


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


class RolloverGuard:
    def __init__(self, cfg):
        self.cfg = cfg
        self._risk_ema = 0.0

    def reset(self):
        self._risk_ema = 0.0

    def smooth(self, risk):
        a = self.cfg.guard_ema
        self._risk_ema = a * self._risk_ema + (1.0 - a) * float(risk)
        return self._risk_ema

    def gate(self, risk, steer, throttle, smooth=True):
        """返回 (steer, throttle, action)；action 取 'pass' / 'slow' / 'brake'。"""
        r = self.smooth(risk) if smooth else float(risk)
        steer = _clamp(steer, -1.0, 1.0)
        throttle = _clamp(throttle, 0.0, 1.0)
        if r >= self.cfg.guard_brake_risk:
            return steer, 0.0, "brake"
        if r > self.cfg.risk_threshold:
            span = max(1e-6, 1.0 - self.cfg.risk_threshold)
            scale = 1.0 - self.cfg.guard_gain * (r - self.cfg.risk_threshold) / span
            return steer, throttle * _clamp(scale, 0.0, 1.0), "slow"
        return steer, throttle, "pass"


def rule_gate(roll_deg, throttle, tilt_limit_deg=20.0, brake_limit_deg=35.0):
    """传统反应式门限：横滚角超限即减速，用当前姿态而非预测。"""
    throttle = _clamp(throttle, 0.0, 1.0)
    a = abs(roll_deg)
    if a >= brake_limit_deg:
        return 0.0, "brake"
    if a >= tilt_limit_deg:
        k = (a - tilt_limit_deg) / max(1e-6, brake_limit_deg - tilt_limit_deg)
        return throttle * (1.0 - k), "slow"
    return throttle, "pass"
