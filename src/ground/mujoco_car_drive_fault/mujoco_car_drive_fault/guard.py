# -*- coding: utf-8 -*-
"""单侧驱动失效的检测与补偿：把"跑偏风险"转成对差速指令的修正。

纯函数实现，不依赖 MuJoCo 与 ROS，便于单元测试。
检测口径（评测时对比）：
    none ：不干预，车被拖滞侧拉着跑偏
    rule ：横向偏差 / 航向越过阈值才启用补偿（反应式，车已经在偏）
    nn   ：按 1.0 s 前瞻的跑偏概率提前启用补偿
补偿本身三组共用：横向偏差 + 航向 + 偏航角速度的 PD，加一项对恒定拖滞的积分，
两侧对称，靠差速把车头拉回车道中心。
"""


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def comp_steer(y, yaw, yaw_rate, cfg, gain=1.0, integ=0.0):
    """按当前位姿算出抵消跑偏的转向修正量（归一化，负号表示往右打）。"""
    raw = -(cfg.comp_k_y * y + cfg.comp_k_psi * yaw
            + cfg.comp_k_yaw * yaw_rate + cfg.comp_ki * integ)
    limit = cfg.comp_limit * _clamp(gain, 0.0, 1.5)
    return _clamp(raw, -limit, limit)


class FaultGuard:
    """按检测结果决定补偿强度。"""

    def __init__(self, cfg):
        self.cfg = cfg
        self._risk_ema = 0.0
        self._active = False
        self.integ = 0.0

    def reset(self):
        self._risk_ema = 0.0
        self._active = False
        self.integ = 0.0

    def smooth(self, risk):
        a = self.cfg.guard_ema
        self._risk_ema = a * self._risk_ema + (1.0 - a) * float(risk)
        return self._risk_ema

    def decide(self, risk, y, yaw_deg, mode="nn"):
        """返回 (gain, action)，gain=0 表示不补偿；action 取 pass/assist/slow。"""
        if mode == "none":
            return 0.0, "pass"
        if mode == "rule":
            # 反应式：已经看到明显的横向偏差或航向偏角才动手
            if abs(y) < self.cfg.rule_y and abs(yaw_deg) < self.cfg.rule_yaw_deg:
                return 0.0, "pass"
            rel_y = abs(y) / max(1e-6, self.cfg.deviate_m)
            rel_psi = abs(yaw_deg) / max(1e-6, self.cfg.rule_yaw_deg)
            return _clamp(max(rel_y, rel_psi), 0.4, 1.0), "assist"
        # nn：风险越高补偿越强，超过高阈值时判为无法维持，减速保安全。
        # 触发后保持补偿，直到车真的回到车道中心——补偿本身会改变状态，
        # 网络在这种"已经被纠正"的轨迹上容易给出低风险，不能一停就松手。
        r = self.smooth(risk)
        if r > self.cfg.risk_threshold:
            self._active = True
        if self._active and r <= self.cfg.risk_threshold \
                and abs(y) < self.cfg.recover_y and abs(yaw_deg) < self.cfg.recover_yaw_deg:
            self._active = False
            self.integ = 0.0
        if not self._active:
            return 0.0, "pass"
        if r >= self.cfg.guard_brake_risk:
            return 1.0, "slow"
        # 补偿一旦触发就给满权限：拖滞是持续扰动，半权限压不住
        return 1.0, "assist"

    def update_integ(self, y):
        """补偿生效期间累积横向偏差，用来吃掉恒定拖滞造成的稳态偏置。"""
        self.integ = _clamp(self.integ + y * self.cfg.dt,
                            -self.cfg.comp_integ_limit, self.cfg.comp_integ_limit)
        return self.integ


def rule_gate(y, yaw_deg, throttle, limit_y=0.15, limit_yaw=3.0, stop_y=0.8):
    """传统门限：只看当前横向偏差 / 航向偏角，不做预测。"""
    throttle = _clamp(throttle, 0.0, 1.0)
    if abs(y) >= stop_y:
        return 0.0, "slow"
    if abs(y) >= limit_y or abs(yaw_deg) >= limit_yaw:
        return throttle * 0.85, "assist"
    return throttle, "pass"
