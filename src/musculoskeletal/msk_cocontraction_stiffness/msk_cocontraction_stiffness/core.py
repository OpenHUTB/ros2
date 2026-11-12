"""拮抗肌共收缩与关节刚度核算（只依赖标准库）.

一对拮抗肌跨过同一个关节：伸肌产生正向力矩，屈肌产生反向力矩。净力矩只取决于两侧
肌力之差；而关节刚度取决于两侧肌力之**和**——这就是共收缩的意义：

    F_m = a * F_max                         线性肌力模型（忽略力-长度/力-速度）
    tau = r * (F_ago - F_ant)               净关节力矩
    k_m = q * F_m / L_slack                 单肌刚度（q 为无量纲系数，文献典型 10~30）
    K   = r^2 * (k_ago + k_ant)             关节刚度
        = r^2 * q * (F_ago + F_ant) / L_slack

反解（给定目标净力矩 tau 与共收缩水平 c，c = (F_ago+F_ant)/(2*F_max)）：

    F_ago = c*F_max + tau/(2r),  F_ant = c*F_max - tau/(2r)
    可行条件: tau <= 2 * c * F_max * r

线性模型下两条结论值得记：**共收缩水平决定刚度，净力矩决定姿势力矩，两者解耦**——
同样的净力矩，共收缩越高关节越"硬"，扰动下的角位移越小：

    delta_theta = tau_disturbance / K
"""

import math
from dataclasses import dataclass

Q_FACTOR_MIN = 10.0
Q_FACTOR_MAX = 30.0


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def muscle_force(activation, f_max_n):
    """线性肌力模型 F = a * F_max."""
    _require(_finite(activation) and 0.0 <= activation <= 1.0, '激活度需在 [0, 1] 内')
    _require(_finite(f_max_n) and f_max_n > 0.0, '最大等长力必须为正数')
    return activation * f_max_n


def cocontraction_solve(torque_nm, moment_arm_m, f_max_n, coactivation):
    """给定净力矩与共收缩水平，反解两侧肌力与激活度."""
    _require(_finite(torque_nm) and torque_nm >= 0.0, '净力矩不能为负（符号由屈/伸侧约定）')
    _require(_finite(moment_arm_m) and moment_arm_m > 0.0, '力臂必须为正数')
    _require(_finite(f_max_n) and f_max_n > 0.0, '最大等长力必须为正数')
    _require(_finite(coactivation) and 0.0 <= coactivation <= 1.0, '共收缩水平需在 [0, 1] 内')
    torque_max = 2.0 * coactivation * f_max_n * moment_arm_m
    if torque_nm > torque_max + 1e-9:
        return {'feasible': False, 'torque_max_nm': torque_max}
    f_net = torque_nm / moment_arm_m
    f_ago = coactivation * f_max_n + f_net / 2.0
    f_ant = max(0.0, coactivation * f_max_n - f_net / 2.0)
    return {
        'feasible': True,
        'torque_max_nm': torque_max,
        'force_agonist_n': f_ago,
        'force_antagonist_n': f_ant,
        'activation_agonist': f_ago / f_max_n,
        'activation_antagonist': f_ant / f_max_n,
    }


def joint_stiffness(moment_arm_m, q_factor, force_sum_n, slack_length_m):
    """关节刚度 N*m/rad：K = r^2 * q * (F_ago + F_ant) / L_slack."""
    _require(_finite(moment_arm_m) and moment_arm_m > 0.0, '力臂必须为正数')
    _require(_finite(q_factor) and Q_FACTOR_MIN <= q_factor <= Q_FACTOR_MAX,
             'q 系数需在 10~30 内（文献典型范围）')
    _require(_finite(force_sum_n) and force_sum_n >= 0.0, '肌力之和不能为负')
    _require(_finite(slack_length_m) and slack_length_m > 0.0, '肌肉静息长度必须为正数')
    return moment_arm_m ** 2 * q_factor * force_sum_n / slack_length_m


def deflection_deg(disturbance_torque_nm, stiffness_nm_per_rad):
    """扰动转矩下的稳态角位移 deg：delta_theta = tau_d / K."""
    _require(_finite(disturbance_torque_nm) and disturbance_torque_nm > 0.0,
             '扰动力矩必须为正数')
    _require(_finite(stiffness_nm_per_rad) and stiffness_nm_per_rad > 0.0,
             '关节刚度必须为正数')
    return math.degrees(disturbance_torque_nm / stiffness_nm_per_rad)


@dataclass
class MskCocontraction:
    """一次拮抗肌共收缩核算."""

    torque_nm: float = 8.0
    moment_arm_m: float = 0.03
    f_max_n: float = 600.0
    slack_length_m: float = 0.25
    q_factor: float = 25.0
    coactivation: float = 0.3
    disturbance_nm: float = 2.0

    def __post_init__(self):
        for name in ('torque_nm', 'moment_arm_m', 'f_max_n', 'slack_length_m',
                     'q_factor', 'coactivation', 'disturbance_nm'):
            _require(_finite(getattr(self, name)), '%s 必须是有限数' % name)
        _require(self.torque_nm >= 0.0, '净力矩不能为负')
        _require(self.moment_arm_m > 0.0 and self.f_max_n > 0.0, '力臂与肌力必须为正数')
        _require(self.slack_length_m > 0.0, '肌肉静息长度必须为正数')
        _require(Q_FACTOR_MIN <= self.q_factor <= Q_FACTOR_MAX, 'q 系数需在 10~30 内')
        _require(0.0 <= self.coactivation <= 1.0, '共收缩水平需在 [0, 1] 内')
        _require(self.disturbance_nm > 0.0, '扰动力矩必须为正数')

    def report(self):
        solution = cocontraction_solve(self.torque_nm, self.moment_arm_m,
                                       self.f_max_n, self.coactivation)
        warnings = []
        level = 'ok'
        if not solution['feasible']:
            level = 'over'
            warnings.append('目标力矩 %.1f N*m 超过当前共收缩水平能给出的 %.1f N*m，'
                            '需提高共收缩或降低目标'
                            % (self.torque_nm, solution['torque_max_nm']))
            stiffness = 0.0
            deflection = float('inf')
        else:
            force_sum = solution['force_agonist_n'] + solution['force_antagonist_n']
            stiffness = joint_stiffness(self.moment_arm_m, self.q_factor, force_sum,
                                        self.slack_length_m)
            deflection = deflection_deg(self.disturbance_nm, stiffness)
            utilization = self.torque_nm / solution['torque_max_nm']
            if utilization > 0.9:
                level = 'marginal'
                warnings.append('目标力矩已达该共收缩水平上限的 %.0f%%，拮抗侧只剩 %.1f N'
                                % (utilization * 100.0, solution['force_antagonist_n']))
            if not warnings:
                warnings.append('可行：拮抗侧肌力 %.1f N，关节刚度 %.2f N*m/rad，'
                                '%.1f N*m 扰动引起 %.2f deg 偏转'
                                % (solution['force_antagonist_n'], stiffness,
                                   self.disturbance_nm, deflection))
        return {
            'parameters': {
                'torque_nm': self.torque_nm,
                'moment_arm_m': self.moment_arm_m,
                'f_max_n': self.f_max_n,
                'slack_length_m': self.slack_length_m,
                'q_factor': self.q_factor,
                'coactivation': self.coactivation,
                'disturbance_nm': self.disturbance_nm,
            },
            'solution': solution,
            'level': level,
            'stiffness': {
                'joint_nm_per_rad': stiffness,
                'deflection_deg': deflection,
            },
            'warnings': warnings,
            'notes': [
                '线性肌力模型，忽略力-长度与力-速度关系（本模块只算准静态共收缩）',
                'q 系数取 %d（文献典型 10~30），实际值随肌纤维类型与激活状态变化' % self.q_factor,
                '两侧肌力臂取相同 r，真实关节的屈/伸肌力臂不对称',
            ],
        }
