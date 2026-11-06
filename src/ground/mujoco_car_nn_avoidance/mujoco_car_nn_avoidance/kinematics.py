# -*- coding: utf-8 -*-
"""两轮差速运动学：(steer, throttle) -> 左右轮角速度。

纯函数实现，不依赖 MuJoCo，便于在未装仿真器的环境里做单元测试。
steer>0 表示向左转（yaw 增大），throttle 取 [0, 1] 控制油门大小。
"""


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def differential_wheel_vels(steer, throttle, v_max, omega_max, wheel_r, track,
                            max_wheel_omega):
    """把归一化指令解算成左右轮目标角速度 (rad/s)。

    运动学关系：
        v = throttle * v_max
        omega = steer * omega_max
        w_l = (v - omega * track / 2) / wheel_r
        w_r = (v + omega * track / 2) / wheel_r

    轮速再按 max_wheel_omega 限幅，与 XML 里 velocity 执行器的 ctrlrange 对齐。
    """
    v = _clamp(throttle, 0.0, 1.0) * v_max
    omega = _clamp(steer, -1.0, 1.0) * omega_max
    half = track / 2.0
    w_l = (v - omega * half) / wheel_r
    w_r = (v + omega * half) / wheel_r
    w_l = _clamp(w_l, -max_wheel_omega, max_wheel_omega)
    w_r = _clamp(w_r, -max_wheel_omega, max_wheel_omega)
    return w_l, w_r
