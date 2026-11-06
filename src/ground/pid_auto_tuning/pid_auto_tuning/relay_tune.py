"""继电器反馈实验：求临界增益 Ku 与临界振荡周期 Tu。"""

import math

from .plant import HeadingPlant


def run_relay_feedback(plant_factory=None, setpoint=math.pi / 2.0,
                       relay_amplitude=0.5, dt=0.01, u_max=2.5,
                       t_max=60.0, measure_window=20.0):
    if plant_factory is None:
        plant_factory = lambda: HeadingPlant(dt=dt, u_max=u_max)
    plant = plant_factory()
    t = 0.0
    prev_sign = 1
    switch_times = []
    samples = []

    for _ in range(int(round(t_max / dt))):
        error = setpoint - plant.theta
        sign = 1 if error >= 0 else -1
        if sign != prev_sign:
            switch_times.append(t)
            prev_sign = sign
        plant.step(sign * relay_amplitude)
        t += dt
        if t > t_max - measure_window:
            samples.append(plant.theta)

    if len(switch_times) < 3:
        raise RuntimeError('继电器反馈未产生足够振荡，请增大 relay_amplitude 或延长 t_max')

    # 相邻两次同方向切换之间为一个周期
    periods = [switch_times[i + 2] - switch_times[i] for i in range(len(switch_times) - 2)]
    tu = sum(periods[-3:]) / len(periods[-3:])
    amplitude = (max(samples) - min(samples)) / 2.0
    ku = 4.0 * relay_amplitude / (math.pi * amplitude)

    return {'ku': ku, 'tu': tu, 'amplitude': amplitude, 'switch_times': switch_times}
