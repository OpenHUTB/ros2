"""闭环仿真：ATO 指令 -> 逐节力分配 -> 车钩动力学积分。"""

import math

from .consist import Actuator, Consist
from .params import (ActuatorParams, ControlParams, CouplerParams, DEFAULT_CAR,
                     Route, SINGLE_BODY)
from .profile import build_reference

DT = 0.002          # 积分步长：车钩刚度 25 MN/m，ω≈18 rad/s，2 ms 足够稳定
CONTROL_DT = 0.02   # 控制周期，与 train_ato_stop 的单帧 0.02 s 对齐
JERK_TAU = 0.10     # 纵向冲动用的低通时间常数


class AtoController:
    """列车级 ATO：速度跟踪 + 站台停车闭环。

    停车精度的一个关键点是执行机构滞后（0.8 s）会让实际速度高于参考曲线，
    等到距离不够时再制动就来不及。这里把滞后折算成"已经多走的距离" feedforward
    到剩余距离上（lead = v * tau），等价于提前 0.8 s 开始制动。
    """

    def __init__(self, reference, route, control, stop_x):
        self.reference = reference
        self.route = route
        self.control = control
        self.stop_x = stop_x

    def command(self, position, speed):
        c = self.control
        remaining = self.stop_x - position
        lead = speed * c.lead_time
        effective = max(0.0, remaining - lead)

        if remaining < c.stop_window and speed < c.stop_speed:
            a = c.kp * effective - c.kd * speed
            return max(-c.a_service, min(c.a_service, a))

        v_reference = self.reference.at(position + c.preview)
        v_stop = math.sqrt(max(0.0, 2.0 * c.a_plan * effective))
        v_limit = self.route.limit_at(position) - c.speed_margin
        target = min(v_reference, v_stop, v_limit)
        a = c.k_v * (target - speed)
        return max(-c.a_service, min(c.a_service, a))


def build_consist(spec):
    route = Route(length=spec.get("length", 2400.0),
                  speed_limit=spec.get("speed_limit", 160.0 / 3.6),
                  grades=tuple(spec.get("grades", ())))
    car = spec.get("car", DEFAULT_CAR)
    n = int(spec.get("n_cars", 4))
    consist = Consist(n, car, spec.get("coupler", CouplerParams()), route)
    return consist, route


def coupling_delays(n_cars, per_car_delay, direction):
    """逐节指令延迟。head：车头先动；tail：车尾先动。"""
    if n_cars <= 1 or per_car_delay <= 0.0:
        return [0.0] * n_cars
    if direction == "tail":
        return [(n_cars - 1 - i) * per_car_delay for i in range(n_cars)]
    return [i * per_car_delay for i in range(n_cars)]


def run_scenario(spec, dt=DT, control_dt=CONTROL_DT, max_time=400.0):
    consist, route = build_consist(spec)
    control = ControlParams()
    actuator_params = ActuatorParams()
    coupler_params = spec.get("coupler", CouplerParams())

    reference, envelope = build_reference(route, consist, control)
    controller = AtoController(reference, route, control, route.length)
    delays = coupling_delays(consist.n, spec.get("delay", 0.0), spec.get("direction", "head"))

    actuators = [Actuator(actuator_params) for _ in range(consist.n)]
    shares = [0.0]
    gamma = consist.car.gamma

    history = {"t": [], "head": [], "v_head": [], "v_tail": [],
               "couplers": [], "accs": [], "a_cmd": [], "x": []}

    t = 0.0
    tick = 0
    next_control = 0.0
    next_record = 0.0
    dwell = 0.0
    a_filtered = [0.0] * consist.n
    a_previous = [0.0] * consist.n
    jerk_max = 0.0
    peak_buff = 0.0
    peak_draft = 0.0
    per_coupler_peak = [0.0] * max(0, consist.n - 1)
    v_max = 0.0
    brake_start = None
    brake_time = None
    accs = [0.0] * consist.n
    a_cmd = 0.0

    while t < max_time:
        if t + 1e-9 >= next_control:
            a_cmd = controller.command(consist.head, consist.v[0])
            demand = (gamma * consist.total_mass * a_cmd
                      + consist.resistance_total()
                      + consist.grade_force_total())
            upper = consist.traction_available()
            lower = -consist.brake_available(control.a_service)
            demand = max(lower, min(upper, demand))
            shares.append(demand / consist.n)
            tick = len(shares) - 1
            next_control += control_dt

        for i in range(consist.n):
            index = tick - int(round(delays[i] / control_dt))
            target = shares[index] if index >= 0 else 0.0
            consist.force[i] = actuators[i].step(target, dt)

        accs = consist.integrate(dt)
        t += dt

        if v_max < consist.v[0]:
            v_max = consist.v[0]
        if brake_start is None and a_cmd < -0.1:
            brake_start = consist.head
            brake_time = t

        if t + 1e-9 >= next_record:
            face = consist.coupler_forces()
            history["t"].append(t)
            history["head"].append(consist.head)
            history["v_head"].append(consist.v[0])
            history["v_tail"].append(consist.v[-1])
            history["couplers"].append(face)
            history["accs"].append(list(accs))
            history["a_cmd"].append(a_cmd)
            history["x"].append(list(consist.x))

            for i in range(consist.n):
                a_filtered[i] += (accs[i] - a_filtered[i]) * control_dt / JERK_TAU
                jerk = abs(a_filtered[i] - a_previous[i]) / control_dt
                a_previous[i] = a_filtered[i]
                if jerk > jerk_max:
                    jerk_max = jerk
            for i, value in enumerate(face):
                if value > peak_buff:
                    peak_buff = value
                if value < peak_draft:
                    peak_draft = value
                if abs(value) > per_coupler_peak[i]:
                    per_coupler_peak[i] = abs(value)
            next_record += control_dt

        if t > 5.0:
            error = consist.head - route.length
            if consist.v[0] < 0.02:
                dwell += dt
                if abs(error) < 0.02:
                    break
                if dwell > 2.0:
                    break
            else:
                dwell = 0.0

    return {
        "name": spec.get("name", "unnamed"),
        "note": spec.get("note", ""),
        "n_cars": consist.n,
        "train_length": consist.length,
        "run_time": t,
        "v_max": v_max,
        "stop_error": consist.head - route.length,
        "peak_buff": peak_buff,
        "peak_draft": peak_draft,
        "per_coupler_peak": per_coupler_peak,
        "max_jerk": jerk_max,
        "brake_start": brake_start if brake_start is not None else float("nan"),
        "brake_time": brake_time if brake_time is not None else float("nan"),
        "final_position": consist.head,
        "history": history,
        "delay": spec.get("delay", 0.0),
        "direction": spec.get("direction", "head"),
        "grades": tuple(spec.get("grades", ())),
    }
