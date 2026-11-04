# -*- coding: utf-8 -*-
"""四种前车工况。

每种工况只回答一个问题：**前车此刻应该以多大速度行驶**（``target_speed(t)``）。
把“速度曲线”这种纯数学部分与控制 CARLA 的部分分开，好处是：
本文件在导入时**不依赖 carla**（carla 只在真正下发控制时才导入），
因此可以在没有 CARLA 的机器上直接跑单元测试。
"""

DEFAULT_V0 = 16.67   # m/s，约 60 km/h


def speed_of(actor):
    import math
    v = actor.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def steer_to_point(vehicle, target, gain=1.0):
    """朝一个世界坐标点转向：纯追踪与面包屑跟踪共用的几何部分。

    ``alpha`` 是目标点相对车头方向的夹角，归一化到 [-pi, pi]；
    转向量与夹角成正比并限幅到 [-1, 1]。
    """
    import math

    transform = vehicle.get_transform()
    location = transform.location
    yaw = math.radians(transform.rotation.yaw)
    alpha = math.atan2(target.y - location.y, target.x - location.x) - yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    return max(-1.0, min(1.0, gain * alpha))


def lane_steer(vehicle, carla_map, base_lookahead=8.0, gain=1.5):
    """纯追踪（Pure Pursuit）横向控制：让车辆沿着车道中心线行驶。

    为什么必须有它：本模块的控制律只输出油门/刹车（纵向）。如果方向盘始终为 0，
    车辆会沿直线冲出弯道撞上马路牙子或建筑，表现为"全油门却越来越慢"。

    做法：在车道中心线上取车辆前方 ``lookahead`` 米处的路点作为追踪目标。
    """
    transform = vehicle.get_transform()
    location = transform.location
    lookahead = max(6.0, base_lookahead + 0.35 * speed_of(vehicle))

    import carla
    waypoint = carla_map.get_waypoint(location, project_to_road=True,
                                      lane_type=carla.LaneType.Driving)
    candidates = waypoint.next(lookahead)
    if not candidates:
        return 0.0
    return steer_to_point(vehicle, candidates[0].transform.location, gain)


def drive_to_speed(vehicle, v_target, carla_map=None, kp=0.5, max_throttle=1.0,
                   throttle_bias=0.25):
    """P 控制器 + 前馈项：把前车开到目标速度（用来制造可复现的工况）。

    ``throttle_bias`` 是前馈项，用来抵消滚动阻力与空气阻力。
    没有它时车辆会在"全油门 -> 到达目标速度 -> 完全松油门 -> 掉速"之间大幅振荡
    （实测速度会在 10~16 m/s 之间来回摆动，污染跟车数据）。

    传入 ``carla_map`` 时会同时做车道保持；否则只控制纵向（车辆走直线）。
    """
    import carla

    error = v_target - speed_of(vehicle)
    raw = throttle_bias + kp * error
    if raw >= 0.0:
        control = carla.VehicleControl(throttle=min(max_throttle, raw), brake=0.0)
    else:
        control = carla.VehicleControl(throttle=0.0, brake=min(1.0, 0.6 * (-raw)))
    if carla_map is not None:
        control.steer = lane_steer(vehicle, carla_map)
    vehicle.apply_control(control)


def full_brake(vehicle, carla_map=None):
    import carla
    control = carla.VehicleControl(throttle=0.0, brake=1.0)
    if carla_map is not None:
        control.steer = lane_steer(vehicle, carla_map)
    vehicle.apply_control(control)


class LeadScenario:
    """工况基类。"""

    name = 'base'
    label = '前车工况'

    def target_speed(self, t):
        raise NotImplementedError

    def extra_speed(self, t):
        """加塞车辆的期望速度；没有加塞时无意义。"""
        return 0.0

    def wants_cut_in(self, t):
        """是否需要（在本帧）生成加塞车辆。"""
        return False

    def mark_spawned(self):
        pass

    # -- 真正下发控制 --
    def update(self, t, dt, lead, extras=(), carla_map=None):
        drive_to_speed(lead, self.target_speed(t), carla_map)
        for vehicle in extras:
            drive_to_speed(vehicle, self.extra_speed(t), carla_map)


class ConstantSpeed(LeadScenario):
    """前车匀速行驶。"""

    name = 'constant'
    label = '前车匀速 60 km/h'

    def __init__(self, v0=DEFAULT_V0):
        self.v0 = float(v0)

    def target_speed(self, t):
        return self.v0


class SlowDown(LeadScenario):
    """前车缓慢减速，用来检验跟车距离是否收敛到期望值。"""

    name = 'decelerate'
    label = '前车缓慢减速（-1.5 m/s^2）'

    def __init__(self, v0=DEFAULT_V0, t_start=3.0, decel=1.5, v_min=5.0):
        self.v0 = float(v0)
        self.t_start = float(t_start)
        self.decel = float(decel)
        self.v_min = float(v_min)

    def target_speed(self, t):
        if t < self.t_start:
            return self.v0
        return max(self.v_min, self.v0 - self.decel * (t - self.t_start))


class EmergencyBrake(LeadScenario):
    """前车急刹（-6 m/s^2），用来检验自车的最小车距与 TTC。"""

    name = 'emergency'
    label = '前车急刹（-6 m/s^2）'

    def __init__(self, v0=DEFAULT_V0, t_start=3.0):
        self.v0 = float(v0)
        self.t_start = float(t_start)

    def target_speed(self, t):
        if t < self.t_start:
            return self.v0
        return 0.0

    def update(self, t, dt, lead, extras=(), carla_map=None):
        if t < self.t_start:
            drive_to_speed(lead, self.v0, carla_map)
        else:
            full_brake(lead, carla_map)
        for vehicle in extras:
            drive_to_speed(vehicle, self.extra_speed(t), carla_map)


class CutIn(LeadScenario):
    """前车匀速，另有旁车在 ``t_start`` 时刻加塞到自车正前方。

    本模块的加塞做了简化：直接在自车前方 ``cut_in_distance`` 米处生成一辆车速较低的车辆，
    而不是先跑一段横向变道过程。这样场景 100% 可复现，也便于统计最小车距。
    """

    name = 'cutin'
    label = '旁车加塞（前车 60 km/h，加塞车 30 km/h）'

    def __init__(self, v0=DEFAULT_V0, t_start=3.0,
                 cut_in_distance=15.0, cut_in_speed=8.33):
        self.v0 = float(v0)
        self.t_start = float(t_start)
        self.cut_in_distance = float(cut_in_distance)
        self.cut_in_speed = float(cut_in_speed)
        self._spawned = False

    def target_speed(self, t):
        return self.v0

    def extra_speed(self, t):
        return self.cut_in_speed

    def wants_cut_in(self, t):
        return (t >= self.t_start) and (not self._spawned)

    def mark_spawned(self):
        self._spawned = True


SCENARIOS = {
    'constant': ConstantSpeed,
    'decelerate': SlowDown,
    'emergency': EmergencyBrake,
    'cutin': CutIn,
}


def build(name):
    """按名字创建工况对象；``all`` 由调用方展开为多次运行。"""
    if name not in SCENARIOS:
        raise ValueError('未知工况：%s（可选：%s）' % (name, ', '.join(sorted(SCENARIOS))))
    return SCENARIOS[name]()


def all_names():
    return sorted(SCENARIOS)
