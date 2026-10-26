# -*- coding: utf-8 -*-
"""CARLA 连接与车辆操作的**精简独立**实现。

本文件从零写起、刻意不依赖仓库里其他模块（例如 ``carla_nn_car_following``），
这样本模块可以单独编译、单独提交，不会因为另一个 PR 是否合并而失效。

只包含动力性测试需要的东西：连接、同步步进、按车型生成车辆、读取车速/位姿、
第三人称跟随镜头、销毁并恢复世界设置。
"""

import math
import time

try:                                                    # CARLA 是可选依赖
    import carla
except ImportError:                                     # pragma: no cover
    carla = None

DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 2000
DEFAULT_TOWN = 'Town03'
DEFAULT_DT = 0.05            # 20 Hz，与真实台架采样率同量级


def require_carla():
    """在使用 CARLA 之前检查依赖是否可用，给出明确的安装提示。"""
    if carla is None:
        raise RuntimeError(
            '未找到 carla 包。请先安装与 Python 版本匹配的 CARLA 客户端：\n'
            '  python3 -m pip install --user carla==0.9.16\n'
            '（Ubuntu 22.04 + Python 3.10 对应 cp310 的 wheel）')


def speed_of(actor):
    """车辆**水平**速度（m/s，标量）。

    只取 x/y 分量：出生点通常离地面十几厘米，车辆生成后会下落、弹跳，
    竖直速度会污染三维模长（实测让"0→60 km/h"的起点凭空变成 1.47 m/s，
    即 5.3 km/h）。纵向动力性测试只关心平面速度，取水平分量才自洽。
    """
    velocity = actor.get_velocity()
    return math.sqrt(velocity.x ** 2 + velocity.y ** 2)


def location_of(actor):
    loc = actor.get_transform().location
    return (loc.x, loc.y, loc.z)


def steer_to_point(vehicle, target, gain=1.6):
    """纯追踪几何：朝世界坐标点 ``target`` 转向，返回归一化方向盘指令（-1~1）。

    ``alpha`` 是目标点相对车头方向的夹角（归一化到 ``[-pi, pi]``），
    转向量与夹角成正比并限幅。该约定与仓库既有的跟车模块一致
    （那套代码已在实际跟车中验证过）。
    """
    transform = vehicle.get_transform()
    location = transform.location
    yaw = math.radians(transform.rotation.yaw)
    alpha = math.atan2(target.y - location.y, target.x - location.x) - yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    return max(-1.0, min(1.0, gain * alpha))


def lane_steer(vehicle, carla_map, base_lookahead=8.0, gain=1.5):
    """沿当前车道行驶的方向盘指令（纯追踪）。

    **为什么必须有横向控制**：若只给油门/刹车不管方向，车辆会在几十米内驶出车道、
    撞上路沿或路边障碍后突然停住——实测这会让"制动距离 7.8 m、减速度 1.8g"
    这种荒谬数据混进结果。加速与制动测试全程都要保持车道。

    前视距离随车速增大（``base + 0.35 * v``），避免高速下画龙。
    """
    location = vehicle.get_transform().location
    lookahead = max(6.0, float(base_lookahead) + 0.35 * speed_of(vehicle))
    waypoint = carla_map.get_waypoint(location, project_to_road=True,
                                      lane_type=carla.LaneType.Driving)
    candidates = waypoint.next(lookahead)
    if not candidates:
        return 0.0
    return steer_to_point(vehicle, candidates[0].transform.location, gain)


def reset_vehicle(session, vehicle, transform, max_frames=60, rest_speed=0.05):
    """把车辆"搬回"起点、清零速度，并**等到真正静止**再开始测试。

    没有这一步时，加速测试跑完车已经向前开了几十米（甚至撞过），
    后面的制动/转向测试就都在一个被污染的状态下开始；
    而车辆刚生成时会从十几厘米高处落到地面并弹跳，不等静止就测量，
    起步速度会凭空多出 1.5 m/s。返回静止后等待的帧数。
    """
    vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
    vehicle.set_transform(transform)
    session.settle(3)
    frames = 3
    while frames < max_frames and speed_of(vehicle) > rest_speed:
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
        session.tick()
        frames += 1
    return frames


def set_spectator_follow(world, vehicle, distance=9.0, height=4.5, pitch=-15.0):
    """让 CARLA 窗口的观察者跟随车辆（否则录屏只能看到静止画面）。"""
    transform = vehicle.get_transform()
    forward = transform.get_forward_vector()
    location = carla.Location(x=transform.location.x - forward.x * distance,
                              y=transform.location.y - forward.y * distance,
                              z=transform.location.z + height)
    world.get_spectator().set_transform(
        carla.Transform(location, carla.Rotation(pitch=pitch,
                                                 yaw=transform.rotation.yaw)))


def _wheelbase_candidates(wheels, vehicle_transform):
    """给出轮距（轴距）的候选值与推荐顺序。

    实测（CARLA 0.9.16）：``WheelPhysicsControl.position`` 返回的是**世界坐标（cm）**，
    不是车身系偏移——例如特斯拉四个轮子的 x 是 ``[-567, -734, -556, -723]``，
    而车辆本身在 ``x = -640 cm`` 附近。

    判别方法（不能只看"数值是否合理"）：世界坐标的轮位必然**落在车辆附近**
    （几米以内），而车身系偏移的轮位减去车辆世界坐标后会是上千厘米的巨值。
    因此先用"轮位到车辆的距离"判定属于哪种解释，再算轴距：

    * 解释 B（世界坐标）：先减去车辆位置，再投影到车头方向；
    * 解释 A（车身系偏移）：直接取前轴与后轴的平均纵向差。
    """
    if len(wheels) < 4:
        return []
    xs = [w.position.x for w in wheels]
    ys = [w.position.y for w in wheels]
    front_x, rear_x = (xs[0] + xs[1]) / 2.0, (xs[2] + xs[3]) / 2.0
    body_relative = ('车身系偏差', abs(front_x - rear_x) / 100.0)

    if vehicle_transform is None:
        return [body_relative]

    origin_x = vehicle_transform.location.x * 100.0
    origin_y = vehicle_transform.location.y * 100.0
    max_distance = max(math.hypot(x - origin_x, y - origin_y) for x, y in zip(xs, ys))

    yaw = math.radians(vehicle_transform.rotation.yaw)
    longitudinal = [(x - origin_x) * math.cos(yaw) + (y - origin_y) * math.sin(yaw)
                    for x, y in zip(xs, ys)]
    front_l = (longitudinal[0] + longitudinal[1]) / 2.0
    rear_l = (longitudinal[2] + longitudinal[3]) / 2.0
    world_projected = ('世界坐标投影', abs(front_l - rear_l) / 100.0)

    # 轮位离车辆不超过 10 m -> 它们就是世界坐标；否则只能是车身系偏移
    if max_distance < 1000.0:
        return [world_projected, body_relative]
    return [body_relative, world_projected]


def parse_physics(physics, vehicle_transform=None):
    """从 CARLA ``VehiclePhysicsControl``（或同结构的假对象）解析轴距/转角/质量。

    单独抽成纯函数的原因：CARLA 不同版本里字段名与含义都变过
    （0.9.16 最大转角字段是 ``max_steer_angle``，轮位是**世界坐标**），
    这样解析逻辑可以用假对象**离线单测**，不必连 CARLA 才暴露问题。
    """
    wheels = list(getattr(physics, 'wheels', []) or [])

    wheelbase = 2.8                                            # 兜底值
    source = '兜底'
    for name, value in _wheelbase_candidates(wheels, vehicle_transform):
        if 1.5 <= value <= 5.0:                                # 真实车辆的轴距区间
            wheelbase, source = value, name
            break

    steer_values = []
    for wheel in wheels:
        value = getattr(wheel, 'max_steer_angle', None)
        if value is None:
            value = getattr(wheel, 'max_steer', None)
        if value:
            steer_values.append(float(value))
    max_steer_deg = max(steer_values) if steer_values else 70.0

    return {'wheelbase': wheelbase,
            'wheelbase_source': source,
            'max_steer_rad': math.radians(max_steer_deg),
            'max_steer_deg': max_steer_deg,
            'mass': float(getattr(physics, 'mass', 0.0) or 0.0),
            # 原始轮位，便于核对单位与顺序（CARLA 各版本字段含义可能不同）
            'raw_wheel_x': [round(w.position.x, 4) for w in wheels],
            'raw_wheel_y': [round(w.position.y, 4) for w in wheels]}


class CarlaSession:
    """一次测试运行所需的 CARLA 会话。"""

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=20.0):
        require_carla()
        self.host = host
        self.port = int(port)
        self.client = carla.Client(host, self.port)
        self.client.set_timeout(float(timeout))
        self.world = self.client.get_world()
        self.original_settings = None
        self.dt = DEFAULT_DT
        self.actors = []

    # ---------------- 世界设置 ----------------
    def ensure_town(self, town):
        if town and town not in self.world.get_map().name:
            print('[世界] 正在加载地图 %s ...' % town)
            self.world = self.client.load_world(town)
            self.original_settings = None          # 换图后设置需要重新记录
        print('[世界] 地图：%s' % self.world.get_map().name)

    def enable_sync(self, dt=DEFAULT_DT):
        """打开同步模式（测试需要固定步长，否则时间戳与帧号对不上）。

        与上一个模块一样做自愈：先把世界恢复成异步模式再记录"原始设置"，
        避免上一次异常退出留下的同步状态被当成原始设置。
        """
        if self.world.get_settings().synchronous_mode:
            print('[世界] 检测到残留的同步模式，已先恢复为异步模式')
            stuck = self.world.get_settings()
            stuck.synchronous_mode = False
            self.world.apply_settings(stuck)
        self.dt = float(dt)
        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.dt
        self.world.apply_settings(settings)
        print('[世界] 同步模式已开启，步长 %.3f s（%.0f Hz）' % (self.dt, 1.0 / self.dt))

    def restore_async(self):
        """恢复异步模式：避免程序结束后 CARLA 窗口因收不到 tick 而冻结。"""
        if self.original_settings is not None:
            try:
                self.world.apply_settings(self.original_settings)
            except Exception:                              # noqa: BLE001
                pass
            self.original_settings = None

    def tick(self):
        return self.world.tick()

    def settle(self, frames=2):
        """生成 actor 后先推进一步：位置数据要下一帧才到达客户端。"""
        for _ in range(int(frames)):
            self.world.tick()

    # ---------------- 车辆 ----------------
    def spawn_vehicle(self, blueprint_filter='vehicle.tesla.model3',
                      spawn_index=-1, role_name='ego_vehicle'):
        """生成一辆指定车型的车，返回 actor。

        ``spawn_index = -1`` 表示自动挑一个空余出生点。
        """
        blueprint = self.world.get_blueprint_library().find(blueprint_filter)
        if blueprint.has_attribute('role_name'):
            blueprint.set_attribute('role_name', role_name)

        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError('当前地图没有出生点')
        order = [int(spawn_index)] if spawn_index >= 0 else range(len(spawn_points))
        last_error = None
        for index in order:
            try:
                vehicle = self.world.spawn_actor(blueprint, spawn_points[index])
            except Exception as exc:                       # noqa: BLE001
                last_error = exc
                continue
            self.actors.append(vehicle)
            self.settle()
            print('[车辆] %s 已生成于出生点 #%d @ (%.1f, %.1f, %.1f)'
                  % (blueprint_filter, index, *location_of(vehicle)))
            return vehicle
        raise RuntimeError('无法生成车辆 %s（出生点可能被占用）：%s'
                           % (blueprint_filter, last_error))

    def vehicle_params(self, vehicle):
        """读取车辆物理参数（轴距、最大前轮转角、质量），见 :func:`parse_physics`。"""
        return parse_physics(vehicle.get_physics_control(), vehicle.get_transform())

    def destroy_actor(self, actor):
        if actor is None:
            return
        try:
            actor.destroy()
        except Exception:                                  # noqa: BLE001
            pass
        if actor in self.actors:
            self.actors.remove(actor)

    def close(self):
        """销毁本会话生成的所有对象并恢复世界设置。"""
        for actor in list(self.actors):
            self.destroy_actor(actor)
        self.restore_async()
        print('[清理] 已销毁车辆，世界设置已恢复')
