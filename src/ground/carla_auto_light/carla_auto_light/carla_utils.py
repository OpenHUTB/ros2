# -*- coding: utf-8 -*-
"""CARLA 连接与灯光相关的精简封装（不依赖仓库其它模块）。

只包含本模块需要的四件事：连接世界、同步步进、生成一辆车、切换天气与灯光。
"""

import math

try:
    import carla
except ImportError:                                     # pragma: no cover
    carla = None

DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 2000
DEFAULT_TOWN = 'Town03'
DEFAULT_DT = 0.05

# 天气预设：太阳高度角（度）与降水强度（CARLA 里是 0~100）
WEATHER_PRESETS = {
    'day': {'sun_altitude_angle': 60.0, 'precipitation': 0.0, 'cloudiness': 10.0,
            'label': '白天'},
    'dusk': {'sun_altitude_angle': 3.0, 'precipitation': 0.0, 'cloudiness': 40.0,
             'label': '黄昏'},
    'night': {'sun_altitude_angle': -12.0, 'precipitation': 0.0, 'cloudiness': 20.0,
              'label': '夜间'},
    'rain': {'sun_altitude_angle': 20.0, 'precipitation': 80.0, 'cloudiness': 90.0,
             'label': '暴雨'},
}


def require_carla():
    """使用 CARLA 前检查依赖，给出明确的安装提示。"""
    if carla is None:
        raise RuntimeError(
            '未找到 carla 包。请先安装与 Python 版本匹配的 CARLA 客户端：\n'
            '  python3 -m pip install --user -r requirements.txt')


def speed_of(actor):
    """水平速度（m/s）：只取 x/y，避免出生点落地时的竖直速度干扰。"""
    velocity = actor.get_velocity()
    return math.sqrt(velocity.x ** 2 + velocity.y ** 2)


def steer_to_point(vehicle, target, gain=1.6):
    """纯追踪：朝世界坐标点转向，返回归一化方向盘指令（-1~1）。"""
    transform = vehicle.get_transform()
    location = transform.location
    yaw = math.radians(transform.rotation.yaw)
    alpha = math.atan2(target.y - location.y, target.x - location.x) - yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    return max(-1.0, min(1.0, gain * alpha))


def lane_steer(vehicle, carla_map, base_lookahead=8.0, gain=1.5):
    """沿当前车道行驶的方向盘指令（纯追踪）。"""
    location = vehicle.get_transform().location
    lookahead = max(6.0, float(base_lookahead) + 0.35 * speed_of(vehicle))
    waypoint = carla_map.get_waypoint(location, project_to_road=True,
                                      lane_type=carla.LaneType.Driving)
    candidates = waypoint.next(lookahead)
    if not candidates:
        return 0.0
    return steer_to_point(vehicle, candidates[0].transform.location, gain)


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


def light_state_to_carla(state):
    """把决策字典转成 CARLA 的 ``VehicleLightState``。

    注意（实测踩过）：``set_light_state`` 只接受 ``carla.VehicleLightState`` 对象，
    **直接传 int 会抛** ``Boost.Python.ArgumentError``。所以位掩码要再用
    ``carla.VehicleLightState(...)`` 包一层。
    """
    flags = 0
    if state.get('low_beam'):
        flags |= int(carla.VehicleLightState.LowBeam)
    if state.get('fog'):
        flags |= int(carla.VehicleLightState.Fog)
    if state.get('left_blinker'):
        flags |= int(carla.VehicleLightState.LeftBlinker)
    if state.get('right_blinker'):
        flags |= int(carla.VehicleLightState.RightBlinker)
    if state.get('brake_light'):
        flags |= int(carla.VehicleLightState.Brake)
    return carla.VehicleLightState(flags)


class LightSession:
    """一次演示所需的 CARLA 会话。"""

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=20.0):
        require_carla()
        self.client = carla.Client(host, int(port))
        self.client.set_timeout(float(timeout))
        self.world = self.client.get_world()
        self.original_settings = None
        self.original_weather = None
        self.original_spectator = None
        self.actors = []

    def ensure_town(self, town=DEFAULT_TOWN):
        if town and town not in self.world.get_map().name:
            print('[世界] 正在加载地图 %s ...' % town)
            self.world = self.client.load_world(town)
            self.original_settings = None
        print('[世界] 地图：%s' % self.world.get_map().name)

    def enable_sync(self, dt=DEFAULT_DT):
        """打开同步模式（同时记录原始设置、原始天气与镜头位置，退出时还原）。"""
        settings = self.world.get_settings()
        if settings.synchronous_mode:
            print('[世界] 检测到残留的同步模式，先恢复为异步')
            settings.synchronous_mode = False
            self.world.apply_settings(settings)
        self.original_settings = self.world.get_settings()
        self.original_weather = self.world.get_weather()
        try:
            self.original_spectator = self.world.get_spectator().get_transform()
        except Exception:                                # noqa: BLE001
            self.original_spectator = None
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = float(dt)
        self.world.apply_settings(settings)
        self.dt = float(dt)
        print('[世界] 同步模式已开启，步长 %.3f s' % self.dt)

    def tick(self):
        return self.world.tick()

    def settle(self, frames=2):
        for _ in range(int(frames)):
            self.world.tick()

    def set_weather(self, preset):
        """应用天气预设，返回实际的 (太阳高度角, 降水强度)。"""
        weather = self.world.get_weather()
        weather.sun_altitude_angle = float(preset['sun_altitude_angle'])
        weather.precipitation = float(preset['precipitation'])
        weather.cloudiness = float(preset.get('cloudiness', 10.0))
        self.world.set_weather(weather)
        self.settle(3)                      # 让天气与光照生效
        actual = self.world.get_weather()
        return actual.sun_altitude_angle, actual.precipitation

    def spawn_vehicle(self, blueprint_filter='vehicle.tesla.model3', spawn_index=-1):
        blueprint = self.world.get_blueprint_library().find(blueprint_filter)
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError('当前地图没有出生点')
        order = [int(spawn_index)] if spawn_index >= 0 else range(len(spawn_points))
        last_error = None
        for index in order:
            try:
                vehicle = self.world.spawn_actor(blueprint, spawn_points[index])
            except Exception as exc:                     # noqa: BLE001
                last_error = exc
                continue
            self.actors.append(vehicle)
            self.settle(2)
            loc = vehicle.get_transform().location
            print('[车辆] %s 已生成于出生点 #%d @ (%.1f, %.1f, %.1f)'
                  % (blueprint_filter, index, loc.x, loc.y, loc.z))
            return vehicle
        raise RuntimeError('无法生成车辆 %s：%s' % (blueprint_filter, last_error))

    def apply_lights(self, vehicle, state):
        """把灯光状态写进车辆。"""
        vehicle.set_light_state(light_state_to_carla(state))

    def close(self):
        """销毁车辆，并还原天气、镜头位置与世界设置。"""
        for actor in list(self.actors):
            try:
                actor.destroy()
            except Exception:                            # noqa: BLE001
                pass
        self.actors = []
        if self.original_weather is not None:
            try:
                self.world.set_weather(self.original_weather)
            except Exception:                            # noqa: BLE001
                pass
        if self.original_spectator is not None:
            # 跟随镜头会把观察者挪到车后，退出时放回原处，否则下次打开 CARLA
            # 视角会停在上一次演示的终点
            try:
                self.world.get_spectator().set_transform(self.original_spectator)
            except Exception:                            # noqa: BLE001
                pass
        if self.original_settings is not None:
            try:
                self.world.apply_settings(self.original_settings)
            except Exception:                            # noqa: BLE001
                pass
        print('[清理] 已销毁车辆，天气、镜头位置与世界设置已恢复')
