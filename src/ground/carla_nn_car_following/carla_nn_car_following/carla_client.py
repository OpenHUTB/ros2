# -*- coding: utf-8 -*-
"""CARLA 连接、场景生成与传感器封装的统一入口。

设计约定
--------
1. 本文件是整个模块中**唯一**直接操作 CARLA 世界的地方，便于在文档第 7 章做源码解析；
2. 所有 sensor 都保存为对象属性（如 ``self.sensor``），避免被 Python 垃圾回收
   ——这是 CARLA 最常见的坑：传感器句柄被回收后回调一次都不会触发；
3. 世界设置（同步模式）的原始值会保存在 ``self.original_settings``，退出时务必恢复，
   否则 CARLA 窗口会“卡住不动”。
"""

import math

try:
    import carla
except ImportError:      # 离线训练与单元测试不需要 CARLA，允许软导入
    carla = None

DEFAULT_TIMEOUT = 60.0   # 秒。默认超时太短，首次加载地图容易失败
DEFAULT_DT = 0.05        # 同步步长，20 Hz


# ----------------------------------------------------------------------------
# 通用小工具
# ----------------------------------------------------------------------------
def speed_of(actor):
    """返回车辆速度标量，单位 m/s。"""
    v = actor.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def distance_between(a, b):
    """两个 actor 参考点之间的欧氏距离，单位 m。"""
    return a.get_location().distance(b.get_location())


def _set_attr(blueprint, name, value):
    """安全设置蓝图属性：属性不存在时忽略（不同 CARLA 版本属性集略有差异）。"""
    try:
        blueprint.set_attribute(name, str(value))
    except Exception:
        pass


# ----------------------------------------------------------------------------
# 前向毫米波雷达
# ----------------------------------------------------------------------------
class FrontRadar:
    """前向毫米波雷达，输出正前方最靠近目标的距离与接近速度。

    **路面回波过滤**（非常重要）：雷达装在车头约 1.2 m 高处，若垂直视场角较宽，
    波束会打到路面上——波束下沿 5° 时路面回波出现在 ``1.2 / tan(5°) ≈ 13.7 m`` 处，
    比前车还近，于是"最短回波距离"测到的是路面（实测表现为雷达恒定读数 15 m 左右，
    而真实车距是 25 m，导致感知网络完全学不出来）。

    过滤方法：对每个回波按下式换算它相对路面的高度，只保留车身高度范围内的回波::

        h = 雷达安装高度 + 回波距离 * sin(回波仰角)

    车身（0.25~3 m）会被保留，路面（h ≈ 0）与天空会被剔除。

    **门限跟踪**（抑制远处杂波）：只做"高度过滤 + 取最近回波"仍然不够——
    车辆过弯时前车会偏出方位角窗口，此时"最近的合格回波"可能是几十米外的建筑，
    表现为雷达读数从 21 m 突然跳到 90 m。因此这里采用真实雷达的跟踪做法：
    用上一帧的距离与接近速度外推得到本帧的预测距离，只接受落在
    ``±gate`` 米范围内的回波；若没有回波落入范围，则**保持预测值**（模拟丢锁）。

    特征向量（与文档 3.3 节的 :math:`x_p` 对应，顺序一致）::

        [d_radar, azimuth, closing_rate, mean_depth, n_points, v_ego]
    """

    def __init__(self, world, vehicle, fov=30.0, radar_range=100.0,
                 azimuth_tolerance_deg=8.0, position=(2.0, 0.0, 1.2), tick=0.05,
                 vertical_fov=4.0, min_height=0.25, max_height=3.0,
                 gate=3.0, recovery_gate=10.0, min_distance=4.0,
                 max_closing_rate=30.0, max_lost_frames=25,
                 max_extrapolate_frames=40, max_extrapolation=5.0,
                 velocity_tolerance=8.0):
        self.azimuth_tolerance = math.radians(azimuth_tolerance_deg)
        self.range_max = float(radar_range)
        self.mount_height = float(position[2])
        self.min_height = float(min_height)
        self.max_height = float(max_height)
        self.gate = float(gate)
        self.recovery_gate = float(recovery_gate)        # 重捕获的宽松门限（米）
        self.tick = float(tick)
        self.min_distance = float(min_distance)          # 近距离盲区（车体自身长度）
        self.max_closing_rate = float(max_closing_rate)  # 接近速度限幅，防止单帧跳变带偏跟踪
        self.max_lost_frames = int(max_lost_frames)
        self.max_extrapolate_frames = int(max_extrapolate_frames)   # 丢锁期间最多匀速外推多少帧
        self.max_extrapolation = float(max_extrapolation)          # 外推距离上限（米）
        self.last_valid_depth = None                                # 最近一次有效测量值
        self.velocity_tolerance = float(velocity_tolerance)   # 多普勒：滤除静止杂波

        self.frame = 0
        self.min_depth = None
        self.min_azimuth = None
        self.mean_depth = None
        self.n_points = 0
        self.closing_rate = 0.0     # >0 表示与前车距离正在减小
        self.tracked_depth = None   # 门限跟踪的当前目标距离
        self.lost_frames = 0        # 连续丢锁帧数（诊断用）

        self._prev_depth = None

        bp = world.get_blueprint_library().find('sensor.other.radar')
        _set_attr(bp, 'role_name', 'front_radar')
        _set_attr(bp, 'horizontal_fov', fov)
        _set_attr(bp, 'vertical_fov', vertical_fov)
        _set_attr(bp, 'range', radar_range)
        _set_attr(bp, 'points_per_second', 1500)
        _set_attr(bp, 'sensor_tick', tick)

        transform = carla.Transform(carla.Location(x=position[0], y=position[1], z=position[2]))
        self.sensor = world.spawn_actor(
            bp, transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
        self.sensor.listen(self._on_data)

    # -- 回调 ---------------------------------------------------------------
    def reset_tracking(self):
        """目标切换时（例如加塞车辆出现）重新初始化门限跟踪。"""
        self.tracked_depth = None
        self.lost_frames = 0

    def _on_data(self, measurement):
        self.frame += 1

        # 第一步：方位角 + 高度 + 近距离盲区 + 多普勒速度过滤，得到候选回波
        candidates = []
        for detection in measurement:
            if abs(detection.azimuth) > self.azimuth_tolerance:
                continue
            if detection.depth < self.min_distance:
                continue                      # 车体自身长度范围内的回波不可信
            height = self.mount_height + detection.depth * math.sin(detection.altitude)
            if height < self.min_height or height > self.max_height:
                continue
            # 多普勒滤除静止杂波：护栏/墙面/路面等静止目标的径向速度约等于自车速度，
            # 而前车与自车速度接近、径向速度很小。没有这一条时，雷达一旦锁到护栏，
            # 就会沿护栏一帧帧"走"出去（实测读数被拖到 49.7 m）。
            if self.velocity_tolerance > 0.0 and abs(detection.velocity) > self.velocity_tolerance:
                continue
            candidates.append(detection)

        self.n_points = len(candidates)
        self.mean_depth = (sum(d.depth for d in candidates) / len(candidates)) if candidates else None

        # 第二步：门限跟踪
        # 注意符号：closing_rate > 0 表示正在接近，因此预测距离应**减小**
        chosen = None
        if candidates:
            if self.tracked_depth is None or self.lost_frames > self.max_lost_frames:
                # 新目标捕获，或者**长时间丢锁后重新捕获**：直接取最近的有效回波。
                #
                # 为什么必须重新捕获：如果航迹滑到很近的杂波上（例如 4 m），
                # ±gate 的门限只能接受 1~7 m 的回波，真正的前车（20 多米）永远进不来，
                # 就会**永久失锁**（实测 constant 工况雷达读数常年停在 4.0 m，
                # 而真实车距是 25 m，偏差均值 +19 m）。
                # 现在有了多普勒速度过滤（静止杂波已被剔除），重新捕获是安全的。
                chosen = min(candidates, key=lambda d: d.depth)
            else:
                predicted = self.tracked_depth - self.closing_rate * self.tick
                inside = [d for d in candidates if abs(d.depth - predicted) <= self.gate]
                if inside:
                    chosen = min(inside, key=lambda d: abs(d.depth - predicted))

        if chosen is not None:
            self.tracked_depth = chosen.depth
            self.min_depth = chosen.depth
            self.min_azimuth = chosen.azimuth
            self.last_valid_depth = chosen.depth
            self.lost_frames = 0
        else:
            # 丢锁：按"最后一次测到的接近速度"做匀速外推（真实雷达跟踪器的常规做法），
            # 但**外推距离限幅在 ±max_extrapolation 米**、并且最多持续 max_extrapolate_frames 帧。
            #
            # 为什么不能"保持上一次测量值不动"：弯道遮挡可能持续好几秒，期间真实车距
            # 可能从 28 m 收窄到 3 m，冻住的读数会让控制网络继续踩油门（实测最小 TTC 0.39 s）。
            # 为什么必须限幅：无限外推会把航迹一路推到量程下限并冻住。
            self.lost_frames += 1
            if (self.tracked_depth is not None and self.last_valid_depth is not None
                    and self.lost_frames <= self.max_extrapolate_frames):
                predicted = self.tracked_depth - self.closing_rate * self.tick
                self.tracked_depth = min(
                    max(predicted,
                        self.last_valid_depth - self.max_extrapolation,
                        self.min_distance),
                    self.last_valid_depth + self.max_extrapolation)
            self.min_depth = self.tracked_depth
            self.min_azimuth = 0.0

    # -- 对外接口 -----------------------------------------------------------
    def features(self, dt):
        """取出当前帧的特征量。

        接近速度用**相邻两帧最短距离的差分**计算，而不是直接使用
        ``detection.velocity``，这样可以完全避开雷达速度字段的符号约定问题。
        丢锁帧不更新接近速度（避免"保持值 → 差分 → 巨值"的反馈），并对其限幅。
        """
        depth = self.min_depth if self.min_depth is not None else self.range_max
        if self._prev_depth is None:
            self.closing_rate = 0.0
        elif self.lost_frames == 0:
            raw = (self._prev_depth - depth) / dt
            self.closing_rate = max(-self.max_closing_rate,
                                    min(self.max_closing_rate, raw))
        # 丢锁帧：保持上一次的接近速度不变
        if self.lost_frames == 0:
            self._prev_depth = depth

        return {
            'd_radar': depth,
            'azimuth': self.min_azimuth if self.min_azimuth is not None else 0.0,
            'closing_rate': self.closing_rate,
            'mean_depth': self.mean_depth if self.mean_depth is not None else self.range_max,
            'n_points': float(self.n_points),
        }

    def destroy(self):
        try:
            if self.sensor is not None:
                self.sensor.stop()
                self.sensor.destroy()
        except Exception:
            pass
        self.sensor = None


# ----------------------------------------------------------------------------
# 会话
# ----------------------------------------------------------------------------
class CarlaSession:
    """一次仿真的完整生命周期：连接 → 生成车辆 → 挂传感器 → 步进 → 清理。"""

    def __init__(self, host, port=2000, timeout=DEFAULT_TIMEOUT):
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)

        self.world = self.client.get_world()
        self.dt = DEFAULT_DT
        self.original_settings = None

        self.radar = None
        self._actors = []

    # ---------------- 世界 ----------------
    def load_town(self, town):
        """切换地图。Town03 是推荐的开发地图（直路多、路口少）。"""
        self.world = self.client.load_world(town)

    def set_weather(self, name):
        if not name:
            return
        params = getattr(carla.WeatherParameters, name, None)
        if params is None:
            raise ValueError('未定义的天气名称：%s' % name)
        self.world.set_weather(params)

    def enable_sync(self, dt=DEFAULT_DT):
        """打开同步模式。ACC 采样必须使用同步步进，否则时间戳与帧号对不上。

        **自愈处理**：先把世界强制恢复成异步模式，再保存"原始设置"。
        否则上一次运行异常退出（被 Ctrl+C 强杀、世界停留在同步模式）时，
        会把"同步"当成原始设置，本次结束又恢复成同步 ——
        表现为 **CARLA 窗口卡死不动**（服务器在等客户端发 tick，但没有客户端）。
        """
        if self.world.get_settings().synchronous_mode:
            print('[世界] 检测到上次运行残留的同步模式，已先恢复为异步模式')
            stuck = self.world.get_settings()
            stuck.synchronous_mode = False
            self.world.apply_settings(stuck)

        self.dt = float(dt)
        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.dt
        self.world.apply_settings(settings)

    def tick(self):
        self.world.tick()

    def _step_once(self):
        """推进一步：同步模式下 tick，异步模式下等待下一帧。"""
        if self.world.get_settings().synchronous_mode:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def settle(self, frames=1):
        """等待新生成的 actor 数据到达客户端。

        为什么必须这么做：CARLA 是客户端-服务器结构，刚 ``try_spawn_actor``
        出来的车辆在下一帧数据返回之前，``get_location()`` 会返回 ``(0, 0, 0)``。
        如果此时就用它去查车道点（``get_waypoint``）或算车距，
        计算结果会跑到地图原点附近去——表现为"前车被生成在几十米甚至几百米外"。
        """
        for _ in range(max(1, int(frames))):
            self._step_once()

    def restore_async(self):
        """恢复启动前的世界设置（异步模式）。

        同步模式下服务器**只有收到客户端 tick 才会推进一步**。如果程序跑完场景后
        就不再 tick、却仍留在同步模式，CARLA 窗口会一直冻结不动。
        因此场景结束时必须显式恢复异步模式。
        """
        if self.original_settings is not None:
            try:
                self.world.apply_settings(self.original_settings)
            except Exception:
                pass
            self.original_settings = None

    def current_map_name(self):
        return self.world.get_map().name

    # ---------------- 车辆 ----------------
    def _vehicle_blueprint(self, blueprint_id, role_name):
        bp_lib = self.world.get_blueprint_library()
        try:
            bp = bp_lib.find(blueprint_id)
        except Exception:
            bp = bp_lib.filter('vehicle.*')[0]
        _set_attr(bp, 'role_name', role_name)
        return bp

    def spawn_ego(self, blueprint_id='vehicle.tesla.model3', spawn_index=-1,
                  preflight=True, preflight_frames=30,
                  min_displacement=4.0, min_speed=5.0):
        """生成自车，并（默认）先做一次"这辆车能不能开动"的预检。

        为什么要预检：CARLA 地图里有些出生点会被墙、护栏或其他车辆卡住，
        车辆明明给了油门却几乎不动，跟车数据会完全失真。预检的做法是：
        在该出生点全油门推进 1.5 秒，位移与末速度都达标才算可用。

        **预检通过后会销毁试驾车并在原出生点重新生成一辆**，
        保证自车以"零速度、原始位置"进入正式场景——
        否则自车会带着预检时的残余速度冲向前车。

        :param spawn_index: >= 0 时只用该序号出生点（此时不做预检）；-1 表示自动挑选。
        """
        bp = self._vehicle_blueprint(blueprint_id, 'ego_vehicle')
        spawn_points = self.world.get_map().get_spawn_points()

        if spawn_index is not None and int(spawn_index) >= 0:
            index = int(spawn_index)
            if index >= len(spawn_points):
                raise RuntimeError('出生点序号 %d 超出范围（共 %d 个）' % (index, len(spawn_points)))
            actor = self.world.try_spawn_actor(bp, spawn_points[index])
            if actor is None:
                raise RuntimeError('出生点 #%d 已被占用' % index)
            self._actors.append(actor)
            self.settle()
            return actor

        candidates = list(enumerate(spawn_points))
        if preflight:
            candidates = candidates[:8]      # 最多试前 8 个，避免启动太慢

        for index, spawn_point in candidates:
            actor = self.world.try_spawn_actor(bp, spawn_point)
            if actor is None:
                continue
            self._actors.append(actor)
            self.settle()

            if not preflight:
                return actor

            start = actor.get_location()
            for _ in range(int(preflight_frames)):
                actor.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0, brake=0.0))
                self._step_once()
            moved = actor.get_location().distance(start)
            speed = speed_of(actor)

            if moved >= float(min_displacement) and speed >= float(min_speed):
                # 预检通过：销毁试驾车，在原出生点重新生成，保证零速度进场
                actor.destroy()
                self._actors.remove(actor)
                actor = self.world.try_spawn_actor(bp, spawn_point)
                if actor is None:
                    continue
                self._actors.append(actor)
                self.settle()
                print('[预检] 出生点 #%d 可用（%d 帧位移 %.1f m，末速 %.2f m/s），已重新就位'
                      % (index, int(preflight_frames), moved, speed))
                return actor

            print('[预检] 出生点 #%d 不可用（%d 帧位移 %.1f m，末速 %.2f m/s），换下一个'
                  % (index, int(preflight_frames), moved, speed))
            actor.destroy()
            self._actors.remove(actor)

        raise RuntimeError('找不到可以正常行驶的出生点；请用 diagnose_drive.py 检查地图与出生点，'
                           '或先用 --spawn-index 手动指定一个')

    def _is_free(self, location, radius):
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.get_location().distance(location) < radius:
                return False
        return True

    def spawn_in_lane(self, reference_actor, distance=25.0,
                      blueprint_id='vehicle.audi.a2', role_name='lead_vehicle',
                      free_radius=8.0):
        """在参考车辆所在车道的正前方 ``distance`` 米处生成一辆车。

        会依次尝试若干个前后偏移，跳过被占用的位置；全部失败返回 ``None``。
        """
        bp = self._vehicle_blueprint(blueprint_id, role_name)
        waypoint = self.world.get_map().get_waypoint(reference_actor.get_location())

        for offset in (0.0, 4.0, 8.0, -4.0, 12.0, -8.0):
            candidates = waypoint.next(distance + offset)
            if not candidates:
                continue
            transform = candidates[0].transform
            if not self._is_free(transform.location, free_radius):
                continue
            actor = self.world.try_spawn_actor(bp, transform)
            if actor is not None:
                self._actors.append(actor)
                self.settle()          # 同上：等位置数据到达
                return actor
        return None

    def attach_radar(self, vehicle, **kwargs):
        self.radar = FrontRadar(self.world, vehicle, **kwargs)
        return self.radar

    # ---------------- 清理 ----------------
    def close(self):
        """先销毁传感器，再销毁车辆，最后恢复世界设置（顺序不能反）。"""
        if self.radar is not None:
            self.radar.destroy()
            self.radar = None
        for actor in self._actors:
            try:
                actor.destroy()
            except Exception:
                pass
        self._actors = []
        if self.original_settings is not None:
            try:
                self.world.apply_settings(self.original_settings)
            except Exception:
                pass
            self.original_settings = None
