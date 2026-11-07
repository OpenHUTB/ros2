"""CARLA 连接、车辆与激光雷达工具函数（点云分割模块版）。

沿用仓库里已验证的惯例：

* ``carla`` 惰性导入，没装的时候给一句人话提示，离线测试完全不依赖它；
* 同步模式 + 固定步长，逐帧取数据，保证"普通点云"和"语义点云"是同一时刻的；
* 退出前调用 :func:`restore_async`，否则服务端会一直停在最后一帧。

本模块只做"和 CARLA 打交道"的事，特征、指标、网络都在别处，方便离线单测。
"""

import os
import queue

import numpy as np

try:
    import carla
except ImportError:      # 没装 carla 库时不影响离线部分
    carla = None

DEFAULT_HOST = os.environ.get('CARLA_HOST', '127.0.0.1')
DEFAULT_PORT = int(os.environ.get('CARLA_PORT', '2000'))
DEFAULT_TOWN = os.environ.get('CARLA_MAP', 'Town03')
EGO_BLUEPRINT = os.environ.get('CARLA_EGO_BP', 'vehicle.tesla.model3')

DT = 0.05      # 同步步长，20 Hz


def check_carla():
    """没装 CARLA 客户端时抛出带操作指引的异常。"""
    if carla is None:
        raise RuntimeError(
            '未找到 carla 模块。请安装 CARLA 0.9.16 的 Python 客户端：\n'
            '  pip3 install <CARLA>/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl')


def connect(host=DEFAULT_HOST, port=DEFAULT_PORT, town=DEFAULT_TOWN,
            sync=True, dt=DT, timeout=60.0, force_reload=False):
    """连接服务端并（按需）加载地图，返回 ``(client, world)``。"""
    check_carla()
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    try:
        current_world = client.get_world()
        current = current_world.get_map().name
        # 上一次运行如果崩在 restore_async 之前，服务端会停在同步模式，
        # 此时再 load_world 极容易卡死甚至把客户端拖崩，所以先恢复异步。
        if current_world.get_settings().synchronous_mode:
            settings = current_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            current_world.apply_settings(settings)
            print('[连接] 检测到服务端停在同步模式（上次退出异常），已恢复异步')
    except Exception:      # noqa: BLE001
        current = None

    def _same(name, want):
        return bool(name) and (name == want or name.endswith('/' + want))

    if current and _same(current, town) and not force_reload:
        world = client.get_world()
        print('[连接] 服务端已在地图 %s，跳过重载' % current)
    else:
        print('[连接] 正在加载地图 %s（当前 %s）...' % (town, current or '未知'))
        world = client.load_world(town)
        print('[连接] 地图加载完成')

    if sync:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
    return client, world


def restore_async(world):
    """恢复异步模式；脚本退出前必须调用，否则服务端会卡在最后一帧。"""
    check_carla()
    try:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        return True
    except Exception as exc:      # noqa: BLE001
        print('[清理] 恢复异步模式失败：%s' % exc)
        return False


def enable_sync_traffic_manager(client, port=8000):
    """让 TrafficManager 也按同步模式推进。

    世界是同步的、TM 还是异步的话，NPC 车不会跟着 world.tick() 走，
    采到的点云里"车"全是静止的，训练出来的模型对运动车辆没有泛化能力。
    """
    check_carla()
    tm = client.get_trafficmanager(port)
    tm.set_synchronous_mode(True)
    return tm



def spawn_ego(world, blueprint=EGO_BLUEPRINT, spawn_index=0, autopilot=True):
    """在指定出生点生成自车，默认打开自动驾驶以便采集多样化的场景。"""
    check_carla()
    library = world.get_blueprint_library()
    bp = library.filter(blueprint)[0]
    points = world.get_map().get_spawn_points()
    if not points:
        raise RuntimeError('地图没有可用出生点')
    index = int(spawn_index) % len(points)
    ego = world.try_spawn_actor(bp, points[index])
    if ego is None:
        raise RuntimeError('出生点 %d 被占用，换 --spawn-index 再试' % index)
    if autopilot:
        ego.set_autopilot(True)
    return ego


def spawn_traffic(world, client, count=12, seed=0, near=None, radius=(10.0, 45.0)):
    """撒一批自动驾驶的 NPC 车，让点云里有更多"车辆"样本。

    ``near`` 给自车时，优先挑自车周围 10~45 m 的空闲出生点。整张地图随机撒的
    话，NPC 车大多在几百米外，雷达根本扫不到——实测车辆点只占 0.17%，
    训练出来的分类器几乎见不到车。
    """
    check_carla()
    library = world.get_blueprint_library()
    tm = client.get_trafficmanager()
    tm.set_synchronous_mode(True)
    spawn_points = list(world.get_map().get_spawn_points())
    rng = np.random.default_rng(seed)

    if near is not None:
        origin = near.get_location()
        low, high = radius
        spawn_points = [sp for sp in spawn_points
                        if low <= sp.location.distance(origin) <= high]
        spawn_points.sort(key=lambda sp: sp.location.distance(origin))

    actors = []
    for spawn_point in spawn_points[:count]:
        bp = rng.choice(library.filter('vehicle.*'))
        npc = world.try_spawn_actor(bp, spawn_point)
        if npc is None:
            continue
        npc.set_autopilot(True)
        actors.append(npc)
    print('[交通] 已生成 %d 辆 NPC 车（%s）'
          % (len(actors), '自车附近' if near is not None else '全图随机'))
    return actors


def spawn_pedestrians(world, ego, count=8, seed=0, radii=(6.0, 10.0, 14.0)):
    """在自车周围撒一群静止行人，补上"行人"这一类样本。

    为什么绕自车放而不是用地图的导航随机点：随机点可能在地图另一头，
    自车雷达根本看不到，采出来的数据集里行人点数还是 0。
    绕自车多圈布点，保证至少有一部分落在量程内。

    行人不带 AI 控制器，站着不动：移动的行人更真实，但需要额外的
    控制器生命周期管理，崩溃面变大；对训练分类器来说，静止行人同样提供
    有效的"行人"标签，先保证数据里有这一类。
    """
    check_carla()
    carla_map = world.get_map()
    library = world.get_blueprint_library()
    rng = np.random.default_rng(seed)
    transform = ego.get_transform()
    yaw0 = np.deg2rad(transform.rotation.yaw)
    actors = []
    forward = np.array([np.cos(yaw0), np.sin(yaw0)])
    for radius in radii:
        for step in rng.permutation(16):
            if len(actors) >= count:
                break
            angle = yaw0 + 2.0 * np.pi * float(step) / 16.0
            location = carla.Location(
                x=transform.location.x + radius * np.cos(angle),
                y=transform.location.y + radius * np.sin(angle),
                z=transform.location.z + 0.5)
            waypoint = carla_map.get_waypoint(location, project_to_road=True,
                                              lane_type=carla.LaneType.Any)
            if waypoint is None:
                continue
            # 别把人放在自车正前方的车道里：采集时那会让"前方余量"永远只有几米，
            # 演示时自车会一直礼让不肯走，看起来像卡死。
            offset = np.array([waypoint.transform.location.x - transform.location.x,
                               waypoint.transform.location.y - transform.location.y])
            longitudinal = float(offset @ forward)
            lateral = float(offset @ np.array([-forward[1], forward[0]]))
            if 0.0 < longitudinal < 25.0 and abs(lateral) < 3.0:
                continue
            walker_bp = rng.choice(library.filter('walker.pedestrian.*'))
            spawn_transform = carla.Transform(
                carla.Location(x=waypoint.transform.location.x,
                               y=waypoint.transform.location.y,
                               z=waypoint.transform.location.z + 0.3),
                waypoint.transform.rotation)
            walker = world.try_spawn_actor(walker_bp, spawn_transform)
            if walker is not None:
                actors.append(walker)
    print('[行人] 已生成 %d 个行人' % len(actors))
    return actors


def _alive(actor):
    """安全地判断 actor 是否还活着；销毁过的 actor 访问属性会抛异常。"""
    try:
        return bool(actor.is_alive)
    except Exception:      # noqa: BLE001
        return False


def teardown(client, world, sensors=(), actors=()):
    """统一清理：先停传感器，再批量销毁 actor，最后恢复异步模式。

    为什么不逐个 ``destroy()``：TrafficManager 在同步模式下还持有这些车辆的引用，
    逐个销毁时客户端会抛 C++ 的 ``trying to operate on a destroyed actor``，
    这个异常跨不过 Python 边界，进程会直接 abort。先关掉 TM 的同步模式，
    再用 ``apply_batch_sync`` 一次性销毁，既快也不会踩到悬空引用。
    """
    check_carla()
    try:
        client.get_trafficmanager().set_synchronous_mode(False)
    except Exception:      # noqa: BLE001
        pass

    for sensor in list(sensors):
        try:
            sensor.stop()
        except Exception:      # noqa: BLE001
            pass

    targets = [a for a in list(sensors) + list(actors) if _alive(a)]
    if targets:
        try:
            # do_tick 传 False：这一批里全是"销毁"，没必要再推进世界，
            # 传 True 反而会让服务端在已销毁的 actor 上继续 tick。
            client.apply_batch_sync([carla.command.DestroyActor(a) for a in targets], False)
        except Exception:      # noqa: BLE001
            # 批量失败时才逐个退让；这里仍要检查存活，否则会重复销毁，
            # 服务端刷一屏 "already dead" 的警告。
            for actor in targets:
                try:
                    if _alive(actor):
                        actor.destroy()
                except Exception:      # noqa: BLE001
                    pass

    try:
        restore_async(world)
    except Exception:      # noqa: BLE001
        pass
    print('[清理] 传感器与车辆已销毁，世界已恢复异步模式')


def lidar_blueprint(world, semantic=False, lidar_range=60.0, channels=32,
                    points_per_second=600000, rotation_frequency=20.0,
                    upper_fov=10.0, lower_fov=-30.0, sensor_tick=0.0):
    """构造激光雷达 blueprint；普通与语义两台必须用**同一组参数**，否则点对不上。"""
    check_carla()
    library = world.get_blueprint_library()
    name = 'sensor.lidar.ray_cast_semantic' if semantic else 'sensor.lidar.ray_cast'
    bp = library.find(name)
    bp.set_attribute('range', str(lidar_range))
    bp.set_attribute('channels', str(channels))
    bp.set_attribute('points_per_second', str(points_per_second))
    bp.set_attribute('rotation_frequency', str(rotation_frequency))
    bp.set_attribute('upper_fov', str(upper_fov))
    bp.set_attribute('lower_fov', str(lower_fov))
    bp.set_attribute('sensor_tick', str(sensor_tick))
    return bp


def attach_lidar(world, ego, blueprint, height=2.4):
    """把雷达装到车顶中心，返回 ``(sensor, 队列)``。"""
    check_carla()
    transform = carla.Transform(carla.Location(x=0.0, z=height))
    sensor = world.spawn_actor(blueprint, transform, attach_to=ego)
    data_queue = queue.Queue()
    sensor.listen(data_queue.put)
    return sensor, data_queue


def _raw_buffer(data):
    """把测量对象里的二进制缓冲区取成 float32 数组。

    不同版本的 Python 客户端给点对象起的字段名不一样：官方 0.9.16 是
    ``detection.x``，但 PyPI 上重新打包的版本把坐标塞进了 ``detection.point``，
    直接按属性读会在采集现场炸掉。缓冲区布局是稳定的（每点 4 个或 6 个 float32），
    所以优先解析缓冲区，属性只作为兜底。
    """
    return np.frombuffer(data.raw_data, dtype=np.float32)


def _point_xyz(item):
    """兜底：按属性取坐标，兼容 ``item.x`` 与 ``item.point.x`` 两种写法。"""
    for owner in (item, getattr(item, 'point', None)):
        if owner is not None and hasattr(owner, 'x'):
            return float(owner.x), float(owner.y), float(owner.z)
    raise AttributeError('这个 CARLA 版本的点对象既没有 x 也没有 point.x，'
                         '请把 dir(point) 的输出反馈给维护者')


def points_from_raw(data):
    """普通点云 -> ``(N, 3)`` 数组（传感器坐标系，单位米）。

    普通激光雷达每点 4 个 float32：``x, y, z, cos_inc_angle``。
    """
    count = len(data)
    if count == 0:
        return np.zeros((0, 3), dtype=np.float32)
    buffer = _raw_buffer(data)
    if buffer.size == count * 4:
        return buffer.reshape(count, 4)[:, :3].astype(np.float32)
    return np.array([_point_xyz(p) for p in data], dtype=np.float32)


def semantic_arrays(data):
    """语义点云 -> ``((N, 3) 坐标, (N,) 标签, (N,) actor 编号)``。

    语义激光雷达每点 6 个 32 位：``x, y, z, cos_inc_angle`` 是 float32，
    ``object_idx, object_tag`` 是 uint32。同一段内存按 uint32 视图取最后两个即可，
    不需要依赖点对象的属性名。

    ``object_idx`` 是射线撞到的 actor 编号，可以用来精确核对"某个已知目标
    对应哪个标签"，比按半径筛点可靠得多（半径筛会被遮挡和位置误差骗）。
    """
    count = len(data)
    if count == 0:
        return (np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.int32),
                np.zeros(0, dtype=np.int32))
    buffer = _raw_buffer(data)
    if buffer.size == count * 6:
        points = buffer.reshape(count, 6)[:, :3].astype(np.float32)
        as_uint = np.ascontiguousarray(buffer).view(np.uint32).reshape(count, 6)
        tags = as_uint[:, 5].astype(np.int32)
        indices = as_uint[:, 4].astype(np.int32)
        return points, tags, indices
    points = np.array([_point_xyz(p) for p in data], dtype=np.float32)
    tags = np.array([int(getattr(p, 'object_tag')) for p in data], dtype=np.int32)
    indices = np.array([int(getattr(p, 'object_idx', -1)) for p in data], dtype=np.int32)
    return points, tags, indices


def points_from_semantic(data):
    """语义点云 -> ``((N, 3) 坐标, (N,) 语义标签)``。

    只要坐标和标签时用这个；需要知道每个点属于哪个 actor 时用 :func:`semantic_arrays`。
    """
    points, tags, _ = semantic_arrays(data)
    return points, tags


def align_pair(raw_points, sem_points, sem_tags, tolerance=0.05):
    """把普通点云和语义点云按坐标对齐。

    两台雷达参数完全相同、射线方向一致，正常情况下点数相等、顺序一致；但服务端
    两个传感器的采样时刻可能差一帧，点数和顺序就不保证。这里用"最近邻 + 距离阈值"
    兜底：普通点云里每个点找最近的语义点，超过 ``tolerance`` 米就不配对。

    返回 ``(points, tags, matched_mask)``。
    """
    if raw_points.shape[0] == 0 or sem_points.shape[0] == 0:
        return (np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.int32),
                np.zeros(0, dtype=bool))

    if raw_points.shape[0] == sem_points.shape[0] and \
            np.allclose(raw_points, sem_points, atol=1e-4):
        return raw_points, sem_tags, np.ones(raw_points.shape[0], dtype=bool)

    # 用 1 cm 网格做哈希加速：候选格子里找最近点
    cell = 0.05
    buckets = {}
    for i, (x, y, z) in enumerate(sem_points):
        key = (int(np.floor(x / cell)), int(np.floor(y / cell)), int(np.floor(z / cell)))
        buckets.setdefault(key, []).append(i)

    matched_points, matched_tags, mask = [], [], []
    for point in raw_points:
        key = tuple(int(np.floor(v / cell)) for v in point[:3])
        best, best_dist = -1, tolerance ** 2
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for idx in buckets.get((key[0] + dx, key[1] + dy, key[2] + dz), ()):
                        diff = sem_points[idx] - point
                        dist = float(diff[0] ** 2 + diff[1] ** 2 + diff[2] ** 2)
                        if dist < best_dist:
                            best, best_dist = idx, dist
        if best >= 0:
            matched_points.append(point)
            matched_tags.append(sem_tags[best])
            mask.append(True)
        else:
            matched_points.append(point)
            matched_tags.append(0)
            mask.append(False)

    return (np.array(matched_points, dtype=np.float32),
            np.array(matched_tags, dtype=np.int32),
            np.array(mask, dtype=bool))


def wait_tick(client, world, timeout=5.0):
    """推进一帧并返回新快照的帧号。"""
    check_carla()
    world.tick()
    return int(world.get_snapshot().frame)