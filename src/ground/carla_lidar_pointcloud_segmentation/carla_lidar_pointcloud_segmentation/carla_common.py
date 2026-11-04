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
        current = client.get_world().get_map().name
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


def spawn_traffic(world, client, count=12, seed=0):
    """撒一批自动驾驶的 NPC 车，让点云里有更多"车辆"样本。"""
    check_carla()
    library = world.get_blueprint_library()
    tm = client.get_trafficmanager()
    tm.set_synchronous_mode(True)
    spawn_points = world.get_map().get_spawn_points()
    rng = np.random.default_rng(seed)
    actors = []
    for index in rng.permutation(len(spawn_points))[:count]:
        bp = library.filter('vehicle.*')[0]
        npc = world.try_spawn_actor(bp, spawn_points[int(index)])
        if npc is None:
            continue
        npc.set_autopilot(True)
        actors.append(npc)
    print('[交通] 已生成 %d 辆 NPC 车' % len(actors))
    return actors


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


def points_from_raw(data):
    """普通点云 -> ``(N, 3)`` 数组（传感器坐标系，单位米）。"""
    return np.array([[p.x, p.y, p.z] for p in data], dtype=np.float32)


def points_from_semantic(data):
    """语义点云 -> ``((N, 3) 坐标, (N,) 语义标签)``。

    ``object_tag`` 的取值与 ``carla.CityObjectLabel`` 一致，映射在 :mod:`labels` 里做。
    """
    points = np.array([[p.x, p.y, p.z] for p in data], dtype=np.float32)
    tags = np.array([int(p.object_tag) for p in data], dtype=np.int32)
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