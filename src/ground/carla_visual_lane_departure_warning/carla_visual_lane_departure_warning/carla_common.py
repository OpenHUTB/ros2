"""CARLA 连接、自车生成与相机挂载。

只有这一层 import carla；纯几何都在 geometry.py 里。
"""
import queue

import numpy as np


def connect(host, port, town=None, timeout=10.0):
    """连接服务端；指定了地图且当前不是这张图时才切换。"""
    import carla

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    if town and not world.get_map().name.endswith(town):
        world = client.load_world(town)
    return client, world


def enable_sync(world, dt):
    """开同步模式，固定步长。"""
    settings = world.get_settings()
    if settings.synchronous_mode:
        print('注意：服务端已经是同步模式，可能是上次崩溃没恢复；'
              '建议重启 CarlaUE4.exe 再跑', flush=True)
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = dt
    world.apply_settings(settings)


def restore_async(world):
    """退出时恢复异步模式，否则服务端会卡在最后一帧。"""
    try:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
    except Exception:      # 服务端已经断开时不必再抛
        pass


def spawn_ego(world, client, blueprint_id='vehicle.tesla.model3',
              spawn_index=0, autopilot=True):
    """生成自车，默认接交通管理器跑自动驾驶。

    刚生成的车位姿是全零，要先 tick 一次再读位置。
    """
    import carla

    blueprints = world.get_blueprint_library().filter(blueprint_id)
    if not blueprints:
        raise RuntimeError('找不到蓝图 %s' % blueprint_id)
    blueprint = blueprints[0]

    points = world.get_map().get_spawn_points()
    if not points:
        raise RuntimeError('地图没有出生点')
    ego = world.try_spawn_actor(blueprint, points[spawn_index % len(points)])
    if ego is None:
        raise RuntimeError('出生点 %d 被占用，换一个 --spawn-index' % spawn_index)

    world.tick()          # 位姿生效
    if autopilot:
        # 用默认的自动驾驶，不单独取 traffic manager：
        # 同步模式下自己再 set_synchronous_mode(True) 会在退出时留下
        # 半开的 TM 状态，上一版就是死在清理阶段
        ego.set_autopilot(True)
    return ego


def attach_camera(world, ego, width=800, height=600, fov=90.0, height_m=1.6):
    """在自车车顶挂一台前视 RGB 相机，返回 (sensor, 图像队列)。"""
    import carla

    blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
    blueprint.set_attribute('image_size_x', str(width))
    blueprint.set_attribute('image_size_y', str(height))
    blueprint.set_attribute('fov', str(fov))
    blueprint.set_attribute('sensor_tick', '0')

    transform = carla.Transform(carla.Location(x=0.8, z=height_m))
    camera = world.spawn_actor(blueprint, transform, attach_to=ego)
    frames = queue.Queue()
    camera.listen(frames.put)
    return camera, frames


def follow_ego(world, ego, distance=9.0, height=4.0, pitch=-14.0):
    """把旁观者相机挪到自车斜后方，否则窗口里看不到车在动。"""
    import math as _math

    import carla

    transform = ego.get_transform()
    yaw = _math.radians(transform.rotation.yaw)
    location = carla.Location(
        x=transform.location.x - distance * _math.cos(yaw),
        y=transform.location.y - distance * _math.sin(yaw),
        z=transform.location.z + height)
    world.get_spectator().set_transform(
        carla.Transform(location,
                        carla.Rotation(pitch=pitch, yaw=transform.rotation.yaw)))


def image_to_array(image):
    """BGRA 原始缓冲转 RGB 的 (H, W, 3) 数组。"""
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape(image.height, image.width, 4)
    return array[:, :, :3][:, :, ::-1].copy()


def destroy_all(*actors):
    """销毁演员。

    顺序很关键：相机挂在自车上，销毁自车时相机会跟着没掉，
    所以先停掉相机回调，再只销毁自车——如果反过来先 destroy 相机，
    自车销毁时再去碰已销毁的子对象，CARLA 会直接抛 C++ 异常把进程打崩。
    """
    for actor in actors:
        if actor is None:
            continue
        try:
            if hasattr(actor, "stop"):
                actor.stop()
        except Exception:
            pass
    for actor in actors:
        if actor is None or hasattr(actor, "stop"):
            continue
        try:
            actor.destroy()
        except Exception:
            pass
