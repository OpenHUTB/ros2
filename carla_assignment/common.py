#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 0.9.16 无人车四次作业 · 公共工具。

封装 CARLA Python API 的常用操作，让每个作业的 main.py 保持简短清晰：

  - connect()         连接并切换/加载地图（同步模式）
  - spawn_vehicle()   在指定出生点生成自车（ego vehicle）
  - CameraManager()   管理 RGB 相机 / 深度相机 / 雷达，回调得到数据
  - apply_control()   施加车辆控制（油门/转向/刹车）
  - get_vehicle_state 读取速度/朝向
  - 常量：HOST / PORT / TOWN / 出生点

适配：CARLA 0.9.16（Python API）。
说明：本模块为纯 Python 实现，不依赖 ROS；作业自带的 *launch 仅用于按
教学要求以 roslaunch/ros2 launch 方式启动入口脚本。
"""

import os

import numpy as np

try:
    import carla
except ImportError as _e:  # 未装 carla 库时给出友好提示
    carla = None

# ---------------------------------------------------------------- 常量与参数
DEFAULT_HOST = os.environ.get("CARLA_HOST", "127.0.0.1")
DEFAULT_PORT = int(os.environ.get("CARLA_PORT", "2000"))
# 地图：Town05 有较多可走道路与场景，适合轨迹/建图/端到端；可改 Town01/Town03。
DEFAULT_MAP = os.environ.get("CARLA_MAP", "Town05")

# 推荐的自车蓝图（0.9.16 中均有）
EGO_BLUEPRINT = os.environ.get("CARLA_EGO_BP", "vehicle.tesla.model3")

# 默认出生点：位于某条路面中央，yaw 沿路面方向
DEFAULT_SPAWN = (36.0, -5.0, 0.6, 0.0, 0.0, 0.0)  # (x, y, z, roll, pitch, yaw)

DT = 0.05  # 同步模式固定步长（秒）


# ------------------------------------------------------------------ 工具函数
def _check_carla():
    if carla is None:
        raise RuntimeError(
            "未找到 carla 模块。请安装 CARLA 0.9.16 的 Python egg，或在 CARLA 提供"
            "的 PythonAPI/carla/dist 中添加其路径："
            "sys.path.append('<CARLA>/PythonAPI/carla/dist/<python-version>.whl')"
        )


def connect(host=DEFAULT_HOST, port=DEFAULT_PORT, town=DEFAULT_MAP,
           sync=True, dt=DT):
    """连接 CARLA 服务端并加载地图，返回 (client, world)。

    - 若 server 端口已有一个世界，load_world 会加载 town（同步阻塞）。
    - 开启同步模式（synchronous_mode），便于逐帧精确控制与传感器对齐。
    """
    _check_carla()
    client = carla.Client(host, port)
    client.set_timeout(20.0)
    world = client.load_world(town)
    if sync:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
    # 清空已存在的天气、车辆以外的无关 actor 保留，便于稳定复现
    return client, world


def reset_sync(world, dt=DT):
    """把世界切回同步模式（多次 connect 后保险用）。"""
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = dt
    world.apply_settings(settings)


def spawn_vehicle(world, blueprint=EGO_BLUEPRINT, transform=DEFAULT_SPAWN):
    """按蓝图生成自车，返回 (vehicle, transform)。出生点不可用时自动搜索最近路面点。"""
    _check_carla()
    bp_lib = world.get_blueprint_library()
    bp = bp_lib.find(blueprint)
    # 关闭车轮物理碰撞带来的扰动（教学演示用，便于稳定控制）
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "hero")
    if bp.has_attribute("color"):
        bp.set_attribute("color", "255,0,0")

    tf = _make_transform(transform)
    vehicle = world.try_spawn_actor(bp, tf)
    if vehicle is None:
        # 出生点不在路面：找最近可行走路点
        m = world.get_map()
        wp = m.get_waypoint(tf.location)
        tf = wp.transform
        tf.location.z += 0.6
        vehicle = world.try_spawn_actor(bp, tf)
    if vehicle is None:
        raise RuntimeError("无法生成自车，请检查 CARLA 服务端与地图。")
    return vehicle, tf


def make_rgb_camera(world, vehicle, callback, blueprint_filter="sensor.camera.rgb",
                    width=640, height=480, fov=90, x=1.6, z=1.4, yall=0.0, tick=False):
    """在当前车辆上挂一个 RGB 相机，返回 sensor。

    callback(rgb): 收到 (H,W,3) uint8 RGB 数组（由 _decode_image 重排 BGRA->RGB 得到）。
    """
    _check_carla()
    bp = world.get_blueprint_library().find(blueprint_filter)
    bp.set_attribute("image_size_x", str(width))
    bp.set_attribute("image_size_y", str(height))
    bp.set_attribute("fov", str(fov))
    tf = _relative_transform(x=x, z=z, yall=yall)
    sensor = world.spawn_actor(bp, tf, attach_to=vehicle)
    if tick:
        _tick_once(world)
    sensor.listen(lambda img: callback(_decode_image(img)))
    return sensor


def make_lidar(world, vehicle, callback, points=12000, range=40.0,
               upper_fov=10.0, lower_fov=-30.0, tick=False):
    """在车辆顶部挂一个射线雷达（RayCast LiDAR）。

    callback(pc): 收到 carla.LidarMeasurement。`pc.data` 为 (N,4) 数组，
      每行为 (x,y,z, intensity)，坐标为**雷达传感器坐标系**（右手系：x 前，y 左，z 上）。
    """
    _check_carla()
    bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
    bp.set_attribute("channels", "32")
    bp.set_attribute("points_per_second", str(int(points * 10)))
    bp.set_attribute("range", str(range))
    bp.set_attribute("rotation_frequency", str(int(1.0 / DT)))
    bp.set_attribute("upper_fov", str(upper_fov))
    bp.set_attribute("lower_fov", str(lower_fov))
    tf = _relative_transform(x=1.0, z=2.8)
    sensor = world.spawn_actor(bp, tf, attach_to=vehicle)
    if tick:
        _tick_once(world)
    sensor.listen(lambda pc: callback(pc))
    return sensor


def make_depth_camera(world, vehicle, callback, width=512, height=256, fov=90,
                      x=1.6, z=1.4, tick=False):
    """挂场景深度相机（绿色编码归一化深度）。callback(rgb) 其中 rgb 为 (H,W,3) uint8
    （灰度绿图的 3 通道重复）。
    """
    return make_rgb_camera(world, vehicle, callback,
                          blueprint_filter="sensor.camera.depth",
                          width=width, height=height, fov=fov,
                          x=x, z=z, tick=tick)


def apply_control(vehicle, throttle=0.0, steer=0.0, brake=0.0, reverse=False):
    """给车辆施加（油门, 转向, 刹车）控制。各值范围：throttle/brake∈[0,1]，steer∈[-1,1]。"""
    vc = carla.VehicleControl(
        throttle=float(np.clip(throttle, 0.0, 1.0)),
        steer=float(np.clip(steer, -1.0, 1.0)),
        brake=float(np.clip(brake, 0.0, 1.0)),
        reverse=bool(reverse),
        hand_brake=False,
    )
    vehicle.apply_control(vc)
    return vc


def get_speed(vehicle):
    """返回车辆前进速度（m/s），基于 Carla 的纵向速度投影到航向。"""
    v = vehicle.get_velocity()
    return float(np.hypot(v.x, v.y))


def get_location(vehicle):
    """返回 (x, y)。"""
    loc = vehicle.get_location()
    return (loc.x, loc.y)


def get_yaw(vehicle):
    """返回车辆当前的航向角 yaw（弧度，CARLA 为北向顺时针，转成 x 轴向逆时针约定）。

    CARLA 默认以 x 为东、y 为南、z 向上；其 yaw 是相对 +x 逆时针（右手系）。
    这里直接返回 transform.rotation.yaw 转弧度，并让上层统一用 atan2(dy,dx) 约定。
    """
    rot = vehicle.get_transform().rotation
    import math
    return math.radians(rot.yaw)


# ------------------------------------------------------------------ 内部工具
def _make_transform(values):
    """由 (x,y,z,roll,pitch,yaw) 构造 carla.Transform，yaw 用角度。"""
    x, y, z, roll, pitch, yaw = values
    loc = carla.Location(x=float(x), y=float(y), z=float(z))
    rot = carla.Rotation(roll=float(roll), pitch=float(pitch), yaw=float(yaw))
    return carla.Transform(loc, rot)


def _relative_transform(x=0.0, z=1.4, yall=0.0):
    """构造一个相对自车的传感器 Transform（CARLA 相机默认朝向 +x，即车头前方）。"""
    return carla.Transform(
        carla.Location(x=float(x), z=float(z)),
        carla.Rotation(pitch=-15.0, yaw=float(yall)),
    )


def _tick_once(world):
    world.tick()


def set_spectator_follow(world, vehicle, dist=8.0, height=6.0):
    """让 CARLA 大窗口（spectator）自动跟随自车后方，方便观察/录屏。

    每帧调用，把 spectator 摆到车辆后方 dist 米、height 米高的位置并看向车辆。
    车辆默认 yaw 0 时车头朝 +x；后方位移取反方向。
    """
    _check_carla()
    import math
    tf = vehicle.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    # 车头单位方向（CARLA +x 为东，yaw 逆时针）
    fx, fy = math.cos(yaw), math.sin(yaw)
    rx, ry = -fx, -fy                 # 车尾方向
    bx, by = -fy, fx                  # 侧向垂直方向
    # 车尾后方 + 侧向一点，抬高
    ex = tf.location.x + rx * dist + bx * 0.5
    ey = tf.location.y + ry * dist + by * 0.5
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        carla.Location(x=ex, y=ey, z=tf.location.z + height),
        carla.Rotation(pitch=-18.0, yaw=tf.rotation.yaw),
    ))


def _decode_image(img):
    """把 carla.Image 重排为 (H,W,3) uint8 RGB 数组并返回。

    CARLA 相机原始数据为 BGRA（raw_data），需重排为 RGB。
    不修改原对象（避免在 Boost.Python 对象上写自定义属性引发 AttributeError）。
    """
    arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)
    bgr = arr[:, :, :3]
    rgb = bgr[:, :, ::-1]
    return rgb


def parse_lidar_pc(pc):
    """从 carla.LidarMeasurement 取 (N,4) 点云（传感器系 x 前 y 左 z 上）。"""
    data = np.frombuffer(pc.raw_data, dtype=np.float32).reshape(-1, 4)
    return data
