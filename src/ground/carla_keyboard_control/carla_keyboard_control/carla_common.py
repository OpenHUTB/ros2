#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 0.9.16 公共封装（地面载具键盘运动控制）。

把 CARLA Python API 的常用操作集中于此，供 `carla_control_node`、
`keyboard_teleop_node` 与 `standalone` 主入口复用：

  - connect()            连接服务端并加载地图、开启同步模式
  - spawn_vehicle()      生成自车（ego vehicle）
  - make_rgb_camera()    挂载前视 RGB 相机
  - apply_control()      施加 (throttle, steer, brake, reverse) 控制
  - set_spectator_follow() 让 CARLA 大窗口镜头跟随自车
  - decode_image()       把 carla.Image 重排为 (H,W,3) RGB 数组

坐标与单位约定（CARLA 右手系）：+x 为东、+y 为南、+z 向上；
yaw 为绕 +z 轴逆时针角度（度）；速度单位为 m/s。
"""

import math

import numpy as np

try:
    import carla
except ImportError:  # 未安装 carla 客户端时给出友好提示
    carla = None

# ------------------------------------------------------------------ 默认参数
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 2000
DEFAULT_TOWN = "Town05"
DEFAULT_EGO_BLUEPRINT = "vehicle.tesla.model3"
# 默认出生点 (x, y, z, roll, pitch, yaw)：位于路面中央，车头沿路面方向
DEFAULT_SPAWN = (36.0, -5.0, 0.6, 0.0, 0.0, 0.0)
DT = 0.05  # 同步模式固定步长（秒）


def _check_carla():
    if carla is None:
        raise RuntimeError(
            "未找到 carla 模块。请安装 CARLA 0.9.16 的 Python 客户端：\n"
            "  pip3 install <CARLA>/PythonAPI/carla/dist/"
            "carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl"
        )


# ------------------------------------------------------------------ 连接与生成
def connect(host=DEFAULT_HOST, port=DEFAULT_PORT, town=DEFAULT_TOWN,
            sync=True, dt=DT):
    """连接 CARLA 服务端并加载地图，返回 (client, world)。

    打开同步模式（synchronous_mode）以便逐帧精确控制，并与传感器数据对齐。
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
    return client, world


def spawn_vehicle(world, blueprint=DEFAULT_EGO_BLUEPRINT, transform=DEFAULT_SPAWN):
    """生成自车，返回 (vehicle, transform)。出生点不可用时自动搜索最近路面点。"""
    _check_carla()
    bp = world.get_blueprint_library().find(blueprint)
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "hero")
    if bp.has_attribute("color"):
        bp.set_attribute("color", "255,0,0")

    tf = make_transform(transform)
    vehicle = world.try_spawn_actor(bp, tf)
    if vehicle is None:
        # 出生点不在路面：改用地图上最近的可行驶路点
        waypoint = world.get_map().get_waypoint(tf.location)
        tf = waypoint.transform
        tf.location.z += 0.6
        vehicle = world.try_spawn_actor(bp, tf)
    if vehicle is None:
        raise RuntimeError("无法生成自车，请检查 CARLA 服务端与地图设置。")
    return vehicle, tf


def make_rgb_camera(world, vehicle, callback, width=640, height=480, fov=90.0,
                    x=1.6, z=1.4, tick=False):
    """在自车车头挂载 RGB 相机，回调参数为 (H,W,3) uint8 RGB 数组。"""
    _check_carla()
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(width))
    bp.set_attribute("image_size_y", str(height))
    bp.set_attribute("fov", str(fov))
    tf = carla.Transform(
        carla.Location(x=float(x), z=float(z)),
        carla.Rotation(pitch=-15.0),
    )
    sensor = world.spawn_actor(bp, tf, attach_to=vehicle)
    if tick:
        world.tick()
    sensor.listen(lambda image: callback(decode_image(image)))
    return sensor


# ------------------------------------------------------------------ 控制
def apply_control(vehicle, throttle=0.0, steer=0.0, brake=0.0, reverse=False):
    """施加车辆控制。throttle/brake ∈ [0,1]，steer ∈ [-1,1]。"""
    _check_carla()
    control = carla.VehicleControl(
        throttle=float(np.clip(throttle, 0.0, 1.0)),
        steer=float(np.clip(steer, -1.0, 1.0)),
        brake=float(np.clip(brake, 0.0, 1.0)),
        reverse=bool(reverse),
        hand_brake=False,
    )
    vehicle.apply_control(control)
    return control


def set_spectator_follow(world, vehicle, dist=8.0, height=6.0):
    """让 CARLA 大窗口旁观镜头跟随自车后方，便于观察与录屏。"""
    _check_carla()
    tf = vehicle.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    fx, fy = math.cos(yaw), math.sin(yaw)   # 车头方向
    bx, by = -fy, fx                        # 侧向
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        carla.Location(
            x=tf.location.x - fx * dist + bx * 0.5,
            y=tf.location.y - fy * dist + by * 0.5,
            z=tf.location.z + height,
        ),
        carla.Rotation(pitch=-18.0, yaw=tf.rotation.yaw),
    ))


# ------------------------------------------------------------------ 状态读取
def get_speed(vehicle):
    """返回自车速率（m/s）。"""
    v = vehicle.get_velocity()
    return float(math.hypot(v.x, v.y))


def get_location(vehicle):
    """返回自车 (x, y, z)。"""
    loc = vehicle.get_location()
    return (loc.x, loc.y, loc.z)


def get_yaw(vehicle):
    """返回自车航向角（弧度）。"""
    return math.radians(vehicle.get_transform().rotation.yaw)


# ------------------------------------------------------------------ 内部工具
def make_transform(values):
    """由 (x, y, z, roll, pitch, yaw) 构造 carla.Transform（yaw 为角度）。"""
    x, y, z, roll, pitch, yaw = values
    return carla.Transform(
        carla.Location(x=float(x), y=float(y), z=float(z)),
        carla.Rotation(roll=float(roll), pitch=float(pitch), yaw=float(yaw)),
    )


def decode_image(image):
    """把 carla.Image 的 BGRA 原始数据重排为 (H,W,3) uint8 RGB 数组。"""
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = arr.reshape(image.height, image.width, 4)
    return arr[:, :, :3][:, :, ::-1].copy()
