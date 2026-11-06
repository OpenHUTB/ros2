"""在 CARLA / HUTB 模拟器里做一轮"附着系数 → 制动距离"扫掠试验。

每次试验：换一个附着系数 ``tire_friction`` → 加速到目标车速 → 全刹车到停 → 量制动距离。
``import carla`` 放在函数内部，没有安装客户端的机器仍可运行离线演示与单元测试。

单个附着系数失败（撞车、超时）不会丢弃整轮数据，失败原因会记录在 ``environment.failures``。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .analyze import BrakingPoint, SweepDataset
from .model import BrakingSpec

#: 默认连接参数、车型与扫掠点
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 2000
DEFAULT_VEHICLE = "vehicle.tesla.model3"
DEFAULT_FRICTIONS: tuple[float, ...] = (0.4, 0.6, 0.8, 1.0, 1.2, 1.6)


@dataclass(frozen=True)
class SimEnvironment:
    """模拟器环境信息，写进指标文件便于复现。"""

    map_name: str
    vehicle_id: str
    server_version: str
    tick_dt_s: float

    def as_dict(self) -> dict:
        return {
            "map": self.map_name,
            "vehicle": self.vehicle_id,
            "server_version": self.server_version,
            "tick_dt_s": self.tick_dt_s,
        }


def connect(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, timeout_s: float = 60.0):
    """连接模拟器；找不到客户端时给出可操作的提示。"""
    try:
        import carla  # noqa: F401  延迟导入
    except ImportError as error:  # pragma: no cover - 依赖外部环境
        raise RuntimeError(
            "未找到 carla 客户端：请安装 PythonAPI/carla/dist 下的 hutb-*.whl；"
            "离线演示用 python main.py demo"
        ) from error
    client = carla.Client(host, port)
    client.set_timeout(timeout_s)
    return client


def run_sweep(
    client,
    frictions=DEFAULT_FRICTIONS,
    vehicle_id: str = DEFAULT_VEHICLE,
    target_kmh: float = 50.0,
    dt: float = 0.05,
    spawn_index: int = 0,
    accel_limit_s: float = 25.0,
    brake_limit_s: float = 20.0,
) -> SweepDataset:
    """对每个附着系数跑一次 50→0 km/h 的全力制动，返回数据集。"""
    values = tuple(float(value) for value in frictions)
    if not values:
        raise ValueError("frictions must not be empty")
    if any(value <= 0.0 for value in values):
        raise ValueError("tire friction must be positive")

    world = client.get_world()
    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = float(dt)
    settings.no_rendering_mode = True
    world.apply_settings(settings)

    library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("当前地图没有可用的出生点")

    points: list[BrakingPoint] = []
    failures: list[str] = []
    try:
        for friction in values:
            try:
                points.append(
                    _run_single(
                        world, library, spawn_points[spawn_index], vehicle_id,
                        friction, target_kmh, dt, accel_limit_s, brake_limit_s,
                    )
                )
            except Exception as error:  # 单点失败不丢弃整轮数据
                failures.append(f"mu={friction}: {error}")
    finally:
        world.apply_settings(original)

    if not points:
        raise RuntimeError("所有附着系数都失败了：" + "；".join(failures))

    environment = SimEnvironment(
        map_name=world.get_map().name,
        vehicle_id=vehicle_id,
        server_version=str(client.get_server_version()),
        tick_dt_s=float(dt),
    ).as_dict()
    if failures:
        environment["failures"] = failures
    return SweepDataset(
        spec=BrakingSpec(reference_speed_kmh=float(target_kmh)),
        points=points,
        source="carla",
        environment=environment,
    )


def _run_single(
    world,
    library,
    spawn_point,
    vehicle_id: str,
    friction: float,
    target_kmh: float,
    dt: float,
    accel_limit_s: float,
    brake_limit_s: float,
) -> BrakingPoint:
    """跑单个附着系数：加速到目标车速后全力制动，返回一个数据点。"""
    import carla

    vehicle = world.spawn_actor(library.find(vehicle_id), spawn_point)
    try:
        physics = vehicle.get_physics_control()
        wheels = list(physics.wheels)
        for wheel in wheels:
            wheel.tire_friction = friction
        physics.wheels = wheels
        vehicle.apply_physics_control(physics)

        def speed_mps() -> float:
            velocity = vehicle.get_velocity()
            return float(np.hypot(velocity.x, velocity.y))

        accelerate = carla.VehicleControl(throttle=1.0, steer=0.0, brake=0.0)
        for _ in range(int(round(accel_limit_s / dt))):
            vehicle.apply_control(accelerate)
            world.tick()
            if speed_mps() >= target_kmh / 3.6:
                break

        entry_speed = speed_mps()
        if entry_speed < 0.8 * target_kmh / 3.6:
            raise RuntimeError(
                f"加速只到 {entry_speed * 3.6:.1f} km/h，可能撞到物体或道路太短"
            )

        start = vehicle.get_location()
        brake = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
        elapsed = 0.0
        for _ in range(int(round(brake_limit_s / dt))):
            vehicle.apply_control(brake)
            world.tick()
            elapsed += dt
            if speed_mps() < 0.3:
                break
        end = vehicle.get_location()
        distance = float(np.hypot(end.x - start.x, end.y - start.y))
        return BrakingPoint(
            tire_friction=friction,
            entry_speed_mps=entry_speed,
            distance_m=distance,
            duration_s=elapsed,
        )
    finally:
        try:
            vehicle.destroy()
        except RuntimeError:
            pass
