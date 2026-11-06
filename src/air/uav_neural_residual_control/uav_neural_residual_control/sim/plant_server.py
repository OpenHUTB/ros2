#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""带未建模效应的仿真服务端。

对外协议、传感器、控制状态机、TCP 并发全部复用底座 ``uav_mujoco_sim``，
唯一区别是机体换成 ``ResidualPlant``、且几何控制器使用**标称参数**——
这正是残差网络要补偿的模型偏差的来源。
"""
from __future__ import annotations

import argparse
from typing import Optional

import numpy as np

from uav_mujoco_sim.core import GeometricController, make_scene
from uav_mujoco_sim.core.sensors import CameraConfig, LidarConfig, LidarSensor, VisionSensors, ImuSensor
from uav_mujoco_sim.sim.server import DEFAULT_PORT, SimCore, SimServer

from ..core.disturbances import PlantConfig, WIND_LEVELS, PAYLOAD_LEVELS, make_plant
from ..core.plant import ResidualPlant

__all__ = ["ResidualSimCore", "ResidualSimServer", "serve", "main"]


class ResidualSimCore(SimCore):
    """SimCore 的机体替换版。

    ``_build`` 整段覆写而不是打补丁：底座的 ``_build`` 没有留出"换机体类"的
    注入口，但它只有机体那一行需要变。其余（相机/雷达/IMU/控制状态机/起飞逻辑）
    与底座逐行一致，改动点一眼可见。
    """

    def __init__(self, *args, plant: Optional[PlantConfig] = None, **kwargs):
        self.plant_cfg = plant if plant is not None else make_plant("heavy", "windy")
        super().__init__(*args, **kwargs)

    def _build(self, scene_name, seed, camera, lidar, imu_noise) -> None:
        sc = make_scene(scene_name, seed=seed, **self.scene_kwargs)
        self.quad = ResidualPlant(plant=self.plant_cfg, scene=sc, dt=self.dt,
                                  substeps=self.substeps)
        # 控制器只拿到标称参数：载荷与气动偏差必须由残差网络补
        self.ctrl = GeometricController(self.quad.nominal_params)

        self.camera_cfg = camera or CameraConfig(width=640, height=480, name="front_cam")
        self.lidar_cfg = lidar or LidarConfig()
        self.vision: Optional[VisionSensors] = None if self.headless else \
            VisionSensors(self.quad.model, self.camera_cfg)
        self.lidar = LidarSensor(self.quad.model, self.lidar_cfg)
        self.imu = ImuSensor(self.quad.model,
                             gyro_noise=0.002 if imu_noise else 0.0,
                             acc_noise=0.02 if imu_noise else 0.0)

        st = self.quad.state()
        from uav_mujoco_sim.sim.server import ControlState, VelocityCommander
        self.cstate = ControlState(goal=st.pos.copy(), ref_pos=st.pos.copy(),
                                   ref_yaw=st.yaw, status="HOVER")
        self.vcmd = VelocityCommander(self.cstate)
        self.cstate.ref_pos = st.pos.copy()
        self.cstate.ref_pos[2] = max(st.pos[2], 1.5)

    def _state_msg(self, want=("odom", "imu", "status")):
        msg = super()._state_msg(want)
        # 把真值扰动一并报出去，供桥接节点可视化"网络到底补了多少"
        q = self.quad
        T = float(np.sum(q.thrust_cmd))
        msg["plant"] = {
            "payload_kg": q.plant_cfg.payload.mass,
            "true_mass": q.true_mass,
            "nominal_mass": q.nominal_mass,
            "wind": q.plant_cfg.wind.vel.tolist(),
            "wind_speed": q.plant_cfg.wind.speed,
            "ground_scale": q.ground_scale,
            "disturbance_accel": q.true_disturbance_accel().tolist(),
            "thrust_total": T,
        }
        return msg


class ResidualSimServer(SimServer):
    """与底座 SimServer 等价，只是让 worker 构造 ResidualSimCore。

    ``_SimWorker.run`` 在**模块全局**里查 ``SimCore``，没有留出注入工厂的钩子；
    这里在启动 worker 之前临时替换那个全局名。``bootstrap()`` 会一直阻塞到
    SimCore 构造完成，所以还原时构造一定已经结束，不存在竞态。
    """

    def __init__(self, plant: Optional[PlantConfig] = None, **kwargs):
        import uav_mujoco_sim.sim.server as srv
        self.plant_cfg = plant
        old, srv.SimCore = srv.SimCore, ResidualSimCore
        try:
            super().__init__(plant=plant, **kwargs)
        finally:
            srv.SimCore = old


def serve(host: str = "0.0.0.0", port: int = DEFAULT_PORT,
          plant: Optional[PlantConfig] = None, **kwargs) -> None:
    from uav_mujoco_sim.sim.server import _Handler, _Server
    sim = ResidualSimServer(plant=plant, **kwargs)
    srv = _Server((host, port), _Handler)
    srv.sim = sim
    cfg = sim.core.plant_cfg
    print("[uav_neural_residual_control] 仿真服务已启动  %s:%d" % (host, port))
    print("  场景=%s  载荷=%.2f kg  真值质量=%.3f kg  标称质量=%.3f kg"
          % (sim.scene_name, cfg.payload.mass, sim.core.quad.true_mass,
             sim.core.quad.nominal_mass))
    print("  风：平均 %s m/s  阵风 σ=%.2f m/s  地面效应增益 %.0f%%"
          % (np.round(cfg.wind.mean, 2).tolist(), cfg.wind.gust,
             100 * cfg.ground_effect))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[uav_neural_residual_control] 正在退出…")
    finally:
        srv.shutdown()
        sim.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="带未建模效应的 MuJoCo 四旋翼仿真服务端")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--scene", default="open_ground")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--payload", default="heavy", choices=list(PAYLOAD_LEVELS))
    ap.add_argument("--wind", default="windy", choices=list(WIND_LEVELS))
    ap.add_argument("--ground-effect", type=float, default=0.12)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    a = ap.parse_args(argv)
    cam = CameraConfig(width=a.width, height=a.height, quality="fast")
    serve(a.host, a.port, scene=a.scene, seed=a.seed, camera=cam,
          headless=a.headless,
          plant=make_plant(a.payload, a.wind, seed=a.seed,
                           ground_effect=a.ground_effect))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
