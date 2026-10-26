# -*- coding: utf-8 -*-
"""飞行仿真运行器：把「场景 + 故障 + 控制器」跑成一条 :class:`FlightResult`。

设计约束
--------
1. **不修改底座**。只用 ``uav_mujoco_sim`` 的公开 API：``Quadrotor`` 负责动力学，
   ``GeometricController.act()`` 负责出桨推力，本模块只在两者之间插入故障算子。
2. **控制器可替换**。所有控制器统一实现
   ``act(quad, p_d, v_d, a_d, yaw_d) -> np.ndarray(4)``，因此基线几何控制器、
   容错分配器、神经网络策略可以同台对比，换控制器不改仿真代码。
3. **记录完整轨迹**。即便坠机也跑满时长，便于对比"坠得多快/多深"，
   而不是只看终局。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import numpy as np

from ._compat import BaseAPI, load_base
from .fault import FaultInjector, FaultSpec
from .metrics import FlightResult
from .scenario import FlightCase

#: 控制器工厂：给定飞行器实体，返回带 ``act`` 方法的控制器
ControllerFactory = Callable[[object], object]


@dataclass
class RunConfig:
    """一次仿真的参数（默认值与底座保持一致）。"""

    #: 仿真总时长，秒
    duration: float = 8.0
    #: 物理步长，秒
    dt: float = 0.01
    #: MuJoCo 每步子步数
    substeps: int = 4
    #: 悬停目标点（世界系 ENU），米
    target: Tuple[float, float, float] = (0.0, 0.0, 1.5)
    #: 目标偏航角，弧度
    yaw_d: float = 0.0
    #: 是否采样 RGB 帧（录 gif 用；开着会慢 5~10 倍）
    record_frames: bool = False
    #: 每多少个仿真步采样一帧
    frame_every: int = 10
    #: 采样分辨率
    frame_width: int = 320
    frame_height: int = 240
    #: 渲染质量："fast" 关阴影/反射（感知用）、"full" 全开（录像用）
    quality: str = "fast"
    #: 采样相机：
    #: ``fpv``   = 机载前视相机（与 ``/camera/image_raw`` 同一视角，用于感知类实验）；
    #: ``track`` = 外部跟拍相机（演示动图更直观：能看见机身姿态和坠地）。
    camera: str = "fpv"


class _TrackCamera:
    """外部跟拍相机：始终看向飞行器，保持固定视角与距离。

    为什么不用机载相机录演示：悬停时前视相机里只有地面和天空，看不出"坠机"，
    也看不出机身翻转。跟拍相机则能同时看到姿态、高度变化和地面。
    它只读 ``data.qpos[:3]``，不与底座内部状态耦合。
    """

    def __init__(self, model, width: int, height: int, distance: float = 2.6,
                 azimuth: float = 135.0, elevation: float = -14.0):
        import mujoco

        self._mj = mujoco
        self._renderer = mujoco.Renderer(model, height, width)
        # 关掉地面反射：反射会在地面上多出一个"幽灵"机体，演示时容易误读成两架
        self._renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        self._cam = mujoco.MjvCamera()
        self._cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self._cam.distance = distance
        self._cam.azimuth = azimuth
        self._cam.elevation = elevation
        self._cam.lookat[:] = [0.0, 0.0, 1.0]

    def rgb(self, data) -> np.ndarray:  # 与 VisionSensors.rgb(data) 同签名，可互换
        pos = np.asarray(data.qpos[:3], dtype=float)
        # 高度限幅在 [0, 2]：镜头不钻到地面以下（那会拍出一片全黑），
        # 飞行器坠地后从画面下缘穿出，配合左上角的高度读数仍然读得懂。
        self._cam.lookat[:] = [float(pos[0]), float(pos[1]),
                              float(np.clip(pos[2], 0.0, 2.0))]
        self._renderer.update_scene(data, camera=self._cam)
        return self._renderer.render()

    def close(self) -> None:
        self._renderer.close()



def _default_controller_factory(api: BaseAPI) -> ControllerFactory:
    """基线：底座自带的几何控制器（不含任何容错逻辑）。"""
    return lambda quad: api.GeometricController(quad.params)


def _make_scene(api: BaseAPI, name: str, seed: int):
    """构造场景，``seed`` 只对随机场景有意义，不接受则退回默认场景。"""
    try:
        return api.make_scene(name, seed=seed)
    except TypeError:
        return api.make_scene(name)


def run_case(
    case: FlightCase,
    cfg: Optional[RunConfig] = None,
    controller_factory: Optional[ControllerFactory] = None,
    spec: Optional[FaultSpec] = None,
    api: Optional[BaseAPI] = None,
) -> FlightResult:
    """跑一个飞行用例。

    Parameters
    ----------
    case:
        物理场景 + 故障场景 + 随机种子的组合。
    cfg:
        仿真参数，``None`` 时用默认值（8 s、dt=0.01）。
    controller_factory:
        控制器工厂；``None`` 时用基线几何控制器。
    spec:
        直接指定故障规格，覆盖 ``case.build_spec()``（做故障参数扫描时用）。
    api:
        底座符号集合，多次调用时可传入以免重复解析路径。

    Returns
    -------
    FlightResult
        完整轨迹与指标。
    """
    cfg = cfg or RunConfig()
    api = api or load_base()
    factory = controller_factory or _default_controller_factory(api)

    scene = _make_scene(api, case.physical_scene, case.seed)
    quad = api.Quadrotor(scene=scene, dt=cfg.dt, substeps=cfg.substeps)
    target = np.asarray(cfg.target, dtype=float)
    quad.reset(pos=target)

    ctrl = factory(quad)
    fault_spec = spec if spec is not None else case.build_spec()
    injector = FaultInjector(fault_spec, max_thrust=float(quad.params.max_thrust))

    visual = None
    if cfg.record_frames:
        if cfg.camera == "track":
            visual = _TrackCamera(quad.model, cfg.frame_width, cfg.frame_height)
        elif cfg.camera == "fpv":
            cam = api.CameraConfig(
                width=cfg.frame_width,
                height=cfg.frame_height,
                quality=cfg.quality,
            )
            visual = api.VisionSensors(quad.model, cam)
        else:
            raise ValueError(f"未知相机 {cfg.camera!r}，可选：'fpv' / 'track'")

    n_steps = int(round(cfg.duration / cfg.dt))
    t_hist = np.empty(n_steps)
    pos = np.empty((n_steps, 3))
    vel = np.empty((n_steps, 3))
    omega = np.empty((n_steps, 3))
    euler = np.empty((n_steps, 3))
    thrusts = np.empty((n_steps, 4))
    desired = np.empty((n_steps, 4))
    health = np.empty((n_steps, 4))
    frames: List[np.ndarray] = []

    v_d = np.zeros(3)
    a_d = np.zeros(3)
    max_thrust = float(quad.params.max_thrust)

    for k in range(n_steps):
        t = k * cfg.dt
        u_cmd = np.asarray(ctrl.act(quad, target, v_d, a_d, cfg.yaw_d), dtype=float)
        u_act = injector.apply(u_cmd, t)
        quad.apply_rotor_thrust(u_act)
        quad.step(1)
        st = quad.state()

        t_hist[k] = t
        pos[k] = st.pos
        vel[k] = st.vel
        omega[k] = st.omega
        euler[k] = np.degrees([st.roll, st.pitch, st.yaw])
        thrusts[k] = u_act
        desired[k] = u_cmd
        health[k] = injector.gains(t)

        if visual is not None and k % cfg.frame_every == 0:
            frames.append(visual.rgb(quad.data).copy())

    if visual is not None:
        visual.close()

    return FlightResult(
        t=t_hist,
        pos=pos,
        vel=vel,
        omega=omega,
        euler=euler,
        thrusts=thrusts,
        health=health,
        target=target,
        onset=injector.spec.earliest_onset if injector.spec.rotor_faults else None,
        scenario=case.fault_scenario,
        scene=case.physical_scene,
        desired=desired,
        frames=frames,
    )


def run_many(
    cases,
    cfg: Optional[RunConfig] = None,
    controller_factory: Optional[ControllerFactory] = None,
    api: Optional[BaseAPI] = None,
):
    """按顺序跑一批用例，共享一次底座解析。"""
    api = api or load_base()
    return [run_case(c, cfg=cfg, controller_factory=controller_factory, api=api)
            for c in cases]
