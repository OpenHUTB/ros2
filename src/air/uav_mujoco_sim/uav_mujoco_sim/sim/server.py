#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 仿真服务端：单进程内跑物理与传感器，经 TCP 对外提供服务。

坐标与状态机约定同 ``carlair_ros_bridge``：``cmd_vel`` 为机体系（x 前 / y 左 / z 上），
位姿与目标点为 ENU 世界系；速度指令实现为参考轨迹 + 几何控制器跟踪。
"""
from __future__ import annotations

import argparse
import json
import queue
import socket
import socketserver
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import numpy as np

from ..core.quadrotor import Quadrotor, GeometricController, quat_to_mat, quat_to_euler
from ..core.scenes import make_scene
from ..core.sensors import (VisionSensors, CameraConfig, LidarSensor,
                            LidarConfig, ImuSensor)

DEFAULT_PORT = 41460          # 避开 AirSim 默认端口 41451，防止与真机冲突


def _optional_vector(v) -> Optional[np.ndarray]:
    """可选向量参数。用 ``is None`` 判断：``[0.0]*4`` 是合法取值，真值判断会误判为未给。"""
    return None if v is None else np.asarray(v, dtype=float)


# --------------------------------------------------------------------------- #
# 控制模式
# --------------------------------------------------------------------------- #
@dataclass
class ControlState:
    """当前控制目标。"""

    mode: str = "hover"                       # hover | velocity | goal | rotor
    goal: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.5]))
    ref_pos: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.5]))
    # 参考轨迹速度，作为前馈交给几何控制器；置零会产生稳态跟随滞后。
    ref_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ref_yaw: float = 0.0
    yaw_rate: float = 0.0
    rotor: np.ndarray = field(default_factory=lambda: np.zeros(4))
    max_horiz_speed: float = 5.0
    max_vert_speed: float = 3.0
    max_yaw_rate: float = 1.0
    goal_tol: float = 0.35
    goal_hold: int = 0
    status: str = "READY"


class VelocityCommander:
    """机体系速度指令 → 参考轨迹：``ref_pos += R_world_body @ [vx, vy, vz] * dt``。

    参考点沿机头方向推进；指令先经一阶低通再积分，未滤波时松键回弹实测 18 cm。
    """

    #: 参考速度一阶滤波时间常数（秒），_research/tune_gains.py 扫描选定：0.12 s（0.25 s 时滑行 0.26 m）。
    TAU = 0.12

    def __init__(self, state: ControlState):
        self.st = state
        self._v_filt = np.zeros(3)      # 世界系滤波后速度

    def update(self, quad: Quadrotor, cmd_vel: np.ndarray, dt: float) -> None:
        vx, vy, vz, wz = (float(x) for x in cmd_vel)
        st = self.st
        vx = float(np.clip(vx, -st.max_horiz_speed, st.max_horiz_speed))
        vy = float(np.clip(vy, -st.max_horiz_speed, st.max_horiz_speed))
        vz = float(np.clip(vz, -st.max_vert_speed, st.max_vert_speed))
        wz = float(np.clip(wz, -st.max_yaw_rate, st.max_yaw_rate))

        cur = quad.state()
        R = quat_to_mat(cur.quat)
        v_cmd_world = R @ np.array([vx, vy, vz])

        # 一阶低通：松键时平滑衰减，而非硬切
        alpha = min(1.0, dt / max(self.TAU, 1e-6))
        self._v_filt = self._v_filt + (v_cmd_world - self._v_filt) * alpha

        # 速度低于阈值直接归零，避免残量引起缓慢漂移
        if np.linalg.norm(self._v_filt) < 1e-3:
            self._v_filt[:] = 0.0

        st.ref_pos = st.ref_pos + self._v_filt * dt
        st.ref_pos[2] = max(st.ref_pos[2], 0.3)          # 高度下限 0.3 m
        st.ref_yaw = st.ref_yaw + wz * dt
        st.ref_vel = self._v_filt.copy()

        active = max(abs(vx), abs(vy), abs(vz), abs(wz)) > 1e-6
        moving = bool(np.any(self._v_filt))
        st.mode = "velocity" if (active or moving) else "hover"
        st.status = "VELOCITY" if (active or moving) else "HOVER"


# --------------------------------------------------------------------------- #
# 仿真服务
# --------------------------------------------------------------------------- #
class SimCore:
    """MuJoCo 仿真实现。所有方法必须在同一线程调用（GL 上下文线程绑定），由 ``_SimWorker`` 保证。"""

    def __init__(self, scene: str = "open_ground", seed: int = 0,
                 dt: float = 0.01, substeps: int = 4,
                 camera: Optional[CameraConfig] = None,
                 lidar: Optional[LidarConfig] = None,
                 imu_noise: bool = False, scene_kwargs: Optional[dict] = None,
                 headless: bool = False):
        self.dt = float(dt)
        self.substeps = int(substeps)
        self.scene_name = scene
        self.headless = bool(headless)
        self.lock = threading.RLock()

        self.scene_kwargs = dict(scene_kwargs or {})
        self._build(scene, seed, camera, lidar, imu_noise)

        self.t0 = time.time()
        self.stats = {"steps": 0, "resets": 0, "started": self.t0}

    # ------------------------------------------------------------------ 构建
    def _build(self, scene_name: str, seed: int, camera, lidar, imu_noise) -> None:
        sc = make_scene(scene_name, seed=seed, **self.scene_kwargs)
        self.quad = Quadrotor(scene=sc, dt=self.dt, substeps=self.substeps)
        self.ctrl = GeometricController(self.quad.params)

        self.camera_cfg = camera or CameraConfig(width=640, height=480,
                                                 name="front_cam")
        self.lidar_cfg = lidar or LidarConfig()
        # 无头模式不建渲染器，省去 OpenGL 上下文开销
        self.vision: Optional[VisionSensors] = None if self.headless else \
            VisionSensors(self.quad.model, self.camera_cfg)
        self.lidar = LidarSensor(self.quad.model, self.lidar_cfg)
        self.imu = ImuSensor(self.quad.model,
                             gyro_noise=0.002 if imu_noise else 0.0,
                             acc_noise=0.02 if imu_noise else 0.0)

        st = self.quad.state()
        self.cstate = ControlState(
            goal=st.pos.copy(), ref_pos=st.pos.copy(), ref_yaw=st.yaw,
            status="HOVER",
        )
        self.vcmd = VelocityCommander(self.cstate)
        # 起飞：参考点抬到 ≥1.5 m
        self.cstate.ref_pos = st.pos.copy()
        self.cstate.ref_pos[2] = max(st.pos[2], 1.5)

    def reset(self, pos=None, yaw: float = 0.0, scene: Optional[str] = None,
              seed: Optional[int] = None) -> Dict[str, Any]:
        with self.lock:
            if scene is not None and scene != self.scene_name:
                self._build_and_reset_scene(scene, seed if seed is not None else 0)
            p = pos if pos is not None else self.scene.drone_pos
            self.quad.reset(pos=tuple(p), yaw=yaw)
            self.imu.reset()
            self.cstate = ControlState(
                goal=np.asarray(p, float).copy(),
                ref_pos=np.asarray(p, float).copy(),
                ref_yaw=yaw, status="HOVER",
            )
            self.vcmd = VelocityCommander(self.cstate)
            self.stats["resets"] += 1
            return self._state_msg()

    def _build_and_reset_scene(self, scene: str, seed: int) -> None:
        self.scene_name = scene
        self._build(scene, seed, self.camera_cfg, self.lidar_cfg, False)

    # ------------------------------------------------------------------ 推进
    def _control_step(self, cmd_vel: Optional[np.ndarray]) -> np.ndarray:
        """一个控制周期：更新参考轨迹 → 几何控制器 → 返回四个桨推力。"""
        st = self.cstate
        cur = self.quad.state()

        if st.mode == "rotor":
            return np.clip(st.rotor, 0.0, self.quad.params.max_thrust)

        if st.mode == "goal":
            st.ref_pos = st.goal
            st.ref_vel = np.zeros(3)
            if np.linalg.norm(cur.pos - st.goal) < st.goal_tol:
                st.goal_hold += 1
                if st.goal_hold > 10:
                    st.status = "GOAL_DONE"
            else:
                st.goal_hold = 0
                st.status = "GOAL"
        elif st.mode == "velocity" and cmd_vel is not None:
            self.vcmd.update(self.quad, cmd_vel, self.dt)
        else:
            st.ref_vel = np.zeros(3)

        # 参考速度作前馈（v_d）交给几何控制器，参考加速度留零
        thrusts = self.ctrl.act(self.quad, st.ref_pos, st.ref_vel,
                                np.zeros(3), st.ref_yaw)
        return thrusts

    def step(self, n: int = 1, cmd_vel: Optional[np.ndarray] = None,
             goal: Optional[np.ndarray] = None, mode: Optional[str] = None,
             rotor: Optional[np.ndarray] = None) -> Dict[str, Any]:
        with self.lock:
            if mode is not None:
                self.cstate.mode = mode
                self.cstate.status = {"hover": "HOVER", "velocity": "VELOCITY",
                                      "goal": "GOAL", "rotor": "ROTOR"}.get(mode, mode.upper())
            if cmd_vel is not None:
                # 收到速度指令即切 velocity 模式：可覆盖 goal，不覆盖 rotor（直通模式，须显式退出）
                if self.cstate.mode != "rotor":
                    self.cstate.mode = "velocity"
            if goal is not None:
                self.cstate.goal = np.asarray(goal, float).copy()
                self.cstate.mode = "goal"
                self.cstate.status = "GOAL"
                self.cstate.goal_hold = 0
            if rotor is not None:
                self.cstate.rotor = np.asarray(rotor, float).copy()
                self.cstate.mode = "rotor"
                self.cstate.status = "ROTOR"

            for _ in range(max(int(n), 1)):
                t = self._control_step(cmd_vel)
                self.quad.step_with_control(t)
                self.stats["steps"] += 1
            return self._state_msg()

    # ------------------------------------------------------------------ 状态
    def _state_msg(self, want=("odom", "imu", "status")) -> Dict[str, Any]:
        q = self.quad
        st = q.state()
        msg: Dict[str, Any] = {
            "t": st.t,
            "wall": time.time() - self.t0,
            "status": self.cstate.status,
            "scene": self.scene_name,
            "odom": {
                "pos": st.pos.tolist(),
                "quat": st.quat.tolist(),
                "vel": st.vel.tolist(),
                "omega": st.omega.tolist(),
                "rpy": [st.roll, st.pitch, st.yaw],
            },
            "params": {
                "mass": q.params.mass,
                "arm": q.params.arm,
                "k_m": q.params.k_m,
                "max_thrust": q.params.max_thrust,
            },
        }
        if "imu" in want:
            r = self.imu.read(q.data, q.time)
            msg["imu"] = {"quat": r.quat.tolist(), "gyro": r.gyro.tolist(),
                          "acc": r.acc.tolist()}
        return msg

    def observe(self, want=("odom", "imu")) -> Dict[str, Any]:
        """取一帧观测（不推进物理）。"""
        with self.lock:
            return self._state_msg(tuple(want))

    def render(self, want=("rgb", "depth", "seg", "lidar")) -> Dict[str, np.ndarray]:
        """取一帧传感器数据（不推进物理；无头模式下视觉部分返回空）。"""
        with self.lock:
            out: Dict[str, np.ndarray] = {}
            if self.vision is not None:
                if "rgb" in want:
                    out["rgb"] = self.vision.rgb(self.quad.data)
                if "depth" in want:
                    out["depth"] = self.vision.depth(self.quad.data)
                if "seg" in want:
                    out["seg"] = self.vision.segmentation(self.quad.data)
            if "lidar" in want:
                st = self.quad.state()
                out["lidar"] = self.lidar.scan(
                    self.quad.data, state_pos=st.pos, state_mat=quat_to_mat(st.quat))
            return out

    # ------------------------------------------------------------------ 全帧
    def frame(self, step_n: int = 1, cmd_vel=None, goal=None, mode=None,
              rotor=None, want=("odom", "imu")) -> tuple:
        """推进 + 取观测，返回 (状态头, blobs)。供 TCP 服务端一次往返完成。"""
        msg = self.step(step_n, cmd_vel=cmd_vel, goal=goal, mode=mode, rotor=rotor)
        blobs = self.render(tuple(want))
        msg["blobs"] = blobs
        return msg, blobs

    def close(self) -> None:
        if self.vision is not None:
            self.vision.close()

    @property
    def scene(self):
        return self.quad.scene


# --------------------------------------------------------------------------- #
# 独占线程 + 对外接口
# --------------------------------------------------------------------------- #
class _SimWorker(threading.Thread):
    """独占仿真与渲染的线程：连接线程经队列提交任务，GL 上下文只属于本线程。"""

    def __init__(self):
        super().__init__(name="uav-sim-worker", daemon=True)
        self._jobs: "queue.Queue" = queue.Queue()
        self.core: Optional[SimCore] = None
        self._init_args = ()
        self._init_kwargs = {}
        self._init_box = None
        # 线程在 __init__ 中 start，构造参数经 bootstrap() 传入，故先等 _init_ready
        self._init_ready = threading.Event()
        self.start()

    # ---- 线程入口
    def run(self):
        self._init_ready.wait()
        args, kwargs, box = self._init_args, self._init_kwargs, self._init_box
        try:
            self.core = SimCore(*args, **kwargs)
            box["result"] = self.core
        except BaseException as e:                 # noqa: BLE001
            box["error"] = e
        finally:
            box["done"].set()

        while True:
            job = self._jobs.get()
            if job is None:                        # 收到哨兵：退出
                return
            fn, a, kw, box = job
            try:
                box["result"] = fn(*a, **kw)
            except BaseException as e:             # noqa: BLE001
                box["error"] = e
            finally:
                box["done"].set()

    # ---- 提交
    def submit(self, fn, *args, **kwargs):
        box = {"done": threading.Event(), "result": None, "error": None}
        self._jobs.put((fn, args, kwargs, box))
        if not box["done"].wait(timeout=600):
            raise TimeoutError("仿真线程 600 s 未响应")
        if box["error"] is not None:
            raise box["error"]
        return box["result"]

    def bootstrap(self, *args, **kwargs):
        """在 worker 线程上构造 SimCore（渲染器只能在此线程创建）。"""
        box = {"done": threading.Event(), "result": None, "error": None}
        self._init_args, self._init_kwargs, self._init_box = args, kwargs, box
        self._init_ready.set()
        if not box["done"].wait(timeout=600):
            raise TimeoutError("仿真线程 600 s 未完成初始化")
        if box["error"] is not None:
            raise box["error"]
        return box["result"]

    def shutdown(self):
        self._jobs.put(None)
        self.join(timeout=10)


class SimServer:
    """仿真服务端对外接口：所有调用经 ``_SimWorker`` 转发到独占线程执行。"""

    def __init__(self, **kwargs):
        self._worker = _SimWorker()
        self._worker.bootstrap(**kwargs)
        self.core: SimCore = self._worker.core            # type: ignore[assignment]

    # ---- 转发到 worker
    def reset(self, **kw):
        return self._worker.submit(self.core.reset, **kw)

    def step(self, *a, **kw):
        return self._worker.submit(self.core.step, *a, **kw)

    def observe(self, want=("odom", "imu")):
        return self._worker.submit(self.core.observe, want)

    def render(self, want=("rgb", "depth", "seg", "lidar")):
        return self._worker.submit(self.core.render, want)

    def frame(self, *a, **kw):
        return self._worker.submit(self.core.frame, *a, **kw)

    def close(self):
        try:
            self._worker.submit(self.core.close)
        finally:
            self._worker.shutdown()

    # ---- 只读属性（返回对象只能在 worker 线程中操作）
    @property
    def scene_name(self):
        return self.core.scene_name

    @property
    def dt(self):
        return self.core.dt

    @property
    def headless(self):
        return self.core.headless

    @property
    def camera_cfg(self):
        return self.core.camera_cfg

    @property
    def scene(self):
        return self.core.scene

    @property
    def quad(self):
        """直接访问机体对象。仅限单线程场景（如 demo.py），不可跨线程。"""
        return self.core.quad

    @property
    def ctrl(self):
        return self.core.ctrl

    @property
    def cstate(self):
        return self.core.cstate

    @property
    def imu(self):
        return self.core.imu


# --------------------------------------------------------------------------- #
# TCP 服务端
# --------------------------------------------------------------------------- #
class _Handler(socketserver.BaseRequestHandler):
    """每条连接一个线程；请求串行化到 SimServer 的大锁上。"""

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.request.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("对端关闭")
            buf.extend(chunk)
        return bytes(buf)

    def handle(self):  # noqa: C901
        srv: SimServer = self.server.sim  # type: ignore[attr-defined]
        # 关 Nagle：图像报文分段发送会被延迟 ACK 拖慢，单次往返多 40 ms
        try:
            self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        try:
            while True:
                head_len = self._recv_exact(4)
                (n,) = struct.unpack(">I", head_len)
                req = json.loads(self._recv_exact(n).decode("utf-8"))
                resp = self._dispatch(srv, req)
                self.request.sendall(resp)
        except (ConnectionError, OSError, struct.error):
            return

    def _dispatch(self, srv: SimServer, req: Dict[str, Any]) -> bytes:
        from .protocol import encode

        op = req.get("op", "frame")
        if op == "ping":
            return encode({"ok": True, "t": time.time()})
        if op == "reset":
            srv.reset(pos=req.get("pos"), yaw=float(req.get("yaw", 0.0)),
                      scene=req.get("scene"), seed=req.get("seed"))
            return encode({"ok": True, **(srv.observe())})
        if op == "frame":
            cmd_vel = _optional_vector(req.get("cmd_vel"))
            goal = _optional_vector(req.get("goal"))
            rotor = _optional_vector(req.get("rotor"))
            want = tuple(req.get("want") or ("odom", "imu"))
            msg, blobs = srv.frame(int(req.get("n", 1)), cmd_vel, goal,
                                   req.get("mode"), rotor, want)
            msg["ok"] = True
            return encode({**msg, "blobs": blobs})
        if op == "observe":
            return encode({"ok": True, **srv.observe(tuple(req.get("want") or ("odom", "imu")))})
        if op == "render":
            blobs = srv.render(tuple(req.get("want") or ("rgb", "depth")))
            return encode({"ok": True, "blobs": blobs})
        if op == "config":
            return encode({"ok": True, **srv.observe(("odom", "imu")),
                           "scene": srv.scene_name})
        return encode({"ok": False, "error": f"未知操作 {op!r}"})


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def serve(host: str = "0.0.0.0", port: int = DEFAULT_PORT,
          server: Optional[SimServer] = None, **kwargs) -> None:
    """启动服务端并阻塞。"""
    sim = server or SimServer(**kwargs)
    srv = _Server((host, port), _Handler)
    srv.sim = sim  # type: ignore[attr-defined]
    print(f"[uav_mujoco_sim] 仿真服务已启动  {host}:{port}")
    print(f"[uav_mujoco_sim] 场景={sim.scene_name}  控制周期 dt={sim.dt}s  "
          f"{'无头模式' if sim.headless else f'相机 {sim.camera_cfg.width}x{sim.camera_cfg.height}'}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[uav_mujoco_sim] 正在退出…")
    finally:
        srv.shutdown()
        sim.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="MuJoCo 四旋翼仿真服务端")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--scene", default="open_ground",
                    choices=["open_ground", "obstacle_field", "corridor",
                             "gate_track", "urban_block"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dt", type=float, default=0.01)
    ap.add_argument("--substeps", type=int, default=4)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--headless", action="store_true", help="不渲染视觉传感器")
    ap.add_argument("--imu-noise", action="store_true")
    ap.add_argument("--lidar-azimuth", type=int, default=72)
    ap.add_argument("--lidar-max-range", type=float, default=15.0)
    ap.add_argument("--quality", choices=["fast", "full"], default="fast",
                    help="fast=关阴影(感知用,快6倍)  full=全开(录演示动图用)")
    ap.add_argument("--rgb-noise", type=float, default=0.0)
    a = ap.parse_args(argv)

    cam = CameraConfig(width=a.width, height=a.height, quality=a.quality,
                       rgb_noise=a.rgb_noise)
    lid = LidarConfig(n_azimuth=a.lidar_azimuth, max_range=a.lidar_max_range)
    serve(a.host, a.port, scene=a.scene, seed=a.seed, dt=a.dt,
          substeps=a.substeps, camera=cam, lidar=lid,
          imu_noise=a.imu_noise, headless=a.headless)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
