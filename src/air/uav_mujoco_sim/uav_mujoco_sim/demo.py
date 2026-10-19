#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""不依赖 ROS 的独立演示与录制/回放，全部走 core 纯 Python 路径。"""
from __future__ import annotations

import os
import time
from typing import Optional

import numpy as np

from .core.quadrotor import Quadrotor, GeometricController, quat_to_euler
from .core.scenes import make_scene
from .core.sensors import VisionSensors, CameraConfig, LidarSensor, LidarConfig
from .core.teleop import TeleopConfig, command_from_keys, key_help, EXIT_KEYS


# --------------------------------------------------------------------------- #
# 独立演示
# --------------------------------------------------------------------------- #
def run_demo(scene: str = "open_ground", seed: int = 0, dt: float = 0.01,
             substeps: int = 4, seconds: float = 15.0, viewer: bool = False,
             quality: str = "fast", verbose: bool = False) -> int:
    """仿真 + 终端键盘遥控，一体进程，不经过 ROS。"""
    from .core.terminal import RawKeyReader
    from .sim.server import SimServer, ControlState

    print("=" * 68)
    print(" uav_mujoco_sim 独立演示")
    print("=" * 68)
    srv = SimServer(scene=scene, seed=seed, dt=dt, substeps=substeps,
                    camera=CameraConfig(quality=quality))
    quad, ctrl = srv.quad, srv.ctrl
    st = srv.cstate
    print(f"场景: {scene} (seed={seed})   {srv.scene.notes}")
    print(f"机体: 质量 {quad.params.mass:.3f} kg, 推重比 "
          f"{4*quad.params.max_thrust/(quad.params.mass*9.81):.2f}")
    print("\n键盘操作：")
    print(key_help())
    if not verbose:
        print("\n（按任意键开始，Ctrl-C 退出）")

    mjv = None
    if viewer:
        try:
            import mujoco.viewer
            mjv = mujoco.viewer.launch_passive(quad.model, quad.data)
            print("已打开 MuJoCo 交互窗口。")
        except Exception as e:                       # noqa: BLE001
            print(f"无法打开 MuJoCo 窗口（{e}），改为无窗口模式。")

    cfg = TeleopConfig()
    keys_reader = RawKeyReader(warn=print)
    if not keys_reader.is_tty:
        print("取不到可读的终端（%s），收不到键盘。请在终端里直接运行，"
              % (keys_reader.error or "未知"))
        print("不要用管道、重定向、nohup 或放在后台启动。")

    pressed = set()
    last_key_t = -1e9
    t0 = time.time()
    n = 0
    try:
        while (time.time() - t0) < seconds:
            loop_t0 = time.perf_counter()

            for ch in keys_reader.read():
                if ch in EXIT_KEYS:
                    raise KeyboardInterrupt
                if ch.lower() in "wasdrfqe":
                    pressed = {ch.lower()}
                    last_key_t = time.monotonic()

            if (time.monotonic() - last_key_t) > 0.2:
                pressed = set()
            cmd = command_from_keys(pressed, cfg)

            srv.step(1, cmd_vel=list(cmd))
            n += 1

            if mjv is not None:
                mjv.sync()
                if not mjv.is_running():
                    break

            if verbose and n % 50 == 0:
                s = quad.state()
                print(f"  t={s.t:6.2f}s pos=({s.pos[0]:6.2f},{s.pos[1]:6.2f},"
                      f"{s.pos[2]:5.2f}) yaw={np.degrees(s.yaw):6.1f}° "
                      f"cmd=({cmd[0]:4.1f},{cmd[1]:4.1f},{cmd[2]:4.1f}) "
                      f"status={st.status}")

            # 按控制周期节拍限速
            slack = dt - (time.perf_counter() - loop_t0)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        print("\n收到中断。")
    finally:
        keys_reader.restore()
        if mjv is not None:
            try:
                mjv.close()
            except Exception:                        # noqa: BLE001
                pass
        s = quad.state()
        print(f"共 {n} 个控制周期（{n*dt:.1f}s 仿真时间）。"
              f"最终位置 {np.round(s.pos, 2).tolist()}")
        srv.close()
    return 0


# --------------------------------------------------------------------------- #
# 录制 / 回放
# --------------------------------------------------------------------------- #
def _scripted_command(t: float) -> tuple:
    """固定机动脚本：起飞 → 前进 → 侧移 → 上升转向 → 悬停，回放轨迹可复现。"""
    if t < 1.0:
        return (0.0, 0.0, 0.0, 0.0)                   # 起飞悬停
    if t < 3.0:
        return (1.0, 0.0, 0.0, 0.0)                   # 前进
    if t < 5.0:
        return (0.0, 0.8, 0.0, 0.0)                   # 左移
    if t < 7.0:
        return (0.0, 0.0, 0.6, 0.0)                   # 上升
    if t < 9.0:
        return (0.0, 0.0, 0.0, 0.8)                   # 右转
    if t < 11.0:
        return (1.0, 0.0, 0.0, 0.0)                   # 转向后再前进
    if t < 12.0:
        return (0.0, 0.0, -0.5, 0.0)                  # 下降
    return (0.0, 0.0, 0.0, 0.0)                       # 悬停


def record_flight(path: str, scene: str = "open_ground", seed: int = 0,
                  dt: float = 0.01, substeps: int = 4, seconds: float = 15.0,
                  save_images: bool = True, img_every: int = 20,
                  img_size=(120, 160), verbose: bool = True) -> int:
    """按脚本飞一次并落盘：状态、IMU、指令、周期性 RGB/深度/分割与雷达。

    示例图像取 120×160 以控制体积。
    """
    from .sim.server import SimServer

    if verbose:
        print(f"录制 {seconds:.0f}s 到 {path} …")
    srv = SimServer(scene=scene, seed=seed, dt=dt, substeps=substeps,
                    camera=CameraConfig(width=img_size[1], height=img_size[0],
                                        quality="fast"))
    quad = srv.quad

    rows, imus, cmds, thrusts = [], [], [], []
    imgs_rgb, imgs_dep, imgs_seg, lds, img_t = [], [], [], [], []

    n = int(seconds / dt)
    for k in range(n):
        t = quad.time
        cmd = _scripted_command(t)
        th = srv.step(1, cmd_vel=list(cmd))
        s = quad.state()
        imu = srv.imu.read(quad.data, quad.time)

        rows.append([s.t, *s.pos, *s.quat, *s.vel, *s.omega, s.roll, s.pitch, s.yaw])
        imus.append([*imu.gyro, *imu.acc])
        cmds.append(list(cmd))
        thrusts.append(list(quad.data.ctrl[:4]))

        if save_images and k % img_every == 0:
            b = srv.render(("rgb", "depth", "seg", "lidar"))
            imgs_rgb.append(b["rgb"])
            imgs_dep.append(b["depth"])
            imgs_seg.append(b["seg"])
            lds.append(b["lidar"])
            img_t.append(s.t)

    arrs = dict(
        t=np.asarray([r[0] for r in rows], dtype=np.float64),
        pos=np.asarray([r[1:4] for r in rows], dtype=np.float64),
        quat=np.asarray([r[4:8] for r in rows], dtype=np.float64),
        vel=np.asarray([r[8:11] for r in rows], dtype=np.float64),
        omega=np.asarray([r[11:14] for r in rows], dtype=np.float64),
        rpy=np.asarray([r[14:17] for r in rows], dtype=np.float64),
        gyro=np.asarray([i[0:3] for i in imus], dtype=np.float64),
        acc=np.asarray([i[3:6] for i in imus], dtype=np.float64),
        cmd=np.asarray(cmds, dtype=np.float64),
        thrust=np.asarray(thrusts, dtype=np.float64),
        img_t=np.asarray(img_t, dtype=np.float64),
        meta=np.asarray([scene, str(seed), str(dt), str(substeps)]),
    )
    if save_images and imgs_rgb:
        arrs["rgb"] = np.stack(imgs_rgb)
        arrs["depth"] = np.stack(imgs_dep).astype(np.float32)
        arrs["seg"] = np.stack(imgs_seg).astype(np.uint8)
        # 点云每帧长度不同，无法堆成定长数组；只存每帧点数 + 拼接数据
        arrs["lidar_counts"] = np.asarray([len(p) for p in lds], dtype=np.int32)
        arrs["lidar_flat"] = np.concatenate(lds, axis=0) if lds else \
            np.zeros((0, 3), dtype=np.float32)

    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    np.savez_compressed(path, **arrs)
    size_mb = os.path.getsize(path) / 1024 / 1024
    if verbose:
        print(f"已保存：{path}  （{size_mb:.2f} MB）")
        print(f"  状态 {len(arrs['t'])} 帧 @ {1/dt:.0f} Hz，"
              f"图像 {len(img_t)} 帧 @ {1/(dt*img_every):.1f} Hz，"
              f"雷达 {sum(arrs['lidar_counts'])} 点")
    srv.close()
    return 0


def replay_flight(path: str) -> int:
    """读取录制的 npz 并打印内容摘要。"""
    if not os.path.exists(path):
        print(f"找不到文件：{path}")
        return 1
    d = np.load(path, allow_pickle=False)
    print("=" * 68)
    print(f"回放 {path}")
    print("=" * 68)
    if "meta" in d:
        scene, seed, dt, sub = d["meta"]
        print(f"场景={scene}  seed={seed}  dt={dt}s  子步={sub}")
    print(f"状态帧数 : {len(d['t'])}  时长 {d['t'][-1]:.2f}s")
    print(f"位置范围 : x[{d['pos'][:,0].min():.2f},{d['pos'][:,0].max():.2f}] "
          f"y[{d['pos'][:,1].min():.2f},{d['pos'][:,1].max():.2f}] "
          f"z[{d['pos'][:,2].min():.2f},{d['pos'][:,2].max():.2f}]")
    total = float(np.linalg.norm(np.diff(d["pos"], axis=0), axis=1).sum())
    print(f"飞行路程 : {total:.2f} m")
    print(f"最大速度 : {np.linalg.norm(d['vel'], axis=1).max():.2f} m/s")
    print(f"最大倾角 : roll {np.degrees(np.abs(d['rpy'][:,0])).max():.1f}°, "
          f"pitch {np.degrees(np.abs(d['rpy'][:,1])).max():.1f}°")
    if "rgb" in d:
        print(f"图像     : rgb{d['rgb'].shape} depth{d['depth'].shape} "
              f"seg{d['seg'].shape}")
        print(f"雷达     : {int(d['lidar_counts'].sum())} 点，"
              f"每帧 {d['lidar_counts'].min()}~{d['lidar_counts'].max()} 点")
    # 指令为机体系、速度为世界系；须先转回机体系再比（偏航非零时直接相减误差虚高，实测 0.4 m/s）
    print("\n各轴速度跟踪误差（机体系指令 vs 世界系速度转机体系）：")
    yaw = d["rpy"][:, 2]
    c, s = np.cos(-yaw), np.sin(-yaw)
    vx_b = c * d["vel"][:, 0] - s * d["vel"][:, 1]
    vy_b = s * d["vel"][:, 0] + c * d["vel"][:, 1]
    vz_b = d["vel"][:, 2]
    err = np.stack([vx_b, vy_b, vz_b], axis=1) - d["cmd"][:, :3]
    for i, ax in enumerate("xyz"):
        print(f"  v{ax}: RMSE {np.sqrt((err[:, i]**2).mean()):.4f} m/s"
              f"   峰值误差 {np.abs(err[:, i]).max():.4f} m/s")
    return 0
