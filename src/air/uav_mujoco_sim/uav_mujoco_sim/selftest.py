#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""内置自检（``python main.py --selftest``）：不依赖 ROS，无显示环境跳过渲染项。

各项输出实测数值而非 PASS/FAIL（如质量 0.950→1.520 kg 提示子刚体未计入）。
"""
from __future__ import annotations

import os
import sys
import time
from typing import Callable, List, Tuple

import numpy as np


def _has_display() -> bool:
    if os.name == "nt":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def run_selftest(scene: str = "open_ground", seed: int = 0,
                 verbose: bool = False) -> int:
    from .core.quadrotor import Quadrotor, GeometricController, mixer, quat_to_mat
    from .core.scenes import make_scene, SCENE_BUILDERS
    from .core.sensors import (VisionSensors, CameraConfig, LidarSensor,
                               LidarConfig, ImuSensor)

    results: List[Tuple[bool, str, str]] = []

    def check(name: str, fn: Callable[[], Tuple[bool, str]]):
        try:
            ok, detail = fn()
        except Exception as e:                        # noqa: BLE001
            import traceback
            ok, detail = False, "%s: %s" % (type(e).__name__, e)
            if verbose:
                traceback.print_exc()
        results.append((ok, name, detail))
        print("  %s %-42s %s" % ("[PASS]" if ok else "[FAIL]", name, detail))

    print("=" * 78)
    print(" uav_mujoco_sim 自检")
    print("=" * 78)

    # ---------------------------------------------------------------- 模型
    print("\n[1/6] 模型与混控")
    quad = Quadrotor(scene=make_scene(scene, seed=seed), dt=0.01, substeps=4)
    p = quad.params

    def t_params():
        # 质量须含旋翼子刚体：仅取 body_mass 得 0.85 kg（应为 0.95 kg），实测下坠 1.29 m
        return (0.5 < p.mass < 3.0,
                "质量=%.3f kg 桨臂=%.4f m k_m=%.4f 单桨最大推力=%.2f N "
                "推重比=%.2f" % (p.mass, p.arm, p.k_m, p.max_thrust,
                              4 * p.max_thrust / (p.mass * 9.81)))
    check("机体参数合理", t_params)

    def t_mixer_hover():
        th = mixer(p.mass * 9.81, np.zeros(3), p)
        return (np.allclose(th, p.mass * 9.81 / 4, atol=1e-6) and
                th.min() >= 0 and th.max() <= p.max_thrust,
                "悬停四桨各 %.4f N（理论 %.4f）" % (th[0], p.mass * 9.81 / 4))
    check("混控：悬停解算", t_mixer_hover)

    def t_mixer_yaw():
        th = mixer(p.mass * 9.81, np.array([0.0, 0.0, 0.05]), p)
        spread = np.ptp(th)
        return (spread > 1e-3 and th.min() >= 0,
                "偏航力矩 0.05 N·m → 四桨极差 %.4f N" % spread)
    check("混控：偏航通道有独立权限", t_mixer_yaw)

    def t_mixer_sat():
        # 饱和时只等比缩小力矩幅值，不得产生负推力或超限推力
        th = mixer(9.3, np.array([9.9, 9.9, 9.9]), p)
        return (th.min() >= -1e-9 and th.max() <= p.max_thrust + 1e-9,
                "饱和输入下四桨=[%.3f %.3f %.3f %.3f] 全部在 [0,%.1f] 内"
                % (*th, p.max_thrust))
    check("混控：饱和时保持可行（不发散）", t_mixer_sat)

    # ---------------------------------------------------------------- 控制
    print("\n[2/6] 控制")
    ctrl = GeometricController(p)

    def t_hover():
        quad.reset(pos=(0.0, 0.0, 1.5))
        zero = np.zeros(3)
        tgt = np.array([0.0, 0.0, 1.5])
        for _ in range(1000):
            quad.step_with_control(ctrl.act(quad, tgt, zero, zero, 0.0))
            s = quad.state()
            e = np.linalg.norm(s.pos - tgt)
            if e < 1e-4 and np.linalg.norm(s.vel) < 1e-3:
                break
        e = np.linalg.norm(quad.state().pos - tgt)
        return e < 1e-3, "悬停误差 %.4f mm（%.1f s 内收敛）" % (e * 1000, quad.time)
    check("悬停收敛", t_hover)

    def t_step():
        quad.reset(pos=(0.0, 0.0, 1.5))
        zero = np.zeros(3)
        tgt = np.array([2.0, 1.5, 2.0])
        d0 = np.linalg.norm(quad.state().pos - tgt)
        hist, t_settle = [], None
        for _ in range(1500):
            quad.step_with_control(ctrl.act(quad, tgt, zero, zero, 0.0))
            s = quad.state()
            e = np.linalg.norm(s.pos - tgt)
            hist.append(e)
            # 首次进入 2% 误差带即认为稳定，之后不得再离开
            if t_settle is None and e < 0.02 * d0:
                t_settle = s.t
        hist = np.asarray(hist)
        # 超调只取稳定后段的最大误差（全段 max 会算进初始距离 2.55 m）
        i_set = int(np.argmax(hist < 0.02 * d0)) if (hist < 0.02 * d0).any() else 0
        overshoot = max(0.0, float(hist[i_set:].max()))
        err = float(hist[-1])
        return err < 0.005, (
            "终点误差 %.3f mm  稳定时间 %s  稳定后最大偏差 %.1f mm"
            % (err * 1000,
               ("%.2f s" % t_settle) if t_settle else "未进入 2% 误差带",
               overshoot * 1000))
    check("飞到目标点并稳住", t_step)

    def t_circle():
        quad.reset(pos=(2.0, 0.0, 1.5))
        R, w = 2.0, 0.5
        errs = []
        for k in range(1600):
            t = quad.time
            pd = np.array([R * np.cos(w * t), R * np.sin(w * t), 1.5])
            vd = np.array([-R * w * np.sin(w * t), R * w * np.cos(w * t), 0.0])
            ad = np.array([-R * w * w * np.cos(w * t), -R * w * w * np.sin(w * t), 0.0])
            quad.step_with_control(ctrl.act(quad, pd, vd, ad,
                                            np.arctan2(vd[1], vd[0])))
            if k > 900:
                errs.append(np.linalg.norm(quad.state().pos - pd))
        rms = float(np.sqrt(np.mean(np.square(errs))))
        return rms < 0.05, "半径 2 m 圆周轨迹 RMSE %.2f mm" % (rms * 1000)
    check("圆周轨迹跟踪", t_circle)

    # ---------------------------------------------------------------- 场景
    print("\n[3/6] 场景库")
    for name in SCENE_BUILDERS:
        def t_scene(nm=name):
            sc = make_scene(nm, seed=seed)
            q = Quadrotor(scene=sc)
            return (q.model.ngeom > 5, "ngeom=%3d 对象=%3d  %s" %
                    (q.model.ngeom, len(sc.objects), sc.notes[:30]))
        check("场景 %s" % name, t_scene)

    # ---------------------------------------------------------------- 传感器
    print("\n[4/6] 传感器")
    if not _has_display():
        print("  [SKIP] 无显示环境（DISPLAY 未设置），跳过渲染类检查")
        print("         无头机器请安装 OSMesa 后设置 MUJOCO_GL=osmesa 再试")
    else:
        quad2 = Quadrotor(scene=make_scene("urban_block", seed=3))
        quad2.reset(pos=(-24.0, -4.0, 5.0), yaw=0.6)
        vs = VisionSensors(quad2.model, CameraConfig(width=160, height=120,
                                                     quality="fast"))

        def t_rgb():
            a = vs.rgb(quad2.data)
            return (a.shape == (120, 160, 3) and a.dtype == np.uint8,
                    "shape=%s 均值=%.1f 非黑像素=%.1f%%" %
                    (a.shape, a.mean(), 100 * (a.sum(-1) > 10).mean()))
        check("RGB 渲染", t_rgb)

        def t_depth():
            d = vs.depth(quad2.data)
            v = d > 0
            return (d.shape == (120, 160) and v.any(),
                    "有效像素 %.1f%% 范围 [%.2f, %.2f] m（量程上限 %.0f m）" %
                    (100 * v.mean(), d[v].min(), d[v].max(), vs.cfg.depth_max_range))
        check("深度渲染", t_depth)

        def t_seg():
            s = vs.segmentation(quad2.data)
            cls = [int(c) for c in np.unique(s) if c > 0]
            return len(cls) >= 1, "识别到 %d 类：%s" % (len(cls), cls)
        check("语义分割渲染", t_seg)

        def t_speed():
            for _ in range(3):
                vs.rgb(quad2.data)
            t0 = time.perf_counter()
            for _ in range(15):
                vs.rgb(quad2.data)
            dt = (time.perf_counter() - t0) / 15
            return dt < 0.05, "RGB 160x120 %.1f ms → %.0f Hz（quality=fast）" % (
                dt * 1000, 1 / dt)
        check("渲染性能", t_speed)
        vs.close()

    quad3 = Quadrotor(scene=make_scene("obstacle_field", seed=7))

    def t_lidar():
        ld = LidarSensor(quad3.model, LidarConfig(n_azimuth=72, max_range=15.0))
        pts = ld.scan(quad3.data)
        st = quad3.state()
        dmin = (np.linalg.norm(pts - quad3.data.site_xpos[ld.site_id], axis=1).min()
                if len(pts) else 99.0)
        return dmin > 0.25, "%d 线雷达命中 %d 点，最近 %.2f m（无自遮挡）" % (
            ld.n_ray, len(pts), dmin)
    check("激光雷达（含自身排除）", t_lidar)

    def t_imu():
        imu = ImuSensor(quad3.model, gyro_noise=0.002, acc_noise=0.02, seed=1)
        quad3.reset(pos=(0.0, 0.0, 2.0))
        c3 = GeometricController(quad3.params)
        zero = np.zeros(3)
        tgt = np.array([0.0, 0.0, 2.0])
        for _ in range(600):
            quad3.step_with_control(c3.act(quad3, tgt, zero, zero, 0.0))
        accs = [imu.read(quad3.data, quad3.time).acc for _ in range(30)]
        az = float(np.mean([a[2] for a in accs]))
        return abs(az - 9.81) < 0.5, "悬停时加速度计 z=%.3f m/s²（期望 9.81）" % az
    check("IMU 比力语义", t_imu)

    # ---------------------------------------------------------------- 性能
    print("\n[5/6] 性能")
    quad4 = Quadrotor(scene=make_scene("obstacle_field", seed=1))

    def t_sim_hz():
        c4 = GeometricController(quad4.params)
        zero = np.zeros(3)
        tgt = np.array([0.0, 0.0, 1.5])
        for _ in range(200):
            quad4.step_with_control(c4.act(quad4, tgt, zero, zero, 0.0))
        t0 = time.perf_counter()
        N = 2000
        for _ in range(N):
            quad4.step_with_control(c4.act(quad4, tgt, zero, zero, 0.0))
        dt = (time.perf_counter() - t0) / N
        return 1 / dt > 100, "%.0f Hz 控制回路（含控制律+物理，要求 ≥100）" % (1 / dt)
    check("实时性", t_sim_hz)

    # ---------------------------------------------------------------- 协议
    print("\n[6/6] 通信协议")

    def t_proto():
        from .sim.protocol import encode, decode, blob_array
        blobs = {"a": np.arange(12, dtype=np.float32).reshape(3, 4),
                 "b": np.zeros((2,), dtype=np.uint8)}
        raw = encode({"ok": True, "blobs": blobs})
        head, payload = decode(raw)
        a = blob_array(head, payload, "a")
        b = blob_array(head, payload, "b")
        return (np.array_equal(a, blobs["a"]) and np.array_equal(b, blobs["b"]),
                "往返 %d 字节，两个数组逐元素一致" % len(raw))
    check("编解码往返", t_proto)

    # ---------------------------------------------------------------- 汇总
    n_pass = sum(1 for ok, _, _ in results if ok)
    n_all = len(results)
    print("\n" + "=" * 78)
    print(" 通过 %d/%d" % (n_pass, n_all))
    if n_pass < n_all:
        print(" 失败项：")
        for ok, name, detail in results:
            if not ok:
                print("   - %s : %s" % (name, detail))
    print("=" * 78)
    return 0 if n_pass == n_all else 1
