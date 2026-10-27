#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""被控对象辨识 + 位置环调参探针（单机，约 2-3 分钟，不录屏）。

为什么需要它：编队日志显示长机侧向以约 3.5 秒周期来回摆、高度长期高出目标
1 米，而 lag_probe 实测「速度指令 -> 实际速度」延迟 0.65 秒、且与指令时长
（0.5 s / 0.05 s）无关 —— 瓶颈在 AirSim 内部的速度跟踪，不在指令队列。
被控对象既然这么迟钝，PID 增益就不能照常取，得先把对象量出来再定。

Phase 1（约 40 秒）：给水平轴、垂直轴分别下发 ±0.5 m/s 的常值速度指令，
        测实际达到的速度（稳态增益）与上升时间。
Phase 2（每组约 40 秒）：用与 formation_controller 完全相同的位置环
        （位置误差 -> PID -> moveByVelocityAsync）做 +2 米水平阶跃并返回，
        量出稳态误差、残余摆幅、稳定时间，挑出能进 0.25 米容差的一组。

用法：
    python3 tune_hover.py [AirSim IP]          # IP 默认取 config/params.yaml 的 airsim/ip
"""

import math
import os
import sys
import time

import airsim

PARAMS_NAME = "uav_formation_flight"


def _find_params():
    """定位 config/params.yaml：直接跑源码、rosrun（devel/install）都能找到。"""
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [os.path.join(here, os.pardir, "config", "params.yaml")]
    for root in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if root:
            cands.append(os.path.join(root, PARAMS_NAME, "config", "params.yaml"))
    for path in cands:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise SystemExit("找不到 config/params.yaml；试过：%s" % "；".join(cands))


def _load_params():
    try:
        import yaml
        with open(_find_params(), "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}
    except Exception as exc:
        raise SystemExit("读取参数失败：%s" % exc)


_CFG = _load_params()
_AIRSIM = _CFG.get("airsim") or {}
_FLIGHT = _CFG.get("flight") or {}
_PID = _CFG.get("pid") or {}


def _num(section, key, default):
    try:
        return float(section.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def _ip():
    value = _AIRSIM.get("ip")
    if not value:
        raise SystemExit("config/params.yaml 里没有 airsim/ip")
    return value


RATE = _num(_FLIGHT, "control_rate", 20.0)
MAX_SPEED = _num(_FLIGHT, "max_speed", 2.0)
MAX_VZ = _num(_FLIGHT, "max_vertical_speed", 1.0)
TOL = _num(_FLIGHT, "waypoint_tolerance", 0.25)
HOVER_Z = _num(_FLIGHT, "takeoff_altitude", 3.0)
CMD_DUR = _num(_FLIGHT, "cmd_duration", 0.1)
AIRSIM_TIMEOUT = _num(_AIRSIM, "timeout", 30.0)
# 当前主回路增益：作为 Phase 2 的第一组候选，并标为“当前值”
KP_NOW = _num(_PID, "kp", 0.4)
KI_NOW = _num(_PID, "ki", 0.02)
KD_NOW = _num(_PID, "kd", 0.05)
# 下面两个是试跑本身的实验参数，不属于主参数
STEP = 2.0
HOLD_SECS = 12.0


class PID(object):
    """与 formation_controller.PID 逐行一致，保证试出来的增益能直接搬过去。"""

    def __init__(self, kp, ki, kd, limit=1.0):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.integral = 0.0
        self.last_err = None

    def reset(self):
        self.integral = 0.0
        self.last_err = None

    def update(self, err, dt):
        self.integral += err * dt
        self.integral = max(-self.limit, min(self.limit, self.integral))
        deriv = 0.0
        if self.last_err is not None and dt > 0:
            deriv = (err - self.last_err) / dt
        self.last_err = err
        return self.kp * err + self.ki * self.integral + self.kd * deriv


def connect(ip):
    client = airsim.MultirotorClient(ip=ip, timeout_value=AIRSIM_TIMEOUT)
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    return client


def pos_enu(client, home):
    p = client.getMultirotorState().kinematics_estimated.position
    return (p.y_val - home[1], p.x_val - home[0], home[2] - p.z_val)


def send(client, v_enu):
    ve = max(-MAX_SPEED, min(MAX_SPEED, v_enu[0]))
    vn = max(-MAX_SPEED, min(MAX_SPEED, v_enu[1]))
    vu = max(-MAX_VZ, min(MAX_VZ, v_enu[2]))
    client.moveByVelocityAsync(vn, ve, -vu, CMD_DUR,
                               drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                               yaw_mode=airsim.YawMode(False, 0))
    return (ve, vn, vu)


def const_velocity(client, home, axis, value, secs=3.5):
    """常值速度指令跑 secs 秒，返回 (采样, 每拍耗时 ms)。"""
    dt = 1.0 / RATE
    t0 = time.perf_counter()
    next_t = t0
    samples = []
    while True:
        now = time.perf_counter()
        if now - t0 >= secs:
            break
        t_rpc = time.perf_counter()
        v = [0.0, 0.0, 0.0]
        v[axis] = value
        send(client, v)
        samples.append((now - t0, pos_enu(client, home),
                        (time.perf_counter() - t_rpc) * 1000.0))
        next_t += dt
        time.sleep(max(0.0, next_t - time.perf_counter()))
    return samples


def vel_profile(samples, axis, win=3):
    """位置中心差分得到的速度曲线。"""
    n = len(samples)
    out = []
    for i in range(n):
        a, b = max(0, i - win), min(n - 1, i + win)
        dt = samples[b][0] - samples[a][0]
        out.append(0.0 if dt <= 0 else
                   (samples[b][1][axis] - samples[a][1][axis]) / dt)
    return out


def hold(client, home, target, secs, pids):
    """位置环把飞机保持在 target（ENU 相对 home）。返回 (采样, 中止原因)。"""
    dt = 1.0 / RATE
    t0 = time.perf_counter()
    next_t = t0
    samples = []
    abort = None
    while True:
        now = time.perf_counter()
        if now - t0 >= secs:
            break
        t_rpc = time.perf_counter()
        pos = pos_enu(client, home)
        err = [target[i] - pos[i] for i in range(3)]
        u = [pids[i].update(err[i], dt) for i in range(3)]
        send(client, u)
        samples.append((now - t0, pos, u,
                        (time.perf_counter() - t_rpc) * 1000.0))
        if pos[2] < 1.2 or abs(pos[2] - target[2]) > 1.8:
            abort = "高度失控（%.2f 米）" % pos[2]
            break
        if math.dist(pos, target) > 5.0:
            abort = "偏离目标 %.1f 米" % math.dist(pos, target)
            break
        next_t += dt
        time.sleep(max(0.0, next_t - time.perf_counter()))
    if abort:
        for _ in range(10):
            send(client, [0.0, 0.0, 0.0])
            time.sleep(dt)
    return samples, abort


def tail_stats(samples, target, window=5.0):
    """取最后 window 秒：3D 误差 RMS、水平残余摆幅、高度误差 RMS。"""
    if not samples:
        return None
    t_end = samples[-1][0]
    tail = [s for s in samples if s[0] >= t_end - window]
    if len(tail) < 10:
        tail = samples[-10:]
    errs = [math.dist(s[1], target) for s in tail]
    xy = [math.hypot(s[1][0] - target[0], s[1][1] - target[1]) for s in tail]
    zs = [s[1][2] - target[2] for s in tail]
    return dict(rms=math.sqrt(sum(e * e for e in errs) / len(errs)),
                osc=max(xy) - min(xy),
                zrms=math.sqrt(sum(z * z for z in zs) / len(zs)))


def settle_time(samples, target, tol=TOL):
    """进入 tol 并此后一直保持的时刻（秒）；始终没进过则返回 None。"""
    for i in range(len(samples)):
        if all(math.dist(s[1], target) < tol for s in samples[i:]):
            return samples[i][0]
    return None


def phase1(client, home):
    print("\n=== Phase 1：被控对象辨识（常值速度指令 -> 实测速度）===")
    print("  %-18s %9s %10s %8s %11s" %
          ("轴", "指令(m/s)", "实测(m/s)", "稳态比", "上升时间(s)"))
    cases = [("水平东向 +0.5", 0, 0.5),
             ("垂直向上 +0.5", 2, 0.5),
             ("垂直向下 -0.5", 2, -0.5)]
    for label, axis, value in cases:
        for _ in range(20):
            send(client, [0.0, 0.0, 0.0])
            time.sleep(0.05)
        samples = const_velocity(client, home, axis, value)
        prof = vel_profile(samples, axis)
        rpc = sum(s[2] for s in samples) / len(samples)
        t_end = samples[-1][0]
        steady = [v for v, s in zip(prof, samples) if s[0] >= t_end - 1.2]
        v_ss = sum(steady) / len(steady)
        rise = None
        for v, s in zip(prof, samples):
            if abs(v) >= 0.63 * abs(value):
                rise = s[0]
                break
        print("  %-18s %9.2f %10.2f %8.2f %11s   每拍 RPC %.0f ms"
              % (label, value, v_ss, v_ss / value,
                 "-" if rise is None else "%.2f" % rise, rpc))


def phase2(client, home, candidates):
    print("\n=== Phase 2：位置环增益试跑（+2 米水平阶跃并返回）===")
    print("  %-18s %11s %11s %11s %11s" %
          ("增益 kp/ki/kd", "稳态RMS(m)", "残余摆幅(m)", "高度RMS(m)", "稳定时间(s)"))
    best = None
    for (kp, ki, kd) in candidates:
        pids = [PID(kp, ki, kd) for _ in range(3)]
        start = (0.0, 0.0, HOVER_Z)
        far = (STEP, 0.0, HOVER_Z)
        rows = []
        ok = True
        for target in (start, far, start):
            samples, abort = hold(client, home, target, HOLD_SECS, pids)
            if abort:
                print("  %-18s 中止：%s" % ("%.2f/%.2f/%.2f" % (kp, ki, kd), abort))
                ok = False
                break
            st = tail_stats(samples, target)
            st["settle"] = settle_time(samples, target)
            rows.append(st)
        if not ok:
            continue
        rms = max(r["rms"] for r in rows)
        osc = max(r["osc"] for r in rows)
        zrms = max(r["zrms"] for r in rows)
        settles = [r["settle"] for r in rows]
        settle_txt = ("未达标" if any(s is None for s in settles)
                      else "%.1f" % max(settles))
        print("  %-18s %11.3f %11.2f %11.3f %11s"
              % ("%.2f/%.2f/%.2f" % (kp, ki, kd), rms, osc, zrms, settle_txt))
        if rms < 0.35 and (best is None or rms < best[0]):
            best = (rms, kp, ki, kd)
    if best:
        print("\n  建议：kp=%.2f ki=%.2f kd=%.2f（试跑里稳态 RMS 最小 %.3f 米）"
              % (best[1], best[2], best[3], best[0]))
    else:
        print("\n  这几组都没进 0.35 米：说明需要改控制结构（例如把 max_speed 进一步"
              "压低，或让长机改用 AirSim 自带的位置控制）。请把这张表发我。")


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else _ip()
    print("连接 AirSim %s ..." % ip)
    client = connect(ip)
    try:
        client.landAsync(timeout_sec=30).join()          # 先落地，保证基准干净
        time.sleep(1.0)
        p = client.getMultirotorState().kinematics_estimated.position
        home = (p.x_val, p.y_val, p.z_val)
        client.takeoffAsync(timeout_sec=20).join()
        client.moveToZAsync(home[2] - HOVER_Z, 1.0, timeout_sec=25).join()
        time.sleep(3.0)
        print("已悬停在离地 %.1f 米（起飞点 %.2f, %.2f, %.2f）"
              % (HOVER_Z, home[0], home[1], home[2]))

        phase1(client, home)
        candidates = [(KP_NOW, KI_NOW, KD_NOW),      # 当前值（config/params.yaml）
                      (1.0, 0.05, 0.20),             # 单机整定最稳的一组（文档 §7 里 0.026 m 的来源）
                      (0.6, 0.02, 0.25),
                      (0.4, 0.02, 0.20),
                      (0.3, 0.01, 0.15)]
        # 万一改参数后和某一组重合，去重，免得重复占 40 秒
        phase2(client, home, [c for i, c in enumerate(candidates) if c not in candidates[:i]])
    finally:
        try:
            send(client, [0.0, 0.0, 0.0])
            client.landAsync(timeout_sec=30).join()
            client.armDisarm(False)
            client.enableApiControl(False)
            print("\n已降落并交还控制权")
        except Exception as exc:                                    # noqa: BLE001
            print("降落阶段出错：%s" % exc)


if __name__ == "__main__":
    main()
