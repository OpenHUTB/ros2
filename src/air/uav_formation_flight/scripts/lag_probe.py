#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""回路延迟探针：单独测 AirSim「下发速度指令 -> 飞机实际速度」的滞后。

只用于诊断，不参与编队任务。用法（虚拟机里，模拟器已启动并切到无人机模式）：

    python3 lag_probe.py [AirSim IP]      # IP 默认取 config/params.yaml 的 airsim/ip

流程：
  1. 让 SimpleFlight 起飞到 3 米并稳定；
  2. 用两组相同的方波速度指令各跑 12 秒，只改指令有效期 duration
     （0.5 秒 vs 0.05 秒），其余完全一致；
  3. 记录每拍的「指令速度」与「实际速度」（位置中心差分），用互相关求延迟。

判读：
  duration=0.50 的延迟明显大于 duration=0.05 -> 延迟主要来自指令有效期
  （AirSim 把每拍指令排进队列），把 cmd_duration 改小就能修好；
  两者差不多（都 0.2-0.4 秒）- 延迟来自 AirSim 内部速度跟踪，
  那就只能从增益上处理（降低 kp、减小 ki）。

整个过程约 40 秒，不需要录屏，跑完自动降落。
"""

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
HOVER_ALT = _num(_FLIGHT, "takeoff_altitude", 3.0)
MAX_VZ = _num(_FLIGHT, "max_vertical_speed", 1.0)
AIRSIM_TIMEOUT = _num(_AIRSIM, "timeout", 30.0)


def connect(ip):
    client = airsim.MultirotorClient(ip=ip, timeout_value=AIRSIM_TIMEOUT)
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    return client


def best_lag(cmd, vel, dt, max_lag=3.0):
    """在滞后 0..max_lag 秒里找使 corr(cmd[t-lag], vel[t]) 最大的滞后。"""
    n = len(cmd)

    def corr(a, b):
        m = len(a)
        if m < 50:
            return -9.0
        ma, mb = sum(a) / m, sum(b) / m
        num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
        da = sum((x - ma) ** 2 for x in a) ** 0.5
        db = sum((y - mb) ** 2 for y in b) ** 0.5
        return num / (da * db) if da and db else -9.0

    best = (-9.0, 0.0)
    k = 0
    while k * dt <= max_lag:
        c = corr(cmd[:n - k], vel[k:]) if k else corr(cmd, vel)
        if c > best[0]:
            best = (c, k * dt)
        k += 1
    return best


def run_phase(client, duration, secs=12.0, amp=0.5, period=3.0, rate=RATE):
    """用给定 duration 下发方波速度（NED 的 y 轴 = 东向），返回采样序列。"""
    dt_loop = 1.0 / rate
    t0 = time.perf_counter()
    ts, cmds, ys = [], [], []
    next_t = t0
    while True:
        now = time.perf_counter()
        if now - t0 >= secs:
            break
        # 方波：前半周期 +amp，后半周期 -amp
        sign = 1.0 if ((now - t0) % period) < (period / 2.0) else -1.0
        cmd = amp * sign
        client.moveByVelocityAsync(0.0, cmd, 0.0, duration,
                                  drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                                  yaw_mode=airsim.YawMode(False, 0))
        p = client.getMultirotorState().kinematics_estimated.position
        ts.append(now - t0)
        cmds.append(cmd)
        ys.append(p.y_val)
        next_t += dt_loop
        rest = next_t - time.perf_counter()
        if rest > 0:
            time.sleep(rest)
    # 停止（连续几个零点指令）
    for _ in range(10):
        client.moveByVelocityAsync(0.0, 0.0, 0.0, duration)
        time.sleep(dt_loop)
    return ts, cmds, ys


def analyze(tag, ts, cmds, ys):
    n = len(ts)
    dt = (ts[-1] - ts[0]) / max(1, n - 1)
    # 位置中心差分 -> 实际速度
    vel = []
    for i in range(n):
        a, b = max(0, i - 5), min(n - 1, i + 5)
        vel.append((ys[b] - ys[a]) / max(1e-6, ts[b] - ts[a]))
    c, lag = best_lag(cmds, vel, dt)
    exc = max(ys) - min(ys)
    print("  %-16s 采样 %.1f Hz  延迟 = %.2f s (corr %.2f)  横向摆幅 %.2f m"
          % (tag, 1.0 / dt, lag, c, exc))
    return lag


def axis_phase(client, home, secs=3.0, rate=RATE):
    """边横飞边下降：水平 +0.5 m/s 与垂直 -0.5 m/s 同时下发，看两者是否都被兑现。

    编队任务里出现过「垂直指令满速下降、飞机却常驻 4~7 米」的现象，
    这段单独判定：是命令通道在移动时不执行垂直分量，还是飞机被东西挡住了。
    """
    dt = 1.0 / rate
    t0 = time.perf_counter()
    next_t = t0
    samples = []
    while time.perf_counter() - t0 < secs:
        # ENU: 东 +0.5，上 -0.5
        client.moveByVelocityAsync(0.0, 0.5, 0.5, 0.1,
                                   drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                                   yaw_mode=airsim.YawMode(False, 0))
        p = client.getMultirotorState().kinematics_estimated.position
        samples.append((time.perf_counter() - t0,
                        (p.y_val - home[1], p.x_val - home[0], home[2] - p.z_val)))
        next_t += dt
        rest = next_t - time.perf_counter()
        if rest > 0:
            time.sleep(rest)
    for _ in range(8):
        client.moveByVelocityAsync(0.0, 0.0, 0.0, 0.1)
        time.sleep(dt)

    n = len(samples)
    def rate_of(axis):
        a, b = samples[max(0, n // 4)], samples[-1]
        return (b[1][axis] - a[1][axis]) / max(1e-3, b[0] - a[0])
    ve, vu = rate_of(0), rate_of(2)
    print("  水平位移 %.2f 米 -> 实测 %.2f m/s（指令 +0.50）" % (
        samples[-1][1][0] - samples[0][1][0], ve))
    print("  高度变化 %+.2f 米 -> 实测 %+.2f m/s（指令 -0.50）" % (
        samples[-1][1][2] - samples[0][1][2], vu))
    if abs(vu) < 0.2:
        print("  -> 横向运动中垂直分量几乎没被执行（这是被控对象行为，不是增益问题）")
    else:
        print("  -> 垂直分量被执行，之前的漂移更可能来自障碍物/树冠")
    return ve, vu


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else _ip()
    print("连接 AirSim %s ..." % ip)
    client = connect(ip)
    try:
        state = client.getMultirotorState().kinematics_estimated.position
        home = (state.x_val, state.y_val, state.z_val)     # 地面起飞点（NED）
        hover_z = state.z_val - HOVER_ALT
        client.takeoffAsync(timeout_sec=20).join()
        client.moveToZAsync(hover_z, MAX_VZ, timeout_sec=25).join()
        client.moveByVelocityAsync(0.0, 0.0, 0.0, 0.5).join()
        time.sleep(3.0)
        print("已悬停在离地 3 米，开始测延迟\n")

        res = {}
        for duration in (0.5, 0.05):
            ts, cmds, ys = run_phase(client, duration)
            res[duration] = analyze("duration=%.2f s" % duration, ts, cmds, ys)
            time.sleep(3.0)          # 两段之间稳一下

        print("\n=== 附加判定：边横飞边下降（先爬到 4.5 米，免得降到地面）===")
        client.moveToZAsync(home[2] - 4.5, MAX_VZ, timeout_sec=20).join()
        time.sleep(2.0)
        axis_phase(client, home)

        print("\n结论：")
        if res[0.5] - res[0.05] > 0.3:
            print("  延迟随指令有效期明显变化 -> 主因是 AirSim 的指令队列，"
                  "把 cmd_duration 设成约一个控制周期即可。")
        else:
            print("  两者接近 -> 延迟主要来自 AirSim 内部速度跟踪，"
                  "需要从 PID 增益上处理（降 kp、减 ki）。")
    finally:
        try:
            client.moveByVelocityAsync(0.0, 0.0, 0.0, 0.5)
            client.landAsync(timeout_sec=30).join()
            client.armDisarm(False)
            client.enableApiControl(False)
            print("已降落并交还控制权")
        except Exception as exc:                                    # noqa: BLE001
            print("降落阶段出错：%s" % exc)


if __name__ == "__main__":
    main()
