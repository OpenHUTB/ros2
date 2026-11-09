#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐架诊断：真值/估计状态对比 + 执行机构响应（单机轮流做，约 2 分钟）。

为什么需要它：同一份代码、同一时刻、同样约 3 米路程，SimpleFlight3 用 4.6 秒
就位，SimpleFlight2 却绕圈 30 秒不收敛（残差一直 1.1-1.9 米、高度还比目标高
约 1 米），循环频率实测 20.0 Hz —— 增益、频率、指令队列都已排除。剩下两种可能：

  A. 某一架的**状态估计**不对（控制器在追一个假位姿）：本脚本把
     getMultirotorState（估计）和 simGetGroundTruthKinematics（真值）并排打印；
  B. 执行机构对某些方向的命令响应差：本脚本分别给东向、垂直方向常值命令，
     量实际位移（窗口 3 秒，避免被上升时间污染）。

最后再让每一架用同一套 PID 从原点飞到 (0, -3, 4) —— 复现 SF2 那个动作。

用法（虚拟机里，模拟器已启动）：
    python3 vehicle_diag.py [AirSim IP]    # IP 默认取 config/params.yaml 的 airsim/ip
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


def _vehicles():
    leader = _AIRSIM.get("leader") or "SimpleFlight"
    followers = _AIRSIM.get("followers") or ["SimpleFlight2", "SimpleFlight3"]
    return [leader] + list(followers)


VEHICLES = _vehicles()
HOVER_Z = 4.0                     # 诊断用固定高度：复现 SF2 当初那个动作，故意不取 params
RATE = _num(_FLIGHT, "control_rate", 20.0)
MAX_SPEED = _num(_FLIGHT, "max_speed", 2.0)
MAX_VZ = _num(_FLIGHT, "max_vertical_speed", 1.0)
CMD_DUR = _num(_FLIGHT, "cmd_duration", 0.1)
KP = _num(_PID, "kp", 0.4)
KI = _num(_PID, "ki", 0.02)
KD = _num(_PID, "kd", 0.05)
TOL = _num(_FLIGHT, "waypoint_tolerance", 0.25)
AIRSIM_TIMEOUT = _num(_AIRSIM, "timeout", 30.0)
SLOT = (0.0, -3.0, 4.0)           # 诊断目标（同上，固定）


class PID(object):
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


class Diag(object):
    def __init__(self, client):
        self.client = client
        self.home = None

    def truth(self, name):
        """真值位置与速度（ENU，相对起飞点）。"""
        k = self.client.simGetGroundTruthKinematics(vehicle_name=name)
        p, v = k.position, k.linear_velocity
        return ((p.y_val - self.home[1], p.x_val - self.home[0], self.home[2] - p.z_val),
                (v.y_val, v.x_val, -v.z_val))

    def estimate(self, name):
        """控制器实际用到的估计位置与速度（ENU，相对起飞点）。"""
        st = self.client.getMultirotorState(vehicle_name=name)
        k = st.kinematics_estimated
        p, v = k.position, k.linear_velocity
        return ((p.y_val - self.home[1], p.x_val - self.home[0], self.home[2] - p.z_val),
                (v.y_val, v.x_val, -v.z_val), st)

    def send(self, name, v_enu):
        ve = max(-MAX_SPEED, min(MAX_SPEED, v_enu[0]))
        vn = max(-MAX_SPEED, min(MAX_SPEED, v_enu[1]))
        vu = max(-MAX_VZ, min(MAX_VZ, v_enu[2]))
        self.client.moveByVelocityAsync(vn, ve, -vu, CMD_DUR,
                                        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                                        yaw_mode=airsim.YawMode(False, 0),
                                        vehicle_name=name)

    def state_report(self, name):
        pos_e, vel_e, st = self.estimate(name)
        pos_t, vel_t = self.truth(name)
        print("    估计 pos=(%6.2f,%6.2f,%6.2f) vel=(%5.2f,%5.2f,%5.2f)  ready=%s arm=%s"
              % (pos_e[0], pos_e[1], pos_e[2], vel_e[0], vel_e[1], vel_e[2],
                 getattr(st, "ready", "?"), getattr(st, "armed", "?")))
        print("    真值 pos=(%6.2f,%6.2f,%6.2f) vel=(%5.2f,%5.2f,%5.2f)  "
              "位置差=(%+.2f,%+.2f,%+.2f) 速度差=%.2f"
              % (pos_t[0], pos_t[1], pos_t[2], vel_t[0], vel_t[1], vel_t[2],
                 pos_e[0] - pos_t[0], pos_e[1] - pos_t[1], pos_e[2] - pos_t[2],
                 math.dist(vel_e, vel_t)))
        return pos_e, pos_t

    def constant_pulse(self, name, axis, value, secs=3.0):
        """常值速度命令跑 secs 秒，返回 (位移, 平均速度)。"""
        rate_dt = 1.0 / RATE
        p0 = None
        t0 = time.time()
        p1 = None
        while time.time() - t0 < secs:
            v = [0.0, 0.0, 0.0]
            v[axis] = value
            self.send(name, v)
            p = self.estimate(name)[0]
            if p0 is None:
                p0 = p
            p1 = p
            time.sleep(max(0.0, rate_dt - (time.time() - t0) % rate_dt))
        elapsed = time.time() - t0
        for _ in range(6):
            self.send(name, [0.0, 0.0, 0.0])
            time.sleep(rate_dt)
        return (p1[axis] - p0[axis]), (p1[axis] - p0[axis]) / max(1e-3, elapsed)

    def pid_hold(self, name, target, secs=14.0):
        """同一套 PID 飞向 target，返回 (是否进容差, 用时, 最终残差, 平均控制频率)。"""
        pids = [PID(KP, KI, KD) for _ in range(3)]
        rate_dt = 1.0 / RATE
        t0 = time.time()
        t_prev = t0
        n = 0
        last_print = 0.0
        while time.time() - t0 < secs:
            t_now = time.time()
            dt = min(max(t_now - t_prev, 0.25 / RATE), 4.0 / RATE)
            t_prev = t_now
            n += 1
            pos = self.estimate(name)[0]
            err = [target[i] - pos[i] for i in range(3)]
            dist = math.sqrt(sum(e * e for e in err))
            if dist < TOL:
                return True, t_now - t0, dist, n / max(1e-3, t_now - t0)
            if t_now - t0 - last_print >= 1.0:
                last_print = t_now - t0
                print("      t=%4.1f pos=(%6.2f,%6.2f,%6.2f) 目标=(%5.1f,%5.1f,%5.1f) "
                      "残差=%.2f" % (t_now - t0, pos[0], pos[1], pos[2],
                                     target[0], target[1], target[2], dist), flush=True)
            self.send(name, [pids[i].update(err[i], dt) for i in range(3)])
            time.sleep(max(0.0, rate_dt - (time.time() - t_now)))
        pos = self.estimate(name)[0]
        return (math.dist(pos, target) < TOL, time.time() - t0,
                math.dist(pos, target), n / max(1e-3, time.time() - t0))


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else _ip()
    client = airsim.MultirotorClient(ip=ip, timeout_value=AIRSIM_TIMEOUT)
    client.confirmConnection()
    print("已连接 AirSim %s\n" % ip)
    diag = Diag(client)

    for name in VEHICLES:
        client.enableApiControl(True, vehicle_name=name)
        client.armDisarm(True, vehicle_name=name)
        client.landAsync(timeout_sec=30, vehicle_name=name).join()
    time.sleep(1.0)
    lead = client.getMultirotorState(vehicle_name=VEHICLES[0]).kinematics_estimated.position
    diag.home = (lead.x_val, lead.y_val, lead.z_val)
    print("起飞点 NED = (%.2f, %.2f, %.2f)\n" % diag.home)

    try:
        for name in VEHICLES:
            print("=== %s ===" % name, flush=True)
            client.takeoffAsync(timeout_sec=20, vehicle_name=name).join()
            client.moveToZAsync(diag.home[2] - HOVER_Z, MAX_VZ, timeout_sec=25,
                                vehicle_name=name).join()
            time.sleep(2.0)
            print("  [1] 悬停在 %.1f 米，状态对比：" % HOVER_Z)
            diag.state_report(name)

            print("  [2] 东向 +0.5 m/s 常值命令 3 秒：", end="", flush=True)
            d, v = diag.constant_pulse(name, 0, 0.5)
            print("位移 %+.2f 米，平均速度 %+.2f m/s（应为 +0.5）" % (d, v))

            print("  [3] 垂直 -0.5 m/s（下降）常值命令 3 秒：", end="", flush=True)
            d, v = diag.constant_pulse(name, 2, -0.5)
            print("位移 %+.2f 米，平均速度 %+.2f m/s（应为 -0.5）" % (d, v))

            print("  [4] 用同一套 PID 从原点飞向 (%.0f, %.0f, %.0f)：" % SLOT)
            ok, secs, dist, hz = diag.pid_hold(name, SLOT)
            print("      -> %s，用时 %.1f 秒，最终残差 %.2f 米，平均控制频率 %.1f Hz"
                  % ("就位" if ok else "未收敛", secs, dist, hz), flush=True)

            client.moveByVelocityAsync(0.0, 0.0, 0.0, 0.5, vehicle_name=name)
            client.landAsync(timeout_sec=30, vehicle_name=name).join()
            time.sleep(1.0)
            print()
    finally:
        for name in VEHICLES:
            try:
                client.armDisarm(False, vehicle_name=name)
                client.enableApiControl(False, vehicle_name=name)
            except Exception:                                       # noqa: BLE001
                pass
        print("三架已降落并交还控制权")


if __name__ == "__main__":
    main()
