#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""展开阶段探针：复现「三架同点起飞、逐架展开」，逐拍打印位置与碰撞。

背景：编队任务每次都在展开阶段卡住 —— SimpleFlight2 报「未完全收敛」，
随后长机下降时把它撞开，之后所有航点超时。而 tune_hover.py 已证明单机
位置环没问题（同一组增益下稳态 RMS 0.026 米）。所以怀疑是**物理阻挡**
（树冠/建筑）或**机间相撞**。

simGetCollisionInfo 会给出撞到的对象名：静态物体会给出 mesh 名，
撞到别的载具会给出载具名 —— 这两者能直接区分开。

用法（虚拟机里，模拟器已启动）：
    python3 deploy_probe.py [AirSim IP]        # IP 默认取 config/params.yaml 的 airsim/ip
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
_SHAPES = _CFG.get("shapes") or {}


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


def _slots(shape_name):
    """把 params.yaml 里的队形偏移转成 (x, y) 列表。"""
    return [(float(s[0]), float(s[1])) for s in (_SHAPES.get(shape_name) or [])]


VEHICLES = _vehicles()
SLOTS = [(0.0, 0.0)] + _slots("line")      # 长机占原点，僚机按 line 队形左右展开
TAKEOFF_ALT = _num(_FLIGHT, "takeoff_altitude", 3.0)
ALT_SEP = _num(_FLIGHT, "alt_separation", 3.0)
KP = _num(_PID, "kp", 0.4)
KI = _num(_PID, "ki", 0.02)
KD = _num(_PID, "kd", 0.05)
TOL = _num(_FLIGHT, "waypoint_tolerance", 0.25)
TIMEOUT = _num(_FLIGHT, "deploy_timeout", 30.0)
RATE = _num(_FLIGHT, "control_rate", 20.0)
MAX_SPEED = _num(_FLIGHT, "max_speed", 2.0)
MAX_VZ = _num(_FLIGHT, "max_vertical_speed", 1.0)
CMD_DUR = _num(_FLIGHT, "cmd_duration", 0.1)
AIRSIM_TIMEOUT = _num(_AIRSIM, "timeout", 30.0)


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


class Probe(object):
    def __init__(self, client):
        self.client = client
        self.home = None
        self.seen = {}

    def pose(self, name):
        kin = self.client.getMultirotorState(vehicle_name=name).kinematics_estimated
        p, v = kin.position, kin.linear_velocity
        return ((p.y_val - self.home[1], p.x_val - self.home[0], self.home[2] - p.z_val),
                math.sqrt(v.x_val ** 2 + v.y_val ** 2 + v.z_val ** 2))

    def send(self, name, v_enu):
        ve = max(-MAX_SPEED, min(MAX_SPEED, v_enu[0]))
        vn = max(-MAX_SPEED, min(MAX_SPEED, v_enu[1]))
        vu = max(-MAX_VZ, min(MAX_VZ, v_enu[2]))
        self.client.moveByVelocityAsync(vn, ve, -vu, CMD_DUR,
                                        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                                        yaw_mode=airsim.YawMode(False, 0),
                                        vehicle_name=name)

    def poll_collisions(self, t):
        """每次状态变化都打印：谁撞到谁、穿透多深、当时在哪。"""
        for name in VEHICLES:
            try:
                info = self.client.simGetCollisionInfo(vehicle_name=name)
            except Exception:                                       # noqa: BLE001
                continue
            key = str(info.object_name) if info.has_collided else ""
            if self.seen.get(name) == key:
                continue
            self.seen[name] = key
            if not info.has_collided:
                continue
            pos, spd = self.pose(name)
            print("    [碰撞] t=%5.1f  %s 撞到 %s  穿透 %.3f 米  "
                  "位置 ENU=(%.2f, %.2f, %.2f)  速度 %.2f m/s"
                  % (t, name, info.object_name, info.penetration_depth,
                     pos[0], pos[1], pos[2], spd), flush=True)


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else _ip()
    client = airsim.MultirotorClient(ip=ip, timeout_value=AIRSIM_TIMEOUT)
    client.confirmConnection()
    print("已连接 AirSim %s，载具：%s"
          % (ip, getattr(client.listVehicles(), "vehicle_names", client.listVehicles())))

    probe = Probe(client)
    for name in VEHICLES:                      # 先全部落地，保证从同一状态开始
        client.enableApiControl(True, vehicle_name=name)
        client.armDisarm(True, vehicle_name=name)
        client.landAsync(timeout_sec=30, vehicle_name=name).join()
    time.sleep(1.0)
    lead = client.getMultirotorState(vehicle_name=VEHICLES[0]).kinematics_estimated.position
    probe.home = (lead.x_val, lead.y_val, lead.z_val)
    print("起飞点（AirSim NED）：(%.2f, %.2f, %.2f)\n" % probe.home)

    summary = []
    try:
        for i, name in enumerate(VEHICLES):
            level = TAKEOFF_ALT + (len(VEHICLES) - 1 - i) * ALT_SEP
            slot = (SLOTS[i][0], SLOTS[i][1], level)
            client.takeoffAsync(timeout_sec=20, vehicle_name=name).join()
            client.moveToZAsync(probe.home[2] - level, MAX_VZ, timeout_sec=25,
                                vehicle_name=name).join()
            print("  %s 起飞并爬升到 %.1f 米，目标空位 ENU=(%.1f, %.1f, %.1f)"
                  % (name, level, slot[0], slot[1], slot[2]), flush=True)

            pid = [PID(KP, KI, KD) for _ in range(3)]
            dt = 1.0 / RATE
            t0 = time.perf_counter()
            next_t = t0
            last_print = 0.0
            last_collision = 0.0
            ok = False
            while time.perf_counter() - t0 < TIMEOUT:
                now = time.perf_counter()
                t = now - t0
                pos, spd = probe.pose(name)
                err = [slot[j] - pos[j] for j in range(3)]
                dist = math.sqrt(sum(e * e for e in err))
                if dist < TOL:
                    ok = True
                    break
                probe.send(name, [pid[j].update(err[j], dt) for j in range(3)])
                if t - last_print >= 0.5:
                    last_print = t
                    print("    t=%5.1f  %-14s pos=(%6.2f,%6.2f,%6.2f) 误差=(%+5.2f,%+5.2f,%+5.2f) "
                          "|e|=%.2f 速度=%.2f" % (t, name, pos[0], pos[1], pos[2],
                                                  err[0], err[1], err[2], dist, spd),
                          flush=True)
                if t - last_collision >= 0.25:
                    last_collision = t
                    probe.poll_collisions(t)
                next_t += dt
                time.sleep(max(0.0, next_t - time.perf_counter()))

            pos, spd = probe.pose(name)
            print("  -> %s %s，用时 %.1f 秒，最终 ENU=(%.2f, %.2f, %.2f)，"
                  "残差 %.2f 米，末速 %.2f m/s\n"
                  % (name, "已就位" if ok else "未完全收敛", time.perf_counter() - t0,
                     pos[0], pos[1], pos[2],
                     math.dist(pos, slot), spd), flush=True)
            summary.append((name, ok, pos, math.dist(pos, slot)))

        print("=== 小结 ===")
        for name, ok, pos, dist in summary:
            print("  %-14s %-8s 残差 %.2f 米  位置 ENU=(%.2f, %.2f, %.2f)"
                  % (name, "就位" if ok else "未收敛", dist, pos[0], pos[1], pos[2]))
    finally:
        for name in reversed(VEHICLES):
            try:
                client.moveByVelocityAsync(0.0, 0.0, 0.0, 0.5, vehicle_name=name)
                client.landAsync(timeout_sec=30, vehicle_name=name).join()
                client.armDisarm(False, vehicle_name=name)
                client.enableApiControl(False, vehicle_name=name)
            except Exception as exc:                                # noqa: BLE001
                print("  %s 降落出错：%s" % (name, exc))
        print("三架已降落并交还控制权")


if __name__ == "__main__":
    main()
