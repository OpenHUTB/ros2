# -*- coding: utf-8 -*-
# MuJoCo 小车自动打靶：车固定，炮塔自动识别立着的靶子并转动瞄准，依次击倒
#
# 用法：
#   python3 main.py                       # 打开可视化窗口（原来的体验）
#   python3 main.py --headless            # 无窗口跑完并打印成绩
#   python3 main.py --headless --wind 0.6 0 --report out.csv
import argparse
import csv
import math
import os
import sys
import time

import mujoco
import mujoco.viewer

from ballistics import score_shots, solve_launch_velocity
XML = """
<mujoco>
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" iterations="60"/>
  <default>
    <geom friction="1.0 0.005 0.0001" solref="0.02 1" solimp="0.9 0.95 0.001"/>
    <joint damping="0.05" armature="0.02"/>
  </default>
  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" directional="true"/>
    <light pos="-3 3 4" dir="1 -1 -1" diffuse="0.3 0.3 0.35"/>
    <geom name="floor" type="plane" size="8 8 0.05" rgba="0.75 0.75 0.75 1"/>
    <geom name="zone" type="box" pos="2.0 0 0.001" size="2.0 2.9 0.001" rgba="0.50 0.60 0.50 1"/>
    <geom name="line" type="box" pos="0.4 0 0.002" size="0.02 2.9 0.002" rgba="0.90 0.90 0.90 1"/>
    <body name="car" pos="0 0 0.05">
      <geom name="chassis" type="box" pos="0 0 0.04" size="0.28 0.16 0.035" rgba="0.20 0.45 0.85 1" mass="4.0"/>
      <body name="turret" pos="0.28 0 0.12">
        <joint name="turret_j" type="hinge" axis="0 0 1"/>
        <geom name="barrel" type="box" pos="0.15 0 0" size="0.15 0.03 0.03" rgba="0.25 0.25 0.25 1"/>
        <geom name="beam" type="cylinder" pos="2.0 0 0" size="0.012 1.7" euler="0 1.5707963 0" rgba="0.10 1.00 0.30 0.30" contype="0" conaffinity="0"/>
      </body>
      <body name="c_fl" pos="0.16 0.125 0">
        <joint name="c_fl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="c_fr" pos="0.16 -0.125 0">
        <joint name="c_fr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="c_rl" pos="-0.16 0.125 0">
        <joint name="c_rl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="c_rr" pos="-0.16 -0.125 0">
        <joint name="c_rr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
    </body>
    <body name="t0" pos="1.55 -1.25 0">
      <joint name="tj0" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.85 0.25 0.25 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx0" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy0" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t1" pos="2.00 0.00 0">
      <joint name="tj1" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.95 0.60 0.15 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx1" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy1" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t2" pos="1.55 1.25 0">
      <joint name="tj2" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.90 0.85 0.20 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx2" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy2" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t3" pos="3.00 -1.35 0">
      <joint name="tj3" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.30 0.70 0.35 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx3" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy3" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t4" pos="3.40 0.00 0">
      <joint name="tj4" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.25 0.55 0.85 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx4" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy4" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t5" pos="3.00 1.35 0">
      <joint name="tj5" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.60 0.35 0.80 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx5" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy5" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="ball0" pos="10 10 0.05">
      <joint name="bj0" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball1" pos="10 10.2 0.05">
      <joint name="bj1" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball2" pos="10 10.4 0.05">
      <joint name="bj2" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball3" pos="10 10.6 0.05">
      <joint name="bj3" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball4" pos="10 10.8 0.05">
      <joint name="bj4" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball5" pos="10 11.0 0.05">
      <joint name="bj5" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball6" pos="10 11.2 0.05">
      <joint name="bj6" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball7" pos="10 11.4 0.05">
      <joint name="bj7" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball8" pos="10 11.6 0.05">
      <joint name="bj8" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball9" pos="10 11.8 0.05">
      <joint name="bj9" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball10" pos="10 12.0 0.05">
      <joint name="bj10" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball11" pos="10 12.2 0.05">
      <joint name="bj11" type="free" damping="0" armature="0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="turret" joint="turret_j" kv="20" ctrlrange="-7 7" forcerange="-30 30"/>
  </actuator>
</mujoco>
"""


# 六个靶子的位置：前三近、后三远（车在原点）
TARGETS = [
    (1.55, -1.25), (2.00, 0.00), (1.55, 1.25),
    (3.00, -1.35), (3.40, 0.00), (3.00, 1.35),
]
BALL_Q = 11          # 球的 qpos 起始：炮塔0 + 轮1-4 + 靶5-10
BALL_V = 11
BALL_COUNT = 12      # XML 里预置的小球数量
BALL_MASS = 0.1
MUZZLE_Z = 0.17      # 炮口高度
TARGET_Z = 0.25      # 靶心高度
FLIGHT_TIME = 0.45   # 固定飞行时间（秒）
HIT_ANGLE = 0.25     # 靶子倒下判定阈值（rad）
DEFAULT_MAX_SHOTS = 12
DEFAULT_TIME_LIMIT = 18.0
TURRET_X = 0.28      # 炮塔安装位置（车身坐标）
TURRET_Y = 0.35      # 炮口到炮塔轴的距离


def clamp(v, lo=-15.0, hi=15.0):
    return max(lo, min(hi, v))


def muzzle_position(yaw):
    """炮口在世界系的位置。"""
    return (TURRET_X + TURRET_Y * math.cos(yaw),
            TURRET_Y * math.sin(yaw),
            MUZZLE_Z)


def run(headless=False, wind=(0.0, 0.0), max_shots=DEFAULT_MAX_SHOTS,
        time_limit=DEFAULT_TIME_LIMIT, report_path=None, verbose=True):
    """跑一次打靶，返回成绩 dict。

    headless=True 时不创建可视化窗口、也不按实时速度 sleep，适合在服务器 /
    CI 上批量评测；wind 是水平风加速度 (ax, ay)，单位 m/s²，会作为恒定外力
    作用在飞行中的小球上，发射时由 ``solve_launch_velocity`` 做补偿。
    """
    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    max_steps = max(1, int(time_limit / m.opt.timestep))
    ball_bodies = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ball%d" % i)
                   for i in range(BALL_COUNT)]

    # 锁定准星的十字 geom（平时透明，锁定目标时亮红）
    cx_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "cx%d" % i) for i in range(6)]
    cy_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "cy%d" % i) for i in range(6)]

    n = 0
    si = 0
    cool = 0
    last_tid = -1
    scan = 1.0
    scan_t = 0
    tid = -1
    down = [False] * 6
    shots = []
    reason = "达到时间上限"

    viewer = None
    if not headless:
        viewer = mujoco.viewer.launch_passive(m, d)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.distance = 9.0
        viewer.cam.elevation = -40
        viewer.cam.azimuth = 90

    try:
        while n < max_steps:
            # 击倒判定：靶子绕铰链翻过阈值就算倒
            for i in range(6):
                if not down[i] and abs(d.qpos[5 + i]) > HIT_ANGLE:
                    down[i] = True
                    # 归因给最近一次打这个靶、还没记为命中的那发
                    for shot in reversed(shots):
                        if shot["target"] == i and not shot["hit"]:
                            shot["hit"] = True
                            break
                    if verbose:
                        print("靶%d 倒下 (t=%.2fs)" % (i, d.time))
            if all(down):
                reason = "全部击倒"
                break
            if si >= max_shots:
                reason = "球用尽"
                break

            if n < 400:
                # 开场雷达扫描：激光左右扫场找目标
                d.ctrl[0] = 5.0 * scan
                scan_t += 1
                if scan_t > 130:
                    scan = -scan
                    scan_t = 0
            elif cool > 0:
                cool -= 1
                d.ctrl[0] = 0
            else:
                # 雷达式选靶：炮塔当前朝向最近的、还立着的靶
                tid = -1
                best = 999.0
                for i in range(6):
                    if not down[i]:
                        tx, ty = TARGETS[i]
                        a = math.atan2(ty, tx)
                        err = (a - d.qpos[0] + math.pi) % (2 * math.pi) - math.pi
                        if abs(err) < best:
                            best = abs(err)
                            tid = i
                if tid == -1:
                    d.ctrl[0] = 0
                else:
                    tx, ty = TARGETS[tid]
                    want = math.atan2(ty, tx)
                    err_t = (want - d.qpos[0] + math.pi) % (2 * math.pi) - math.pi
                    d.ctrl[0] = clamp(err_t * 6.0 - d.qvel[0] * 2.0)
                    if abs(err_t) < 0.05 and abs(d.qvel[0]) < 0.05:
                        # 炮塔对准了，解算初速度并发射
                        bq = BALL_Q + si * 7
                        bv = BALL_V + si * 6
                        start = muzzle_position(d.qpos[0])
                        aim = (tx, ty, TARGET_Z)
                        vx, vy, vz = solve_launch_velocity(
                            start, aim, FLIGHT_TIME, wind=wind)
                        d.qpos[bq:bq + 3] = list(start)
                        d.qpos[bq + 3:bq + 7] = [0, 0, 0, 1]
                        d.qvel[bv:bv + 3] = [vx, vy, vz]
                        d.qvel[bv + 3:bv + 6] = [0, 0, 0]
                        mujoco.mj_forward(m, d)

                        dist = math.hypot(tx - start[0], ty - start[1])
                        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
                        shots.append({
                            "shot": si + 1,
                            "time": float(d.time),
                            "target": tid,
                            "distance": dist,
                            "speed": speed,
                            "hit": False,
                        })
                        if tid != last_tid and verbose:
                            print("锁定靶%d 距离%.2f m" % (tid, dist))
                            last_tid = tid
                        if verbose:
                            print("第%d发 发射 力度%.1f" % (si + 1, speed))
                        si += 1
                        cool = 300

            # 风：对已经打出去的球施加恒定水平力（F = m·a）
            if wind[0] or wind[1]:
                for k in range(si):
                    bid = ball_bodies[k]
                    d.xfrc_applied[bid, 0] = BALL_MASS * wind[0]
                    d.xfrc_applied[bid, 1] = BALL_MASS * wind[1]

            mujoco.mj_step(m, d)

            # 锁定准星：被锁定的靶子上方亮起十字
            for i in range(6):
                on = 1.0 if i == tid and not down[i] else 0.0
                m.geom_rgba[cx_ids[i]] = [1.0, 1.0, 1.0, on]
                m.geom_rgba[cy_ids[i]] = [1.0, 1.0, 1.0, on]

            if viewer is not None:
                viewer.cam.lookat[0] = 0.6 * math.cos(d.qpos[0])
                viewer.cam.lookat[1] = 0.6 * math.sin(d.qpos[0])
                viewer.sync()
                time.sleep(0.002)
                if not viewer.is_running():
                    reason = "窗口关闭"
                    break

            if verbose and viewer is not None and n % 500 == 0:
                ang = [f"{d.qpos[5 + i]:.1f}" for i in range(6)]
                print(f"step={n} 炮塔={d.qpos[0]:.2f} 靶角度={ang}")
            n += 1
    finally:
        if viewer is not None:
            viewer.close()

    stats = score_shots(shots, sum(down), d.time, reason)
    stats["wind"] = tuple(float(x) for x in wind)
    stats["shots_detail"] = shots

    if report_path:
        write_report(report_path, shots, stats)
    return stats


def write_report(path, shots, stats):
    """把逐发记录写成 CSV，末尾附一行汇总。"""
    folder = os.path.dirname(os.path.abspath(path))
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    with open(path, "w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.writer(fh)
        writer.writerow(["shot", "sim_time_s", "target", "distance_m", "speed_m_s", "hit"])
        for shot in shots:
            writer.writerow([shot["shot"], "%.3f" % shot["time"], shot["target"],
                             "%.3f" % shot["distance"], "%.3f" % shot["speed"],
                             1 if shot["hit"] else 0])
        writer.writerow([])
        writer.writerow(["summary", "shots=%d" % stats["shots"],
                         "downed=%d" % stats["downed"],
                         "hit_rate=%.2f" % stats["hit_rate"],
                         "duration=%.2fs" % stats["duration"],
                         "reason=%s" % stats["reason"]])
    print("逐发记录已写入: %s" % os.path.abspath(path))


def format_summary(stats):
    """把成绩排成一段可读文本。"""
    avg = stats["avg_shots_per_down"]
    lines = [
        "=" * 46,
        "打靶成绩",
        "=" * 46,
        "  结束原因    : %s" % stats["reason"],
        "  发射发数    : %d" % stats["shots"],
        "  击倒靶数    : %d / 6" % stats["downed"],
        "  命中率      : %.1f%%" % (stats["hit_rate"] * 100.0),
        "  一发一个靶  : %d 个" % stats["first_shot_hits"],
        "  平均用弹    : %s" % ("%.2f 发/靶" % avg if avg is not None else "—"),
        "  耗时        : %.2f s" % stats["duration"],
        "  风加速度    : (%.2f, %.2f) m/s²" % tuple(stats.get("wind", (0.0, 0.0))),
        "=" * 46,
    ]
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="MuJoCo 小车自动打靶（可无界面批量评测）")
    ap.add_argument("--headless", action="store_true",
                    help="不打开可视化窗口，直接跑完并输出成绩")
    ap.add_argument("--wind", nargs=2, type=float, default=(0.0, 0.0),
                    metavar=("AX", "AY"), help="水平风加速度 m/s²，默认 0 0")
    ap.add_argument("--max-shots", type=int, default=DEFAULT_MAX_SHOTS,
                    help="最多发射几发，默认 %d" % DEFAULT_MAX_SHOTS)
    ap.add_argument("--time-limit", type=float, default=DEFAULT_TIME_LIMIT,
                    help="仿真时长上限（秒），默认 %.0f" % DEFAULT_TIME_LIMIT)
    ap.add_argument("--report", default=None,
                    help="把逐发记录写成 CSV")
    args = ap.parse_args(argv)

    stats = run(headless=args.headless, wind=tuple(args.wind),
                max_shots=args.max_shots, time_limit=args.time_limit,
                report_path=args.report)
    print()
    print(format_summary(stats))
    return 0 if stats["finished"] else 2


if __name__ == "__main__":
    sys.exit(main())
