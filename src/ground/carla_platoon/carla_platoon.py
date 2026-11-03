#!/usr/bin/env python3
"""
carla_platoon.py —— CARLA 多车协同编队（一字纵队 platoon）
================================================================================
与 2D 版共用同一套编队控制律（platoon_core）：
  - Leader 沿参考轨迹匀速领航；
  - follower 沿同一条轨迹保持恒定弧长间距跟进（队形不散）。
这里车辆状态取自 CARLA 车辆真实位姿（get_transform），控制量经
apply_control(VehicleControl(throttle/brake, steer)) 下发，横向用纯跟踪，
纵向用「进度误差 P 控制」调节车速以维持间距。

前置条件（本地机器）：
  - 已安装并启动 CARLA 模拟器（含 Python API，路径如
    /opt/carla/PythonAPI/carla/dist/carla-<ver>-py3.7-linux-x86_64.egg）
  - 安装依赖：pip install numpy
运行：
  python3 carla_platoon.py                     # 连 localhost:2000，4 车编队
  python3 carla_platoon.py --map Town03 --n 5 --spacing 8 --v 6 --frames 600
参数：
  --host/--port CARLA 服务地址   --map 地图名   --n 车辆数
  --spacing 期望车距(m)          --v 巡航车速(m/s)  --frames 运行帧数  --dt 步长
"""
import argparse
import sys
import time

import numpy as np

from platoon_core import (Bicycle, pure_pursuit_steer, project_to_path,
                          sample_path, build_path_from_waypoints,
                          normalize_angle)


# ---------------- 参考轨迹（与 2D 版同一形状：直线 + S 形横摆 + 直线）-------
def make_reference_path():
    xs = np.linspace(0.0, 160.0, 400)
    mid, w = 80.0, 12.0
    ys = np.where(
        xs < mid - 20, 0.0,
        np.where(xs > mid + 20, 0.0,
                 w * np.sin((xs - (mid - 20)) / 40.0 * np.pi))
    )
    wpts = np.column_stack([xs, ys])[::8]
    return build_path_from_waypoints(wpts, dense_per_m=2.0)


# ---------------- CARLA 连接与 spawn -------------------------------
def import_carla():
    try:
        import carla
        return carla
    except ImportError as e:
        print("[-] 未找到 carla Python API。请在本机启动 CARLA 并设置 PythonAPI 路径，例如：")
        print("    export PYTHONPATH=/opt/carla/PythonAPI/carla/dist/"
              "carla-0.9.15-py3.7-linux-x86_64.egg:$PYTHONPATH")
        raise SystemExit(e)


def spawn_platoon(carla, world, n_vehicles, spacing, start_xy=(40.0, 0.0)):
    """在参考轨迹起点附近按队形 spawn n_vehicles 辆车，返回 actor 列表。"""
    blueprint_lib = world.get_blueprint_library()
    bp = blueprint_lib.filter("vehicle.*")[0]   # 用默认车辆蓝图
    spawn_points = []
    path, s = make_reference_path()
    for i in range(n_vehicles):
        px, py = sample_path(path, s, max(-i * spacing, 0.0))
        yaw = np.degrees(np.arctan2(path[1][1] - path[0][1],
                                    path[1][0] - path[0][0]))
        spawn_points.append((start_xy[0] + px, start_xy[1] + py, yaw))

    actors = []
    for (sx, sy, syaw) in spawn_points:
        transform = carla.Transform(carla.Location(sx, sy, 0.5),
                                    carla.Rotation(yaw=syaw))
        actor = world.try_spawn_actor(bp, transform)
        if actor is None:
            print(f"[!] spawn 失败于 ({sx:.1f}, {sy:.1f})，尝试移除遮挡后重试")
            # 尝试物理冲突位置：清空再试
            time.sleep(0.2)
            actor = world.try_spawn_actor(bp, transform)
        if actor is None:
            continue
        actor.set_autopilot(False)
        actors.append(actor)
    print(f"[i] 已 spawn {len(actors)}/{n_vehicles} 辆车")
    return actors


# ---------------- 主控制循环 -------------------------------
def run(args):
    carla = import_carla()
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.load_world(args.map)
    print(f"[i] 已连接 CARLA，地图 {args.map}")

    actors = spawn_platoon(carla, world, args.n, args.spacing)
    if len(actors) < 2:
        raise SystemExit("[-] 车辆 spawn 数量不足，无法编队")

    path, s = make_reference_path()
    # 目标间距、纯跟踪参数（与 2D 版一致）
    spacing, v_set, lookahead, L = args.spacing, args.v, 6.0, 2.8
    kp, v_min, v_max = 1.2, 0.0, 10.0
    max_steer = 0.6

    s_leader = 0.0
    prev_pos = [actors[0].get_transform().location.x,
                actors[0].get_transform().location.y]
    t0 = time.time()

    for frame in range(args.frames):
        dt = time.time() - t0
        t0 = time.time()
        dt = min(dt, 0.05)                      # 防卡顿导致 dt 过大

        # 读取所有车辆真实位姿
        states = []
        for a in actors:
            tf = a.get_transform()
            states.append([tf.location.x, tf.location.y,
                           np.radians(tf.rotation.yaw)])
        states = np.asarray(states)

        # leader 实际进度（投影到路径）
        s_leader, _ = project_to_path(path, s, states[0, :2])

        # 各车控制
        for i, (actor, state) in enumerate(zip(actors, states)):
            if i == 0:
                # leader：追自己前方 lookahead 处的参考点，定速 v_set
                tgt = sample_path(path, s, s_leader + lookahead)
                v_des = v_set
            else:
                # follower：追沿轨迹滞后 leader 的参考点，纵向控间距
                s_des = max(s_leader - i * spacing, 0.0)
                tgt = sample_path(path, s, s_des)
                s_actual, _ = project_to_path(path, s, state[:2])
                v_des = v_set + kp * (s_des - s_actual)
                v_des = float(np.clip(v_des, v_min, v_max))

            delta = pure_pursuit_steer(state[0], state[1], state[2],
                                       tgt, lookahead, L, max_steer)

            # 速度 -> throttle/brake（简单 P 控）
            cur_speed = actor.get_velocity().length()
            kv = 0.8
            acc_cmd = kv * (v_des - cur_speed)
            throttle = float(np.clip(acc_cmd, 0.0, 1.0))
            brake = float(np.clip(-acc_cmd, 0.0, 1.0))
            steer = float(np.clip(delta / max_steer, -1.0, 1.0))

            ctrl = carla.VehicleControl(throttle=throttle, steer=steer,
                                        brake=brake)
            actor.apply_control(ctrl)

        if frame % 30 == 0:
            gap12 = np.linalg.norm(states[0, :2] - states[1, :2])
            print(f"[frame {frame:4d}] v={v_set}  gap(L-F1)={gap12:.1f} m"
                  f"  (target {spacing} m)  s_leader={s_leader:.1f} m")
        time.sleep(args.dt)

    print("[OK] 编队巡航结束，清理车辆")
    for a in actors:
        a.destroy()


def main():
    ap = argparse.ArgumentParser(description="CARLA 多车协同编队")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--map", default="Town03")
    ap.add_argument("--n", type=int, default=4, help="车辆数(含 leader)")
    ap.add_argument("--spacing", type=float, default=8.0, help="期望车距(m)")
    ap.add_argument("--v", type=float, default=6.0, help="巡航车速(m/s)")
    ap.add_argument("--frames", type=int, default=600, help="运行帧数")
    ap.add_argument("--dt", type=float, default=0.05, help="控制周期(s)")
    run(ap.parse_args())


if __name__ == "__main__":
    main()
