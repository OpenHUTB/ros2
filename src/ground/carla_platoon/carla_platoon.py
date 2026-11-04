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
  - 已安装并启动 CARLA 模拟器（含 Python API）。CARLA 的 .egg 需与所用 Python
    版本匹配（例如 CARLA 0.9.15 对应 py3.7，更高版本提供 py3.8/py3.10 等），
    请按本地 Python 选择对应 .egg，或将 carla 目录加入 PYTHONPATH：
    /opt/carla/PythonAPI/carla/dist/carla-<ver>-py<X.Y>-linux-x86_64.egg
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

from platoon_core import (pure_pursuit_steer, project_to_path,
                          sample_path, build_path_from_waypoints,
                          platoon_initial_progress, leader_target_s,
                          follower_target_s)


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
              "carla-<ver>-py<X.Y>-linux-x86_64.egg:$PYTHONPATH")
        print("    注意：.egg 中的 py<X.Y> 需与本地 Python 版本匹配。")
        raise SystemExit(e)


def spawn_platoon(carla, world, n_vehicles, spacing):
    """按队形 spawn n_vehicles 辆车，返回 [(原始序号i, actor)]。

    与 2D 版 Platoon 使用同一套共享初始基准（platoon_initial_progress）：
    leader 置于路径起点前方 (n-1)*spacing 处，follower 沿路径依次滞后 spacing，
    起步位姿沿轨迹排开、互不重合（车辆有碰撞体积，重叠会导致后车生成失败）。

    保留**原始序号映射**：即使某辆车 spawn 失败被跳过，其余车仍按其原始序号 i
    计算间距基准（follower 目标 = 基准进度 - i*spacing），不会因列表压缩而错位。
    失败车辆不做无效重试（相同位姿重试不会成功）。"""
    blueprint_lib = world.get_blueprint_library()
    bp = blueprint_lib.filter("vehicle.*")[0]   # 用默认车辆蓝图
    path, s = make_reference_path()
    yaw_deg = np.degrees(np.arctan2(path[1][1] - path[0][1],
                                    path[1][0] - path[0][0]))
    s_leader0 = platoon_initial_progress(n_vehicles, spacing)
    spawn_points = []
    for i in range(n_vehicles):
        px, py = sample_path(path, s, s_leader0 - i * spacing)
        spawn_points.append((i, float(px), float(py), yaw_deg))

    actors = []
    for (i, px, py, syaw) in spawn_points:
        transform = carla.Transform(carla.Location(px, py, 0.5),
                                    carla.Rotation(yaw=syaw))
        actor = world.try_spawn_actor(bp, transform)
        if actor is None:
            print(f"[!] 车辆 {i} spawn 失败于 ({px:.1f}, {py:.1f})，跳过（保留序号 {i}）")
            continue
        actor.set_autopilot(False)
        actors.append((i, actor))
    print(f"[i] 已 spawn {len(actors)}/{n_vehicles} 辆车")
    return actors


# ---------------- 主控制循环 -------------------------------
def run(args):
    carla = import_carla()
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.load_world(args.map)
    print(f"[i] 已连接 CARLA，地图 {args.map}")

    actor_list = spawn_platoon(carla, world, args.n, args.spacing)
    if len(actor_list) < 2:
        for _, a in actor_list:
            a.destroy()
        raise SystemExit("[-] 车辆 spawn 数量不足(<2)，无法编队，已清理")

    # 原始序号映射：idxs 与 actors 一一对应（follower 用原始序号算间距）
    idxs = [i for i, _ in actor_list]
    actors = [a for _, a in actor_list]
    leader_i = min(idxs)                       # 最小序号的车辆为 leader
    path, s = make_reference_path()
    # 目标间距、纯跟踪参数（与 2D 版一致）
    spacing, v_set, lookahead, L = args.spacing, args.v, 6.0, 2.8
    kp, v_min, v_max = 1.2, 0.0, 10.0
    max_steer = 0.6

    s_leader = 0.0
    # 每车上次投影弧长（供 project_to_path 近邻搜索，避免每帧全量遍历路径）
    last_s = np.zeros(len(actors))
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

        # leader（最小序号车）实际进度（投影到路径，用上次弧长做近邻搜索）
        s_leader, _ = project_to_path(path, s, states[0, :2],
                                      hint_s=last_s[0])
        last_s[0] = s_leader

        # 各车控制（按原始序号 i 计算编队目标，两版共用共享核心函数）
        for k, (i, actor, state) in enumerate(zip(idxs, actors, states)):
            if i == leader_i:
                # leader：目标 = 当前进度 + 前视（共享核心，保证能转向）
                tgt = sample_path(path, s, leader_target_s(s_leader, lookahead))
                v_des = v_set
            else:
                # follower：目标 = 基准进度 - 原始序号 i*spacing（共享核心）
                s_des = follower_target_s(s_leader, i, spacing)
                tgt = sample_path(path, s, s_des)
                s_actual, _ = project_to_path(path, s, state[:2],
                                              hint_s=last_s[k])
                last_s[k] = s_actual
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
    for _, a in actor_list:
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
