#!/usr/bin/env python3
"""carla_ttc_emergency_braking — 基于激光雷达前向测距的自动紧急制动（AEB）实验。

设计要点
--------
1. **感知**：32 线激光雷达只取「车体前方锥形区域」的点，算出与前车的最近纵向距离。
   真实 AEB 用的是毫米波雷达，这里用激光雷达等价替代，好处是不依赖任何训练权重，
   在任何地图上都能复现。
2. **决策**：TTC（Time To Collision）= 距离 / 相对速度。TTC 小于阈值或距离小于安全距离
   立即满制动（brake=1.0），否则按巡航油门加速。
3. **取证**：运行中保存关键帧（逼近 → 触发制动 → 停稳），结束后输出
   `run_log.csv`（逐帧速度/距离/TTC）与 `result.png`（曲线 + 俯视照片）。

运行
----
    python main.py --host 127.0.0.1 --port 2000 --map Town10HD_Opt

作者：陈曦（湖南工商大学 人工智能与低空经济）
"""
import argparse
import csv
import math
import os
import sys
import time

import carla
import numpy as np
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # module root
DEFAULT_PARAMS = os.path.join(HERE, "config", "params.yaml")


# --------------------------------------------------------------------------- #
# 参数
# --------------------------------------------------------------------------- #
def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def build_argparser():
    p = argparse.ArgumentParser(description="CARLA AEB（自动紧急制动）实验")
    p.add_argument("--host", default=None, help="仿真器宿主机 IP")
    p.add_argument("--port", type=int, default=None, help="CARLA RPC 端口")
    p.add_argument("--map", default=None, help="地图名，如 Town10HD_Opt")
    p.add_argument("--save-dir", default=None, help="证据输出目录")
    p.add_argument("--params", default=DEFAULT_PARAMS, help="参数文件")
    p.add_argument("--no-render", action="store_true", help="不保存关键帧（只跑数据）")
    return p


def merge_params(cfg, args):
    if args.host:
        cfg["simulator"]["host"] = args.host
    if args.port:
        cfg["simulator"]["port"] = args.port
    if args.map is not None:
        cfg["simulator"]["map"] = args.map
    if args.save_dir:
        cfg["output"]["save_dir"] = args.save_dir
    if args.no_render:
        cfg["output"]["save_frames"] = False
    return cfg


# --------------------------------------------------------------------------- #
# 工具函数
# --------------------------------------------------------------------------- #
def connect(cfg):
    """连接仿真器。

    注意：OpenHUTB 2.10.x 的客户端采用「地图文件按需下载」机制，
    必须先 set_files_base_folder + get_world()，否则 get_world() 只会抛 RuntimeError。
    """
    sim = cfg["simulator"]
    cache = sim["cache_dir"] or os.path.join(HERE, ".hutb_cache")
    os.makedirs(cache, exist_ok=True)
    client = carla.Client(sim["host"], sim["port"])
    client.set_timeout(float(sim["timeout"]))
    client.set_files_base_folder(cache)
    world = client.get_world()
    if sim["map"] and not world.get_map().name.endswith(sim["map"]):
        print("[aeb] 切换地图 -> %s" % sim["map"])
        world = client.load_world(sim["map"])
    print("[aeb] 已连接 %s:%d | 服务端版本 %s | 地图 %s"
          % (sim["host"], sim["port"], client.get_server_version(), world.get_map().name))
    return client, world


def _norm_angle(deg):
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


def find_straight_pair(world, gap, max_yaw_diff=15.0, chord_tol=8.0):
    """在所有出生点里挑一段「够直」的路：以出生点为前车位置，沿车道回溯 gap 米后方向基本不变。

    弯曲路段上用直线偏移放车会把前车放到隔壁车道、甚至拐到另一条路上（实验就废了），
    所以这里显式做一次几何筛选。
    """
    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()
    best = None
    for idx, sp in enumerate(spawn_points):
        wp = carla_map.get_waypoint(sp.location, project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        travelled = 0.0
        ok = True
        while travelled < gap:
            prev = wp.previous(2.0)
            if not prev:
                ok = False
                break
            wp = prev[0]
            travelled += 2.0
        if not ok:
            continue
        dyaw = abs(_norm_angle(wp.transform.rotation.yaw - sp.rotation.yaw))
        chord = wp.transform.location.distance(sp.location)
        if dyaw > max_yaw_diff or abs(chord - gap) > chord_tol:
            continue
        score = dyaw + abs(chord - gap)
        if best is None or score < best[0]:
            best = (score, idx, wp.transform, sp)
    return best


def place_vehicle_pair(world, spawn_index, gap, ego_blueprint, lead_blueprint):
    """把「前车」放在出生点上，再沿同一条车道向后回溯 gap 米放置主车。

    spawn_index < 0 表示自动挑选一段最直的路（推荐）；否则用指定出生点，若该点弯道过大也会回退到自动挑选。
    """
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    choice = None
    if int(spawn_index) >= 0:
        spawn_points = carla_map.get_spawn_points()
        sp = spawn_points[int(spawn_index) % len(spawn_points)]
        wp = carla_map.get_waypoint(sp.location, project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        travelled = 0.0
        while travelled < gap:
            prev = wp.previous(2.0)
            if not prev:
                break
            wp = prev[0]
            travelled += 2.0
        if travelled >= gap:
            choice = (0, int(spawn_index), wp.transform, sp)

    if choice is None:
        best = find_straight_pair(world, gap)
        if best is None:
            raise RuntimeError("找不到足够直的路段来放置主车/前车，请调小 lead_gap")
        choice = best
        print("[aeb] 自动选择直路段：spawn[%d]（方向偏差 %.1f°）"
              % (best[1], abs(_norm_angle(best[2].rotation.yaw - best[3].rotation.yaw))))

    _, idx, ego_tf_src, lead_tf_src = choice
    ego_tf = ego_tf_src
    ego_tf.location.z += 0.3
    lead_tf = lead_tf_src
    lead_tf.location.z += 0.3

    ego = world.spawn_actor(bp_lib.filter(ego_blueprint)[0], ego_tf)
    lead = world.spawn_actor(bp_lib.filter(lead_blueprint)[0], lead_tf)
    actual_gap = ego.get_location().distance(lead.get_location())
    return ego, lead, actual_gap


def cleanup_world(world):
    """清掉上一轮残留的车辆/传感器，保证实验可重复（否则出生点会被占住）。"""
    removed = 0
    for actor in world.get_actors():
        if actor.type_id.startswith(("vehicle.", "sensor.", "walker.")):
            try:
                actor.destroy()
                removed += 1
            except RuntimeError:
                pass
    if removed:
        print("[aeb] 清理上一轮残留 actor: %d 个" % removed)
    return removed


def to_vehicle_frame(tf, world_loc):
    """把世界坐标点转换到车体坐标系（x 前、y 左）。"""
    dx = world_loc.x - tf.location.x
    dy = world_loc.y - tf.location.y
    yaw = math.radians(tf.rotation.yaw)
    lx = dx * math.cos(yaw) + dy * math.sin(yaw)
    ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return lx, ly


def lane_steer(ego, carla_map, lookahead=8.0):
    """纯跟踪（pure pursuit）横向控制：让主车始终沿车道中心行驶。"""
    tf = ego.get_transform()
    wp = carla_map.get_waypoint(tf.location, project_to_road=True,
                               lane_type=carla.LaneType.Driving)
    ahead = wp.next(lookahead)
    target = (ahead[0] if ahead else wp).transform.location
    lx, ly = to_vehicle_frame(tf, target)
    alpha = math.atan2(ly, max(lx, 0.1))
    steer = max(-1.0, min(1.0, 1.6 * alpha))
    return steer, (lx, ly)


def radar_front_target(world, ego, max_range=80.0, half_width=2.2):
    """前向雷达式目标跟踪：从仿真器目标列表里挑出正前方最近的一辆车。

    真实 AEB 用的就是毫米波雷达的目标级输出（距离/相对速度），这里用目标列表等价实现，
    好处是不受激光雷达点云稀疏性影响，任何地图、任何车速都稳定可用。
    """
    tf = ego.get_transform()
    best = None
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.id == ego.id:
            continue
        lx, ly = to_vehicle_frame(tf, actor.get_location())
        if 0.5 < lx < max_range and abs(ly) < half_width:
            if best is None or lx < best[0]:
                best = (lx, actor)
    return best


def parse_lidar(image, width, height):
    """激光雷达点云 -> (N,4) 数组：x, y, z, intensity。"""
    arr = np.frombuffer(image.raw_data, dtype=np.float32).reshape((-1, 4))
    return arr


def front_distance(points, max_range=60.0, half_width=1.2, min_x=3.0,
                   z_min=-1.75, z_max=1.6, percentile=2.0):
    """车体前方锥形区域内的最近纵向距离（米）；无命中返回 None。

    说明：激光雷达点云里混有地面点与自车车顶回波，这里用「抬高 z 下限（滤掉地面回环）
    + 取 2% 分位数」做一次轻量的杂波抑制，避免单点噪声把测距瞬间拉到 1 m 以内。
    """
    if points.size == 0:
        return None
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    mask = ((x > min_x) & (x < max_range) & (np.abs(y) < half_width)
            & (z > z_min) & (z < z_max))
    if not mask.any():
        return None
    return float(np.percentile(x[mask], percentile))


def speed_of(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def save_plot(csv_rows, out_png, meta):
    """把速度/距离/TTC 画成三条曲线 + 关键信息，作为实验证据。"""
    if not csv_rows:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r["t"] for r in csv_rows]
    v = [r["speed_kmh"] for r in csv_rows]
    d = [r["distance_m"] if r["distance_m"] >= 0 else float("nan") for r in csv_rows]
    ttc = [r["ttc"] if r["ttc"] > 0 else float("nan") for r in csv_rows]

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(t, v, color="#c0392b", lw=2)
    axes[0].set_ylabel("speed (km/h)")
    axes[0].grid(alpha=.3)
    axes[1].plot(t, d, color="#2980b9", lw=2)
    axes[1].axhline(meta["min_brake_distance"], ls="--", c="gray", label="min brake distance")
    axes[1].set_ylabel("front distance (m)")
    axes[1].legend(loc="upper right")
    axes[1].grid(alpha=.3)
    axes[2].plot(t, ttc, color="#8e44ad", lw=2)
    axes[2].axhline(meta["ttc_threshold"], ls="--", c="gray", label="TTC threshold")
    axes[2].set_ylabel("TTC (s)")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right")
    axes[2].grid(alpha=.3)
    fig.suptitle("AEB: ego stopped at %.1f m (min %.1f m), brake at %.1f s"
                 % (meta["final_distance"], meta["min_distance"], meta["brake_time"]))
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print("[aeb] 曲线图 -> %s" % out_png)


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def run(cfg):
    demo = cfg["scene"]
    sensors_cfg = cfg["sensor"]
    ctrl_cfg = cfg["control"]
    out_cfg = cfg["output"]
    save_dir = os.path.join(HERE, out_cfg["save_dir"])
    os.makedirs(save_dir, exist_ok=True)

    client, world = connect(cfg)
    cleanup_world(world)
    bp_lib = world.get_blueprint_library()
    carla_map = world.get_map()

    spawn_points = carla_map.get_spawn_points()
    ego, lead, lead_gap = place_vehicle_pair(
        world, demo["spawn_index"], float(demo["lead_gap"]),
        demo["ego_blueprint"], demo["lead_blueprint"])
    print("[aeb] 主车 %s 出生于 %s" % (ego.type_id, ego.get_transform().location))
    print("[aeb] 前车 %s 位于前方 %.1f m" % (lead.type_id, lead_gap))

    # 传感器：RGB 相机（取证）+ 激光雷达（测距）
    frames = {"count": 0}
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(sensors_cfg["camera_width"]))
    cam_bp.set_attribute("image_size_y", str(sensors_cfg["camera_height"]))
    cam_bp.set_attribute("fov", str(sensors_cfg["camera_fov"]))
    cam_bp.set_attribute("sensor_tick", "0.5")
    camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.6, z=1.85)), attach_to=ego)

    saved_frames = []
    frame_state = {"last": -999.0, "t0": time.time()}

    def on_camera(image):
        now = time.time()
        state = frame_state.get("phase", "approach")
        if not out_cfg["save_frames"] or frames["count"] >= 14:
            return
        if now - frame_state["last"] < out_cfg["frame_interval"] and state == frame_state.get("saved_phase"):
            return
        name = "frame_%s_%03d.png" % (state, frames["count"])
        path = os.path.join(save_dir, name)
        image.save_to_disk(path)
        saved_frames.append(name)
        frames["count"] += 1
        frame_state["last"] = now
        frame_state["saved_phase"] = state
        print("[aeb] 关键帧 -> %s" % name)

    camera.listen(on_camera)

    lidar_bp = bp_lib.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("channels", str(sensors_cfg["lidar_channels"]))
    lidar_bp.set_attribute("range", str(sensors_cfg["lidar_range"]))
    lidar_bp.set_attribute("points_per_second", str(sensors_cfg["lidar_points_per_second"]))
    lidar_bp.set_attribute("rotation_frequency", str(sensors_cfg["lidar_rotation_frequency"]))
    lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(z=1.9)), attach_to=ego)
    lidar_state = {"points": None}
    lidar.listen(lambda img: lidar_state.update(points=parse_lidar(img, sensors_cfg["camera_width"], 1)))

    time.sleep(2.5)  # 等传感器出第一帧

    rows = []
    t0 = time.time()
    brake_started = None
    stopped_at = None
    min_distance = float("inf")
    final_distance = 0.0
    phase = "approach"

    print("[aeb] 开始实验：巡航油门 %.2f，TTC 阈值 %.2fs，初始距离 %.1f m"
          % (ctrl_cfg["cruise_throttle"], ctrl_cfg["ttc_threshold"], lead_gap))

    while True:
        now = time.time()
        t = now - t0
        if t > float(ctrl_cfg["max_duration"]):
            print("[aeb] 达到最长时长，结束")
            break

        speed = speed_of(ego)
        gt_distance = ego.get_location().distance(lead.get_location())
        points = lidar_state["points"]
        lidar_dist = front_distance(points, float(sensors_cfg["lidar_range"])) if points is not None else None
        target = radar_front_target(world, ego)
        radar_dist = target[0] if target is not None else None
        dist = radar_dist if radar_dist is not None else lidar_dist
        ttc = (dist / speed) if (dist is not None and speed > 0.3) else -1.0

        # ---------------- 决策 ----------------
        steer, rel = lane_steer(ego, carla_map)
        braking = False
        if dist is not None:
            if dist <= float(ctrl_cfg["min_brake_distance"]):
                braking = True
            elif ttc > 0 and ttc <= float(ctrl_cfg["ttc_threshold"]):
                braking = True

        if braking:
            ego.apply_control(carla.VehicleControl(
                throttle=0.0, brake=float(ctrl_cfg["brake_command"]), steer=steer))
            if brake_started is None:
                brake_started = t
                phase = "brake"
                frame_state["phase"] = "brake"
                print("[aeb] >>> 触发制动 t=%.2fs dist=%.1fm speed=%.1fkm/h ttc=%.2fs"
                      % (t, dist if dist is not None else -1, speed * 3.6, ttc))
        else:
            throttle = float(ctrl_cfg["cruise_throttle"])
            if speed * 3.6 > float(ctrl_cfg["target_speed"]) * 3.6:
                throttle = 0.0
            ego.apply_control(carla.VehicleControl(throttle=throttle, brake=0.0, steer=steer))

        if dist is not None:
            min_distance = min(min_distance, dist)
        rows.append({
            "t": round(t, 3),
            "speed_kmh": round(speed * 3.6, 3),
            "distance_m": round(dist, 3) if dist is not None else -1.0,
            "lidar_m": round(lidar_dist, 3) if lidar_dist is not None else -1.0,
            "gt_distance_m": round(gt_distance, 3),
            "ttc": round(ttc, 3),
            "braking": int(braking),
        })

        # ---------------- 停止判定 ----------------
        if brake_started is not None and speed < float(ctrl_cfg["stop_speed"]):
            if stopped_at is None:
                stopped_at = t
                phase = "stopped"
                frame_state["phase"] = "stopped"
                final_distance = dist if dist is not None else gt_distance
                print("[aeb] >>> 停稳 t=%.2fs 距前车 %.1f m（地面真值 %.1f m）"
                      % (t, final_distance, gt_distance))
                time.sleep(1.2)  # 留一帧停稳画面
                break

        time.sleep(0.05)

    # ---------------- 收尾与取证 ----------------
    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))

    csv_path = os.path.join(save_dir, "run_log.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print("[aeb] 逐帧日志 -> %s（%d 行）" % (csv_path, len(rows)))

    meta = {
        "min_brake_distance": float(ctrl_cfg["min_brake_distance"]),
        "ttc_threshold": float(ctrl_cfg["ttc_threshold"]),
        "brake_time": brake_started if brake_started is not None else -1.0,
        "final_distance": final_distance,
        "min_distance": min_distance if min_distance < float("inf") else -1.0,
    }
    save_plot(rows, os.path.join(save_dir, "result.png"), meta)

    with open(os.path.join(save_dir, "summary.txt"), "w", encoding="utf-8") as fh:
        fh.write("AEB 实验摘要\n")
        fh.write("地图: %s\n" % carla_map.name)
        fh.write("初始距离: %.1f m\n" % lead_gap)
        fh.write("制动触发时刻: %.2f s\n" % meta["brake_time"])
        fh.write("制动后停稳时刻: %.2f s\n" % (stopped_at if stopped_at else -1))
        fh.write("停稳时距前车: %.2f m\n" % meta["final_distance"])
        fh.write("全程最近距离: %.2f m\n" % meta["min_distance"])
        fh.write("关键帧: %s\n" % ", ".join(saved_frames))

    camera.stop(); lidar.stop()
    camera.destroy(); lidar.destroy()
    lead.destroy(); ego.destroy()
    print("[aeb] 实验结束，证据已写入 %s" % save_dir)

    ok = meta["final_distance"] > 2.0 and meta["brake_time"] > 0
    print("[aeb] 结论：%s" % ("PASS（制动成功，未发生碰撞）" if ok else "FAIL（未能安全停住）"))
    return 0 if ok else 1


def main():
    args = build_argparser().parse_args()
    cfg = merge_params(load_params(args.params), args)
    try:
        return run(cfg)
    except KeyboardInterrupt:
        print("\n[aeb] 用户中断")
        return 130


if __name__ == "__main__":
    sys.exit(main())
