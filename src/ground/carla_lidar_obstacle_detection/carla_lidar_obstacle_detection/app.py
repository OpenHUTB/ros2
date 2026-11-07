#!/usr/bin/env python3
"""carla_lidar_obstacle_detection — 激光雷达点云 3D 目标检测实验。

设计要点
--------
1. **数据**：语义激光雷达（sensor.lidar.semantic）每帧点云里除 xyz 外还带 cos_incidence、
   物体类别号、物体实例号，等价于「已经做过实例分割的点云」。
2. **检测**：按实例号把点分组 → 每组算质心、尺寸、点数、到自车距离；
   点数太少的目标直接丢弃（去噪）。类别用 actor 的 type_id 归一化（vehicle/walker/...）。
3. **证据**：一帧俯视（BEV）点云图 + 每个目标的检测框与标注，外加 12 帧统计表
   `detections.csv`，最后和仿真器里真实存在的目标数对账（召回率）。

运行
----
    python main.py --host 127.0.0.1 --map Town10HD_Opt

作者：陈曦（湖南工商大学 人工智能与低空经济）
"""
import argparse
import csv
import math
import os
import random
import sys
import time

import carla
import numpy as np
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # module root
DEFAULT_PARAMS = os.path.join(HERE, "config", "params.yaml")

CLASS_TAG = {"vehicle": 10, "walker": 4, "other": 3}


def parse_semantic_lidar(raw_data):
    """解析语义激光雷达原始数据。

    这个传感器每点 6 个 32 位字：x, y, z, cos(incidence), **ObjIdx(u32)**, **ObjTag(u32)**。
    后两个是整数而不是浮点数，必须先按 uint32 读、再单独把前 3 个重新解释成 float32，
    否则会把实例号当成「极小浮点数」而永远分不出目标。
    """
    u = np.frombuffer(raw_data, dtype=np.uint32).reshape((-1, 6)).copy()
    xyz = u[:, :3].view(np.float32)
    obj_idx = u[:, 4]
    obj_tag = u[:, 5]
    return xyz, obj_idx, obj_tag


def class_from_tag(tag):
    if tag in (14, 15, 16, 18):   # Car / Truck / Bus / Motorcycle
        return "vehicle"
    if tag in (12, 13):           # Pedestrian / Rider
        return "walker"
    return "other"


def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def build_argparser():
    p = argparse.ArgumentParser(description="CARLA 激光雷达 3D 目标检测实验")
    p.add_argument("--host", default=None)
    p.add_argument("--port", type=int, default=None)
    p.add_argument("--map", default=None)
    p.add_argument("--save-dir", default=None)
    p.add_argument("--params", default=DEFAULT_PARAMS)
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
    return cfg


def connect(cfg):
    sim = cfg["simulator"]
    cache = sim["cache_dir"] or os.path.join(HERE, ".hutb_cache")
    os.makedirs(cache, exist_ok=True)
    client = carla.Client(sim["host"], sim["port"])
    client.set_timeout(float(sim["timeout"]))
    client.set_files_base_folder(cache)
    world = client.get_world()
    if sim["map"] and not world.get_map().name.endswith(sim["map"]):
        world = client.load_world(sim["map"])
    print("[lidar] 已连接 %s:%d | 地图 %s" % (sim["host"], sim["port"], world.get_map().name))
    return client, world


def cleanup_world(world):
    n = 0
    for actor in world.get_actors():
        if actor.type_id.startswith(("vehicle.", "sensor.", "walker.")):
            try:
                actor.destroy()
                n += 1
            except RuntimeError:
                pass
    if n:
        print("[lidar] 清理上一轮残留 actor: %d 个" % n)


def choose_ego_spawn(world):
    """挑一个周围比较空旷的出生点，方便把检测结果数清楚。"""
    carla_map = world.get_map()
    spawns = carla_map.get_spawn_points()
    best = None
    for idx, sp in enumerate(spawns):
        wp = carla_map.get_waypoint(sp.location, project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        nxt = wp.next(25.0)
        if not nxt:
            continue
        dyaw = abs(((nxt[0].transform.rotation.yaw - sp.rotation.yaw + 180) % 360) - 180)
        if best is None or dyaw < best[0]:
            best = (dyaw, idx, sp)
    return best[1], best[2]


def spawn_traffic(world, ego, n_vehicles, n_walkers, seed, near_range=(8.0, 35.0)):
    """在自车周围 8~35 m 的**出生点**上摆车。

    注意：激光雷达水平角分辨率很有限（每圈只有几十个采样点），
    沿车道用 waypoint 偏移摆的车经常一个点都扫不到；直接借用地图自带的出生点
    （它们本来就落在车道中心、且彼此有重叠的可见关系）成功率最高。
    """
    rng = random.Random(seed)
    bp_lib = world.get_blueprint_library()
    ego_loc = ego.get_location()
    spawns = []
    for sp in world.get_map().get_spawn_points():
        d = sp.location.distance(ego_loc)
        if near_range[0] <= d <= near_range[1]:
            spawns.append((d, sp))
    spawns.sort(key=lambda x: x[0])
    vehicles = []
    for d, sp in spawns[: int(n_vehicles) * 2]:
        if len(vehicles) >= n_vehicles:
            break
        bp = rng.choice(bp_lib.filter("vehicle.*"))
        tf = sp
        tf.location.z += 0.3
        try:
            vehicles.append(world.spawn_actor(bp, tf))
        except RuntimeError:
            continue
    walkers = []
    for _ in range(n_walkers):
        bp = rng.choice(bp_lib.filter("walker.pedestrian.*"))
        loc = world.get_random_location_from_navigation()
        if loc is None or loc.distance(ego_loc) > 20.0:
            continue
        try:
            walkers.append(world.spawn_actor(bp, carla.Transform(loc)))
        except RuntimeError:
            pass
    print("[lidar] 布置目标：车 %d 辆（8~35 m 内的出生点，候选 %d 个）、行人 %d 个"
          % (len(vehicles), len(spawns), len(walkers)))
    return vehicles, walkers


def classify(actor):
    tid = actor.type_id if actor is not None else ""
    if tid.startswith("vehicle."):
        return "vehicle"
    if tid.startswith("walker."):
        return "walker"
    return "other"


def detect_objects(world, cloud, cfg):
    """按实例号分组做 3D 检测，返回 [dict(instance, cls, x, y, z, dx, dy, dz, n, dist)]。"""
    if cloud is None:
        return []
    xyz, obj_idx, obj_tag = cloud
    if xyz.size == 0:
        return []
    min_pts = int(cfg["detect"]["min_points_per_object"])
    max_range = float(cfg["detect"]["max_range"])
    out = []
    for inst in np.unique(obj_idx):
        if inst == 0:      # 0 = 静态背景（建筑/地面/路灯），不是交通参与者
            continue
        sel_mask = obj_idx == inst
        if sel_mask.sum() < min_pts:
            continue
        pts = xyz[sel_mask]
        c = pts.mean(axis=0)
        dist = float(math.sqrt(c[0] ** 2 + c[1] ** 2))
        if not np.isfinite(dist) or dist > max_range:
            continue
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        actor = world.get_actor(int(inst))
        cls = classify(actor) if actor is not None else class_from_tag(int(obj_tag[sel_mask][0]))
        out.append({
            "instance": int(inst),
            "cls": cls,
            "x": float(c[0]), "y": float(c[1]), "z": float(c[2]),
            "dx": float(hi[0] - lo[0]), "dy": float(hi[1] - lo[1]), "dz": float(hi[2] - lo[2]),
            "n": int(sel_mask.sum()), "dist": dist,
        })
    out.sort(key=lambda d: d["dist"])
    return out


def plot_bev(points, dets, out_png, meta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(9, 8))
    if points is not None and len(points):
        pts = points[np.isfinite(points).all(axis=1)]
        if len(pts):
            ax.scatter(pts[:, 0], pts[:, 1], s=0.6, c="#9aa5b1", label="raw points")
    for d in dets:
        color = "#c0392b" if d["cls"] == "vehicle" else ("#2980b9" if d["cls"] == "walker" else "#7f8c8d")
        ax.add_patch(Rectangle((d["x"] - d["dx"] / 2, d["y"] - d["dy"] / 2), d["dx"], d["dy"],
                               fill=False, ec=color, lw=1.6))
        ax.text(d["x"], d["y"], "%s %.1fm" % (d["cls"], d["dist"]), fontsize=7, color=color)
    ax.scatter([0], [0], marker="^", s=90, c="#f39c12", label="ego")
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_title("LiDAR detection (BEV): %d objects | vehicle=%d walker=%d other=%d\n"
                 "range=%.0f m, accumulated frames=%d"
                 % (len(dets), meta["n_vehicle"], meta["n_walker"], meta["n_other"],
                    meta["range"], meta["clips"]))
    ax.grid(alpha=.3)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def run(cfg):
    save_dir = os.path.join(HERE, cfg["output"]["save_dir"])
    os.makedirs(save_dir, exist_ok=True)

    client, world = connect(cfg)
    cleanup_world(world)
    bp_lib = world.get_blueprint_library()

    idx, sp = choose_ego_spawn(world)
    tf = sp
    tf.location.z += 0.3
    ego = world.spawn_actor(bp_lib.filter(cfg["scene"]["ego_blueprint"])[0], tf)
    print("[lidar] 主车出生于 spawn[%d] %s" % (idx, tf.location))

    vehicles, walkers = spawn_traffic(world, ego, int(cfg["scene"]["n_traffic_vehicles"]),
                                      int(cfg["scene"]["n_walkers"]),
                                      int(cfg["scene"]["traffic_seed"]))

    sc = cfg["sensor"]
    lidar_bp = bp_lib.find("sensor.lidar.ray_cast_semantic")
    lidar_bp.set_attribute("channels", str(sc["lidar_channels"]))
    lidar_bp.set_attribute("range", str(sc["lidar_range"]))
    lidar_bp.set_attribute("points_per_second", str(sc["lidar_points_per_second"]))
    lidar_bp.set_attribute("rotation_frequency", str(sc["lidar_rotation_frequency"]))
    lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(z=2.2)), attach_to=ego)
    latest = {"p": None}
    lidar.listen(lambda img: latest.update(p=parse_semantic_lidar(img.raw_data)))
    time.sleep(3.0)

    rows = []
    last_points = None
    last_dets = []
    buf = []
    n_acc = max(1, int(cfg["detect"].get("accumulate_sweeps", 3)))
    for k in range(int(cfg["detect"]["scans"])):
        cloud = latest["p"]
        if cloud is not None:
            buf.append(cloud)
            if len(buf) > n_acc:
                buf.pop(0)
        merged = None
        if buf:
            merged = (np.concatenate([c[0] for c in buf]),
                      np.concatenate([c[1] for c in buf]),
                      np.concatenate([c[2] for c in buf]))
        dets = detect_objects(world, merged, cfg)
        last_points = merged[0] if merged is not None else None
        last_dets = dets
        n_pts = 0 if merged is None else len(merged[0])
        rows.append({"scan": k, "points": n_pts,
                     "detected": len(dets),
                     "vehicles": sum(1 for d in dets if d["cls"] == "vehicle"),
                     "walkers": sum(1 for d in dets if d["cls"] == "walker")})
        print("[lidar] scan %2d: %6d 点 -> 检出 %d 个目标（车 %d / 人 %d）"
              % (k, rows[-1]["points"], len(dets), rows[-1]["vehicles"], rows[-1]["walkers"]))
        time.sleep(float(cfg["detect"]["scan_interval"]))

    with open(os.path.join(save_dir, "detections.csv"), "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=["instance", "cls", "x", "y", "z", "dx", "dy", "dz",
                                                "n", "dist"])
        writer.writeheader()
        for d in last_dets:
            writer.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()})

    with open(os.path.join(save_dir, "scans.csv"), "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    n_vehicle = sum(1 for d in last_dets if d["cls"] == "vehicle")
    n_walker = sum(1 for d in last_dets if d["cls"] == "walker")
    meta = {"n_vehicle": n_vehicle, "n_walker": n_walker,
            "n_other": len(last_dets) - n_vehicle - n_walker,
            "range": float(cfg["detect"]["max_range"]), "clips": int(cfg["detect"]["scans"])}
    plot_bev(last_points if last_points is not None else np.zeros((0, 3)), last_dets,
             os.path.join(save_dir, "bev_detection.png"), meta)
    truth_v = truth_w = 0
    for actor in world.get_actors():
        if actor.id == ego.id:
            continue
        if actor.type_id.startswith(("vehicle.", "walker.")):
            d = math.sqrt((actor.get_location().x - ego.get_location().x) ** 2
                          + (actor.get_location().y - ego.get_location().y) ** 2)
            if d <= meta["range"]:
                truth_v += 1 if actor.type_id.startswith("vehicle.") else 0
                truth_w += 1 if actor.type_id.startswith("walker.") else 0

    with open(os.path.join(save_dir, "summary.txt"), "w", encoding="utf-8") as fh:
        fh.write("激光雷达检测实验摘要\n")
        fh.write("地图: %s\n" % world.get_map().name)
        fh.write("点云帧数: %d，每帧点数约 %d\n" % (len(rows), rows[-1]["points"]))
        fh.write("最后一帧检出: 车 %d / 人 %d / 其他 %d\n" % (n_vehicle, n_walker, meta["n_other"]))
        fh.write("60 m 内真值: 车 %d / 人 %d\n" % (truth_v, truth_w))
        fh.write("召回率: 车 %.2f / 人 %.2f\n"
                 % (n_vehicle / max(truth_v, 1), n_walker / max(truth_w, 1)))
        fh.write("平均每帧检出: %.1f 个\n" % (sum(r["detected"] for r in rows) / len(rows)))

    lidar.stop()
    time.sleep(0.3)
    try:
        lidar.destroy()
    except RuntimeError:
        pass
    for a in vehicles + walkers:
        try:
            a.destroy()
        except RuntimeError:
            pass
    try:
        ego.destroy()
    except RuntimeError:
        pass

    ok = n_vehicle >= 2 and len(last_dets) >= 3
    print("[lidar] 最后一帧检出 %d 个目标（车 %d / 人 %d），60 m 内真值 车 %d / 人 %d"
          % (len(last_dets), n_vehicle, n_walker, truth_v, truth_w))
    print("[lidar] 结论：%s" % ("PASS（检测器稳定输出多目标）" if ok else "FAIL（检出目标过少）"))
    return 0 if ok else 1


def main():
    args = build_argparser().parse_args()
    cfg = merge_params(load_params(args.params), args)
    try:
        return run(cfg)
    except KeyboardInterrupt:
        print("\n[lidar] 用户中断")
        return 130


if __name__ == "__main__":
    sys.exit(main())
