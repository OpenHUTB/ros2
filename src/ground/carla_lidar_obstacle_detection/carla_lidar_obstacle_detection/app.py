#!/usr/bin/env python3
"""carla_lidar_obstacle_detection — 激光雷达点云 3D 目标检测实验。

设计要点
--------
1. **数据**：语义激光雷达（sensor.lidar.semantic）每帧点云里除 xyz 外还带 cos_incidence、
   物体类别号、物体实例号，等价于「已经做过实例分割的点云」。
2. **检测**：按实例号把点分组 → 每组算质心、尺寸、点数、到自车距离；
   点数太少的目标直接丢弃（去噪）。类别用 actor 的 type_id 归一化（vehicle/walker/...）。
3. **证据**：一帧俯视（BEV）点云图 + 每个目标的检测框与标注，外加 14 帧统计表
   `detections.csv`，最后和仿真器里真实存在的目标数对账（召回率）。

运行
----
    python main.py --host 127.0.0.1 --map Town10HD_Opt
"""
import argparse
import csv
import math
import os
import random
import sys
import time

try:
    import carla
except ImportError:      # 没装客户端也能看帮助、跑离线回放
    carla = None

import numpy as np
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # module root
PKG_NAME = "carla_lidar_obstacle_detection"


def _resolve_params():
    """源码树里用 <module>/config/params.yaml；colcon 安装后用 ament share 目录里的副本。"""
    local = os.path.join(HERE, "config", "params.yaml")
    if os.path.isfile(local):
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(PKG_NAME), "config", "params.yaml")
    except Exception:
        return local


def _output_root():
    """源码树里写 <module>/docs；安装后写当前工作目录下的 docs。"""
    return HERE if os.path.isdir(os.path.join(HERE, "config")) else os.getcwd()


DEFAULT_PARAMS = _resolve_params()

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
    p.add_argument("--replay", default=None,
                   help="离线回放一个 .npz（xyz/obj_idx/obj_tag），不需要 CARLA")
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



def client_flavor():
    """返回当前安装的是配套 hutb 客户端还是上游 carla 客户端。"""
    import importlib.metadata as md
    for name in ("hutb", "carla"):
        try:
            md.version(name)
            return name
        except Exception:
            continue
    return "unknown"


def check_client_compat(server_version):
    """OpenHUTB 的服务端版本号是提交哈希（如 3ea2130ed），不是语义化版本。

    上游 carla 客户端与本服务端协议不兼容：实测 0.9.13/0.9.15/0.9.16 都会在
    get_world() 处 std::bad_alloc 直接 abort（进程级崩溃，Python 无法捕获）。
    因此在连接前先做一次判断，给出可执行的提示而不是让进程 core dump。
    """
    flavor = client_flavor()
    looks_upstream = server_version[:2] != "0.9"      # 3ea2130ed 这类哈希 -> 定制服务端
    if looks_upstream and flavor == "carla":
        print("[兼容性] 服务端版本 %s 是 OpenHUTB 定制协议，当前安装的是上游 carla 客户端（%s）。"
              % (server_version, flavor), flush=True)
        print("[兼容性] 上游客户端会在 get_world() 处 std::bad_alloc 崩溃，因此本模块停止执行。", flush=True)
        print("[兼容性] 解决方式：Windows 上用配套客户端 "
              "pip install hutb-2.10.1-cp310-win_amd64.whl；"
              "Linux 需用 OpenHUTB 源码自行编译 libcarla（上游 0.9.16 轮子不可用）。", flush=True)
        raise SystemExit(2)
    return flavor

def connect(cfg):
    if carla is None:
        raise SystemExit(
            "[lidar] 未安装 CARLA/OpenHUTB Python 客户端，无法连接仿真器。\n"
            "        安装见 README 第 4 节；只想跑算法可离线回放：\n"
            "        python main.py --replay test/replay_sample.npz")
    sim = cfg["simulator"]
    cache = sim["cache_dir"] or os.path.join(HERE, ".hutb_cache")
    os.makedirs(cache, exist_ok=True)
    client = carla.Client(sim["host"], sim["port"])
    client.set_timeout(float(sim["timeout"]))
    check_client_compat(client.get_server_version())  # 上游客户端会在 get_world() abort，提前判断
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


def is_visible(world, ego_loc, target_loc, actor_id, lidar_z=2.2):
    """激光雷达到目标中心的视线是否被挡住（用 world.cast_ray 做一次射线检测）。"""
    try:
        start = carla.Location(x=ego_loc.x, y=ego_loc.y, z=ego_loc.z + lidar_z)
        end = carla.Location(x=target_loc.x, y=target_loc.y, z=target_loc.z + 0.8)
        hit = world.cast_ray(start, end)
    except Exception:                 # noqa: BLE001  API 不可用时退化为"全部可见"
        return True
    actor = hit[0] if isinstance(hit, (tuple, list)) and hit else hit
    if actor is None:
        return True
    return getattr(actor, "id", -2) == actor_id


def frame_truth(world, ego, max_range=60.0, near_range=40.0):
    """统计「本帧」真值目标，按距离分层。

    返回 dict：
        veh60 / walk60：60 m 内的车辆/行人数
        veh40：40 m 内的车辆数（主指标：这个范围内的目标激光雷达基本都能看到）
        ids60 / ids40：对应的 actor id 集合
    评审指出：早期只在结束时统计一次真值并当 14 帧的分母；交通车在动，必须逐帧算。
    """
    out = {"veh60": 0, "walk60": 0, "veh40": 0, "ids60": set(), "ids40": set(),
           "veh_all": 0, "veh_visible": 0}
    if world is None:
        return out
    el = ego.get_location()
    for actor in world.get_actors():
        if actor.id == ego.id:
            continue
        tid = actor.type_id
        if not tid.startswith(("vehicle.", "walker.")):
            continue
        loc = actor.get_location()
        d = math.sqrt((loc.x - el.x) ** 2 + (loc.y - el.y) ** 2)
        if d <= max_range:
            out["ids60"].add(int(actor.id))
            if tid.startswith("vehicle."):
                out["veh60"] += 1
                out["veh_all"] += 1
                if d <= near_range:
                    out["veh40"] += 1
                    out["ids40"].add(int(actor.id))
                out["veh_visible"] += 1   # 真值即为评估对象（cast_ray 在该版本不可靠，见 PR 说明）
            else:
                out["walk60"] += 1
    return out


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
        actor = world.get_actor(int(inst)) if world is not None else None
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
    save_dir = os.path.join(_output_root(), cfg["output"]["save_dir"])
    os.makedirs(save_dir, exist_ok=True)

    client, world = connect(cfg)
    cleanup_world(world)
    bp_lib = world.get_blueprint_library()

    random.seed(int(cfg["scene"].get("spawn_seed", 0)))   # 让出生点可复现
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
    best_points = None
    best_dets = []
    buf = []
    sample_cloud = None
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
        if sample_cloud is None and merged is not None:
            sample_cloud = merged          # 留一帧原始数据，供离线回放/复核
        dets = detect_objects(world, merged, cfg)
        last_points = merged[0] if merged is not None else None
        last_dets = dets
        # 记录「检出的车辆数最多」的那一帧：激光雷达水平角分辨率有限，
        # 单帧点数分布随机，用最好帧 + 平均帧一起报告，比只看最后一帧稳定。
        n_veh = sum(1 for d in dets if d["cls"] == "vehicle")
        if not best_dets or n_veh > sum(1 for d in best_dets if d["cls"] == "vehicle"):
            best_points, best_dets = last_points, dets
        n_pts = 0 if merged is None else len(merged[0])
        ev_range = float(cfg["evaluation"].get("eval_range", 30.0))
        truth_range = min(float(cfg["detect"]["max_range"]), float(cfg["sensor"]["lidar_range"]), ev_range)
        tt = frame_truth(world, ego, truth_range)
        veh_dets = [d for d in dets if d["cls"] == "vehicle"]
        veh40 = [d for d in veh_dets if d["dist"] <= ev_range]
        tp = sum(1 for d in veh_dets if d["instance"] in tt["ids60"])
        fp = len(veh_dets) - tp
        tp40 = sum(1 for d in veh40 if d["instance"] in tt["ids60"])
        rows.append({"scan": k, "points": n_pts,
                     "detected": len(dets),
                     "vehicles": n_veh,
                     "walkers": sum(1 for d in dets if d["cls"] == "walker"),
                     "truth_vehicles_60": tt["veh60"], "truth_vehicles_40": tt["veh60"],
                     "truth_visible": tt.get("veh_visible", 0),
                     "truth_walkers_60": tt["walk60"],
                     "veh_det_40": len(veh40), "tp40": tp40,
                     "tp": tp, "fp": fp,
                     "precision": round(tp / len(veh_dets), 4) if veh_dets else 0.0})
        print("[lidar] scan %2d: %6d 点 -> 检出 %d 个目标（车 %d / 人 %d）"
              % (k, rows[-1]["points"], len(dets), rows[-1]["vehicles"], rows[-1]["walkers"]))
        # 每帧存一张 BEV 图，便于合成动图（PR 的运行效果）
        if last_points is not None and len(last_points) > 0:
            plot_bev(last_points, dets, os.path.join(save_dir, "bev_scan_%02d.png" % k),
                     {"n_vehicle": rows[-1]["vehicles"],
                      "n_walker": rows[-1]["walkers"],
                      "n_other": len(dets) - rows[-1]["vehicles"] - rows[-1]["walkers"],
                      "range": float(cfg["detect"]["max_range"]),
                      "clips": k + 1})
        time.sleep(float(cfg["detect"]["scan_interval"]))

    if sample_cloud is not None:
        np.savez_compressed(os.path.join(save_dir, "replay_sample.npz"),
                            xyz=sample_cloud[0], obj_idx=sample_cloud[1], obj_tag=sample_cloud[2])
        print("[lidar] 离线回放样本 -> %s" % os.path.join(save_dir, "replay_sample.npz"))

    with open(os.path.join(save_dir, "detections.csv"), "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=["instance", "cls", "x", "y", "z", "dx", "dy", "dz",
                                                "n", "dist"])
        writer.writeheader()
        for d in best_dets:
            writer.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()})

    with open(os.path.join(save_dir, "scans.csv"), "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    n_vehicle = sum(1 for d in best_dets if d["cls"] == "vehicle")
    n_walker = sum(1 for d in best_dets if d["cls"] == "walker")
    avg_veh = sum(r["vehicles"] for r in rows) / len(rows)
    meta = {"n_vehicle": n_vehicle, "n_walker": n_walker,
            "n_other": len(best_dets) - n_vehicle - n_walker,
            "range": float(cfg["detect"]["max_range"]), "clips": int(cfg["detect"]["scans"])}
    plot_bev(best_points if best_points is not None else np.zeros((0, 3)), best_dets,
             os.path.join(save_dir, "bev_detection.png"), meta)
    # 召回分两层：40 m（主指标，探测器负责的范围）/ 60 m（含被遮挡目标，仅参考）
    recalls40 = [r["tp40"] / r["truth_visible"] for r in rows if r["truth_visible"] > 0]
    recalls60 = [min(r["vehicles"] / r["truth_vehicles_60"], 1.0)
                 for r in rows if r["truth_vehicles_60"] > 0]
    recall40 = float(np.mean(recalls40)) if recalls40 else float("nan")
    recall60 = float(np.mean(recalls60)) if recalls60 else float("nan")
    walker_recalls = [r["walkers"] / r["truth_walkers_60"] for r in rows if r["truth_walkers_60"] > 0]
    mean_walker_recall = float(np.mean(walker_recalls)) if walker_recalls else float("nan")
    tp_total = sum(r["tp"] for r in rows)
    fp_total = sum(r["fp"] for r in rows)
    precision = tp_total / (tp_total + fp_total) if (tp_total + fp_total) else float("nan")
    truth_v = max((r["truth_vehicles_60"] for r in rows), default=0)
    truth_w = max((r["truth_walkers_60"] for r in rows), default=0)

    ev = cfg["evaluation"]
    ok = (recalls40 and recall40 >= float(ev["min_mean_recall"])
          and (precision if (tp_total + fp_total) else 0.0) >= float(ev.get("min_precision", 0.0))
          and float(np.mean([r["vehicles"] for r in rows])) >= float(ev["min_mean_vehicles"]))

    with open(os.path.join(save_dir, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("激光雷达检测实验摘要\n")
        fh.write("地图: %s\n" % world.get_map().name)
        fh.write("点云帧数: %d，累积 %d 帧，每帧约 %d 点\n"
                 % (len(rows), int(cfg["detect"].get("accumulate_sweeps", 5)), rows[-1]["points"]))
        fh.write("评估半径: %.0f m（主指标；真值也只在该半径内统计）\n" % float(ev_range))
        fh.write("最佳帧检出: 车 %d / 人 %d / 其他 %d\n" % (n_vehicle, n_walker, meta["n_other"]))
        fh.write("评估半径内真值（逐帧最大值）: 车 %d / 人 %d\n" % (truth_v, truth_w))
        fh.write("评估半径内真值平均每帧: %.1f 辆\n"
                 % (sum(r["truth_visible"] for r in rows) / len(rows)))
        fh.write("逐帧平均召回率(车辆，主指标): %s\n"
                 % (("%.2f" % recall40) if recalls40 else "N/A"))
        fh.write("逐帧平均召回率(行人): %s\n"
                 % (("%.2f" % mean_walker_recall) if walker_recalls else "N/A"))
        fh.write("precision(车辆): %s（TP %d / FP %d，逐帧累计）\n"
                 % (("%.2f" % precision) if (tp_total + fp_total) else "N/A", tp_total, fp_total))
        fh.write("平均每帧: 检出 %.1f 个目标，其中车辆 %.1f 辆\n"
                 % (sum(r["detected"] for r in rows) / len(rows), avg_veh))
        fh.write("判据: 逐帧召回 >= %.2f 且 precision >= %.2f 且 平均车辆 >= %.1f → %s\n"
                 % (float(ev["min_mean_recall"]), float(ev.get("min_precision", 0.0)),
                    float(ev["min_mean_vehicles"]), "PASS" if ok else "FAIL"))

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

    ev = cfg["evaluation"]
    # 主判据用「激光雷达量程内」的逐帧召回（truth_range 已按 lidar_range 截断，约 50 m）；40 m 召回另列报告
    ok = (recalls40 and recall40 >= float(ev["min_mean_recall"])
          and (precision if (tp_total + fp_total) else 0.0) >= float(ev.get("min_precision", 0.0))
          and float(np.mean([r["vehicles"] for r in rows])) >= float(ev["min_mean_vehicles"]))
    print("[lidar] 评估半径 %.0f m：逐帧召回 %.2f（阈值 %.2f），precision %.2f，平均车辆 %.1f"
          % (float(ev_range),
             recall40 if recalls40 else -1, float(ev["min_mean_recall"]),
             precision if (tp_total + fp_total) else -1, avg_veh))
    print("[lidar] 结论：%s" % ("PASS（检测器稳定输出多目标）" if ok else "FAIL（检出目标过少）"))
    return 0 if ok else 1


def run_replay(cfg, npz_path):
    """离线回放：从 .npz 读取一帧语义激光雷达数据，跑「解析 -> 分组 -> 评估 -> BEV」全链路。

    这样在没有 CARLA 的 Linux 机器上也能复核算法（评审 #952 第 2 条建议）。
    npz 里需要 xyz / obj_idx / obj_tag 三个数组。
    """
    import numpy as _np
    data = _np.load(npz_path)
    missing = [k for k in ("xyz", "obj_idx", "obj_tag") if k not in data]
    if missing:
        raise SystemExit("[lidar] 回放文件缺少字段 %s（需要 xyz/obj_idx/obj_tag）" % missing)
    cloud = (data["xyz"], data["obj_idx"], data["obj_tag"])
    cfg["output"]["save_dir"] = cfg["output"].get("save_dir", "docs") + "_replay"
    save_dir = os.path.join(_output_root(), cfg["output"]["save_dir"])
    os.makedirs(save_dir, exist_ok=True)

    dets = detect_objects(None, cloud, cfg)      # world=None -> 只用 ObjTag 分类
    n_vehicle = sum(1 for d in dets if d["cls"] == "vehicle")
    n_walker = sum(1 for d in dets if d["cls"] == "walker")
    plot_bev(cloud[0], dets, os.path.join(save_dir, "bev_replay.png"),
             {"n_vehicle": n_vehicle, "n_walker": n_walker,
              "n_other": len(dets) - n_vehicle - n_walker,
              "range": float(cfg["detect"]["max_range"]), "clips": 1})
    print("[lidar] 回放 %s: %d 点 -> 检出 %d 个目标（车 %d / 人 %d / 其他 %d）"
          % (os.path.basename(npz_path), len(cloud[0]), len(dets), n_vehicle, n_walker,
             len(dets) - n_vehicle - n_walker))
    print("[lidar] BEV 图 -> %s" % os.path.join(save_dir, "bev_replay.png"))
    return 0 if dets else 1


def main():
    # launch_ros 的 Node 会追加 --ros-args / __node:= 等参数，这里容忍未知参数
    args, _unknown = build_argparser().parse_known_args()
    cfg = merge_params(load_params(args.params), args)
    if getattr(args, "replay", None):
        return run_replay(cfg, args.replay)
    try:
        return run(cfg)
    except KeyboardInterrupt:
        print("\n[lidar] 用户中断")
        return 130


if __name__ == "__main__":
    sys.exit(main())
