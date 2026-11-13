#!/usr/bin/env python3
"""carla_ttc_emergency_braking — 基于语义激光雷达前向测距的自动紧急制动（AEB）实验。

设计要点
--------
1. **感知**：语义激光雷达（`sensor.lidar.ray_cast_semantic`）按实例号分组，只保留车辆
   语义标签，取前方本车道最近实例的最近点作为与前车的距离。仿真器 actor 真值只用于
   评估对账，不参与制动。真实 AEB 用的是毫米波雷达，这里用激光雷达等价替代，好处是
   不依赖任何训练权重，在任何地图上都能复现。
2. **决策**：TTC（Time To Collision）= 距离 / 相对速度。TTC 小于阈值或距离小于安全距离
   立即满制动（brake=1.0），否则按巡航油门加速；触发后锁存到停稳。
3. **取证**：运行中保存关键帧（逼近 → 触发制动 → 停稳），结束后输出
   `run_log.csv`（逐帧速度/距离/TTC）与 `result.png`（速度/距离/TTC 三联图）。

运行
----
    python main.py --host 127.0.0.1 --port 2000 --map Town10HD_Opt
"""
import argparse
import csv
import math
import os
import sys
import time

try:
    import carla
except ImportError:      # 没装客户端也能 --help
    carla = None

import numpy as np
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # module root
PKG_NAME = "carla_ttc_emergency_braking"


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

def client_flavor():
    """返回实际 import 到的客户端是配套 hutb 还是上游 carla。

    不能只看「哪个 distribution 装上了」：配套客户端的顶层包名同样叫 ``carla``
    （distribution 名是 ``hutb``），两个都装时按安装顺序取第一个会把上游客户端
    误判成 hutb，``check_client_compat`` 不退出，接着 ``get_world()`` 就会 core dump。
    这里优先看实际 import 到的模块能力：``set_files_base_folder`` 是 OpenHUTB
    地图按需下载机制的接口，只有配套客户端才有。
    """
    if carla is None:
        return "unknown"
    client_cls = getattr(carla, "Client", None)
    if client_cls is not None:
        return "hutb" if hasattr(client_cls, "set_files_base_folder") else "carla"

    # 没有 Client 时退回 distribution 归属判断（只用于报告，不参与安全判定）
    import importlib.metadata as md
    mod_root = (getattr(carla, "__name__", "") or "carla").split(".")[0]
    try:
        dists = [d.lower() for d in (md.packages_distributions().get(mod_root) or [])]
    except Exception:
        dists = []
    for name in dists:
        if name.startswith("hutb"):
            return "hutb"
    return "carla" if dists else "unknown"


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
            "[ttc] 未安装 CARLA/OpenHUTB Python 客户端，连不上仿真器。\n"
            "        安装见 README 第 4 节（Windows 用配套 hutb wheel；Linux 需自行编译 libcarla）。")
    """连接仿真器。

    注意：OpenHUTB 2.10.x 的客户端采用「地图文件按需下载」机制，
    必须先 set_files_base_folder + get_world()，否则 get_world() 只会抛 RuntimeError。
    """
    sim = cfg["simulator"]
    cache = sim["cache_dir"] or os.path.join(HERE, ".hutb_cache")
    os.makedirs(cache, exist_ok=True)
    client = carla.Client(sim["host"], sim["port"])
    client.set_timeout(float(sim["timeout"]))
    check_client_compat(client.get_server_version())  # 上游客户端会在 get_world() abort，提前判断
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

    ego_bp = bp_lib.filter(ego_blueprint)[0]
    lead_bp = bp_lib.filter(lead_blueprint)[0]
    ego_bp.set_attribute("role_name", ROLE_PREFIX + "ego")
    lead_bp.set_attribute("role_name", ROLE_PREFIX + "lead")
    ego = world.spawn_actor(ego_bp, ego_tf)
    lead = world.spawn_actor(lead_bp, lead_tf)
    # 刚 spawn 的 actor 在服务端 tick 之前 get_location() 还是 (0,0,0)，
    # 直接量距会把初始车距算成「到世界原点」，等一拍再量（实测曾打出 110.5 m）。
    time.sleep(0.2)
    actual_gap = ego.get_location().distance(lead.get_location())
    return ego, lead, actual_gap


ROLE_PREFIX = "aeb_"


def owned_by_module(role_name, prefix=ROLE_PREFIX):
    """该 actor 是否由本模块创建（用 role_name 前缀标识）。纯函数，便于单元测试。"""
    return bool(role_name) and str(role_name).startswith(prefix)


def cleanup_leftovers(world, prefix=ROLE_PREFIX):
    """只清理「本模块上一次运行残留」的 actor。

    早期版本会遍历全世界的 vehicle/sensor/walker 无差别 destroy，这会把别人正在用的
    车辆、交通参与者和传感器一起删掉（PR #955 评审指出）。现在生成时给 actor 打
    role_name = aeb_*，清理时只认这个前缀，不碰世界里的其他对象。
    """
    removed = 0
    for actor in list(world.get_actors()):
        attrs = getattr(actor, "attributes", {}) or {}
        if not owned_by_module(attrs.get("role_name", ""), prefix):
            continue
        try:
            actor.destroy()
            removed += 1
        except RuntimeError:
            pass
    if removed:
        print("[aeb] 清理本模块上一次运行的残留 actor: %d 个" % removed)
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


def should_brake(distance, speed, min_brake_distance, ttc_threshold):
    """AEB 判据（纯函数，便于单元测试）。

    distance: 与前方目标的距离（米）；None 表示前方无目标
    speed:    自车速度（m/s）
    返回 (是否制动, TTC)；TTC 无意义时返回 -1.0。
    """
    if distance is None:
        return False, -1.0
    ttc = distance / speed if speed > 0.3 else -1.0
    brake = distance <= min_brake_distance or (0.0 < ttc <= ttc_threshold)
    return bool(brake), float(ttc)


def truth_front_target(world, ego, max_range=80.0, half_width=2.2):
    """从仿真器真值里取正前方最近车辆的距离——**仅用于评估，不参与制动决策**。

    早期版本用这个函数的结果当制动输入（激光雷达只作回退），等于验证了真值通路而不是感知通路，
    PR #955 评审指出后已改为：决策只吃激光雷达，这个函数只写进日志/摘要用于对账。
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


def decide_brake(lidar_distance, speed, min_brake_distance, ttc_threshold, sensor_lost=False):
    """制动决策（纯函数）：输入只有激光雷达观测。

    lidar_distance: 激光雷达测得的前方距离；None 表示本帧没有观测到目标
    sensor_lost:    传感器失效标志（连续多帧收不到点云/测距全部无效）

    语义：
    - 正常无目标 -> 不制动；
    - 传感器失效（遮挡/掉线）-> 失效安全，直接制动（否则会盲开）；
    - 有目标 -> 交给 TTC/距离判据。
    """
    if sensor_lost:
        return True, -1.0, "sensor_fault"
    brake, ttc = should_brake(lidar_distance, speed, min_brake_distance, ttc_threshold)
    source = "lidar" if lidar_distance is not None else "no_target"
    return brake, ttc, source


# 语义激光雷达里「车辆」的 ObjectTag。本环境（OpenHUTB 2.10.1 / 服务端 3ea2130ed）
# 用 probe 读 lead.semantic_tags 实测为 [14]；15/16/18 在 CARLA 的 CityObjectLabel 里
# 分别是 bridge / railtrack / traffic light，不是车辆，混进来会把桥、轨道当成前车。
# 需要换环境时改 config/params.yaml 的 sensor.vehicle_tags，不要在这里硬编码。
VEHICLE_TAGS = (14,)


def parse_semantic_lidar(image):
    """语义激光雷达 -> (xyz(N,3) float32, obj_idx(N,) uint32, obj_tag(N,) uint32)。

    每点 6 个 32 位字：x, y, z, cos(incidence), ObjIdx(u32), ObjTag(u32)。
    后两个必须按 uint32 读，否则会被当成极小浮点数。
    """
    u = np.frombuffer(image.raw_data, dtype=np.uint32).reshape((-1, 6)).copy()
    return u[:, :3].view(np.float32), u[:, 4], u[:, 5]


def front_vehicle_distance(xyz, obj_idx, obj_tag, max_range=60.0, min_x=3.0,
                           half_width=2.2, z_min=-1.75, z_max=1.6, min_points=4,
                           vehicle_tags=VEHICLE_TAGS):
    """正前方最近的**车辆实例**距离（米）；没有返回 (None, None)。

    只用传感器观测：按实例号分组、只保留车辆语义标签，取该实例点的最小 x。

    横向门限用「实例质心」而不是单个点：相邻车道的车（中心 y≈3.5 m）常有边缘点
    落进 2.2 m 以内，按单点判断会把隔壁车道更近的车当成本车道前车而误刹
    （PR #955 评审给出过 x=10/y=3.5 的反例）。质心判断与真值对账口径
    （truth_front_target 的 |y| < half_width）一致。
    """
    if xyz is None or len(xyz) == 0:
        return None, None
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    cand = ((x > min_x) & (x < max_range) & (z > z_min) & (z < z_max)
            & np.isin(obj_tag, vehicle_tags))
    if not cand.any():
        return None, None
    best = None
    for inst in np.unique(obj_idx[cand]):
        sel = cand & (obj_idx == inst)
        if sel.sum() < min_points:
            continue
        if abs(float(y[sel].mean())) > half_width:      # 实例中心不在本车道 -> 不是前车
            continue
        d = float(x[sel].min())
        if best is None or d < best[0]:
            best = (d, int(inst))
    return (best if best is not None else (None, None))


def speed_of(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def save_plot(csv_rows, out_png, meta):
    """把速度/距离/TTC 画成三联图 + 关键信息，作为实验证据。

    距离面板同时给两条线：激光雷达观测（决策输入，只在检出帧有点）和真值距离
    （评估口径，逐帧都有）。只画激光雷达时，远距稀疏漏检会让曲线出现大片空白，
    看不出逼近过程（PR #955 评审要求证据与实现一致）。
    """
    if not csv_rows:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r["t"] for r in csv_rows]
    v = [r["speed_kmh"] for r in csv_rows]
    lidar = [(r["t"], r["lidar_m"]) for r in csv_rows if r["lidar_m"] > 0]
    truth = [(r["t"], r["truth_distance_m"]) for r in csv_rows if r["truth_distance_m"] > 0]
    ttc_pts = [(r["t"], r["ttc"]) for r in csv_rows if r["ttc"] > 0]

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(t, v, color="#c0392b", lw=2)
    axes[0].set_ylabel("speed (km/h)")
    axes[0].grid(alpha=.3)
    if truth:
        axes[1].plot([p[0] for p in truth], [p[1] for p in truth], color="#7f8c8d", lw=1.6,
                     label="truth distance (eval only)")
    if lidar:
        axes[1].plot([p[0] for p in lidar], [p[1] for p in lidar], "o", color="#2980b9",
                     ms=4, label="lidar distance (decision)")
    axes[1].axhline(meta["min_brake_distance"], ls="--", c="gray", label="min brake distance")
    axes[1].set_ylabel("front distance (m)")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(alpha=.3)
    if ttc_pts:
        axes[2].plot([p[0] for p in ttc_pts], [p[1] for p in ttc_pts], "o", color="#8e44ad", ms=4,
                     label="TTC from lidar")
    axes[2].axhline(meta["ttc_threshold"], ls="--", c="gray", label="TTC threshold")
    axes[2].set_ylim(0, max(3.0, 2.0 * float(meta["ttc_threshold"])))   # 远处 TTC 很大，截掉不占版面
    axes[2].set_ylabel("TTC (s)")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(alpha=.3)
    fig.suptitle("AEB: ego stopped at %.1f m (truth min %.1f m), brake at %.1f s"
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
    save_dir = os.path.join(_output_root(), out_cfg["save_dir"])
    os.makedirs(save_dir, exist_ok=True)
    # 目标横向门限与车辆语义标签都从配置读：不同平台/地图的标签编号可能不同
    target_half_width = float(sensors_cfg.get("target_half_width", 2.2))
    vehicle_tags = tuple(int(t) for t in sensors_cfg.get("vehicle_tags", VEHICLE_TAGS))

    client, world = connect(cfg)
    cleanup_leftovers(world)
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
    cam_bp.set_attribute("role_name", ROLE_PREFIX + "camera")
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

    lidar_bp = bp_lib.find("sensor.lidar.ray_cast_semantic")
    lidar_bp.set_attribute("channels", str(sensors_cfg["lidar_channels"]))
    lidar_bp.set_attribute("range", str(sensors_cfg["lidar_range"]))
    lidar_bp.set_attribute("points_per_second", str(sensors_cfg["lidar_points_per_second"]))
    lidar_bp.set_attribute("rotation_frequency", str(sensors_cfg["lidar_rotation_frequency"]))
    lidar_bp.set_attribute("role_name", ROLE_PREFIX + "lidar")
    lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(z=1.9)), attach_to=ego)
    lidar_state = {"xyz": None, "idx": None, "tag": None, "ts": 0.0}

    def on_lidar(image):
        xyz, idx, tag = parse_semantic_lidar(image)
        lidar_state.update(xyz=xyz, idx=idx, tag=tag, ts=time.time())

    lidar.listen(on_lidar)

    time.sleep(2.5)  # 等传感器出第一帧

    rows = []
    t0 = time.time()
    brake_started = None
    stopped_at = None
    min_distance = float("inf")
    final_distance = 0.0
    phase = "approach"
    no_sensor_frames = 0
    sensor_faults = 0
    final_from_lidar = False
    brake_latched = False
    truth_at_stop = None
    last_lidar_dist = None
    lidar_hits = 0

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
        lidar_dist, lidar_inst = (front_vehicle_distance(
            lidar_state["xyz"], lidar_state["idx"], lidar_state["tag"],
            max_range=float(sensors_cfg["lidar_range"]),
            half_width=target_half_width,
            vehicle_tags=vehicle_tags) if lidar_state["xyz"] is not None else (None, None))
        # 真值只用于评估/日志，不参与决策
        truth = truth_front_target(world, ego, half_width=target_half_width)
        truth_dist = truth[0] if truth is not None else None
        lidar_tag = -1
        if lidar_inst is not None:
            sel_tag = lidar_state["tag"][lidar_state["idx"] == lidar_inst]
            lidar_tag = int(sel_tag[0]) if len(sel_tag) else -1

        # 失效安全：只有「点云流停了」才算传感器故障；
        # 走廊里暂时没有目标（例如前车还在 100 m 外、超出量程）是正常工况，不能刹停。
        sensor_timeout = float(ctrl_cfg.get("sensor_timeout_s", 1.0))
        sensor_lost = (lidar_state["xyz"] is None) or (now - float(lidar_state["ts"]) > sensor_timeout)

        # ---------------- 决策（只吃激光雷达观测） ----------------
        steer, rel = lane_steer(ego, carla_map)
        braking, ttc, decision_source = decide_brake(
            lidar_dist, speed,
            float(ctrl_cfg["min_brake_distance"]),
            float(ctrl_cfg["ttc_threshold"]),
            sensor_lost=sensor_lost)
        # 一旦触发就锁存：AEB 不能因为下一帧暂时丢目标就松开刹车重新加速
        if braking:
            brake_latched = True
        if brake_latched:
            braking = True
            decision_source = "latched" if decision_source != "sensor_fault" else decision_source
        if sensor_lost:
            sensor_faults += 1
        dist = lidar_dist
        if dist is not None:
            last_lidar_dist = dist
            lidar_hits += 1

        if braking:
            ego.apply_control(carla.VehicleControl(
                throttle=0.0, brake=float(ctrl_cfg["brake_command"]), steer=steer))
            if brake_started is None:
                brake_started = t
                phase = "brake"
                frame_state["phase"] = "brake"
                print("[aeb] >>> 触发制动 t=%.2fs 激光雷达 dist=%.1fm speed=%.1fkm/h ttc=%.2fs source=%s"
                      % (t, dist if dist is not None else -1, speed * 3.6, ttc, decision_source))
        else:
            throttle = float(ctrl_cfg["cruise_throttle"])
            if speed * 3.6 > float(ctrl_cfg["target_speed"]) * 3.6:
                throttle = 0.0
            ego.apply_control(carla.VehicleControl(throttle=throttle, brake=0.0, steer=steer))

        if gt_distance is not None:
            min_distance = min(min_distance, gt_distance)
        rows.append({
            "t": round(t, 3),
            "speed_kmh": round(speed * 3.6, 3),
            "lidar_m": round(lidar_dist, 3) if lidar_dist is not None else -1.0,
            "truth_distance_m": round(truth_dist, 3) if truth_dist is not None else -1.0,
            "ttc": round(ttc, 3),
            "braking": int(braking),
            "sensor_lost": int(sensor_lost),
            "decision_source": decision_source,
            "instance": lidar_inst if lidar_inst is not None else -1,
            "tag": lidar_tag,
        })

        # ---------------- 停止判定 ----------------
        if brake_started is not None and speed < float(ctrl_cfg["stop_speed"]):
            if stopped_at is None:
                stopped_at = t
                phase = "stopped"
                frame_state["phase"] = "stopped"
                final_distance = last_lidar_dist if last_lidar_dist is not None else gt_distance
                final_from_lidar = last_lidar_dist is not None
                truth_at_stop = gt_distance
                print("[aeb] >>> 停稳 t=%.2fs 激光雷达测距 %s（真值 %.1f m；传感器失效帧 %d）"
                      % (t, ("%.1f m" % final_distance) if final_from_lidar else "无目标",
                         gt_distance, sensor_faults))
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
        "final_from_lidar": final_from_lidar,
        "truth_at_stop": truth_at_stop if truth_at_stop is not None else -1.0,
        "min_brake_distance": float(ctrl_cfg["min_brake_distance"]),
        "ttc_threshold": float(ctrl_cfg["ttc_threshold"]),
        "brake_time": brake_started if brake_started is not None else -1.0,
        "final_distance": final_distance,
        "min_distance": min_distance if min_distance < float("inf") else -1.0,
    }
    save_plot(rows, os.path.join(save_dir, "result.png"), meta)

    with open(os.path.join(save_dir, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("AEB 实验摘要\n")
        fh.write("地图: %s\n" % carla_map.name)
        fh.write("决策依据: 激光雷达观测（真值仅用于评估对账）\n")
        fh.write("初始距离(首帧真值): %.1f m（配置 lead_gap=%.1f m，实际按路段取整）\n"
                 % (float(rows[0]["truth_distance_m"]), float(demo["lead_gap"])))
        fh.write("制动触发时刻: %.2f s\n" % meta["brake_time"])
        fh.write("制动后停稳时刻: %.2f s\n" % (stopped_at if stopped_at else -1))
        fh.write("激光雷达有效检出帧: %d / %d（语义实例点数 >= 4 才成目标；远距稀疏会漏检，"
                 "决策按锁存与最后一次有效测量处理）\n" % (lidar_hits, len(rows)))
        fh.write("停稳时激光雷达测距（最后一次有效测量，到前车最近表面）: %s\n"
                 % (("%.2f m" % meta["final_distance"]) if meta.get("final_from_lidar") else "无有效目标（回退真值 %.2f m）"
                    % meta["final_distance"]))
        fh.write("停稳时真值距离（中心距，仅评估）: %.2f m\n" % meta.get("truth_at_stop", -1.0))
        fh.write("全程最近距离(真值): %.2f m\n" % meta["min_distance"])
        fh.write("传感器失效帧数: %d（失效安全策略：点云流超过 %.1f s 未更新即判故障并制动；"
                 "「走廊内暂时无目标」不算故障）\n"
                 % (sensor_faults, float(ctrl_cfg.get("sensor_timeout_s", 1.0))))
        fh.write("关键帧: %s\n" % ", ".join(saved_frames))

    camera.stop(); lidar.stop()
    camera.destroy(); lidar.destroy()
    lead.destroy(); ego.destroy()
    print("[aeb] 实验结束，证据已写入 %s" % save_dir)

    ev = cfg["evaluation"]
    ok = (meta["final_distance"] >= float(ev["min_stop_distance"])
          and (meta["brake_time"] > 0 or not bool(ev["require_brake"])))
    print("[aeb] 结论：%s" % ("PASS（制动成功，未发生碰撞）" if ok else "FAIL（未能安全停住）"))
    return 0 if ok else 1


def main():
    # launch_ros 的 Node 会追加 --ros-args / __node:= 等参数，这里容忍未知参数
    args, _unknown = build_argparser().parse_known_args()
    cfg = merge_params(load_params(args.params), args)
    try:
        return run(cfg)
    except KeyboardInterrupt:
        print("\n[aeb] 用户中断")
        return 130


if __name__ == "__main__":
    sys.exit(main())
