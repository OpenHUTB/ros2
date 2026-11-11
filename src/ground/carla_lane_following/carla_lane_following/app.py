#!/usr/bin/env python3
"""carla_lane_following — 基于语义分割的车道线巡线（视觉横向控制）实验。

设计要点
--------
1. **感知**：语义分割相机（sensor.camera.semantic_segmentation），车道线类别 tag=24（RoadLines）。
   只取画面下半部分做 ROI，左右半屏分别统计车道线像素质心，得到「车道中心」的横向偏移。
2. **控制**：把像素偏移归一化成 [-1,1] 的误差，交给 PID 输出转向角；
   速度用简单比例控制维持目标车速。
3. **取证**：每 2 秒存一张带标注的 RGB 画面（车道线高亮 + 偏移数值），
   结束后输出 `run_log.csv` 与 `result.png`（横向误差曲线 + 转向曲线）。

运行
----
    python main.py --host 127.0.0.1 --map Town10HD_Opt
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
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # module root
PKG_NAME = "carla_lane_following"


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


def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def build_argparser():
    p = argparse.ArgumentParser(description="CARLA 车道线巡线（视觉横向控制）实验")
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
            "[lane] 未安装 CARLA/OpenHUTB Python 客户端，连不上仿真器。\n"
            "        安装见 README 第 4 节（Windows 用配套 hutb wheel；Linux 需自行编译 libcarla）。")
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
    print("[lane] 已连接 %s:%d | 地图 %s" % (sim["host"], sim["port"], world.get_map().name))
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
        print("[lane] 清理上一轮残留 actor: %d 个" % n)


def _norm_angle(deg):
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


def rank_spawns(world, min_len=45.0, max_yaw_diff=12.0):
    """按「前方一段是否够直、是否不穿路口」给所有出生点排序，返回 [(score, idx, transform)]。"""
    carla_map = world.get_map()
    candidates = []
    for idx, sp in enumerate(carla_map.get_spawn_points()):
        wp = carla_map.get_waypoint(sp.location, project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        road_id = wp.road_id
        travelled = 0.0
        same_road = True
        ok = True
        while travelled < min_len:
            nxt = wp.next(4.0)
            if not nxt:
                ok = False
                break
            if nxt[0].road_id != road_id:
                same_road = False
            wp = nxt[0]
            travelled += 4.0
        if not ok:
            continue
        dyaw = abs(_norm_angle(wp.transform.rotation.yaw - sp.rotation.yaw))
        if dyaw > max_yaw_diff:
            continue
        score = (0 if same_road else 100) + dyaw
        candidates.append((score, idx, sp))
    candidates.sort(key=lambda c: c[0])
    return candidates


def _band_positions(tags, roi_top, roi_bottom, lane_tag, min_col, mid, wd):
    """在 [roi_top, roi_bottom) 横带里找本车道左右两条车道线，返回 (left_x, right_x)。

    缺失的一侧返回 None（城市地图上车道线常是虚线，单侧可见很常见）。
    每侧只取离画面中心最近的那条，排除邻车道线与路口导流线。
    """
    h = tags.shape[0]
    y0, y1 = int(h * roi_top), int(h * roi_bottom)
    band = tags[y0:y1, :] == lane_tag
    cols = band.sum(axis=0)
    left_idx = [i for i in range(mid) if cols[i] >= min_col]
    right_idx = [i for i in range(mid, wd) if cols[i] >= min_col]
    xl = max(left_idx) if left_idx else None     # 离中心最近的左侧线
    xr = min(right_idx) if right_idx else None   # 离中心最近的右侧线
    if xl is not None and xr is not None and 0 < (xr - xl) < wd * 0.15:
        # 两条线靠得太近，多半是同一条线被劈成两半，只保留右侧
        xl = None
    return xl, xr


def _band_center(tags, roi_top, roi_bottom, lane_tag, min_col, mid, wd, state):
    """带单侧估计的车道中心：两侧都有 -> 中点；只有一侧 -> 用历史车道宽度补另一侧。"""
    xl, xr = _band_positions(tags, roi_top, roi_bottom, lane_tag, min_col, mid, wd)
    if xl is not None and xr is not None:
        width = xr - xl
        if 0.1 * wd <= width <= 0.9 * wd:
            state["width"] = width          # 记住可信的车道宽度（像素）
        return (xl + xr) / 2.0, "dual"
    width = state.get("width")
    if xl is not None and width:
        return xl + width / 2.0, "left_only"
    if xr is not None and width:
        return xr - width / 2.0, "right_only"
    if xl is not None or xr is not None:
        x = xl if xl is not None else xr
        return x, "single"
    return None, "none"


def semantic_lane_offset(sem_img, cfg, state=None):
    """语义分割 -> (近场横向偏移, 航向误差, 掩码, 来源)。

    来源：dual（两侧都看到）/ left_only / right_only（单侧 + 历史宽度估计）/ none（完全丢失）。
    只有 none 才算丢线，这样统计反映真实感知能力，而不是虚线的随机性。
    """
    if state is None:
        state = {}
    arr = np.frombuffer(sem_img.raw_data, dtype=np.uint8).reshape((sem_img.height, sem_img.width, 4))
    tags = arr[..., 2]  # 语义分割：红通道存类别号
    h, w = tags.shape
    mid = w // 2
    lane_tag = int(cfg["perception"]["lane_tag"])
    min_col = int(cfg["perception"]["min_col_pixels"])
    roi_top = float(cfg["perception"]["roi_top"])

    mask = np.zeros_like(tags, dtype=bool)
    mask[int(h * roi_top):, :] = tags[int(h * roi_top):, :] == lane_tag

    near_c, near_src = _band_center(tags, (roi_top + 1.0) / 2, 1.0, lane_tag, min_col, mid, w, state)
    far_c, far_src = _band_center(tags, roi_top, (roi_top + 1.0) / 2, lane_tag, min_col, mid, w, state)
    if near_c is None and far_c is None:
        return None, None, mask, "none"
    # 只有一侧线、而且还没有历史宽度时，中点估计会偏半个车道宽，不可靠：
    # 不参与统计、也不用于控制（按丢线处理），单独记为 "single" 以便追溯。
    if near_c is None or far_c is None:
        only = near_src if near_c is None else far_src
        if only == "single":
            return None, None, mask, "single"
    if near_c is not None and far_c is not None:
        src = "dual" if "dual" in (near_src, far_src) else near_src
    else:
        src = near_src if near_c is not None else far_src
    near = near_c if near_c is not None else far_c
    far = far_c if far_c is not None else near_c
    offset = (near - mid) / (w / 2.0)
    heading = (far - near) / (w / 2.0)
    return float(offset), float(heading), mask, src


def annotate(rgb_img, mask, offset, steer, out_path):
    """把车道线掩码画到 RGB 图上，并写上偏移/转向，作为实验证据。"""
    rgb = np.frombuffer(rgb_img.raw_data, dtype=np.uint8).reshape((rgb_img.height, rgb_img.width, 4))
    rgb = rgb[:, :, :3][:, :, ::-1]  # BGRA -> RGB
    canvas = rgb.copy()
    canvas[mask] = (0.35 * canvas[mask] + 0.65 * np.array([0, 255, 90])).astype(np.uint8)
    img = Image.fromarray(canvas)
    draw = ImageDraw.Draw(img)
    x_center = int(rgb_img.width / 2)
    draw.line([(x_center, 0), (x_center, rgb_img.height)], fill=(255, 60, 60), width=2)
    draw.text((10, 10), "offset=%.3f  steer=%.3f" % (offset, steer), fill=(255, 255, 0))
    img.save(out_path)


def save_plot(rows, out_png, meta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r["t"] for r in rows]
    off = [r["offset"] for r in rows]
    steer = [r["steer"] for r in rows]
    speed = [r["speed_kmh"] for r in rows]

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(t, off, color="#2c7fb8", lw=1.6)
    axes[0].axhline(0, color="gray", ls="--", lw=1)
    axes[0].set_ylabel("lane offset (norm.)")
    axes[0].grid(alpha=.3)
    axes[1].plot(t, steer, color="#d95f0e", lw=1.6)
    axes[1].set_ylabel("steer")
    axes[1].grid(alpha=.3)
    axes[2].plot(t, speed, color="#31a354", lw=1.8)
    axes[2].set_ylabel("speed (km/h)")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(alpha=.3)
    fig.suptitle("Lane following: mean |offset|=%.3f (measured frames), lost=%d/%d (%.1f%%)"
                 % (meta["mean_abs_offset"], meta["lost"], meta["frames"],
                    meta["lost_ratio"] * 100))
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def run(cfg):
    out_cfg = cfg["output"]
    save_dir = os.path.join(_output_root(), out_cfg["save_dir"])
    os.makedirs(save_dir, exist_ok=True)

    client, world = connect(cfg)
    cleanup_world(world)
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    sem_cfg = cfg["sensor"]
    data = {"sem": None, "rgb": None}

    def on_sem(img):
        data["sem"] = img

    sem_bp = bp_lib.find("sensor.camera.semantic_segmentation")
    for k, v in (("image_size_x", sem_cfg["camera_width"]), ("image_size_y", sem_cfg["camera_height"]),
                 ("fov", sem_cfg["camera_fov"]), ("sensor_tick", sem_cfg["sensor_tick"])):
        sem_bp.set_attribute(k, str(v))

    # 起点不是在「最直的路」里盲选，而是逐个候选点实际看一眼：能同时看到左右车道线才算数
    candidates = rank_spawns(world)
    if not candidates:
        raise RuntimeError("找不到合适的直路段")
    ego = sem_cam = None
    chosen = None
    for score, idx, sp in candidates[:12]:
        tf = sp
        tf.location.z += 0.3
        ego = world.spawn_actor(bp_lib.filter(cfg["scene"]["ego_blueprint"])[0], tf)
        data["sem"] = None
        sem_cam = world.spawn_actor(sem_bp, carla.Transform(carla.Location(x=1.5, z=1.6)), attach_to=ego)
        sem_cam.listen(on_sem)
        time.sleep(1.6)
        if data["sem"] is not None:
            off, hd, _mask, _src = semantic_lane_offset(data["sem"], cfg)
            if off is not None and abs(off) < 0.35:
                chosen = (idx, sp, off, score)
                print("[lane] 选定起点 spawn[%d]（score=%.1f，初始偏移 %.3f）%s"
                      % (idx, score, off, tf.location))
                break
        sem_cam.stop(); sem_cam.destroy(); ego.destroy()
        ego = sem_cam = None
    if ego is None:
        raise RuntimeError("12 个候选起点都看不清左右车道线，请换地图或调整 roi_top")

    rgb_bp = bp_lib.find("sensor.camera.rgb")
    for k, v in (("image_size_x", sem_cfg["camera_width"]), ("image_size_y", sem_cfg["camera_height"]),
                 ("fov", sem_cfg["camera_fov"]), ("sensor_tick", "1.0")):
        rgb_bp.set_attribute(k, str(v))
    rgb_cam = world.spawn_actor(rgb_bp, carla.Transform(carla.Location(x=1.5, z=1.6)), attach_to=ego)
    rgb_cam.listen(lambda img: data.update(rgb=img))

    time.sleep(3.0)
    print("[lane] 开始巡线：目标车速 %.1f m/s" % cfg["control"]["target_speed"])

    ctrl = cfg["control"]
    ev = cfg["evaluation"]
    integral = 0.0
    prev_offset = 0.0
    prev_steer = 0.0
    offset_used = None
    lost_total = 0
    source_count = {"dual": 0, "left_only": 0, "right_only": 0, "single": 0, "none": 0}
    perception_state = {}
    lost_run = 0
    lost_run_max = 0
    rows = []
    t0 = time.time()
    last_annot = -999.0
    annotated = []
    completed = False

    try:
        while True:
            t = time.time() - t0
            if t > float(ctrl["max_duration"]):
                print("[lane] 达到设定时长 %.1fs，正常结束" % float(ctrl["max_duration"]))
                completed = True
                break

            sem = data["sem"]
            offset = None
            heading = None
            mask = None
            if sem is not None:
                offset, heading, mask, src = semantic_lane_offset(sem, cfg, perception_state)

            speed = ego.get_velocity()
            speed_ms = math.sqrt(speed.x ** 2 + speed.y ** 2 + speed.z ** 2)

            source_count[src] = source_count.get(src, 0) + 1
            if src in ("none", "single"):
                lost_total += 1
                lost_run += 1
                lost_run_max = max(lost_run_max, lost_run)
                steer = prev_steer * 0.85   # 丢线时柔性保持上一帧转向，避免猛打方向
                throttle_cap = float(ctrl.get("lost_throttle", 0.3))
                if lost_run > int(ctrl["max_lost_frames"]):
                    print("[lane] 连续丢线 %d 帧，超过上限，提前结束（实验未跑满设定时长）" % lost_run)
                    break
            else:
                lost_run = 0
                throttle_cap = 0.85
                # EMA 平滑，抑制逐帧抖动
                smooth_offset = 0.6 * offset + 0.4 * prev_offset
                integral = max(-1.0, min(1.0, integral + smooth_offset * 0.05))
                steer = (float(ctrl["kp"]) * smooth_offset
                         + float(ctrl["ki"]) * integral
                         + float(ctrl["kd"]) * heading)
                steer = max(-float(ctrl["max_steer"]), min(float(ctrl["max_steer"]), steer))
                prev_offset = smooth_offset
                offset_used = smooth_offset
            prev_steer = steer

            # 速度环：比例控制
            err_v = float(ctrl["target_speed"]) - speed_ms
            throttle = max(0.0, min(throttle_cap, err_v * 0.25))
            brake = 0.4 if err_v < -2.0 else 0.0
            ego.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))

            # offset 列只记「本帧真实测到」的值；offset_used 是实际用于控制的平滑值。
            rows.append({
                "t": round(t, 3),
                "offset": round(offset, 4) if offset is not None else "",
                "offset_used": round(offset_used, 4) if offset_used is not None else "",
                "steer": round(steer, 4),
                "speed_kmh": round(speed_ms * 3.6, 3),
                "lost": int(src in ("none", "single")),
                "source": src,
            })

            if (out_cfg["save_annotated"] and data["rgb"] is not None and mask is not None
                    and offset is not None
                    and t - last_annot >= float(out_cfg["annotated_interval"])):
                name = "lane_%05.1fs.png" % t
                annotate(data["rgb"], mask, offset, steer, os.path.join(save_dir, name))
                annotated.append(name)
                last_annot = t
                print("[lane] 标注帧 -> %s（offset=%.3f steer=%.3f）" % (name, offset, steer))

            time.sleep(0.05)
    finally:
        ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))

    csv_path = os.path.join(save_dir, "run_log.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    # 统计口径：均值只算「真实测到车道线」的帧；丢线帧单独统计占比。
    measured = [float(r["offset"]) for r in rows if r["offset"] != ""]
    mean_abs = float(np.mean(np.abs(measured))) if measured else -1.0
    max_abs = float(np.max(np.abs(measured))) if measured else -1.0
    n_frames = len(rows)
    detected = len(measured)
    lost_ratio = (n_frames - detected) / n_frames if n_frames else 1.0
    duration = rows[-1]["t"] if rows else 0.0
    speeds = [r["speed_kmh"] for r in rows]
    mean_speed = float(np.mean(speeds)) if speeds else 0.0
    max_speed = float(np.max(speeds)) if speeds else 0.0

    meta = {"mean_abs_offset": mean_abs, "lost": n_frames - detected, "frames": n_frames,
            "lost_ratio": lost_ratio}
    save_plot(rows, os.path.join(save_dir, "result.png"), meta)

    limits = {"max_mean_offset": float(ev["max_mean_offset"]),
              "max_lost_ratio": float(ev["max_lost_ratio"]),
              "require_complete": bool(ev.get("require_complete", True))}
    ok = (detected > 0
          and mean_abs <= limits["max_mean_offset"]
          and lost_ratio <= limits["max_lost_ratio"]
          and (completed or not limits["require_complete"]))

    with open(os.path.join(save_dir, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("车道线巡线实验摘要\n")
        fh.write("地图: %s\n" % carla_map.name)
        fh.write("总帧数: %d，可靠测量 %d 帧，丢线 %d 帧（%.1f%%；含单侧无历史宽度的不可靠帧 %d）\n"
                 % (n_frames, detected, n_frames - detected, lost_ratio * 100, source_count["single"]))
        fh.write("感知来源分布: dual=%d, left_only=%d, right_only=%d, single=%d, none=%d\n"
                 % (source_count["dual"], source_count["left_only"], source_count["right_only"],
                    source_count["single"], source_count["none"]))
        fh.write("横向偏移(归一化，仅统计测到的帧): 平均 %.4f，最大 %.4f\n" % (mean_abs, max_abs))
        fh.write("车速: 平均 %.1f km/h，最大 %.1f km/h（目标 %.1f m/s = %.1f km/h）\n"
                 % (mean_speed, max_speed, float(ctrl["target_speed"]), float(ctrl["target_speed"]) * 3.6))
        fh.write("运行时长: %.1f s / 设定 %.1f s（%s）\n"
                 % (duration, float(ctrl["max_duration"]), "跑满" if completed else "提前结束"))
        fh.write("PASS 判据: 平均偏移 <= %.2f 且 丢线占比 <= %.0f%% 且 跑满设定时长=%s → %s\n"
                 % (limits["max_mean_offset"], limits["max_lost_ratio"] * 100,
                    limits["require_complete"], "PASS" if ok else "FAIL"))
        fh.write("标注帧: %s\n" % ", ".join(annotated))

    sem_cam.stop(); rgb_cam.stop(); sem_cam.destroy(); rgb_cam.destroy(); ego.destroy()
    print("[lane] 平均偏移 %.4f（%d/%d 帧测到，丢线 %.1f%%），时长 %.1fs/%ss，结论：%s"
          % (mean_abs, detected, n_frames, lost_ratio * 100, duration,
             float(ctrl["max_duration"]), "PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    # launch_ros 的 Node 会追加 --ros-args / __node:= 等参数，这里容忍未知参数
    args, _unknown = build_argparser().parse_known_args()
    cfg = merge_params(load_params(args.params), args)
    try:
        return run(cfg)
    except KeyboardInterrupt:
        print("\n[lane] 用户中断")
        return 130


if __name__ == "__main__":
    sys.exit(main())
