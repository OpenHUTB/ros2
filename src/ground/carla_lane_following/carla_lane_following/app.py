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

import carla
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


def _band_center(tags, roi_top, roi_bottom, lane_tag, min_col, mid, wd):
    """在 [roi_top, roi_bottom) 这条横带里找出本车道左右两条车道线，返回其中点横坐标。"""
    h = tags.shape[0]
    y0, y1 = int(h * roi_top), int(h * roi_bottom)
    band = tags[y0:y1, :] == lane_tag
    cols = band.sum(axis=0)
    left_idx = [i for i in range(mid) if cols[i] >= min_col]
    right_idx = [i for i in range(mid, wd) if cols[i] >= min_col]
    if not left_idx or not right_idx:
        return None
    xl = max(left_idx)    # 离画面中心最近的左侧车道线（排除隔壁车道的线）
    xr = min(right_idx)   # 离画面中心最近的右侧车道线
    if xr - xl < wd * 0.15:
        return None       # 两条线靠得太近，多半是同一条线被劈成两半
    return (xl + xr) / 2.0


def semantic_lane_offset(sem_img, cfg):
    """语义分割 -> (近场横向偏移, 航向误差, 掩码)。

    - 近场（画面最下部）给出车相对车道中心的横向偏移；
    - 远场（画面中部）给出同样定义的偏移，远-近之差即「车头是否对正车道」。
    - 每侧只取离画面中心最近的那条车道线，可排除邻车道线与路口导流线。
    """
    arr = np.frombuffer(sem_img.raw_data, dtype=np.uint8).reshape((sem_img.height, sem_img.width, 4))
    tags = arr[..., 2]  # 语义分割：红通道存类别号
    h, w = tags.shape
    mid = w // 2
    lane_tag = int(cfg["perception"]["lane_tag"])
    min_col = int(cfg["perception"]["min_col_pixels"])
    roi_top = float(cfg["perception"]["roi_top"])

    mask = np.zeros_like(tags, dtype=bool)
    mask[int(h * roi_top):, :] = tags[int(h * roi_top):, :] == lane_tag

    near_c = _band_center(tags, (roi_top + 1.0) / 2, 1.0, lane_tag, min_col, mid, w)
    far_c = _band_center(tags, roi_top, (roi_top + 1.0) / 2, lane_tag, min_col, mid, w)
    if near_c is None and far_c is None:
        return None, None, mask
    near = near_c if near_c is not None else far_c
    far = far_c if far_c is not None else near_c
    offset = (near - mid) / (w / 2.0)
    heading = (far - near) / (w / 2.0)
    return float(offset), float(heading), mask


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
    fig.suptitle("Lane following: mean |offset|=%.3f, lost frames=%d/%d"
                 % (meta["mean_abs_offset"], meta["lost"], len(rows)))
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
            off, hd, _ = semantic_lane_offset(data["sem"], cfg)
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
    integral = 0.0
    prev_offset = 0.0
    prev_steer = 0.0
    offset_display = None
    lost = 0
    rows = []
    t0 = time.time()
    last_annot = -999.0
    annotated = []

    try:
        while True:
            t = time.time() - t0
            if t > float(ctrl["max_duration"]):
                print("[lane] 达到最长时长，结束")
                break

            sem = data["sem"]
            offset = None
            heading = None
            mask = None
            if sem is not None:
                offset, heading, mask = semantic_lane_offset(sem, cfg)

            speed = ego.get_velocity()
            speed_ms = math.sqrt(speed.x ** 2 + speed.y ** 2 + speed.z ** 2)

            if offset is None:
                lost += 1
                steer = prev_steer * 0.85   # 丢线时柔性保持上一帧转向，避免猛打方向
                if lost > int(out_cfg["max_lost_frames"]):
                    print("[lane] 车道线持续丢失，结束实验")
                    break
            else:
                lost = 0
                # EMA 平滑，抑制逐帧抖动
                smooth_offset = 0.6 * offset + 0.4 * prev_offset
                integral = max(-1.0, min(1.0, integral + smooth_offset * 0.05))
                steer = (float(ctrl["kp"]) * smooth_offset
                         + float(ctrl["ki"]) * integral
                         + float(ctrl["kd"]) * heading)
                steer = max(-float(ctrl["max_steer"]), min(float(ctrl["max_steer"]), steer))
                prev_offset = smooth_offset
                offset_display = smooth_offset
            prev_steer = steer

            # 速度环：比例控制
            err_v = float(ctrl["target_speed"]) - speed_ms
            throttle = max(0.0, min(0.85, err_v * 0.25))
            brake = 0.4 if err_v < -2.0 else 0.0
            ego.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))

            rows.append({
                "t": round(t, 3),
                "offset": round(offset_display, 4) if offset_display is not None else "",
                "steer": round(steer, 4),
                "speed_kmh": round(speed_ms * 3.6, 3),
                "lost": int(offset is None),
            })

            if (out_cfg["save_annotated"] and data["rgb"] is not None and mask is not None
                    and offset_display is not None
                    and t - last_annot >= float(out_cfg["annotated_interval"])):
                name = "lane_%05.1fs.png" % t
                annotate(data["rgb"], mask, offset_display, steer, os.path.join(save_dir, name))
                annotated.append(name)
                last_annot = t
                print("[lane] 标注帧 -> %s（offset=%.3f steer=%.3f）" % (name, offset_display, steer))

            time.sleep(0.05)
    finally:
        ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))

    csv_path = os.path.join(save_dir, "run_log.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    valid = [r["offset"] for r in rows if r["offset"] != ""]
    mean_abs = float(np.mean(np.abs(valid))) if valid else -1.0
    meta = {"mean_abs_offset": mean_abs, "lost": lost}
    save_plot(rows, os.path.join(save_dir, "result.png"), meta)

    with open(os.path.join(save_dir, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("车道线巡线实验摘要\n")
        fh.write("地图: %s\n" % carla_map.name)
        fh.write("帧数: %d（其中丢失车道线 %d 帧）\n" % (len(rows), lost))
        fh.write("平均横向偏移(归一化): %.4f\n" % mean_abs)
        fh.write("最大横向偏移(归一化): %.4f\n" % (max(abs(v) for v in valid) if valid else -1))
        fh.write("标注帧: %s\n" % ", ".join(annotated))

    sem_cam.stop(); rgb_cam.stop(); sem_cam.destroy(); rgb_cam.destroy(); ego.destroy()
    ok = valid and mean_abs < 0.25 and lost < len(rows) * 0.2
    print("[lane] 平均横向偏移 %.4f，丢失 %d/%d 帧" % (mean_abs, lost, len(rows)))
    print("[lane] 结论：%s" % ("PASS（车道保持稳定）" if ok else "FAIL（偏离过大或丢线过多）"))
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
