# -*- coding: utf-8 -*-
"""毫米波雷达目标跟踪与前碰撞预警（TTC/FCW）—— 模块入口。

对应报告：docs/ground/radar_ttc_fcw.md
示例数据：data/radar_frames.json（OpenHUTB v2.10.0 实测，56 帧 / 双目标）

用法（无需安装任何第三方库、无需启动仿真即可完整运行）：
    python main.py                 # 默认：回放实测数据，输出表格/CSV/曲线图
    python main.py --self-test     # 合成数据自检，验证算法链路
    python main.py --online        # 连接正在运行的 OpenHUTB/CARLA 实时演示
    python main.py --no-plot       # 跳过绘图（环境里没有 Pillow 时）

模块只依赖 Python 标准库；Pillow 为可选依赖，仅用于生成运行效果 PNG。
"""
import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

from radar_fcw import DEFAULTS, Tracker, decide, run_pipeline

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data", "radar_frames.json")


def repo_img_dir():
    """定位仓库里的 docs/img 目录；若模块被单独拷走则回退到模块内 output/。"""
    root = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
    cand = os.path.join(root, "docs", "img", "radar_ttc_fcw")
    if os.path.isdir(os.path.join(root, "docs", "img")):
        os.makedirs(cand, exist_ok=True)
        return cand
    out = os.path.join(HERE, "output")
    os.makedirs(out, exist_ok=True)
    return out


# ---------------------------------------------------------------- 输出
def print_table(records, stats):
    """只展示主目标航迹，避免近距离反射分裂的短暂航迹干扰阅读。"""
    print("\n=== 逐帧预警记录（每 5 帧抽样）===")
    print(f"{'t(s)':>6} {'trk':>3} {'depth':>7} {'v_rad':>7} {'az':>6} "
          f"{'ttc':>7} {'level':>9} {'gt_a':>7}")
    shown = [r for r in records
             if r["level"] in ("SAFE", "WARN", "STOP")
             and r["track"] == stats.get("main_track")]
    for r in shown[::5]:
        print(f"{r['t']:6.2f} {r['track']:3d} {r['depth']:7.2f} "
              f"{r['v_radial']:+7.2f} {r['az_deg']:+6.1f} "
              f"{(r['ttc'] if r['ttc'] is not None else float('nan')):7.2f} "
              f"{r['level']:>9} {(r['gt_a'] if r['gt_a'] is not None else float('nan')):7.2f}")

    print("\n=== 统计 ===")
    print(f"  回放帧数              : {stats['frames']}")
    print(f"  点云总点数 / 滤除杂波 : {stats['clutter_total']} / "
          f"{stats['clutter_rejected']} ({stats['clutter_ratio']*100:.1f}%)")
    print(f"  建立航迹数            : {stats['tracks']}")
    print(f"  首次 WARN / 首次 STOP : {stats.get('first_warn')} s / {stats.get('first_stop')} s")
    print(f"  TTC 首帧 -> 末帧      : {stats.get('ttc_first')} s -> {stats.get('ttc_last')} s")
    print(f"  测距 MAE / 最大误差   : {stats.get('depth_mae')} m / {stats.get('depth_max_err')} m")
    print(f"  远离目标帧 / 误报帧   : {stats['receding_records']} / {stats['false_alarms']}")


def save_results(records, stats, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "ttc_result.csv")
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(records[0].keys()))
        w.writeheader()
        w.writerows(records)
    json_path = os.path.join(out_dir, "ttc_stats.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(stats, f, ensure_ascii=False, indent=2)
    print(f"\n已输出: {csv_path}\n        {json_path}")


# ---------------------------------------------------------------- 绘图（可选）
def plot_curves(records, frames, out_dir, stats):
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        print("提示：未安装 Pillow，跳过 PNG 生成（--no-plot 可关闭此提示）")
        return None

    W, H, PAD = 1200, 960, 70
    img = Image.new("RGB", (W, H), "#ffffff")
    d = ImageDraw.Draw(img)
    fonts = []
    for p in (r"C:\Windows\Fonts\arial.ttf", r"C:\Windows\Fonts\msyh.ttc",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.exists(p):
            try:
                fonts = [ImageFont.truetype(p, 15), ImageFont.truetype(p, 20),
                         ImageFont.truetype(p, 13)]
                break
            except OSError:
                pass
    if not fonts:
        fonts = [ImageFont.load_default()] * 3
    f_small, f_title, f_tiny = fonts

    main = [r for r in records
            if r["level"] in ("SAFE", "WARN", "STOP")
            and r["track"] == stats.get("main_track")]
    recd = [r for r in records if r["level"] == "RECEDING"]
    if not main:
        print("无有效预警记录，跳过绘图")
        return None

    ts = [r["t"] for r in main]
    t0, t1 = min(ts), max(ts)
    gt = {f["t"]: f.get("gt_a") for f in frames}

    def axis(box, title, ylabel, ymin, ymax):
        x0, y0, x1, y1 = box
        d.rectangle(box, outline="#cccccc", width=1)
        d.text((x0, y0 - 28), title, fill="#222222", font=f_title)
        # 单位放在标题行右端，避免与左侧 y 轴刻度数字重叠
        d.text((x1 - 100, y0 - 26), f"[{ylabel}]", fill="#666666", font=f_small)
        for i in range(5):
            v = ymin + (ymax - ymin) * i / 4
            yy = y1 - (y1 - y0) * i / 4
            d.line((x0, yy, x1, yy), fill="#eeeeee", width=1)
            d.text((x0 - 52, yy - 8), f"{v:.1f}", fill="#666666", font=f_tiny)

    def px(t, x0, x1):
        return x0 + (x1 - x0) * (t - t0) / max(t1 - t0, 1e-6)

    def py(v, y0, y1, ymin, ymax):
        return y1 - (y1 - y0) * (v - ymin) / max(ymax - ymin, 1e-6)

    def series(rows, key, box, ymin, ymax, color, width=2):
        x0, y0, x1, y1 = box
        pts = [(px(r["t"], x0, x1), py(r[key], y0, y1, ymin, ymax))
               for r in rows if r.get(key) is not None]
        if len(pts) >= 2:
            d.line(pts, fill=color, width=width, joint="curve")
        return pts

    # --- 图 1: TTC 曲线 + 分级色带 ---
    box1 = (PAD, 60, W - PAD, 310)
    x0, y0, x1, y1 = box1
    ymin, ymax = 0.0, 8.0
    for r in main:
        c = {"SAFE": "#e8f5ec", "WARN": "#fff2d9", "STOP": "#fde3e3"}[r["level"]]
        xa = px(r["t"], x0, x1)
        d.rectangle((xa, y0, xa + 8, y1), fill=c)
    for th, color, label in ((DEFAULTS["ttc_warn"], "#e8a33d",
                              f"WARN {DEFAULTS['ttc_warn']:.1f} s"),
                             (DEFAULTS["ttc_stop"], "#d64545",
                              f"STOP {DEFAULTS['ttc_stop']:.1f} s")):
        yy = py(th, y0, y1, ymin, ymax)
        d.line((x0, yy, x1, yy), fill=color, width=2)
        d.text((x1 - 130, yy - 18), label, fill=color, font=f_small)
    axis(box1, "TTC curve and warning levels (target A, approaching 6 m/s)",
         "TTC (s)", ymin, ymax)
    series(main, "ttc", box1, ymin, ymax, "#2b6cb0", 3)

    # --- 图 2: 雷达测距 vs Ground Truth ---
    box2 = (PAD, 380, W - PAD, 630)
    ymin2, ymax2 = 0.0, 50.0
    axis(box2, "Radar range vs ground truth (bias calibrated)", "Range (m)", ymin2, ymax2)
    series(main, "depth", box2, ymin2, ymax2, "#2b6cb0", 3)
    gt_rows = [{"t": f["t"], "gt": f["gt_a"]} for f in frames if f.get("gt_a")]
    pts = [(px(r["t"], box2[0], box2[2]), py(r["gt"], box2[1], box2[3], ymin2, ymax2))
           for r in gt_rows if t0 - 0.2 <= r["t"] <= t1 + 0.2]
    if len(pts) >= 2:
        for i in range(0, len(pts) - 1, 2):
            d.line((pts[i], pts[min(i + 1, len(pts) - 1)]), fill="#888888", width=2)
    d.text((box2[2] - 250, box2[1] + 8), "blue: radar   gray dashed: GT",
           fill="#555555", font=f_small)

    # --- 图 3: 两个目标的径向速度 ---
    box3 = (PAD, 700, W - PAD, 900)
    ymin3, ymax3 = -8.0, 5.0
    axis(box3, "Radial velocity: approaching (A) vs receding (B)", "v (m/s)", ymin3, ymax3)
    d.line((box3[0], py(0, box3[1], box3[3], ymin3, ymax3),
            box3[2], py(0, box3[1], box3[3], ymin3, ymax3)), fill="#cccccc", width=1)
    series(main, "v_radial", box3, ymin3, ymax3, "#d64545", 3)
    series(recd, "v_radial", box3, ymin3, ymax3, "#2e9e5b", 3)
    d.text((box3[2] - 250, box3[1] + 8), "red: A (-6)   green: B (+3)",
           fill="#555555", font=f_small)

    path = os.path.join(out_dir, "ttc_curve.png")
    img.save(path)
    print(f"已输出: {path}")
    return path


# ---------------------------------------------------------------- 自检
def synthetic_frames():
    """构造一段合成点云：静止杂波 + 接近目标 + 远离目标。"""
    frames = []
    for i in range(60):
        t = round(i * 0.1, 2)
        pts = []
        # 静止地面杂波：v=0，深度分散
        for k in range(20):
            pts.append([0.0, -4.0 + k * 0.4, -3.0, 12.0 + k * 2.0])
        da = 42.0 - 6.0 * t
        db = 30.0 + 3.0 * t
        for j in range(8):                       # 接近目标 A
            pts.append([-6.0, -0.5 + j * 0.12, -1.0, da + j * 0.05])
        for j in range(5):                       # 远离目标 B
            pts.append([3.0, 7.0 + j * 0.1, -1.0, db + j * 0.05])
        frames.append({"t": t, "pts": pts, "gt_a": round(da, 2), "gt_b": round(db, 2)})
    return frames


def self_test():
    print("=== 合成数据自检 ===")
    frames = synthetic_frames()
    records, stats = run_pipeline(frames)
    levels = {r["level"] for r in records}
    assert stats["clutter_rejected"] > 0, "静止杂波未被剔除"
    assert "WARN" in levels and "STOP" in levels, "预警等级未覆盖 WARN/STOP"
    assert "RECEDING" in levels, "远离目标未被识别"
    assert stats["false_alarms"] == 0, "远离目标被误报为碰撞预警"
    ttc = [r["ttc"] for r in records if r["level"] in ("WARN", "STOP") and r["ttc"]]
    assert ttc and ttc[0] > ttc[-1], "TTC 未随接近过程单调下降"
    print(f"  PASS  杂波滤除 {stats['clutter_ratio']*100:.1f}%  "
          f"等级 {sorted(levels)}  TTC {ttc[0]:.2f} -> {ttc[-1]:.2f} s  "
          f"误报 {stats['false_alarms']}")
    return records, stats


# ---------------------------------------------------------------- 在线模式
def online_demo():
    """连接已启动的 OpenHUTB/CARLA，实时打印 TTC 与预警等级。"""
    try:
        import carla
    except ImportError:
        # 优先用外部解压好的 carla 包目录（环境变量指定），再退回平台自带 dist
        root = os.environ.get("OPENHUTB_ROOT", r"C:\hutb_windows_v2.10.0")
        for p in (os.environ.get("CARLA_PYTHON_PATH", ""),
                  os.path.join(root, "PythonAPI", "carla", "dist")):
            if p and os.path.isdir(p):
                sys.path.insert(0, p)
        import carla  # noqa: F401

    import struct
    import time
    from radar_fcw import Tracker

    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    bp = world.get_blueprint_library()
    for a in list(world.get_actors()):
        if "vehicle" in a.type_id or "sensor.other.radar" in a.type_id:
            a.destroy()
    time.sleep(0.5)
    sp = world.get_map().get_spawn_points()[0]
    fwd = sp.get_forward_vector()
    ego = world.try_spawn_actor(bp.filter("vehicle.tesla.model3")[0], sp)
    p = sp.location + fwd * 42.0
    tgt = world.try_spawn_actor(
        bp.filter("vehicle.audi.tt")[0],
        carla.Transform(carla.Location(p.x, p.y, sp.location.z + 0.3), sp.rotation))
    rbp = bp.find("sensor.other.radar")
    for k, v in {"range": "100", "horizontal_fov": "30", "vertical_fov": "14",
                 "points_per_second": "20000", "sensor_tick": "0.1"}.items():
        rbp.set_attribute(k, v)
    radar = world.spawn_actor(rbp, carla.Transform(carla.Location(z=2.0)), attach_to=ego)

    tracker = Tracker()
    state = {"last": None}

    def on_data(measurement):
        n = len(measurement.raw_data) // 16
        v = struct.unpack("<" + "ffff" * n, measurement.raw_data)
        pts = [[v[i], math.degrees(v[i + 1]), math.degrees(v[i + 2]), v[i + 3]]
               for i in range(0, n * 4, 4)]
        t = time.time() - state["t0"]
        for r in tracker.update(pts, t):
            print(f"t={r['t']:5.2f} trk={r['track']} d={r['depth']:6.2f}m "
                  f"v={r['v_radial']:+6.2f} ttc={r['ttc']} [{r['level']}]")

    try:
        state["t0"] = time.time()
        radar.listen(on_data)
        tgt.enable_constant_velocity(carla.Vector3D(-fwd.x * 6.0, -fwd.y * 6.0, 0))
        time.sleep(6.0)
    finally:
        tgt.disable_constant_velocity()
        radar.stop()
        radar.destroy()
        tgt.destroy()
        ego.destroy()
    print("在线演示结束")


# ---------------------------------------------------------------- 入口
def main():
    ap = argparse.ArgumentParser(description="毫米波雷达 TTC 前向碰撞预警")
    ap.add_argument("--data", default=DATA, help="离线回放的 JSON 数据路径")
    ap.add_argument("--self-test", action="store_true", help="合成数据自检")
    ap.add_argument("--online", action="store_true", help="连接 CARLA 实时演示")
    ap.add_argument("--no-plot", action="store_true", help="不生成 PNG")
    ap.add_argument("--out", default=None, help="结果输出目录")
    args = ap.parse_args()

    if args.online:
        online_demo()
        return

    if args.self_test:
        self_test()
        print("\n自检通过。运行 `python main.py` 可回放真实雷达数据。")
        return

    if not os.path.isfile(args.data):
        print(f"找不到示例数据 {args.data}，自动切换到合成数据自检。")
        self_test()
        return

    with open(args.data, encoding="utf-8") as f:
        meta = json.load(f)
    frames = meta["frames"]
    print(f"=== 实测数据回放: {meta.get('simulator', 'OpenHUTB')} ===")
    print(f"场景: {meta.get('scene', '')}")
    print(f"帧数: {len(frames)}  雷达: {meta.get('radar', {})}")

    records, stats = run_pipeline(frames)
    print_table(records, stats)
    out_dir = args.out or os.path.join(HERE, "data")   # 表格/统计放示例数据旁
    save_results(records, stats, out_dir)
    if not args.no_plot:
        plot_curves(records, frames, repo_img_dir(), stats)  # 效果图放 docs/img

    assert stats["false_alarms"] == 0, "远离目标出现误报"
    print("\n结论: 杂波被速度域分离抑制，接近目标触发 WARN -> STOP，"
          "远离目标不误报，TTC 曲线可用于前向碰撞预警。")


if __name__ == "__main__":
    main()
