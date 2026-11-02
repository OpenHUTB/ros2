"""CSL 一键演示。

两种模式：

  demo（默认，离线）      合成"直道-弯道-直道"路径，自车以巡航速度行驶，
                         CSL 预瞄限速 + 斜坡控制，导出速度跟踪曲线 PNG + CSV；
  demo --live（需 CARLA） 真实场景：自车沿 Town01 圆环路巡航，弯道自动减速、
                         直道恢复巡航，记录目标/实际速度全过程并自动截图。

运行：python3 main.py demo [--live] [--host H] [--out-dir DIR] [--laps N]
"""

import csv
import math
import sys
from pathlib import Path

import numpy as np

ARTIFACTS = Path(__file__).resolve().parent.parent / "artifacts"


def write_csv(path, rows, header):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def plot_curve(history, out_png, title="CSL offline demo"):
    """三行子图：目标速度与实际速度 / 弯道半径 / 状态与事件。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[demo] 未安装 matplotlib，跳过绘图（pip install matplotlib）")
        return False

    t = [r["t"] for r in history]
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

    ax = axes[0]
    ax.plot(t, [r["v_max"] for r in history], ls="--", lw=1,
            label="curve limit (preview)", color="tab:red")
    ax.plot(t, [r["commanded"] for r in history], label="commanded (ramp)",
            color="tab:orange")
    ax.plot(t, [r["v"] for r in history], label="actual speed", color="tab:blue")
    ax.axhline(8.0, ls=":", lw=1, color="gray", label="cruise 8 m/s")
    ax.set_ylabel("speed (m/s)")
    ax.legend(loc="lower left", fontsize=8)
    ax.set_title(title)

    ax = axes[1]
    ax.plot(t, [r["radius"] for r in history], color="tab:green")
    ax.set_ylabel("curve radius (m)")
    ax.set_yscale("log")

    ax = axes[2]
    lvl = {"cruise": 0, "slowdown": 1}
    ax.plot(t, [lvl[r["state"]] for r in history], drawstyle="steps-post",
            color="tab:purple")
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["cruise", "slowdown"])
    ax.set_ylabel("state")
    ax.set_xlabel("time (s)")

    for i, r in enumerate(history):
        if r["event"]:
            axes[0].annotate(r["event"], (r["t"], r["commanded"]),
                             fontsize=7, rotation=45,
                             xytext=(2, 4), textcoords="offset points")

    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    return True


def run_offline(out_dir=ARTIFACTS, ticks=600, dt=0.05):
    """离线场景：合成"直道 60m → 半径 15m 圆弧 → 直道 60m"路径。

    自车以巡航 8 m/s 沿路径行驶（恒速仿真），CSL 以每拍前方 30m 路点
    预瞄限速，展示 slowdown（入弯减速）→ cruise（出弯恢复）完整闭环。
    """
    from carla_curve_speed_limit.curve_speed_core import (CSLController,
                                                          CSLParams)

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 合成路径：直道 → 左转圆弧（R=15） → 直道
    path = [(60.0 * i / 60, 0.0) for i in range(61)]
    x0, y0, r = path[-1][0], path[-1][1], 15.0
    for i in range(1, 60):
        ang = (math.pi / 2) * i / 60
        path.append((x0 + r * math.sin(ang), y0 + r * (1 - math.cos(ang))))
    x1, y1 = path[-1]
    for i in range(1, 61):
        path.append((x1, y1 + 60.0 * i / 60))   # 沿 +y 直行 60 m

    p = CSLParams()
    ctrl = CSLController(p)
    history = []
    pos_idx = 0.0   # 沿路径累计弧长（m）
    arc_len = [0.0]
    for i in range(1, len(path)):
        arc_len.append(arc_len[-1] + math.hypot(path[i][0] - path[i - 1][0],
                                                path[i][1] - path[i - 1][1]))
    total = arc_len[-1]

    v = p.cruise_speed
    for k in range(ticks):
        t = k * dt
        pos_idx = min(pos_idx + v * dt, total)   # 走到路径尽头即停（不取模，避免首尾直道方向跳变形成假急弯）
        # 自车当前位置沿路径最近点 idx（简化：直接用弧长定位）
        idx = min(range(len(arc_len)), key=lambda i: abs(arc_len[i] - pos_idx))
        # 前瞻 30m 的路点切片
        look = [path[idx]]
        j = idx
        while j < len(path) - 1 and arc_len[j] - arc_len[idx] < p.look_ahead:
            j += 1
            look.append(path[j])
        target = ctrl.target_from_points(look)
        r = ctrl.tick(target, dt)
        v = r["commanded_speed"]   # 仿真自车速度直接跟随斜坡指令
        # 记录用：同一切片的最急弯半径
        from carla_curve_speed_limit.curve_speed_core import waypoint_curvatures
        ka = waypoint_curvatures(np.array(look)) if len(look) >= 3 else [0.0]
        m = max(abs(x) for x in ka)
        radius = 1.0 / m if m > 1e-6 else 999.0
        history.append({
            "t": round(t, 2),
            "v_max": round(ctrl.target_from_points(look), 2),
            "commanded": round(r["commanded_speed"], 2),
            "v": round(v, 2),
            "radius": round(radius, 1),
            "state": r["state"],
            "event": r["event"],
        })

    csv_path = out_dir / "csl_offline_data.csv"
    write_csv(csv_path, [(h["t"], h["v_max"], h["commanded"], h["v"],
                          h["radius"], h["state"], h["event"])
                         for h in history],
              ["t", "v_max", "commanded", "v", "radius", "state", "event"])
    png_path = out_dir / "csl_offline_curve.png"
    ok = plot_curve(history, png_path, title="CSL offline demo")

    slow = [h["t"] for h in history if h["state"] == "slowdown"]
    events = [(h["t"], h["event"]) for h in history if h["event"]]
    print(f"[demo] 离线演示完成：{len(history)} 拍（{len(history)*dt:.1f}s）")
    print(f"  slowdown {len(slow)} 拍（{len(slow)*dt:.1f}s），事件: {events}")
    print(f"  输出: {csv_path}")
    if ok:
        print(f"        {png_path}")
    return True


def _pick_blueprint(world, candidates):
    """按候选顺序返回第一个真实存在的车辆蓝图。"""
    lib = world.get_blueprint_library()
    for name in candidates:
        try:
            return lib.find(name)
        except Exception:  # noqa: BLE001
            continue
    raise RuntimeError(f"候选蓝图均不可用: {candidates}")


def _screenshot(world, ego, path, fov=110):
    """同步挂临时 RGB 相机拍一帧并销毁。"""
    import carla
    import numpy as np
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", "800")
    bp.set_attribute("image_size_y", "450")
    bp.set_attribute("fov", str(fov))
    tf = carla.Transform(carla.Location(x=-6.5, z=3.0),
                         carla.Rotation(pitch=-10))
    cam = world.spawn_actor(bp, tf, attach_to=ego)
    holder = {}

    def _cb(img):
        if "arr" not in holder:
            arr = np.frombuffer(bytes(img.raw_data), dtype=np.uint8)
            holder["arr"] = arr.reshape(img.height, img.width, 4)[:, :, :3]

    cam.listen(_cb)
    for _ in range(4):
        world.tick()
    cam.stop()
    cam.destroy()
    if "arr" in holder:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            plt.imsave(str(path), holder["arr"])
            print(f"[demo] 截图: {path}")
            return True
        except Exception as e:  # noqa: BLE001
            print(f"[demo] 截图保存失败: {e}")
    return False


def run_live(host="127.0.0.1", port=2000, out_dir=ARTIFACTS, max_seconds=60,
             town=None, spawn_index=None):
    """真实 CARLA 场景：自车沿指定地图环路巡航，弯道自动减速、直道恢复。

    town 默认 None 用 carla_common.DEFAULT_MAP（Town01_Opt）；
    spawn_index 默认 None 沿用"依次尝试前 10 个出生点"的回退逻辑，
    显式给 N 则取地图第 N 个出生点（换场景出不同弯道的演示图）。
    桥接节点逻辑内嵌：地图路点预瞄 → 目标速度 → 滞回+斜坡 → 控车。
    每次状态跃迁自动截图，全程导出 CSV + 曲线。
    """
    from carla_curve_speed_limit import carla_common as cc
    from carla_curve_speed_limit.curve_speed_core import (CSLController,
                                                          CSLParams,
                                                          target_speed_along,
                                                          waypoint_curvatures)
    import numpy as np
    import carla

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[demo] 连接 CARLA {host}:{port} ...")
    client, world = cc.connect(host, port, town=town or cc.DEFAULT_MAP)
    cc.cleanup_actors(world)
    if spawn_index is not None:
        spawns = world.get_map().get_spawn_points()
        tf0 = spawns[int(spawn_index) % len(spawns)] if spawns else None
        ego, tf = cc.spawn_vehicle(world, transform=tf0)
    else:
        ego, tf = cc.spawn_vehicle(world)
    road_map = world.get_map()
    p = CSLParams()
    ctrl = CSLController(p)
    print(f"[demo] 自车已生成于 ({tf.location.x:.1f}, {tf.location.y:.1f})，"
          f"巡航 {p.cruise_speed} m/s，前瞻 {p.look_ahead:.0f} m")

    history = []
    shots = {}
    n_ticks = int(max_seconds / 0.05)

    def _lane_keep_steer():
        """航点跟随转向：瞄向前方 6 m 路点，按航向角差比例修正。

        CSL 模块本身只控速度不控转向；演示里自车需要沿车道行驶才能
        展示"入弯减速、出弯恢复"，故内嵌一个最小车道保持器。
        """
        wp = road_map.get_waypoint(ego.get_location())
        nxt = wp.next(6.0)
        if not nxt:
            return 0.0
        t = nxt[0].transform.location
        loc = ego.get_location()
        desired = math.atan2(t.y - loc.y, t.x - loc.x)
        current = math.radians(ego.get_transform().rotation.yaw)
        err = math.atan2(math.sin(desired - current), math.cos(desired - current))
        return max(-0.3, min(0.3, 1.5 * err))

    def _front_points():
        loc = ego.get_location()
        wp = road_map.get_waypoint(loc)
        pts = [(loc.x, loc.y)]
        wp_next = wp
        dist, step = 0.0, 2.0
        while dist < p.look_ahead:
            nxt = wp_next.next(step)
            if not nxt:
                break
            wp_next = nxt[0]
            t = wp_next.transform.location
            pts.append((t.x, t.y))
            dist += step
        return pts

    def _shot(tag):
        if tag not in shots:
            shots[tag] = out_dir / f"csl_live_{tag}.png"
            _screenshot(world, ego, shots[tag])

    try:
        dt = 0.05
        for k in range(n_ticks):
            pts = _front_points()
            target = ctrl.target_from_points(pts)
            speed = cc.get_speed(ego)
            r = ctrl.tick(target, dt)
            e = r["commanded_speed"] - speed
            if e > 0:
                cc.apply_control(ego, throttle=min(0.45 * e, 1.0),
                                 steer=_lane_keep_steer())
            else:
                cc.apply_control(ego, brake=min(0.28 * (-e), 1.0),
                                 steer=_lane_keep_steer())

            ka = waypoint_curvatures(np.asarray(pts))
            m = float(np.max(np.abs(ka))) if len(ka) else 0.0
            radius = 1.0 / m if m > 1e-6 else 999.0
            history.append({
                "t": round(k * dt, 2),
                "v_max": round(r["target_speed"], 2),
                "commanded": round(r["commanded_speed"], 2),
                "v": round(speed, 2),
                "radius": round(radius, 1),
                "state": r["state"],
                "event": r["event"],
            })

            if r["state"] == "slowdown":
                _shot("slowdown")
            if r["event"] == "cruise_resume":
                _shot("resume")

            if k % 20 == 0:
                print(f"  t={k*dt:5.1f}s  目标={target:4.1f}  指令={r['commanded_speed']:4.1f}  "
                      f"实际={speed:4.1f}  R={radius:5.0f}  {r['state']}")

            world.tick()   # 同步模式：每拍推进仿真，缺 tick 则世界完全静止
    finally:
        csv_path = out_dir / "csl_live_data.csv"
        write_csv(csv_path, [(h["t"], h["v_max"], h["commanded"], h["v"],
                              h["radius"], h["state"], h["event"])
                             for h in history],
                  ["t", "v_max", "commanded", "v", "radius", "state", "event"])
        png_path = out_dir / "csl_live_curve.png"
        ok = plot_curve(history, png_path, title="CSL CARLA live demo")
        slows = [h["t"] for h in history if h["state"] == "slowdown"]
        events = [(h["t"], h["event"]) for h in history if h["event"]]
        print(f"[demo] live 完成：{len(history)} 拍（{len(history)*dt:.1f}s）")
        print(f"  slowdown {len(slows)} 拍（{len(slows)*dt:.1f}s），事件: {events}")
        print(f"  输出: {csv_path}")
        if ok:
            print(f"        {png_path}")
        for tag, path in shots.items():
            print(f"        {path}")
        try:
            ego.destroy()
        except Exception:  # noqa: BLE001
            pass
        cc.restore_async(world)
    return True


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    out_dir = ARTIFACTS
    host, port = "127.0.0.1", 2000
    live = False
    seconds = 60
    town = None
    spawn_index = None
    it = iter(argv)
    for a in it:
        if a == "--live":
            live = True
        elif a == "--host":
            host = next(it)
        elif a == "--port":
            port = int(next(it))
        elif a == "--out-dir":
            out_dir = Path(next(it))
        elif a == "--seconds":
            seconds = int(next(it))
        elif a == "--town":
            town = next(it)
        elif a == "--spawn-index":
            spawn_index = int(next(it))
        else:
            print(f"未知参数 {a}，忽略")
    if live:
        return 1 if not run_live(host, port, out_dir, max_seconds=seconds,
                                 town=town, spawn_index=spawn_index) else 0
    return 0 if run_offline(out_dir) else 1


if __name__ == "__main__":
    sys.exit(main())
