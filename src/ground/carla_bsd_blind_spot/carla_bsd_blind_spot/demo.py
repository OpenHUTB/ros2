"""BSD 一键演示。

两种模式：

  demo（默认，离线）      合成“邻车从后方接近并超越”的点云场景，跑 BSD 决策，
                         导出左右盲区距离/分级/事件的曲线 PNG + CSV；
  demo --live（需 CARLA） 真实场景：自车巡航，邻车从左后方加速超越穿过盲区，
                         记录盲区监测全过程，刹车瞬间/报警瞬间自动截图。

运行：python3 main.py demo [--live] [--host H] [--out-dir DIR]
"""

import csv
import math
import sys
from pathlib import Path

ARTIFACTS = Path(__file__).resolve().parent.parent / "artifacts"


def write_csv(path, rows, header):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def plot_curve(history, out_png, title="BSD offline demo"):
    """三行子图：左右盲区距离 / 预警分级 / 事件时刻。"""
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
    ax.plot(t, [r["left_d"] for r in history], label="left zone", color="tab:blue")
    ax.plot(t, [r["right_d"] for r in history], label="right zone", color="tab:orange")
    ax.axhline(12.0, ls="--", lw=1, color="tab:red", label="warn < 12 m")
    ax.set_ylabel("distance (m)")
    ax.set_ylim(0, 42)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(title)

    ax = axes[1]
    level_num = {"normal": 0, "approach": 1, "warn": 2}
    ax.plot(t, [level_num[r["left_level"]] for r in history],
            drawstyle="steps-post", label="left level", color="tab:blue")
    ax.plot(t, [level_num[r["right_level"]] for r in history],
            drawstyle="steps-post", label="right level", color="tab:orange")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["normal", "approach", "warn"])
    ax.set_ylabel("alert level")
    ax.set_ylim(-0.3, 2.3)
    ax.legend(loc="upper left", fontsize=8)

    ax = axes[2]
    events = [(r["t"], r["event"]) for r in history if r["event"]]
    if events:
        ax.scatter([e[0] for e in events], [1] * len(events), marker="v", color="red")
        for tt, name in events:
            ax.annotate(name, (tt, 1), rotation=45, fontsize=7,
                        xytext=(2, 4), textcoords="offset points")
    ax.set_ylim(0, 2)
    ax.set_yticks([])
    ax.set_xlabel("time (s)")
    ax.set_ylabel("events")

    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    return True


def run_offline(out_dir=ARTIFACTS, ticks=400, dt=0.05):
    """离线场景：邻车从左后方 35 m 处以 8 m/s 相对速度接近并超越自车。

    合成规则：邻车位于左车道 y≈3.2，自车 x=0；
    邻车纵向位置 x(t) = -35 + 8*t，当 x ∈ [-25, 2]、y ∈ [2.9, 4.2] 时
    落入盲区窗口（与 bsd_core.blind_zone_distance 同参），
    越窗前后测不到目标（返回 valid_max）。
    """
    from carla_bsd_blind_spot.bsd_core import BSDController, BSDParams

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    ctrl = BSDController(BSDParams())
    history = []
    left_d = 40.0
    for k in range(ticks):
        t = k * dt
        # 邻车纵向位置：x(t) = -35 + 8*t（从左后方 35m 逼近，8 m/s 相对速度）。
        # 只有邻车在侧向窗口 x ∈ [-25, 2] 内才"测得到"（越过后脱离窗口）。
        x_rel = -35.0 + 8.0 * t
        left_d = math.hypot(x_rel, 3.2) if -25.0 <= x_rel <= 2.0 else 40.0
        right_d = 40.0  # 右侧无车
        r = ctrl.tick(left_d, right_d, dt)
        lv = {}
        for side, rr in r.items():
            lv[side] = ("warn" if rr["state"] == "warn"
                        else ("approach" if rr["approach"] else "normal"))
        history.append({
            "t": round(t, 2), "left_d": round(left_d, 1), "right_d": right_d,
            "left_level": lv["left"], "right_level": lv["right"],
            "event": r["left"]["event"] or r["right"]["event"],
        })

    csv_path = out_dir / "bsd_offline_data.csv"
    write_csv(csv_path, [(h["t"], h["left_d"], h["right_d"],
                          h["left_level"], h["right_level"], h["event"])
                         for h in history],
              ["t", "left_d", "right_d", "left_level", "right_level", "event"])
    png_path = out_dir / "bsd_offline_curve.png"
    ok = plot_curve(history, png_path, title="BSD offline demo")

    warns = [h["t"] for h in history if h["left_level"] == "warn"]
    events = [(h["t"], h["event"]) for h in history if h["event"]]
    print(f"[demo] 离线演示完成：{len(history)} 拍")
    print(f"  预警时长: {len(warns)} 拍（{len(warns)*dt:.1f}s），事件: {events}")
    print(f"  输出: {csv_path}")
    if ok:
        print(f"        {png_path}")
    return True


def run_live(host="127.0.0.1", port=2000, out_dir=ARTIFACTS, gap=45.0, ego_speed=8.0):
    """真实 CARLA 场景：自车巡航，邻车从左后方加速超越穿过盲区。

    gap: 邻车初始位于自车左后方 gap 米处。
    自车 8 m/s 定速巡航（内置简单巡航控制），邻车 20 m/s 从左后超越——
    邻车依次经过 approach（远处接近）→ 盲区窗口（warn）→ 超越离开（clear）。
    """
    from carla_bsd_blind_spot import carla_common as cc
    from carla_bsd_blind_spot.bsd_core import (BSDController, BSDParams,
                                               blind_zone_distance,
                                               parse_lidar_points)
    import numpy as np
    import carla

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[demo] 连接 CARLA {host}:{port} ...")
    client, world = cc.connect(host, port)
    cc.cleanup_actors(world)
    ego, tf = cc.spawn_vehicle(world)
    buf = []

    def _on_pts(pc):
        buf.append(parse_lidar_points(bytes(pc.raw_data)))
        if len(buf) > 8:
            buf.pop(0)

    lidar = cc.make_lidar(world, ego, _on_pts, tick=True)
    print(f"[demo] 自车已生成于 ({tf.location.x:.1f}, {tf.location.y:.1f})，LiDAR 就绪（8 帧融合）")

    # 邻车：自车左后方 gap 米，车速 20 m/s（约 72 km/h），直线同向
    yaw = math.radians(tf.rotation.yaw)
    back = carla.Location(
        x=tf.location.x - gap * math.cos(yaw) - 3.2 * math.sin(yaw),
        y=tf.location.y - gap * math.sin(yaw) + 3.2 * math.cos(yaw),
        z=tf.location.z + 0.3)
    obs_bp = _pick_blueprint(world, [
        "vehicle.audi.tt", "vehicle.chevrolet.impala", "vehicle.audi.a2"])
    obs = world.try_spawn_actor(
        obs_bp, carla.Transform(back, carla.Rotation(yaw=tf.rotation.yaw)))
    if obs is None:
        print("[demo] 邻车生成失败，退出")
        return False
    obs.set_autopilot(False)
    print(f"[demo] 邻车已生成于左后方 {gap} m，将加速超越")

    ctrl = BSDController(BSDParams())
    history = []
    png_left = png_alert = None
    road_map = world.get_map()

    def _lane_keep_steer(vehicle):
        """航点跟随转向：瞄向前方 6 m 路点，按航向角差比例修正。

        无转向控制时车辆在微弯道路上会横向漂移（实测邻车 steer=0 漂出
        侧向窗口导致盲区跟踪断续），自车与邻车共用该保持器。
        """
        wp = road_map.get_waypoint(vehicle.get_location())
        nxt = wp.next(6.0)
        if not nxt:
            return 0.0
        target = nxt[0].transform.location
        loc = vehicle.get_location()
        desired = math.atan2(target.y - loc.y, target.x - loc.x)
        current = math.radians(vehicle.get_transform().rotation.yaw)
        err = math.atan2(math.sin(desired - current), math.cos(desired - current))
        return max(-0.3, min(0.3, 1.5 * err))

    try:
        dt = 0.05
        for k in range(600):  # 最长 30 s
            # 自车定速巡航 + 车道保持；邻车油门全开 + 车道保持（同向超越）
            v = cc.get_speed(ego)
            throttle = 0.5 if v < ego_speed else 0.0
            cc.apply_control(ego, throttle=throttle,
                             steer=_lane_keep_steer(ego))
            obs.apply_control(carla.VehicleControl(
                throttle=0.85, steer=_lane_keep_steer(obs)))

            if buf:
                pts = np.vstack(buf)
                left_d = blind_zone_distance(pts, side="left")
                right_d = blind_zone_distance(pts, side="right")
            else:
                left_d = right_d = 40.0

            r = ctrl.tick(left_d, right_d, dt)
            lv = {}
            for side, rr in r.items():
                lv[side] = ("warn" if rr["state"] == "warn"
                            else ("approach" if rr["approach"] else "normal"))
            event = r["left"]["event"] or r["right"]["event"]
            history.append({
                "t": round(k * dt, 2),
                "left_d": round(left_d, 1), "right_d": round(right_d, 1),
                "ego_v": round(v, 1),
                "left_level": lv["left"], "right_level": lv["right"],
                "event": event,
            })

            if r["left"]["event"] == "left_warn" and png_alert is None:
                png_alert = out_dir / "bsd_live_alert.png"
                _screenshot(world, ego, png_alert)
            if k % 4 == 0:
                print(f"  t={k*dt:5.1f}s  L={left_d:5.1f}  R={right_d:5.1f}  "
                      f"v={v:4.1f}  {lv['left']:>8s}/{lv['right']:<8s} {event}")

            # 结束条件：已报过警且左盲区恢复无目标（超越完成）
            if png_alert is not None and left_d >= 39.0 and k * dt > 8:
                png_left = out_dir / "bsd_live_pass.png"
                _screenshot(world, ego, png_left)
                break
            world.tick()
    finally:
        csv_path = out_dir / "bsd_live_data.csv"
        write_csv(csv_path, [(h["t"], h["left_d"], h["right_d"], h["ego_v"],
                              h["left_level"], h["right_level"], h["event"])
                             for h in history],
                  ["t", "left_d", "right_d", "ego_v",
                   "left_level", "right_level", "event"])
        png_path = out_dir / "bsd_live_curve.png"
        ok = plot_curve(history, png_path, title="BSD CARLA live demo")
        warns = [h["t"] for h in history if h["left_level"] == "warn"]
        events = [(h["t"], h["event"]) for h in history if h["event"]]
        print(f"[demo] live 完成：{len(history)} 拍（{len(history)*0.05:.1f}s）")
        print(f"  左盲区预警 {len(warns)} 拍，事件: {events}")
        print(f"  输出: {csv_path}")
        if ok:
            print(f"        {png_path}")
        if png_alert:
            print(f"        {png_alert}")
        if png_left:
            print(f"        {png_left}")
        for a in (lidar, obs, ego):
            try:
                a.destroy()
            except Exception:  # noqa: BLE001
                pass
        cc.restore_async(world)
    return True


def _pick_blueprint(world, candidates):
    """按候选顺序返回第一个真实存在的车辆蓝图。"""
    lib = world.get_blueprint_library()
    for name in candidates:
        try:
            return lib.find(name)
        except Exception:  # noqa: BLE001 IndexError/RuntimeError
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
    tf = carla.Transform(carla.Location(x=-5.5, z=2.8),
                         carla.Rotation(pitch=-12))
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


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    out_dir = ARTIFACTS
    host, port = "127.0.0.1", 2000
    live = False
    gap = 45.0
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
        elif a == "--gap":
            gap = float(next(it))
        else:
            print(f"未知参数 {a}，忽略")
    if live:
        return 1 if not run_live(host, port, out_dir, gap=gap) else 0
    return 0 if run_offline(out_dir) else 1


if __name__ == "__main__":
    sys.exit(main())
