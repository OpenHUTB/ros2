"""RLW 一键演示。

两种模式：

  demo（默认，离线）      合成"红灯接近"场景：自车 8 m/s 巡航、周期性
                         遇红灯，驱动三级滞回状态机，导出 CSV + 曲线；
  demo --live（需 CARLA） 真实场景：自车沿 Town01 行驶，遇红灯接近时
                         预警并演示刹停（演示基建），绿灯通过后恢复，
                         状态跃迁自动截图。

运行：python3 main.py demo [--live] [--host H] [--out-dir DIR] [--seconds N]
"""

import csv
import math
import sys
from pathlib import Path

ARTIFACTS = Path(__file__).resolve().parent.parent / "artifacts"

LVL = {"normal": 0, "approaching": 1, "violation_risk": 2}


def write_csv(path, rows, header):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def plot_curve(history, out_png, title="RLW offline demo"):
    """三行子图：停止线距离与预警区间 / 状态台阶 / 事件注记在距离线。"""
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
    ax.plot(t, [r["dist"] for r in history], color="tab:blue",
            label="dist to stop line")
    ax.axhline(30.0, ls="--", lw=1, color="tab:orange", label="approach < 30")
    d_stop_max = 0.5 * 8.0 ** 2 / 2.5 * 1.2   # 8 m/s 满速的判停距离
    ax.axhline(d_stop_max, ls="--", lw=1, color="tab:red",
               label=f"brake d_stop ≈ {d_stop_max:.1f}")
    ax.set_ylabel("dist (m)")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(title)

    ax = axes[1]
    ax.plot(t, [LVL[r["state"]] for r in history], drawstyle="steps-post",
            color="tab:purple")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["normal", "approaching", "violation_risk"])
    ax.set_ylabel("state")
    ax.set_ylim(-0.3, 2.3)

    ax = axes[2]
    colors = {"red": "tab:red", "yellow": "tab:orange", "green": "tab:green",
              "off": "gray"}
    ax.plot(t, [0.5 + 0.3 * {"red": 0, "yellow": 1, "green": 2, "off": 0.5}[r["light"]]
                for r in history], drawstyle="steps-post", color="black")
    ax.set_yticks([0.8, 1.1, 1.4])
    ax.set_yticklabels(["red", "yellow", "green"])
    ax.set_ylabel("light")
    ax.set_xlabel("time (s)")

    for r in history:
        if r["event"]:
            axes[0].annotate(r["event"], (r["t"], r["dist"]),
                             fontsize=7, rotation=45,
                             xytext=(2, 5), textcoords="offset points")

    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    return True


def _offline_light(t, cycle=25.0, red_start=10.0, red_end=20.0):
    """离线灯态：周期 25 s，其中 [10, 20) s 为红灯。"""
    phase = t % cycle
    if red_start <= phase < red_end:
        return "red"
    return "green"


def run_offline(out_dir=ARTIFACTS, ticks=500, dt=0.05):
    """离线场景：自车以 8 m/s 巡航循环接近停止线，灯按 25 s 周期切换。

    每轮：dist<30 红灯 → approaching；dist < d_stop(15.4) → violation_risk；
    变绿灯 → 恢复 normal。
    """
    from carla_red_light_warning.red_light_core import RLWController, RLWParams

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    p = RLWParams()
    ctrl = RLWController(p)
    history = []
    v = 8.0
    period_len = 25.0 / dt   # 每轮 25 s：dist 100 → 0 后重置
    for k in range(ticks):
        t = k * dt
        dist = 100.0 - (v * t) % (100.0)   # 循环接近：100 → 0
        light = _offline_light(t)
        r = ctrl.tick(light, dist, v, dt)
        history.append({
            "t": round(t, 2),
            "dist": round(dist, 2),
            "light": light,
            "state": r["state"],
            "event": r["event"],
            "v": round(v, 2),
        })

    csv_path = out_dir / "rlw_offline_data.csv"
    write_csv(csv_path, [(h["t"], h["dist"], h["light"], h["state"],
                          h["event"], h["v"]) for h in history],
              ["t", "dist", "light", "state", "event", "v"])
    png_path = out_dir / "rlw_offline_curve.png"
    ok = plot_curve(history, png_path, title="RLW offline demo")

    events = [(h["t"], h["event"]) for h in history if h["event"]]
    vio = [h["t"] for h in history if h["state"] == "violation_risk"]
    print(f"[demo] 离线演示完成：{len(history)} 拍（{len(history)*dt:.1f}s）")
    print(f"  violation_risk {len(vio)} 拍，事件 {len(events)} 条: {events}")
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
    try:
        for _ in range(4):
            world.tick()
    finally:
        try:
            cam.stop()
            cam.destroy()
        except Exception:  # noqa: BLE001  异常路径也要销毁临时相机，防 actor 泄漏
            pass
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


def _stop_distance(ego, tl):
    """停止线距离（m）：duck-typing 取 stop_locations，回退灯位近似。"""
    import carla
    loc = ego.get_location()
    getter = getattr(tl, "get_stop_locations", None)
    pts = list(getter()) if callable(getter) else [tl.get_transform().location]
    if not pts:
        return None
    nearest = min(pts, key=lambda p: (p.x - loc.x) ** 2 + (p.y - loc.y) ** 2)
    return math.hypot(nearest.x - loc.x, nearest.y - loc.y)


def run_live(host="127.0.0.1", port=2000, out_dir=ARTIFACTS, max_seconds=60):
    """真实 CARLA 场景：自车沿 Town01 行驶，遇红灯预警并演示刹停。

    桥接测量逻辑内嵌：vehicle.get_traffic_light() 给出自车受控的灯
    （仅红灯前有效），停止线距离 duck-typing 取 get_stop_locations。
    演示基建：violation_risk 时施加制动刹停（RLW 本身只预警不控车），
    绿灯后起步通过，形成"红灯预警刹停 → 绿灯通过恢复"循环。
    """
    from carla_red_light_warning import carla_common as cc
    from carla_red_light_warning.red_light_core import (
        RLWController, RLWParams, braking_decider)
    import carla

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[demo] 连接 CARLA {host}:{port} ...")
    client, world = cc.connect(host, port)
    cc.cleanup_actors(world)
    ego, tf = cc.spawn_vehicle(world)
    road_map = world.get_map()
    p = RLWParams()
    ctrl = RLWController(p)
    print(f"[demo] 自车已生成于 ({tf.location.x:.1f}, {tf.location.y:.1f})，"
          "遇红灯预警+演示刹停，绿灯通过")

    history = []
    shots = {}
    n_ticks = int(max_seconds / 0.05)

    def _measure():
        """桥接测量：灯态 + 停止线距离（自车不受灯控时返回 off/None）。"""
        tl = ego.get_traffic_light()
        if tl is None:
            return "off", None
        d = _stop_distance(ego, tl)
        if d is None:
            return "off", None
        return ego.get_traffic_light_state().name.lower(), d

    def _lane_keep_steer():
        """航点跟随转向：瞄向前方 8 m 路点，按航向角差比例修正。"""
        wp = road_map.get_waypoint(ego.get_location(), project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        nxt = wp.next(8.0)
        if not nxt:
            return 0.0
        t = nxt[0].transform.location
        loc = ego.get_location()
        desired = math.atan2(t.y - loc.y, t.x - loc.x)
        current = math.radians(ego.get_transform().rotation.yaw)
        err = math.atan2(math.sin(desired - current), math.cos(desired - current))
        return max(-0.3, min(0.3, 1.2 * err))

    def _shot(tag):
        if tag not in shots:
            shots[tag] = out_dir / f"rlw_live_{tag}.png"
            _screenshot(world, ego, shots[tag])

    try:
        dt = 0.05
        prev_dist = None
        for k in range(n_ticks):
            t = k * dt
            light_raw, dist_raw = _measure()
            # 投影跳变剔除：0.05s 内停止线距离突变超 5m 为灯切换假值
            dist = dist_raw
            if prev_dist is not None and dist is not None \
                    and abs(dist - prev_dist) > 5.0:
                dist = prev_dist
            prev_dist = dist
            light = light_raw if dist is not None else "off"
            eff_dist = dist if dist is not None else 1.0e4
            r = ctrl.tick(light, eff_dist, cc.get_speed(ego), dt)

            # 车道保持行驶；violation_risk 时演示刹停（演示基建）
            if r["state"] == "violation_risk":
                brake = braking_decider(cc.get_speed(ego), eff_dist, p)
                cc.apply_control(ego, throttle=0.0, brake=brake)
            else:
                cc.apply_control(ego, throttle=0.35, steer=_lane_keep_steer(),
                                 brake=0.0)

            history.append({
                "t": round(t, 2),
                "dist": round(eff_dist, 2),
                "light": light,
                "state": r["state"],
                "event": r["event"],
                "v": round(cc.get_speed(ego), 2),
            })

            if r["state"] == "approaching":
                _shot("approach")
            if r["state"] == "violation_risk":
                _shot("violation")
            if r["event"] == "violation_clear":
                _shot("recover")

            if k % 20 == 0:
                print(f"  t={t:5.1f}s  灯={light:6s} 停止线={eff_dist:6.1f}  "
                      f"v={history[-1]['v']:4.1f}  {r['state']}")

            world.tick()   # 同步模式：每拍推进仿真
    finally:
        csv_path = out_dir / "rlw_live_data.csv"
        write_csv(csv_path, [(h["t"], h["dist"], h["light"], h["state"],
                              h["event"], h["v"]) for h in history],
                  ["t", "dist", "light", "state", "event", "v"])
        png_path = out_dir / "rlw_live_curve.png"
        ok = plot_curve(history, png_path, title="RLW CARLA live demo")
        events = [(h["t"], h["event"]) for h in history if h["event"]]
        vio = [h["t"] for h in history if h["state"] == "violation_risk"]
        print(f"[demo] live 完成：{len(history)} 拍（{len(history)*dt:.1f}s）")
        print(f"  violation_risk {len(vio)} 拍，事件 {len(events)} 条: {events}")
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
        else:
            print(f"未知参数 {a}，忽略")
    if live:
        return 1 if not run_live(host, port, out_dir, max_seconds=seconds) else 0
    return 0 if run_offline(out_dir) else 1


if __name__ == "__main__":
    sys.exit(main())
