"""LDW 一键演示。

两种模式：

  demo（默认，离线）      合成车道中心线 + 正弦偏离轨迹，驱动三级滞回
                         状态机，导出偏移跟踪曲线 PNG + CSV；
  demo --live（需 CARLA） 真实场景：自车沿 Town01 行驶，周期性注入偏置
                         转向缓慢偏离车道触发预警再回正，记录全程并自动截图。

运行：python3 main.py demo [--live] [--host H] [--out-dir DIR] [--seconds N]
"""

import csv
import math
import sys
from pathlib import Path

import numpy as np

ARTIFACTS = Path(__file__).resolve().parent.parent / "artifacts"

LVL = {"normal": 0, "deviation": 1, "departure": 2}


def write_csv(path, rows, header):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def plot_curve(history, out_png, title="LDW offline demo"):
    """三行子图：横向偏移与阈值 / 状态台阶 / 车速。"""
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
    ax.plot(t, [r["offset"] for r in history], color="tab:blue",
            label="lateral offset (+left)")
    ax.axhline(0.5, ls="--", lw=1, color="tab:orange", label="warn ±0.5")
    ax.axhline(-0.5, ls="--", lw=1, color="tab:orange")
    ax.axhline(0.9, ls="--", lw=1, color="tab:red", label="depart ±0.9")
    ax.axhline(-0.9, ls="--", lw=1, color="tab:red")
    ax.axhline(0.0, ls=":", lw=1, color="gray")
    ax.set_ylabel("offset (m)")
    ax.legend(loc="lower left", fontsize=8)
    ax.set_title(title)

    ax = axes[1]
    ax.plot(t, [LVL[r["state"]] for r in history], drawstyle="steps-post",
            color="tab:purple")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["normal", "deviation", "departure"])
    ax.set_ylabel("state")
    ax.set_ylim(-0.3, 2.3)

    ax = axes[2]
    ax.plot(t, [r["v"] for r in history], color="tab:green")
    ax.set_ylabel("speed (m/s)")
    ax.set_xlabel("time (s)")

    for r in history:
        if r["event"]:
            axes[0].annotate(r["event"], (r["t"], r["offset"]),
                             fontsize=7, rotation=45,
                             xytext=(2, 5), textcoords="offset points")

    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    return True


def run_offline(out_dir=ARTIFACTS, ticks=400, dt=0.05):
    """离线场景：自车沿直道 y=0 行驶，横按正弦 y(t)=1.2·sin(2πt/20) 偏离。

    偏移幅值跨过 warn(0.5) 与 depart(0.9) 阈值，左右两侧各触发一轮
    deviation → departure → 恢复 完整事件链。
    """
    from carla_lane_departure_warning.lane_departure_core import LDWController, LDWParams

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    p = LDWParams()
    ctrl = LDWController(p)
    history = []
    v = 6.0   # 离线自车恒速 6 m/s（LDW 只预警不控车）
    for k in range(ticks):
        t = k * dt
        offset = 1.2 * math.sin(2 * math.pi * t / 20.0)
        r = ctrl.tick(offset, dt)
        history.append({
            "t": round(t, 2),
            "offset": round(offset, 3),
            "heading_err": 0.0,
            "state": r["state"],
            "event": r["event"],
            "v": round(v, 2),
        })

    csv_path = out_dir / "ldw_offline_data.csv"
    write_csv(csv_path, [(h["t"], h["offset"], h["heading_err"],
                          h["state"], h["event"], h["v"]) for h in history],
              ["t", "offset", "heading_err", "state", "event", "v"])
    png_path = out_dir / "ldw_offline_curve.png"
    ok = plot_curve(history, png_path, title="LDW offline demo")

    events = [(h["t"], h["event"]) for h in history if h["event"]]
    dep = [h["t"] for h in history if h["state"] == "departure"]
    print(f"[demo] 离线演示完成：{len(history)} 拍（{len(history)*dt:.1f}s）")
    print(f"  departure {len(dep)} 拍，事件链: {events}")
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


def run_live(host="127.0.0.1", port=2000, out_dir=ARTIFACTS, max_seconds=60):
    """真实 CARLA 场景：自车沿 Town01 行驶，周期注入偏置转向偏离车道。

    桥接测量逻辑内嵌：地图路点投影 → 带符号横向偏移 → LDW 状态机。
    车道保持拉回 + 定期偏置注入形成"偏离→预警→回正→恢复"循环。
    """
    from carla_lane_departure_warning import carla_common as cc
    from carla_lane_departure_warning.lane_departure_core import (
        LDWController, LDWParams, lateral_offset, heading_error, reject_jump)
    import carla

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[demo] 连接 CARLA {host}:{port} ...")
    client, world = cc.connect(host, port)
    cc.cleanup_actors(world)
    ego, tf = cc.spawn_vehicle(world)
    road_map = world.get_map()
    p = LDWParams()
    ctrl = LDWController(p)
    print(f"[demo] 自车已生成于 ({tf.location.x:.1f}, {tf.location.y:.1f})，"
          "16 s 周期纯追踪偏离：居中 → 左偏 1.2 m（车道内）→ 回中")

    history = []
    shots = {}
    n_ticks = int(max_seconds / 0.05)

    def _measure():
        """桥接测量：自车位姿 → 带符号横向偏移/航向偏差。"""
        tf = ego.get_transform()
        wp = road_map.get_waypoint(tf.location, project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        road_yaw = math.radians(wp.transform.rotation.yaw)
        wxy = (wp.transform.location.x, wp.transform.location.y)
        offset = lateral_offset((tf.location.x, tf.location.y), wxy, road_yaw)
        herr = heading_error(math.radians(tf.rotation.yaw), road_yaw)
        return offset, herr, float(wp.lane_width)

    def _steer_toward_offset(target_offset):
        """纯追踪转向：瞄向前方 8 m 路点沿车道横向平移 target_offset 后的点。

        追固定横向偏差设定值（0=居中，+1.2=车道内左偏）——收敛后稳定保持，
        不会产生偏置对抗式车道保持的欠阻尼蛇形振荡。
        """
        tf = ego.get_transform()
        wp = road_map.get_waypoint(tf.location, project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        nxt = wp.next(8.0)
        if not nxt:
            return 0.0
        ahead = nxt[0].transform.location
        road_yaw = math.radians(wp.transform.rotation.yaw)
        nx, ny = -math.sin(road_yaw), math.cos(road_yaw)   # 左侧法向
        tx = ahead.x + nx * target_offset
        ty = ahead.y + ny * target_offset
        desired = math.atan2(ty - tf.location.y, tx - tf.location.x)
        current = math.radians(tf.rotation.yaw)
        err = math.atan2(math.sin(desired - current), math.cos(desired - current))
        return max(-0.5, min(0.5, 2.0 * err))

    def _shot(tag):
        if tag not in shots:
            shots[tag] = out_dir / f"ldw_live_{tag}.png"
            _screenshot(world, ego, shots[tag])

    try:
        dt = 0.05
        prev_offset = None
        for k in range(n_ticks):
            t = k * dt
            offset_raw, herr, width = _measure()
            # 投影跳变剔除：0.05s 内偏移突变超 1.5m 为相邻车道假值
            offset, rej = reject_jump(prev_offset, offset_raw)
            prev_offset = offset
            r = ctrl.tick(offset, dt)

            # 16 s 周期纯追踪：0~4s 居中 → 4~10s 左偏 +1.2 m（车道内偏移）→ 回中
            phase = t % 16.0
            target_offset = 1.2 if 4.0 <= phase < 10.0 else 0.0
            steer = _steer_toward_offset(target_offset)
            cc.apply_control(ego, throttle=0.3, steer=steer)

            history.append({
                "t": round(t, 2),
                "offset": round(offset, 3),
                "heading_err": round(herr, 3),
                "state": r["state"],
                "event": r["event"],
                "v": round(cc.get_speed(ego), 2),
            })

            if r["state"] == "deviation":
                _shot("deviation")
            if r["state"] == "departure":
                _shot("departure")
            if r["event"] == "deviation_clear":
                _shot("recover")

            if k % 20 == 0:
                print(f"  t={t:5.1f}s  偏移={offset:+5.2f}  航向差={herr:+5.2f}  "
                      f"v={history[-1]['v']:4.1f}  {r['state']}")

            world.tick()   # 同步模式：每拍推进仿真
    finally:
        csv_path = out_dir / "ldw_live_data.csv"
        write_csv(csv_path, [(h["t"], h["offset"], h["heading_err"],
                              h["state"], h["event"], h["v"])
                             for h in history],
                  ["t", "offset", "heading_err", "state", "event", "v"])
        png_path = out_dir / "ldw_live_curve.png"
        ok = plot_curve(history, png_path, title="LDW CARLA live demo")
        events = [(h["t"], h["event"]) for h in history if h["event"]]
        dep = [h["t"] for h in history if h["state"] == "departure"]
        print(f"[demo] live 完成：{len(history)} 拍（{len(history)*dt:.1f}s）")
        print(f"  departure {len(dep)} 拍，事件链: {events}")
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
