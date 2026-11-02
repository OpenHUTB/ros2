"""AEB 一键演示。

用法：
    python3 main.py demo            # 离线：制动曲线仿真（不需要 ROS / CARLA）
    python3 main.py demo --live     # 真实：CARLA 场景（生成障碍车 → 触发 AEB → 截图 + 数据导出）

输出到 artifacts/（本目录已 gitignore）：
    离线   aeb_offline_curve.png / aeb_offline_data.csv
    --live aeb_live_curve.png / aeb_live_data.csv / aeb_live_brake.png

离线场景：自车 43 km/h 巡航逼近前方以 18 km/h 行驶的车辆，
AEB 依次经历 预警(18m) → 全力制动(7m) → 刹停 → 恢复 → 重新巡航
（完整状态链 + 恢复过程，滞回阈值防抖）。
"""
import csv
import math
from pathlib import Path

ARTIFACTS = Path(__file__).resolve().parent.parent / "artifacts"

# ---- 离线纵动力学玩具模型（仅用于演示与自检，非精确车辆模型）----
THRU_ACCEL = 4.0    # 油门满行程加速度 (m/s^2)
BRAKE_DECEL = 8.6   # 制动满行程减速度 (m/s^2，~0.9g，真实 AEB 量级)
DRAG = 0.18         # 线性气动/滚动阻力系数（终端巡航速 ≈ 油门*4/DRAG ≈ 12.2 m/s）

STATE_ORDER = {"cruise": 0, "warning": 1, "braking": 2, "recover": 3}


def simulate_aeb_scenario(v0=12.0, d0=60.0, v_obs=5.0, dt=0.05, t_max=90.0, p=None):
    """离线纵动力学仿真：自车以 v0 巡航，前方车辆以 v_obs 同向行驶，AEB 决策。

    参数：
        v0:    自车初速 (m/s，默认 12 ≈ 43 km/h)
        d0:    初始前向距离 (m)
        v_obs: 障碍车速度 (m/s，默认 5 ≈ 18 km/h)
        dt:    步长 (s)
        p:     AEBParams

    返回 (history, meta)：
        history: [(t, state, d, v, closing, throttle, brake, event), ...]
        meta:    {"min_gap", "brake_time", "stopped", "final_state", "t_end"}
    """
    from carla_aeb_braking.aeb_core import AEBController, AEBParams
    p = p or AEBParams()
    ctl = AEBController(p)
    d, v = float(d0), float(v0)
    t = 0.0
    history = []
    min_gap = float("inf")
    brake_time = None
    stopped = False
    while t < t_max:
        closing = max(v - v_obs, 0.0)
        state, throttle, brake, event = ctl.tick(d, closing, t=t)
        accel = throttle * THRU_ACCEL - brake * BRAKE_DECEL - DRAG * v
        v = max(0.0, v + accel * dt)
        d = max(0.0, d - (v - v_obs) * dt)
        min_gap = min(min_gap, d)
        if event == "brake" and brake_time is None:
            brake_time = t
        history.append((t, state, d, v, closing, throttle, brake, event))
        t += dt
        if v < 0.05 and d < p.brake_exit:
            stopped = True
        # 一次完整 AEB 事件结束：全力刹过车、已刹停、且恢复巡航
        if brake_time is not None and stopped and state == "cruise":
            break
    meta = {
        "min_gap": min_gap,
        "brake_time": brake_time,
        "stopped": stopped,
        "final_state": ctl.state,
        "t_end": t,
    }
    return history, meta


def _write_csv(rows, path, fields):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        w.writerows(rows)


def plot_curve(history, meta, out_png, title="AEB offline demo"):
    """三行图：前向距离 / 车速 / 制动与状态。返回 True 表示成功。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[demo] 未安装 matplotlib，跳过绘图（pip install matplotlib）")
        return False

    t = [h[0] for h in history]
    d = [h[2] for h in history]
    v = [h[3] for h in history]
    brake = [h[6] for h in history]
    state = [STATE_ORDER.get(h[1], 0) for h in history]
    from carla_aeb_braking.aeb_core import AEBParams
    pp = AEBParams()

    bt = meta["brake_time"]
    bt_txt = f"t={bt:.1f} s" if bt is not None else "not triggered"
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f"{title} (min gap {meta['min_gap']:.1f} m, brake {bt_txt})")
    ax = axes[0]
    ax.plot(t, d, lw=1.5)
    ax.axhline(pp.warn_enter, color="orange", ls="--", lw=1, label=f"warn < {pp.warn_enter:g} m")
    ax.axhline(pp.brake_enter, color="red", ls="--", lw=1, label=f"brake < {pp.brake_enter:g} m")
    ax.set_ylabel("distance (m)")
    ax.legend(loc="upper right")
    ax = axes[1]
    ax.plot(t, v, lw=1.5)
    ax.set_ylabel("speed (m/s)")
    ax = axes[2]
    ax.step(t, brake, where="post", color="red", lw=1.5, label="brake")
    ax.fill_between(t, 0, state, step="post", alpha=0.15, color="blue", label="state")
    ax.set_ylim(-0.1, 2.6)
    ax.set_yticks([0, 1, 2, 3], ["cruise", "warning", "braking", "recover"])
    ax.set_ylabel("brake / state")
    ax.set_xlabel("time (s)")
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(str(out_png), dpi=110)
    plt.close(fig)
    return True


def run_offline(out_dir=ARTIFACTS):
    """离线演示：完整状态链仿真 + 曲线图 + CSV。"""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    history, meta = simulate_aeb_scenario()
    csv_path = out_dir / "aeb_offline_data.csv"
    _write_csv(history, csv_path,
               ["t", "state", "distance_m", "speed_mps", "closing_mps",
                "throttle", "brake", "event"])
    png_path = out_dir / "aeb_offline_curve.png"
    plot_curve(history, meta, png_path)
    print(f"[demo] 离线演示完成（{meta['t_end']:.1f} s）：")
    print(f"  刹车时刻   t = {meta['brake_time']:.2f} s")
    print(f"  最小间隙   {meta['min_gap']:.2f} m")
    print(f"  是否刹停   {meta['stopped']}")
    print(f"  最终状态   {meta['final_state']}")
    print(f"  数据       {csv_path}")
    print(f"  曲线图     {png_path}")
    return 0


def _pick_clean_spawn(world, max_spawns=15, ticks=10):
    """扫描出生点，返回 (出生点变换, 前向车道干净距离)。

    逐点挂前向 LiDAR 采 8 帧融合点云，取“车身高度带 + 前向车道内最近簇”
    的距离作为车道干净距离，选最远者——保证障碍车放得下且不会被车道内
    的路侧设施（标志/立柱）抢先触发。
    """
    import numpy as np
    import carla
    from carla_aeb_braking.aeb_core import parse_lidar_points

    sps = world.get_map().get_spawn_points()[:max_spawns]
    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    best = None
    for sp in sps:
        ego = world.try_spawn_actor(bp, sp)
        if ego is None:
            continue
        lb = world.get_blueprint_library().find("sensor.lidar.ray_cast")
        lb.set_attribute("channels", "64")
        lb.set_attribute("points_per_second", "240000")
        lb.set_attribute("range", "100")
        lb.set_attribute("rotation_frequency", "20")
        lb.set_attribute("upper_fov", "15")
        lb.set_attribute("lower_fov", "-25")
        buf = []

        def _cb(pc, _buf=buf):
            _buf.append(parse_lidar_points(bytes(pc.raw_data)))
            if len(_buf) > 8:
                _buf.pop(0)

        lidar = world.spawn_actor(
            lb, carla.Transform(carla.Location(z=2.0)), attach_to=ego)
        lidar.listen(_cb)
        for _ in range(ticks):
            world.tick()
        if buf:
            p = np.vstack(buf)
            lane = ((np.abs(p[:, 1]) < 2.0) & (p[:, 2] >= -1.7)
                    & (p[:, 2] <= -0.4) & (p[:, 0] > 3) & (p[:, 0] < 100))
            lp = p[lane]
            if len(lp):
                bx = np.floor(lp[:, 0]).astype(int)
                counts = np.bincount(bx)
                good = np.flatnonzero(counts >= 6)
                nearest = float(good.min()) if len(good) else 100.0
            else:
                nearest = 100.0
        else:
            nearest = 0.0
        for a in (lidar, ego):
            try:
                a.destroy()
            except Exception:  # noqa: BLE001
                pass
        if best is None or nearest > best[1]:
            best = (sp, nearest)
    if best is None:
        raise RuntimeError("没有可用的出生点")
    return best[0], best[1]


def run_live(host="127.0.0.1", port=2000, out_dir=ARTIFACTS, gap=50.0):
    """真实 CARLA 演示：生成自车 + 前向障碍车，AEB 触发后截图并导出数据。

    gap: 障碍车初始距离（m）。自车从静止起步，需要足够加速段才能以巡航
    速度冲入全力制动区（d<7m）。实际取 min(gap, 车道干净距离-3m)：
    扫描选出前向车道最干净的出生点，障碍车放在车道干净范围内，
    保证它一定是 AEB 的制动目标（不会被路侧设施抢先）。
    """
    from carla_aeb_braking import carla_common as cc
    from carla_aeb_braking.aeb_core import (AEBController, lidar_obstacle_distance,
                                            parse_lidar_points)
    import numpy as np
    import carla

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[demo] 连接 CARLA {host}:{port} ...")
    client, world = cc.connect(host, port)
    cc.cleanup_actors(world)
    # 扫描出生点，选前向车道最干净的（放得下障碍车并留加速段）
    spawn_tf, clean_range = _pick_clean_spawn(world)
    gap = min(gap, max(clean_range - 3.0, 15.0))
    ego, tf = cc.spawn_vehicle(world, transform=spawn_tf)
    print(f"[demo] 选定出生点 ({spawn_tf.location.x:.1f}, {spawn_tf.location.y:.1f})，"
          f"前向车道 {clean_range:.0f}m 无障碍，障碍车距离 {gap:.0f}m")
    buf = []

    def _on_pts(pc):
        buf.append(parse_lidar_points(bytes(pc.raw_data)))
        if len(buf) > 8:
            buf.pop(0)

    lidar = cc.make_lidar(world, ego, _on_pts, tick=True)
    print(f"[demo] 自车已生成于 ({tf.location.x:.1f}, {tf.location.y:.1f})，前向 LiDAR 就绪（8 帧融合）")

    ctl = AEBController()
    p = ctl.p
    history = []
    shot_done = False
    stopped_until = None
    obstacle = None
    try:
        # 在自车正前方 gap 米处生成静止障碍车
        # （自车从静止起步，gap 给足加速距离，保证触发全力制动时仍有接近速度。
        #   实测出生点朝向可能与最近车道相反（双向道路），故直接按自车航向定位；
        #   路点 z 为 0，需抬高防止车辆陷入地面）
        obs_bp = None
        for name in ("vehicle.audi.tt", "vehicle.chevrolet.impala",
                     "vehicle.lincoln.mkz_2017"):
            try:
                obs_bp = world.get_blueprint_library().find(name)
                break
            except Exception:  # find() 在不同版本抛 IndexError/RuntimeError
                continue
        if obs_bp is None:
            raise RuntimeError("蓝图库中没有可用的障碍车模型")
        yaw = math.radians(tf.rotation.yaw)
        obs_tf = carla.Transform(
            carla.Location(x=tf.location.x + gap * math.cos(yaw),
                           y=tf.location.y + gap * math.sin(yaw),
                           z=tf.location.z + 0.3),
            tf.rotation)
        obstacle = world.try_spawn_actor(obs_bp, obs_tf)
        if obstacle is None:
            raise RuntimeError("无法生成障碍车")
        print(f"[demo] 障碍车已生成于 ({obs_tf.location.x:.1f}, "
              f"{obs_tf.location.y:.1f})")

        for step in range(int(90.0 / cc.DT)):
            world.tick()
            t = step * cc.DT
            v = cc.get_speed(ego)
            if buf:
                d = lidar_obstacle_distance(np.vstack(buf), valid_max=p.valid_max)
            else:
                d = p.valid_max
            closing = max(v, 0.0)  # 障碍车静止
            state, throttle, brake, event = ctl.tick(d, closing, t=t)
            cc.apply_control(ego, throttle, 0.0, brake)
            history.append((t, state, d, v, closing, throttle, brake, event))
            if event == "brake" and not shot_done:
                print(f"[demo] ★ 紧急制动触发（t={t:.1f}s，d={d:.1f}m，v={v:.1f}m/s）")
                shot_done = _grab_screenshot(world, ego, out_dir / "aeb_live_brake.png")
            # 刹停且障碍仍在前方：持续 2 s 后结束
            if v < 0.2 and d < p.brake_exit:
                if stopped_until is None:
                    stopped_until = t + 2.0
                elif t >= stopped_until:
                    break
            print(f"  t={t:5.1f}s  state={state:8s} d={d:5.1f}m  v={v:4.1f}m/s  "
                  f"brake={brake:.2f}", end="\r")
        print()
    finally:
        for actor in (lidar, ego, obstacle):
            if actor is not None:
                try:
                    actor.destroy()
                except Exception:  # noqa: BLE001
                    pass
        cc.restore_async(world)

    # 导出
    csv_path = out_dir / "aeb_live_data.csv"
    _write_csv(history, csv_path,
               ["t", "state", "distance_m", "speed_mps", "closing_mps",
                "throttle", "brake", "event"])
    meta = {
        "min_gap": min(h[2] for h in history) if history else float("inf"),
        "brake_time": next((h[0] for h in history if h[7] == "brake"), None),
        "stopped": any(h[3] < 0.2 and h[2] < p.brake_exit for h in history[-10:]),
        "final_state": ctl.state,
        "t_end": history[-1][0] if history else 0.0,
    }
    png_path = out_dir / "aeb_live_curve.png"
    plot_curve(history, meta, png_path, title="AEB CARLA live demo")
    print(f"[demo] CARLA 演示完成（{meta['t_end']:.1f} s）：")
    print(f"  刹车时刻   t = {meta['brake_time']} s")
    print(f"  最小间隙   {meta['min_gap']:.2f} m")
    print(f"  是否刹停   {meta['stopped']}")
    print(f"  最终状态   {meta['final_state']}")
    print(f"  数据       {csv_path}")
    print(f"  曲线图     {png_path}")
    return 0


def _grab_screenshot(world, ego, out_path):
    """在自车后上方挂临时 RGB 相机抓一帧存 PNG（offscreen 下 get_screenshot 易黑屏）。"""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import carla

    shot = {"img": None}

    def cb(img):
        if shot["img"] is None:
            shot["img"] = bytes(img.raw_data)

    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", "800")
    bp.set_attribute("image_size_y", "450")
    tf = ego.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    cam_tf = carla.Transform(
        carla.Location(x=tf.location.x - 6.0 * math.cos(yaw),
                       y=tf.location.y - 6.0 * math.sin(yaw),
                       z=tf.location.z + 2.5),
        carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw))
    cam = world.spawn_actor(bp, cam_tf)
    try:
        cam.listen(cb)
        # 同步模式下传感器按自身频率出帧（RGB 默认 10 Hz），
        # 需循环 tick 直到收到一帧
        for _ in range(12):
            world.tick()
            if shot["img"] is not None:
                break
    finally:
        try:
            cam.stop()
            cam.destroy()
        except Exception:  # noqa: BLE001
            pass
    if shot["img"] is None:
        print("[demo] 截图失败（未收到图像）")
        return False
    if not any(shot["img"]):
        print("[demo] 截图失败（图像全黑，可能为离屏渲染异常）")
        return False
    arr = np.frombuffer(shot["img"], dtype=np.uint8).reshape(450, 800, 4)
    rgb = arr[:, :, :3][:, :, ::-1].copy()
    plt.imsave(str(out_path), rgb)
    print(f"[demo] 刹车瞬间截图已保存 {out_path}")
    return True


if __name__ == "__main__":
    import sys
    if "--live" in sys.argv:
        raise SystemExit(run_live())
    raise SystemExit(run_offline())
