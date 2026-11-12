#!/usr/bin/env python3
"""airsim_multirotor_waypoint_nav — 多旋翼无人机航点飞行实验（AirSim API）。

背景
----
OpenHUTB 的空地一体模式（CarlaAir）把 CARLA 与 AirSim 合到同一个 UE4 进程里：
CARLA 在 2000 端口，AirSim 在 41451 端口，两套 API 共享同一个世界。
本模块只用 AirSim 客户端飞无人机。

流程
----
1. 连接 AirSim，接管控制权（enableApiControl / armDisarm）；
2. takeoff 到指定高度，记录**起飞点**作为航点原点；
3. 航点为「相对起飞点的东/北偏移」，依次飞越，途中由采样线程持续记录位置与速度；
4. 每个航点用**三维**误差（含高度）判定到达，要求连续 N 次采样都在容差内；
5. 回到起飞点上方悬停、降落；
6. 输出 flight_log.csv（全程采样）、trajectory.png（3D + 俯视）、summary.md。

依赖（版本敏感）
----------------
    pip install msgpack-rpc-python "msgpack<1.0" "tornado<5"
msgpack>=1 去掉了 `encoding` 参数、tornado>=5 去掉了 `tornado.platform.auto`，
msgpackrpc 0.4.1 两者都不兼容；缺依赖时本模块会给出明确提示而不是抛 ImportError。

运行
----
    python main.py --host 127.0.0.1
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # module root
PKG_NAME = "airsim_multirotor_waypoint_nav"


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

DEP_HINT = (
    "[uav] 缺少 AirSim 客户端依赖。请安装：\n"
    "      pip install msgpack-rpc-python \"msgpack<1.0\" \"tornado<5\"\n"
    "      （msgpack>=1 去掉了 encoding 参数、tornado>=5 去掉了 tornado.platform.auto，\n"
    "        msgpackrpc 0.4.1 都不兼容）"
)


def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def build_argparser():
    p = argparse.ArgumentParser(description="AirSim 无人机航点飞行实验")
    p.add_argument("--host", default=None, help="AirSim RPC 主机")
    p.add_argument("--port", type=int, default=None, help="AirSim RPC 端口")
    p.add_argument("--save-dir", default=None, help="证据输出目录")
    p.add_argument("--params", default=DEFAULT_PARAMS, help="参数文件")
    return p


def merge_params(cfg, args):
    if args.host:
        cfg["simulator"]["host"] = args.host
    if args.port:
        cfg["simulator"]["airsim_port"] = args.port
    if args.save_dir:
        cfg["output"]["save_dir"] = args.save_dir
    return cfg


def import_airsim():
    try:
        import airsim  # noqa: WPS433
        return airsim
    except Exception as exc:                      # noqa: BLE001
        print(DEP_HINT)
        raise SystemExit("import airsim 失败: %r" % (exc,))


def connect(airsim, host, port, timeout=20.0):
    """连 AirSim；OpenHUTB 的 AirSim 端口要等无人机世界加载完才可用，所以带重试。"""
    client = airsim.MultirotorClient(ip=host, port=port, timeout_value=timeout)
    for attempt in range(10):
        try:
            client.confirmConnection()
            print("[uav] AirSim 已连接 %s:%d" % (host, port))
            return client
        except Exception as exc:              # noqa: BLE001
            print("[uav] 连接重试 %d/10: %s" % (attempt + 1, exc))
            time.sleep(3.0)
    raise RuntimeError("AirSim RPC 连不上，确认仿真器是用 CarlaAir 模式（Multirotor）启动的")


def position_of(client):
    kin = client.getMultirotorState().kinematics_estimated
    p = kin.position
    v = kin.linear_velocity
    return (p.x_val, p.y_val, p.z_val), (v.x_val, v.y_val, v.z_val)


def distance_3d(position, target):
    """三维距离（含高度）。纯函数，便于单元测试。

    position: (x, y, z)，NED；target: (x, y, z)
    """
    return math.sqrt(sum((position[i] - target[i]) ** 2 for i in range(3)))


def absolute_waypoints(home, offsets, altitude):
    """把「相对起飞点的水平偏移」换算成 NED 绝对目标点。纯函数，便于单元测试。"""
    return [(home[0] + float(dx), home[1] + float(dy), -abs(float(altitude)))
            for dx, dy in offsets]


def save_plot(rows, waypoints, home, out_png, meta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    zs = [-r["z"] for r in rows]   # NED 的 -z 就是高度

    fig = plt.figure(figsize=(11, 5))
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax1.plot(xs, ys, zs, color="#2c7fb8", lw=1.4, label="flight path")
    ax1.scatter([home[0]], [home[1]], [meta["altitude"]], marker="^", c="#f39c12", s=70, label="takeoff")
    for i, (wx, wy, wz) in enumerate(waypoints):
        ax1.scatter([wx], [wy], [-wz], c="#c0392b", s=30)
        ax1.text(wx, wy, -wz, "W%d" % (i + 1), fontsize=8)
    ax1.set_xlabel("north x (m)  [NED]")
    ax1.set_ylabel("east y (m)  [NED]")
    ax1.set_zlabel("altitude (m)")
    ax1.set_title("3D trajectory (%d samples)" % len(rows))
    ax1.legend(fontsize=8)

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(xs, ys, color="#2c7fb8", lw=1.4, label="flight path")
    ax2.scatter([home[0]], [home[1]], marker="^", c="#f39c12", s=70, label="takeoff")
    for i, (wx, wy, _wz) in enumerate(waypoints):
        ax2.scatter([wx], [wy], c="#c0392b", s=30)
        ax2.text(wx + 0.6, wy + 0.6, "W%d" % (i + 1), fontsize=8)
    ax2.set_xlabel("north x (m)  [NED]")
    ax2.set_ylabel("east y (m)  [NED]")
    ax2.set_title("top view: max 3D arrival error %.2f m" % meta["max_error"])
    ax2.grid(alpha=.3)
    ax2.set_aspect("equal", adjustable="box")
    ax2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def _hutb_client_installed():
    """只有装了配套 hutb 客户端，才敢去连 OpenHUTB 的 CARLA 端口。

    上游 carla 客户端（0.9.13/0.9.15/0.9.16）与 OpenHUTB 2.10.x 协议不兼容，
    会在 get_world() 处 std::bad_alloc 直接 abort（进程级崩溃，Python 捕获不到）。
    """
    try:
        import importlib.metadata as md
        md.version("hutb")
        return True
    except Exception:
        return False


def read_carla_map(cfg):
    """读取当前地图名（可选信息，失败不影响飞行实验）。

    注意：这里不拍照、不写文件——AirSim 客户端拿不到 CARLA 相机画面，
    早期版本函数名叫 capture_from_carla 但从未真正保存过文件，已改名以免误导。
    地图缓存放到系统临时目录，避免在源码树里留下 .hutb_cache。
    """
    if not _hutb_client_installed():
        print("[uav] 未安装配套 hutb 客户端，跳过读取 CARLA 地图名（不影响飞行实验）")
        return None
    try:
        import tempfile

        import carla
        cache = os.path.join(tempfile.gettempdir(), "openhutb_map_cache")
        os.makedirs(cache, exist_ok=True)
        client = carla.Client(cfg["simulator"]["host"], int(cfg["simulator"]["carla_port"]))
        client.set_timeout(15.0)
        client.set_files_base_folder(cache)
        return client.get_world().get_map().name
    except Exception as exc:      # noqa: BLE001
        print("[uav] 读取地图名失败（不影响飞行实验）: %s" % exc)
        return None


def grab_scene(client, airsim, frames_dir, index):
    """用 simGetImages 抓一帧机载视角图（PR 里的飞行动图就由这些帧合成）。"""
    try:
        resp = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        if not resp or resp[0].width == 0:
            return False
        arr = np.frombuffer(resp[0].image_data_uint8, dtype=np.uint8)
        img = arr.reshape(resp[0].height, resp[0].width, 3)
        from PIL import Image
        Image.fromarray(img).save(os.path.join(frames_dir, "frame_%04d.png" % index))
        return True
    except Exception:                      # noqa: BLE001  截图失败不影响飞行
        return False


def fly_to(client, target, cfg, sampler_rows, phase, t0, on_sample=None):
    """下发一次位置指令后，在主循环里边采样边等到达（不做后台线程 RPC）。

    返回 (悬停中位误差, 是否闭锁到达, 耗时, 离开时刻误差)。

    - 悬停中位误差：进入容差后 hoverAsync + 悬停 settle_time 秒、取中位数（PASS 用这个口径）；
    - 离开时刻误差：即将飞向下一个航点前再测一次，反映"离开时实际偏离多少"。
      AirSim 的位置控制在重新给指令后会有加速度带来的偏移，因此这个值天然偏大，
      仅作为透明度指标一并输出，不参与 PASS 判定（口径说明见 README 第 9 节）。
    moveToPositionAsync 不 join：指令已在飞控里执行，我们只负责轮询状态，避免阻塞导致无法采样。
    """
    flight = cfg["flight"]
    tol = float(flight["arrive_tolerance"])
    hits_needed = int(flight.get("consecutive_hits", 3))
    interval = float(flight.get("log_interval", 0.1))
    timeout = float(flight["timeout_per_waypoint"])

    last_cmd = target        # 最后一次实际下发的目标（补偿后会变）
    try:
        client.moveToPositionAsync(target[0], target[1], target[2], float(flight["speed"]))
    except Exception as exc:                    # noqa: BLE001
        print("[uav] 位置指令异常: %s" % exc)

    t_wp = time.time()
    deadline = t_wp + timeout
    err = float("inf")
    hits = 0
    while time.time() < deadline:
        p, v = position_of(client)
        err = distance_3d(p, target)
        hits = hits + 1 if err <= tol else 0
        sampler_rows.append({"t": round(time.time() - t0, 3), "phase": phase,
                             "x": round(p[0], 3), "y": round(p[1], 3), "z": round(p[2], 3),
                             "vx": round(v[0], 3), "vy": round(v[1], 3), "vz": round(v[2], 3),
                             "err": round(err, 3)})
        if on_sample is not None:
            on_sample()
        if hits >= hits_needed:
            # 进入容差后先下发悬停指令（把速度压到 0），再观察 settle_time 秒取中位误差，
            # 否则「飞越航点」过程中的瞬时距离会被误当成到达精度。
            try:
                client.hoverAsync()
            except Exception:                  # noqa: BLE001
                pass
            settle = float(flight.get("settle_time", 1.5))
            t_settle = time.time()
            settled = []
            while time.time() - t_settle < settle:
                p2, v2 = position_of(client)
                e2 = distance_3d(p2, target)
                settled.append(e2)
                sampler_rows.append({"t": round(time.time() - t0, 3), "phase": phase + "_settle",
                                     "x": round(p2[0], 3), "y": round(p2[1], 3), "z": round(p2[2], 3),
                                     "vx": round(v2[0], 3), "vy": round(v2[1], 3), "vz": round(v2[2], 3),
                                     "err": round(e2, 3)})
                time.sleep(interval)
            settled.sort()
            median = settled[len(settled) // 2] if settled else err
            # 稳态偏差修正：AirSim 内置位置控制在悬停时有约 1 m 的稳态偏差，
            # 这里按实测偏差把目标点反向补偿一次/两次，再复测（类似位置环的积分项）。
            corrections = 0
            while median > float(flight.get("fine_tolerance", 0.5)) and corrections < 2:
                p3, _v3 = position_of(client)
                tx = target[0] + (target[0] - p3[0])
                ty = target[1] + (target[1] - p3[1])
                tz = target[2] + (target[2] - p3[2])
                last_cmd = (tx, ty, tz)
                try:
                    client.moveToPositionAsync(tx, ty, tz, float(flight["speed"]))
                except Exception:              # noqa: BLE001
                    break
                time.sleep(0.5)
                try:
                    client.hoverAsync()
                except Exception:              # noqa: BLE001
                    pass
                settled = []
                t_settle = time.time()
                while time.time() - t_settle < settle:
                    p4, v4 = position_of(client)
                    e4 = distance_3d(p4, target)
                    settled.append(e4)
                    sampler_rows.append({"t": round(time.time() - t0, 3),
                                         "phase": phase + "_fix%d" % (corrections + 1),
                                         "x": round(p4[0], 3), "y": round(p4[1], 3), "z": round(p4[2], 3),
                                         "vx": round(v4[0], 3), "vy": round(v4[1], 3), "vz": round(v4[2], 3),
                                         "err": round(e4, 3)})
                    time.sleep(interval)
                settled.sort()
                median = settled[len(settled) // 2] if settled else median
                corrections += 1
            depart, _vd = position_of(client)
            departure = distance_3d(depart, last_cmd)   # 与补偿后的实际目标点比较
            return median, True, time.time() - t_wp, departure
        time.sleep(interval)
    depart, _vd = position_of(client)
    return err, False, time.time() - t_wp, distance_3d(depart, target)


def run(cfg):
    airsim = import_airsim()
    out_cfg = cfg["output"]
    save_dir = os.path.join(_output_root(), out_cfg["save_dir"])
    os.makedirs(save_dir, exist_ok=True)
    sim = cfg["simulator"]
    flight = cfg["flight"]
    ev = cfg["evaluation"]
    interval = float(flight.get("log_interval", 0.1))

    client = connect(airsim, sim["host"], int(sim["airsim_port"]))

    print("[uav] 接管控制权")
    client.enableApiControl(True)
    client.armDisarm(True)

    altitude = float(flight["takeoff_height"])
    print("[uav] 起飞到 %.1f m" % altitude)
    client.takeoffAsync().join()
    client.moveToZAsync(-altitude, float(flight["speed"])).join()
    time.sleep(1.0)

    home, _ = position_of(client)
    offsets = [(float(w[0]), float(w[1])) for w in flight["waypoints"]]
    waypoints = absolute_waypoints(home, offsets, altitude)
    print("[uav] 起飞点 NED=(%.2f, %.2f, %.2f)，航点为相对起飞点偏移 %s"
          % (home[0], home[1], home[2], offsets))

    frames_dir = os.path.join(_output_root(), out_cfg["save_dir"], "flight_frames")
    want_frames = bool(out_cfg.get("capture_images", True))
    if want_frames:
        os.makedirs(frames_dir, exist_ok=True)
        for old in os.listdir(frames_dir):
            if old.startswith("frame_"):
                os.remove(os.path.join(frames_dir, old))
    shot = {"n": 1, "last": 0.0}
    image_interval = float(out_cfg.get("image_interval", 1.0))

    def maybe_capture():
        if not want_frames:
            return
        now = time.time()
        if now - shot["last"] < image_interval:
            return
        if grab_scene(client, airsim, frames_dir, shot["n"]):
            shot["n"] += 1
            shot["last"] = now

    rows = []
    t0 = time.time()
    errors = []
    departures = []
    durations = []
    reached = []
    max_flight = float(flight.get("max_flight_time", 150.0))
    landed = False

    def record(phase):
        p, v = position_of(client)
        rows.append({"t": round(time.time() - t0, 3), "phase": phase,
                     "x": round(p[0], 3), "y": round(p[1], 3), "z": round(p[2], 3),
                     "vx": round(v[0], 3), "vy": round(v[1], 3), "vz": round(v[2], 3),
                     "err": 0.0})
        return p

    def land_and_sample(tag):
        """降落并全程采样：landAsync 不 join，靠轮询 z 判断落地。"""
        nonlocal landed
        try:
            client.landAsync()
        except Exception as exc:            # noqa: BLE001
            print("[uav] 降落指令异常: %s" % exc)
        # 落地判据：NED 的 z 从 -高度 回到接近 0（地面）。注意不能用"起飞悬停高度"判断，
        # 否则一开始就满足条件，降落过程只记一行（评审 #954 第 3 条）。
        deadline = time.time() + 90.0
        while time.time() < deadline:
            p = record(tag)
            maybe_capture()
            if p[2] >= -0.3:                # 离地 0.3 m 以内视为已落地
                landed = True
                break
            time.sleep(interval)
        for _ in range(3):                  # 落地后再补 3 个点，方便看出已停稳
            record(tag + "_done")
            time.sleep(interval)
        if not landed:
            print("[uav] 降落超时（90 s），仍继续交还控制权")
        return landed

    try:
        for i, target in enumerate(waypoints):
            if time.time() - t0 > max_flight:
                print("[uav] 超过总飞行时限 %.0f s，中止" % max_flight)
                break
            print("[uav] 飞往航点 %d/%d：相对起飞点偏移 (%.1f, %.1f)"
                  % (i + 1, len(waypoints), offsets[i][0], offsets[i][1]))
            err, ok_wp, dur, dep = fly_to(client, target, cfg, rows, "wp%d" % (i + 1), t0,
                                        on_sample=maybe_capture)
            errors.append(err)
            departures.append(dep)
            durations.append(dur)
            reached.append(ok_wp)
            print("[uav] 航点 %d 悬停中位误差 %.2f m（离开时 %.2f m），%s，耗时 %.1f s"
                  % (i + 1, err, dep, "已闭锁到达" if ok_wp else "超时未闭锁", dur))

        print("[uav] 返回起飞点上方并降落")
        client.moveToPositionAsync(home[0], home[1], -altitude, float(flight["speed"]))
        # 返程同样全程采样：要连续 3 次进入容差才收尾，避免"一进入就 break"导致只有 1 行
        deadline = time.time() + 30.0
        hits = 0
        while time.time() < deadline:
            p = record("return")
            maybe_capture()
            if distance_3d(p, (home[0], home[1], -altitude)) <= float(flight["arrive_tolerance"]):
                hits += 1
                if hits >= 3:
                    break
            else:
                hits = 0
            time.sleep(interval)
        land_and_sample("landing")
        record("landed")
        landed = True
    except Exception as exc:              # noqa: BLE001
        # 关键安全修复：飞行中出异常必须先降落，再交还控制权，否则无人机会从高空直接掉下来
        print("[uav] 飞行过程异常: %r，执行紧急降落" % (exc,))
        land_and_sample("emergency_landing")
        raise
    finally:
        try:
            client.armDisarm(False)
            client.enableApiControl(False)
        except Exception:                  # noqa: BLE001
            pass

    with open(os.path.join(save_dir, "flight_log.csv"), "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=["t", "phase", "x", "y", "z", "vx", "vy", "vz", "err"])
        writer.writeheader()
        writer.writerows(rows)

    max_error = max(errors) if errors else -1.0
    max_departure = max(departures) if departures else -1.0
    total_time = rows[-1]["t"] if rows else 0.0
    meta = {"max_error": max_error, "altitude": altitude}
    save_plot(rows, waypoints, home, os.path.join(save_dir, "trajectory.png"), meta)

    map_name = read_carla_map(cfg)

    all_reached = (len(errors) == len(waypoints) and all(reached)
                   and max_error <= float(ev["pass_max_error"]) and landed)
    with open(os.path.join(save_dir, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("无人机航点飞行实验摘要\n")
        fh.write("地图: %s\n" % (map_name or "n/a（未安装 hutb 客户端）"))
        fh.write("起飞点 NED: (%.2f, %.2f, %.2f)\n" % home)
        fh.write("航点（相对起飞点偏移, x=北 y=东）: %s\n" % offsets)
        fh.write("巡航高度: %.1f m，巡航速度: %.1f m/s\n" % (altitude, float(flight["speed"])))
        fh.write("到达判定: 三维误差 <= %.1f m 且连续 %d 次采样命中；单点超时 %.0f s，总时长上限 %.0f s\n"
                 % (float(flight["arrive_tolerance"]), int(flight.get("consecutive_hits", 3)),
                    float(flight["timeout_per_waypoint"]), max_flight))
        fh.write("误差口径: 主指标 = 悬停 settle_time=%.1f s 的中位误差（PASS 依据）；"
                 "离开时刻误差仅作透明度参考\n" % float(flight.get("settle_time", 1.5)))
        fh.write("各航点悬停中位误差(m): %s\n" % ", ".join("%.2f" % e for e in errors))
        fh.write("各航点离开时刻误差(m): %s\n" % ", ".join("%.2f" % e for e in departures))
        fh.write("各航点是否闭锁到达: %s\n" % ", ".join("是" if r else "否" for r in reached))
        fh.write("各航点耗时(s): %s\n" % ", ".join("%.1f" % d for d in durations))
        fh.write("全程时长: %.1f s，采样点: %d（间隔 %.2f s），是否落地: %s\n"
                 % (total_time, len(rows), interval, "是" if landed else "否"))
        fh.write("最大悬停中位误差: %.2f m（PASS 阈值 %.2f m）；最大离开时刻误差: %.2f m\n"
                 % (max_error, float(ev["pass_max_error"]), max_departure))
        fh.write("结论: %s\n" % ("PASS" if all_reached else "FAIL"))

    print("[uav] 最大悬停中位误差 %.2f m（阈值 %.2f m），最大离开时刻误差 %.2f m，"
          "全程 %.1f s，采样 %d 点"
          % (max_error, float(ev["pass_max_error"]), max_departure, total_time, len(rows)))
    print("[uav] 结论：%s" % ("PASS（航点全部到达并落地）" if all_reached else "FAIL（有航点未能到达）"))
    return 0 if all_reached else 1


def main():
    # launch_ros 的 Node 会追加 --ros-args / __node:= 等参数，这里容忍未知参数
    args, _unknown = build_argparser().parse_known_args()
    cfg = merge_params(load_params(args.params), args)
    try:
        return run(cfg)
    except KeyboardInterrupt:
        print("\n[uav] 用户中断")
        return 130


if __name__ == "__main__":
    sys.exit(main())
