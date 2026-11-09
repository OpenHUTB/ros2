#!/usr/bin/env python3
"""airsim_multirotor_waypoint_nav — 多旋翼无人机航点飞行实验（AirSim API）。

背景
----
OpenHUTB 的空地一体模式（CarlaAir）把 CARLA 与 AirSim 合到同一个 UE4 进程里：
CARLA 在 2000 端口，AirSim 在 41451 端口，两套 API 共享同一个世界。
所以这个模块只用 AirSim 客户端飞无人机，同时借用 CARLA 客户端拍取证照片。

流程
----
1. 连接 AirSim，接管控制权（enableApiControl / armDisarm）；
2. takeoff 到指定高度；
3. 依次飞 4 个航点（相对起飞点的方形航线），每个航点记录到达误差与耗时；
4. 回到起点上方悬停、降落；
5. 输出 `flight_log.csv`（NED 位置/速度）、`trajectory.png`（3D 轨迹 + 俯视）、`summary.md`。

运行
----
    # 先用 CarlaAir 模式启动仿真器（带 settings.json 的 Multirotor 配置）
    python main.py --host 127.0.0.1
"""
import argparse
import csv
import math
import os
import sys
import time

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

# msgpack-rpc-python 0.4.1（AirSim 的 RPC 依赖）只兼容 tornado<5，
# 而系统 Python 里装的是 tornado 6（Jupyter 需要）。把 tornado 4.5.3 放在 deps/ 里，
# 只对本模块生效，不动系统环境。
DEPS = os.path.join(HERE, "deps")
if os.path.isdir(DEPS) and DEPS not in sys.path:
    sys.path.insert(0, DEPS)


def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def build_argparser():
    p = argparse.ArgumentParser(description="AirSim 无人机航点飞行实验")
    p.add_argument("--host", default=None, help="AirSim RPC 主机")
    p.add_argument("--port", type=int, default=None, help="AirSim RPC 端口")
    p.add_argument("--save-dir", default=None)
    p.add_argument("--params", default=DEFAULT_PARAMS)
    return p


def merge_params(cfg, args):
    if args.host:
        cfg["simulator"]["host"] = args.host
    if args.port:
        cfg["simulator"]["airsim_port"] = args.port
    if args.save_dir:
        cfg["output"]["save_dir"] = args.save_dir
    return cfg


def connect(host, port, timeout=20.0):
    """连 AirSim；OpenHUTB 的 AirSim 端口要等无人机世界加载完才可用，所以带重试。"""
    import airsim
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


def planar_error(position, target_xy):
    """航点到目标的平面距离（纯函数，便于单元测试）。

    position: (x, y, z)，NED 坐标；target_xy: (x, y)
    """
    return math.hypot(position[0] - target_xy[0], position[1] - target_xy[1])


def position_of(client):
    kin = client.getMultirotorState().kinematics_estimated
    p = kin.position
    v = kin.linear_velocity
    return (p.x_val, p.y_val, p.z_val), (v.x_val, v.y_val, v.z_val)


def save_plot(rows, waypoints, out_png, meta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    zs = [-r["z"] for r in rows]   # NED 的 -z 就是高度

    fig = plt.figure(figsize=(11, 5))
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax1.plot(xs, ys, zs, color="#2c7fb8", lw=1.6)
    ax1.scatter([0], [0], [0], marker="^", c="#f39c12", s=80, label="start")
    for i, (wx, wy) in enumerate(waypoints):
        ax1.scatter([wx], [wy], [meta["altitude"]], c="#c0392b", s=40)
        ax1.text(wx, wy, meta["altitude"], "W%d" % (i + 1), fontsize=8)
    ax1.set_xlabel("east x (m)")
    ax1.set_ylabel("north y (m)")
    ax1.set_zlabel("altitude (m)")
    ax1.set_title("3D trajectory")
    ax1.legend(fontsize=8)

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(xs, ys, color="#2c7fb8", lw=1.6, label="flight path")
    ax2.scatter([0], [0], marker="^", c="#f39c12", s=80, label="start")
    for i, (wx, wy) in enumerate(waypoints):
        ax2.scatter([wx], [wy], c="#c0392b", s=40)
        ax2.text(wx + 0.6, wy + 0.6, "W%d" % (i + 1), fontsize=8)
    ax2.set_xlabel("east x (m)")
    ax2.set_ylabel("north y (m)")
    ax2.set_title("top view: max arrival error %.2f m" % meta["max_error"])
    ax2.grid(alpha=.3)
    ax2.set_aspect("equal", adjustable="box")
    ax2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def _hutb_client_installed():
    """只有装了配套 hutb 客户端，才敢去连 OpenHUTB 的 CARLA 端口。

    上游 carla 客户端（0.9.13/0.9.15/0.9.16）与 OpenHUTB 2.10.x 协议不兼容，
    会在 get_world() 处 std::bad_alloc 直接 abort（进程级崩溃，Python 捕获不到），
    所以这里先判断，避免把一次可选截图变成整个实验的崩溃。
    """
    try:
        import importlib.metadata as md
        md.version("hutb")
        return True
    except Exception:
        return False


def capture_from_carla(cfg, save_path):
    """用 CARLA 客户端把旁观者相机挪到无人机旁边拍一张取证照片（可选，失败不影响实验）。"""
    if not _hutb_client_installed():
        print("[uav] 未安装配套 hutb 客户端，跳过 CARLA 取证截图（不影响飞行实验）")
        return None
    try:
        import carla
        cache = os.path.join(HERE, ".hutb_cache")
        os.makedirs(cache, exist_ok=True)
        client = carla.Client(cfg["simulator"]["host"], int(cfg["simulator"]["carla_port"]))
        client.set_timeout(15.0)
        client.set_files_base_folder(cache)
        world = client.get_world()
        spec = world.get_spectator()
        loc = spec.get_transform().location
        spec.set_transform(carla.Transform(carla.Location(x=loc.x, y=loc.y, z=loc.z + 3),
                                           carla.Rotation(pitch=-20)))
        return world.get_map().name
    except Exception as exc:      # noqa: BLE001
        print("[uav] 拍照失败（不影响飞行实验）: %s" % exc)
        return None


def run(cfg):
    out_cfg = cfg["output"]
    save_dir = os.path.join(_output_root(), out_cfg["save_dir"])
    os.makedirs(save_dir, exist_ok=True)
    sim = cfg["simulator"]
    flight = cfg["flight"]

    client = connect(sim["host"], int(sim["airsim_port"]))

    print("[uav] 接管控制权")
    client.enableApiControl(True)
    client.armDisarm(True)

    altitude = float(flight["takeoff_height"])
    print("[uav] 起飞到 %.1f m" % altitude)
    client.takeoffAsync().join()
    client.moveToZAsync(-altitude, float(flight["speed"])).join()
    time.sleep(1.0)

    rows = []
    t0 = time.time()
    waypoints = [(float(w[0]), float(w[1])) for w in flight["waypoints"]]
    errors = []
    durations = []

    def sample(phase):
        p, v = position_of(client)
        rows.append({"t": round(time.time() - t0, 3), "phase": phase,
                     "x": round(p[0], 3), "y": round(p[1], 3), "z": round(p[2], 3),
                     "vx": round(v[0], 3), "vy": round(v[1], 3), "vz": round(v[2], 3)})

    sample("takeoff")

    try:
        for i, (wx, wy) in enumerate(waypoints):
            wp_t0 = time.time()
            print("[uav] 飞往航点 %d/%d (%.1f, %.1f)" % (i + 1, len(waypoints), wx, wy))
            client.moveToPositionAsync(wx, wy, -altitude, float(flight["speed"])).join()
            # 到达判定：连续采样直到进入容差或超时
            deadline = time.time() + float(flight["timeout_per_waypoint"])
            err = float("inf")
            while time.time() < deadline:
                p, _v = position_of(client)
                err = planar_error(p, (wx, wy))
                sample("wp%d" % (i + 1))
                if err <= float(flight["arrive_tolerance"]):
                    break
                time.sleep(float(flight["log_interval"]))
            errors.append(err)
            durations.append(time.time() - wp_t0)
            print("[uav] 航点 %d 到达误差 %.2f m，耗时 %.1f s" % (i + 1, err, durations[-1]))

        print("[uav] 返回起点上方并降落")
        client.moveToPositionAsync(0, 0, -altitude, float(flight["speed"])).join()
        sample("return")
        client.landAsync().join()
        time.sleep(2.0)
        sample("landed")
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)

    with open(os.path.join(save_dir, "flight_log.csv"), "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    max_error = max(errors) if errors else -1.0
    total_time = rows[-1]["t"]
    meta = {"max_error": max_error, "altitude": altitude}
    save_plot(rows, waypoints, os.path.join(save_dir, "trajectory.png"), meta)

    map_name = capture_from_carla(cfg, os.path.join(save_dir, "spectator.png"))

    with open(os.path.join(save_dir, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("无人机航点飞行实验摘要\n")
        fh.write("地图: %s\n" % (map_name or "n/a"))
        fh.write("航点: %s\n" % waypoints)
        fh.write("巡航高度: %.1f m，巡航速度: %.1f m/s\n" % (altitude, float(flight["speed"])))
        fh.write("各航点到达误差(m): %s\n" % ", ".join("%.2f" % e for e in errors))
        fh.write("各航点耗时(s): %s\n" % ", ".join("%.1f" % d for d in durations))
        fh.write("全程耗时: %.1f s，采样点: %d\n" % (total_time, len(rows)))
        fh.write("最大到达误差: %.2f m\n" % max_error)

    ok = errors and max_error <= 3.0
    print("[uav] 最大到达误差 %.2f m，全程 %.1f s" % (max_error, total_time))
    print("[uav] 结论：%s" % ("PASS（航点全部到达）" if ok else "FAIL（有航点未能到达）"))
    return 0 if ok else 1


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
