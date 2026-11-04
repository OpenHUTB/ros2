#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CARLA + ROS 2 Humble 环境自检脚本
模块：carla_nn_car_following（基于神经网络的无人小车自适应跟车 / ACC）
用途：在运行模块前，先确认 Python/carla 包、CARLA 连接、雷达与相机数据、ROS 2 环境都正常。

用法：
    python3.10 check_env.py --host 192.168.148.1

它会依次检查：
  1. Python 版本是否匹配 CARLA 0.9.16 的 wheel（只提供 cp310 / cp311 / cp312）
  2. carla 包能否导入、版本号
  3. 能否连上 Windows 宿主机上的 CARLA 服务端（TCP 2000）
  4. 能否读取地图、生成自车
  5. 雷达 + 相机能否正常出数据（雷达是 ACC 跟车的核心传感器）
  6. rclpy（ROS 2 Humble）能否导入
最后打印 PASS / FAIL 汇总和排查建议。

注意：运行前请先在 Windows 上双击 CARLA 安装目录下的 CarlaUE4.exe 启动服务端
（本文档中的命令一律使用相对路径或 `~`，读者按自己的实际安装位置操作即可）。
"""

import argparse
import math
import sys


# --------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------
def hr(title):
    print("\n" + "=" * 68)
    print(title)
    print("=" * 68)


def set_attr(blueprint, name, value):
    """安全地设置蓝图属性：属性不存在时只警告，不中断。"""
    try:
        blueprint.set_attribute(name, str(value))
    except Exception as exc:  # IndexError / RuntimeError 都见过
        print("  [警告] 蓝图属性 %s 设置失败（已忽略）：%s" % (name, exc))


def hint(reason):
    """把失败原因翻译成新手能直接照做的排查步骤。"""
    print("\n排查建议：")
    for line in reason:
        print("  - " + line)


# --------------------------------------------------------------------------
# 第 1~2 步：Python 环境
# --------------------------------------------------------------------------
def check_python_and_carla():
    ok = True
    hr("步骤 1/6：Python 版本与 carla 包")

    ver = "%d.%d.%d" % sys.version_info[:3]
    print("  当前解释器：%s" % sys.executable)
    print("  Python 版本：%s" % ver)

    if sys.version_info[:2] not in ((3, 10), (3, 11), (3, 12)):
        print("  [失败] CARLA 0.9.16 的官方 wheel 只提供 cp310 / cp311 / cp312")
        print("         请改用 python3.10 运行本脚本，例如：")
        print("         python3.10 -m pip install carla==0.9.16")
        print("         python3.10 check_env.py --host <宿主机IP>")
        ok = False
    else:
        print("  [通过] Python 版本与 CARLA 0.9.16 wheel 匹配")

    try:
        import carla  # noqa: F401
    except ImportError as exc:
        print("  [失败] 无法导入 carla：%s" % exc)
        print("         安装（注意用同一个解释器）：")
        print("           python3.10 -m pip install carla==0.9.16")
        print("         若无法访问 PyPI，可用发行包自带的 wheel：")
        print('           pip3 install "<CARLA>/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl"')
        return None, None, False

    carla_version = getattr(carla, "__version__", "unknown")
    print("  [通过] carla 包导入成功，版本：%s" % carla_version)
    return carla, carla_version, ok


# --------------------------------------------------------------------------
# 第 3~4 步：连接与生成自车
# --------------------------------------------------------------------------
def connect_and_spawn(carla, host, port):
    hr("步骤 2/6：连接 CARLA 服务端（%s:%d）" % (host, port))
    client = carla.Client(host, port)
    client.set_timeout(60.0)  # 默认超时太短，第一次连接地图加载会很慢

    try:
        world = client.get_world()
    except Exception as exc:
        print("  [失败] 连接失败：%s" % exc)
        hint([
            "Windows 宿主机上 CarlaUE4.exe 是否已经启动（窗口能看到城市画面）？",
            "宿主机 IP 是否用 ipconfig 查过？填错就是这个报错（端口是 2000）。",
            "虚拟机网络：VMware 用『NAT』或『桥接』，NAT 下主机 IP 通常是 VMnet8 的地址。",
            "Windows 防火墙是否放行了 CarlaUE4.exe / 2000 端口？",
            "在宿主机浏览器访问 http://<宿主机IP>:2000 应能看到一段 JSON，看不到就是网络问题。",
        ])
        return None, None, False

    map_name = world.get_map().name
    print("  [通过] 已连接，服务端 CARLA 版本：%s" % client.get_server_version())
    print("  [通过] 当前地图：%s" % map_name)
    if "Town03" not in map_name:
        print("  [提示] 建议用 Town03 做开发（直路多、红绿灯少）：")
        print("         client.load_world('Town03')  # 或命令行 --town Town03")

    hr("步骤 3/6：生成自车")
    bp_lib = world.get_blueprint_library()
    vehicles = bp_lib.filter("vehicle.*")
    print("  可用车辆蓝图数量：%d" % len(vehicles))
    print("  地图出生点数量：%d" % len(world.get_map().get_spawn_points()))

    if "vehicle.tesla.model3" in [b.id for b in vehicles]:
        ego_bp = bp_lib.find("vehicle.tesla.model3")
    else:
        ego_bp = vehicles[0]
    set_attr(ego_bp, "role_name", "ego_vehicle")

    # 用官方出生点，它自带正确的车道朝向，能避免"出生就逆行"这个经典坑
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("  [失败] 地图没有出生点")
        return world, None, False

    ego = None
    for idx, sp in enumerate(spawn_points[:10]):
        ego = world.try_spawn_actor(ego_bp, sp)
        if ego is not None:
            print("  [通过] 自车已生成于出生点 #%d" % idx)
            print("         位置：%s" % ego.get_transform().location)
            print("         朝向 yaw：%.1f 度" % ego.get_transform().rotation.yaw)
            break
    if ego is None:
        print("  [失败] 前 10 个出生点都被占用，无法生成自车")
        return world, None, False

    return world, ego, True


# --------------------------------------------------------------------------
# 第 5 步：雷达 + 相机
# --------------------------------------------------------------------------
def check_sensors(carla, world, ego, ticks, radar_fov, radar_range):
    hr("步骤 4/6：雷达 + 相机数据链路")

    bp_lib = world.get_blueprint_library()
    sensors = []
    stats = {"radar": 0, "camera": 0, "nearest_gap": None}

    # ---- 雷达（ACC 跟车的核心传感器）----
    radar_bp = bp_lib.find("sensor.other.radar")
    set_attr(radar_bp, "role_name", "front_radar")
    set_attr(radar_bp, "horizontal_fov", radar_fov)
    set_attr(radar_bp, "vertical_fov", "10.0")
    set_attr(radar_bp, "range", radar_range)
    set_attr(radar_bp, "points_per_second", "1500")
    set_attr(radar_bp, "sensor_tick", "0.05")

    radar_tf = carla.Transform(carla.Location(x=2.0, z=1.2))
    radar = world.spawn_actor(radar_bp, radar_tf, attach_to=ego,
                              attachment_type=carla.AttachmentType.Rigid)
    sensors.append(radar)

    def on_radar(measurement):
        stats["radar"] += 1
        # 只统计正前方 ±8 度内的点，最小的 depth 就是与前车的距离
        for det in measurement:
            if abs(det.azimuth) < 0.14:
                if stats["nearest_gap"] is None or det.depth < stats["nearest_gap"]:
                    stats["nearest_gap"] = det.depth

    radar.listen(on_radar)

    # ---- 相机（后面可选，用来做可视化/加分）----
    cam_bp = bp_lib.find("sensor.camera.rgb")
    set_attr(cam_bp, "role_name", "front_camera")
    set_attr(cam_bp, "image_size_x", "640")
    set_attr(cam_bp, "image_size_y", "360")
    set_attr(cam_bp, "fov", "90")
    set_attr(cam_bp, "sensor_tick", "0.05")
    cam_tf = carla.Transform(carla.Location(x=1.6, z=1.6))
    camera = world.spawn_actor(cam_bp, cam_tf, attach_to=ego,
                               attachment_type=carla.AttachmentType.Rigid)
    sensors.append(camera)
    camera.listen(lambda image: stats.__setitem__("camera", stats["camera"] + 1))

    print("  传感器已挂载：雷达 1 个、相机 1 个")
    print("  正在以同步模式推进 %d 帧（fixed_delta_seconds=0.05，即约 %.1f 秒仿真时间）"
          % (ticks, ticks * 0.05))

    ok = True
    for i in range(ticks):
        world.tick()
        if (i + 1) % 25 == 0:
            print("    已推进 %d 帧：雷达=%d 相机=%d 前车距离=%s"
                  % (i + 1, stats["radar"], stats["camera"],
                     "%.1f m" % stats["nearest_gap"] if stats["nearest_gap"] else "无目标"))

    print("\n  结果：")
    print("    雷达回调帧数：%d" % stats["radar"])
    print("    相机回调帧数：%d" % stats["camera"])
    if stats["nearest_gap"] is not None:
        print("    正前方最近目标距离：%.2f m" % stats["nearest_gap"])
    else:
        print("    正前方最近目标距离：无（出生点附近没车，属正常）")

    if stats["radar"] == 0:
        print("  [失败] 雷达一帧数据都没收到 —— 90% 是传感器句柄被垃圾回收了")
        print("         检查：sensor 必须保存为对象成员/列表元素，不能只放在局部变量里")
        print("         本脚本里放在 sensors 列表里，所以是正确写法")
        ok = False
    elif stats["radar"] < ticks * 0.5:
        print("  [警告] 雷达帧数偏少，可能是 sensor_tick 与同步步长不匹配")
    else:
        print("  [通过] 雷达数据链路正常")

    if stats["camera"] == 0:
        print("  [警告] 相机无数据（后续若不用相机可忽略）")
    else:
        print("  [通过] 相机数据链路正常")

    return sensors, ok


# --------------------------------------------------------------------------
# 第 6 步：ROS 2
# --------------------------------------------------------------------------
def check_ros2():
    hr("步骤 5/6：ROS 2 Humble（rclpy）")
    try:
        import rclpy
    except ImportError as exc:
        print("  [失败] 无法导入 rclpy：%s" % exc)
        print("         说明当前解释器不是 ROS 2 的解释器。先 source 环境：")
        print("           source /opt/ros/humble/setup.bash")
        print("         再确认：which python3.10 && python3.10 -c 'import rclpy; print(rclpy.__file__)'")
        print("         注意：conda/venv 环境里通常没有 rclpy，跑 ROS 节点时请不要用 conda。")
        return False
    print("  [通过] rclpy 可导入：%s" % rclpy.__file__)
    try:
        from rclpy.utilities import get_rmw_implementation_identifier
        print("  [通过] RMW 实现：%s" % get_rmw_implementation_identifier())
    except Exception:
        pass
    return True


# --------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="CARLA + ROS 2 Humble 环境自检")
    parser.add_argument("--host", default="192.168.148.1",
                        help="Windows 宿主机 IP（ipconfig 查看），虚拟机里必须填宿主机 IP")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--ticks", type=int, default=100)
    parser.add_argument("--town", default=None, help="可选：自动加载地图，如 Town03")
    parser.add_argument("--radar-fov", default="30.0")
    parser.add_argument("--radar-range", default="100.0")
    args = parser.parse_args()

    print("CARLA + ROS 2 Humble 环境自检  |  模块：carla_nn_car_following")
    print("目标服务端：%s:%d" % (args.host, args.port))

    carla, carla_version, py_ok = check_python_and_carla()
    if carla is None:
        hr("汇总")
        print("  结果：FAIL（Python / carla 包未就绪，后面的检查已跳过）")
        return 2

    world = ego = None
    sensors = []
    original_settings = None
    carla_ok = sensors_ok = ros_ok = False

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)
        if args.town:
            print("\n正在加载地图 %s（首次会比较慢）..." % args.town)
            client.load_world(args.town)

        world, ego, carla_ok = connect_and_spawn(carla, args.host, args.port)
        if world is None:
            raise RuntimeError("world is None")
        if ego is None:
            carla_ok = False
        else:
            # 同步模式：ACC 采样必须用同步步进，否则数据时间戳对不上
            original_settings = world.get_settings()
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

            sensors, sensors_ok = check_sensors(
                carla, world, ego, args.ticks, args.radar_fov, args.radar_range)

            # 让自车以 50% 油门行驶，确认控制链路可用
            hr("步骤 6/6：车辆控制链路（油门/转向/刹车）")
            for _ in range(40):
                ego.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0, brake=0.0))
                world.tick()
            v = ego.get_velocity()
            speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            print("  施加 50%% 油门 40 帧后车速：%.2f m/s（约 %.1f km/h）" % (speed, speed * 3.6))
            if speed > 0.5:
                print("  [通过] 控制链路正常（车辆确实动起来了）")
            else:
                print("  [警告] 车速几乎为 0，检查是否被卡住/在墙里/apply_control 未生效")
            ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            for _ in range(20):
                world.tick()

        ros_ok = check_ros2()

    except KeyboardInterrupt:
        print("\n[中断] 用户手动终止")
    except Exception as exc:
        print("\n[异常] %s: %s" % (type(exc).__name__, exc))
    finally:
        # 清理：先传感器，再车辆，最后务必恢复世界设置
        for s in sensors:
            try:
                s.stop()
                s.destroy()
            except Exception:
                pass
        if ego is not None:
            try:
                ego.destroy()
            except Exception:
                pass
        if world is not None and original_settings is not None:
            try:
                world.apply_settings(original_settings)
                print("\n[清理] 已销毁传感器与自车，世界设置已恢复（异步模式）")
            except Exception:
                pass

    hr("汇总")
    print("  Python / carla 包 ：%s" % ("PASS" if py_ok else "FAIL"))
    print("  连接与生成自车    ：%s" % ("PASS" if carla_ok else "FAIL"))
    print("  雷达+相机数据     ：%s" % ("PASS" if sensors_ok else "FAIL"))
    print("  ROS 2 Humble      ：%s" % ("PASS" if ros_ok else "FAIL"))

    all_ok = py_ok and carla_ok and sensors_ok and ros_ok
    if all_ok:
        print("\n  全部通过，可以进入下一步：写参考控制器 + 采集跟车数据。")
        return 0
    print("\n  有项目未通过，请把上面的完整输出发给我，我据此调整代码。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
