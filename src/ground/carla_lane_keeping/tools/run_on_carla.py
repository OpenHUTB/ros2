#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在真实 CARLA 中运行车道线检测（不需要 ROS，Windows / Linux 均可）。

作用
----
把 `lane_detection.py` 的算法接到 CARLA 的前视相机上：
  1. 连接 CARLA 服务器，加载地图
  2. 生成一辆车，挂一个前视 RGB 相机
  3. 采集若干帧，对每帧跑车道线检测
  4. 打印横向偏差 / 航向偏差 / 曲率，并把带检测结果的图保存到磁盘
  5. 在终端用 ASCII 图展示鸟瞰掩码（便于无图形界面时快速判断）

两种模式
--------
  * 默认（单帧验证）：采若干帧做离线评估，输出图与掩码，用于调参。
  * `--closed-loop`：把检测结果直接送进控制律（与 ROS 控制器同一套参数），
    同时向地图查询车道中心线作为**独立真值**，逐帧记录到 CSV。
    车道保持控制文档第 9 节的结果就是这条命令跑出来的。

运行（需要 `carla` Python 包 + 已启动的 CARLA 服务器）::

    python3 tools/run_on_carla.py --frames 30
    python3 tools/run_on_carla.py --closed-loop --duration 30 --spawn-index 10 \
        --target-speed 3 --disturbance 0.06 --disturbance-sec 2.0 --outdir out_final

输出
----
  <outdir>/closed_raw_XXX.ppm    原始前视帧
  <outdir>/closed_mask_XXX.ppm   带检测结果的叠加图
  <outdir>/lateral_trace.csv     逐帧检测值 / 真值 / 转向 / 车速（闭环模式）
"""

import argparse
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
# 算法模块在 ../scripts 下。为了也支持把本脚本放在包外直接跑，
# 这里按存在性依次尝试几个候选目录，取第一个命中的。
for _sub in (os.path.join("..", "scripts"),
             os.path.join("src", "carla_lane_keeping", "scripts"),
             "scripts"):
    _p = os.path.join(HERE, _sub)
    if os.path.isdir(_p):
        sys.path.insert(0, os.path.abspath(_p))
        break

from lane_detection import (  # noqa: E402
    CameraModel,
    LaneDetector,
    LaneFitConfig,
    _label_components,
    find_lane_bases,
    to_grayscale,
)


def camera_fov_deg(width, height, hfov_deg):
    """由水平视场角给出等效焦距（像素），仅用于日志与后续标定参考。

    车道线检测本身在像素空间完成，不依赖相机内参。
    """
    return (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def safe_str(value):
    """把 CARLA 返回的字符串安全转成可打印的 str。

    CARLA 的部分字符串来自 C++ 侧，可能带有非 UTF-8 字节（例如地图名）。
    直接放进 f-string 打印会在 Windows 上抛 UnicodeDecodeError，因此这里统一
    做一次容错解码。
    """
    if value is None:
        return "<None>"
    if isinstance(value, str):
        return value
    if isinstance(value, (bytes, bytearray)):
        for enc in ("utf-8", "gbk", "latin-1"):
            try:
                return value.decode(enc)
            except (UnicodeDecodeError, LookupError):
                continue
        return repr(bytes(value))
    return str(value)


def ascii_view(mask, cols=78, rows=18):
    """把布尔掩码降采样为 ASCII 图。"""
    h, w = mask.shape
    lines = []
    for r in range(rows):
        v0, v1 = int(r * h / rows), int((r + 1) * h / rows)
        line = []
        for c in range(cols):
            u0, u1 = int(c * w / cols), int((c + 1) * w / cols)
            line.append("#" if mask[v0:v1, u0:u1].any() else ".")
        lines.append("".join(line))
    return "\n".join(lines)


def save_ppm(path, rgb_or_gray):
    """把 uint8 数组写成 PPM（P6 彩色 / P5 灰度），无需额外依赖。"""
    arr = np.asarray(rgb_or_gray)
    if arr.ndim == 3:
        # 输入按 RGB 排列
        h, w, _ = arr.shape
        with open(path, "wb") as fh:
            fh.write(f"P6\n{w} {h}\n255\n".encode())
            fh.write(arr.astype(np.uint8).tobytes())
    else:
        h, w = arr.shape
        with open(path, "wb") as fh:
            fh.write(f"P5\n{w} {h}\n255\n".encode())
            fh.write(arr.astype(np.uint8).tobytes())


def draw_overlay(bgr, result, cfg):
    """在图像上画出拟合的车道线（warped 坐标直接画，仅用于快速目视）。"""
    out = bgr.copy()
    h, w = out.shape[0], out.shape[1]
    for fit, color in ((result.left_fit, (255, 0, 0)), (result.right_fit, (0, 0, 255))):
        if fit is None:
            continue
        v = np.arange(h)
        u = np.polyval(fit, v).astype(int)
        valid = (u >= 0) & (u < w)
        for r in np.nonzero(valid)[0]:
            out[r, max(0, u[r] - 1): min(w, u[r] + 2)] = color
    return out


def evaluate_scene(world, bp_lib, detector, cfg, x, y, z, yaw, settle_frames=8):
    """在给定位置与朝向生成车+相机，采几帧看能否稳定检出车道线。

    返回 (得分, 详情字典)。得分越高越适合做演示：
      * 必须能检出两侧车道线
      * 车道宽度要落在真实范围（2.5~5.5 m）
      * 车道线在鸟瞰图中应细长（宽高比小）
    探测完成后立即清理演员，不留残余。
    """
    import carla as _carla

    vehicle_bp = bp_lib.filter("vehicle.tesla.model3")[0]
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", "800")
    cam_bp.set_attribute("image_size_y", "600")
    cam_bp.set_attribute("fov", "90")

    tf = _carla.Transform(_carla.Location(x=x, y=y, z=z), _carla.Rotation(yaw=yaw))
    vehicle = world.try_spawn_actor(vehicle_bp, tf)
    if vehicle is None:
        return None, {"reason": "无法生成车辆"}

    camera = None
    holder = {"bgr": None, "n": 0}

    def on_image(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        holder["bgr"] = arr.reshape((image.height, image.width, 4))[:, :, :3]
        holder["n"] += 1

    try:
        cam_tf = _carla.Transform(
            _carla.Location(x=1.8, y=0.0, z=1.6),
            _carla.Rotation(pitch=-8.0, yaw=0.0, roll=0.0),
        )
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)
        camera.listen(on_image)

        deadline = time.time() + 6.0
        while holder["n"] < settle_frames and time.time() < deadline:
            time.sleep(0.02)

        frame = holder["bgr"]
        if frame is None:
            return None, {"reason": "相机未出图"}

        res = detector.detect(frame)
        if not res.detected:
            return None, {"reason": "未检出两侧车道线"}

        u_l = float(np.polyval(res.left_fit, res.v_eval))
        u_r = float(np.polyval(res.right_fit, res.v_eval))
        lane_w = (u_r - u_l) * cfg.meters_per_pixel_x
        if not (2.5 <= lane_w <= 5.5):
            return None, {"reason": f"车道宽度不合理 {lane_w:.2f} m"}

        # 车道线是否细长（竖直）
        mask = detector.warped_mask(frame)
        labels, ids = _label_components(mask)
        slenderness = None
        for lab in ids:
            ys, xs = np.nonzero(labels == lab)
            if ys.size < 300:
                continue
            wid = xs.max() - xs.min() + 1
            hei = ys.max() - ys.min() + 1
            ratio = wid / max(1, hei)
            slenderness = ratio if slenderness is None else min(slenderness, ratio)

        score = res.confidence
        if slenderness is not None:
            score += max(0.0, 1.0 - slenderness) * 2.0
        return score, {
            "lane_width": lane_w,
            "confidence": res.confidence,
            "slenderness": slenderness,
            "lateral": res.lateral_offset_m,
        }
    finally:
        if camera is not None:
            try:
                camera.stop()
                camera.destroy()
            except Exception:  # noqa: BLE001
                pass
        try:
            vehicle.destroy()
        except Exception:  # noqa: BLE001
            pass


def lane_ground_truth(world_map, carla, tf_now):
    """用 CARLA 地图给出车辆相对所在车道中心的真值。

    返回 (e_y, e_psi)：

    * ``e_y`` 车辆中心到车道中心线的横向距离，**车辆偏右为正**（米）
    * ``e_psi`` 车头方向与车道方向的夹角，**车头偏右为正**（弧度）

    这两个量是检验检测器的唯一硬标准：球门柱式的自洽检查（比如「左右线
    分居两侧」）都只能排除荒谬值，不能证明偏差量本身对不对。
    """
    try:
        loc = tf_now.location
        wp = world_map.get_waypoint(
            loc, project_to_road=True, lane_type=carla.LaneType.Driving
        )
    except Exception:  # noqa: BLE001
        return None, None
    yaw = math.radians(tf_now.rotation.yaw)
    # CARLA 车体局部 +y 指向右侧；右向量 = R(yaw) @ (0, 1)
    rx, ry = -math.sin(yaw), math.cos(yaw)
    dx = loc.x - wp.transform.location.x
    dy = loc.y - wp.transform.location.y
    e_y = dx * rx + dy * ry
    dpsi = math.radians(tf_now.rotation.yaw - wp.transform.rotation.yaw)
    dpsi = (dpsi + math.pi) % (2.0 * math.pi) - math.pi
    return float(e_y), float(dpsi)


def run_closed_loop(carla, world, vehicle, captured, detector, args, outdir):
    """车道保持闭环：把每帧检测结果反馈到方向盘。

    控制律与 ROS 控制器 `lane_keeping_controller.py` 的 `on_timer()` 逐项一致：

        pid_out  = Kp*e_y + Ki*∫e_y dt + Kd*de_y/dt     （限幅 ±steer_limit）
        steering = -(pid_out) - Kpsi*e_psi              （限幅 ±steer_limit）
        throttle = 0.18 * target_speed * (1 - 0.25*|e_y|)，夹在 [0, 1]

    感知失效（未检出 / 置信度不足）时与控制器同样处理：**清零转向输出并复位
    PID 积分**，避免"没检到"被当成"车在正中"，也避免积分残留导致恢复瞬间
    的转向突变。

    与 ROS 版的唯一差别是不经过话题，检测结果直接在同一进程内送进控制律，
    目的是在真实 CARLA 里验证这套控制律与参数能否把车稳定在车道内。
    """
    print()
    print("=" * 68)
    print("车道保持闭环（检测 → 控制 → 车辆）")
    print("=" * 68)
    print(f"  控制律与 ROS 控制器一致：Kp={args.kp} Ki={args.ki} "
          f"Kd={args.kd} Kpsi={args.k_psi}")
    print(f"  目标速度 {args.target_speed} m/s  转向限幅 {args.steer_limit}  "
          f"运行 {args.duration} s")
    if args.disturbance != 0.0:
        print(f"  先开环施加 {args.disturbance_sec:.1f} s、steer={args.disturbance:+.2f} "
              f"的扰动，制造初始横向偏差，再切闭环")

    rows = []  # (t, e_y, e_psi_deg, steer, throttle, speed, conf, detected, phase)
    e_y_f = None  # 一阶低通后的横向偏差（与 ROS 控制器同一条滤波）
    e_psi_f = None
    integr = 0.0
    prev_err = None
    prev_t = None
    last_count = -1
    t_start = None
    stalled_since = None
    next_report = 0.0

    # 同步模式：由客户端控制仿真节奏，每 tick 处理一帧并下发一次控制。
    #
    # 异步模式下相机以数百 fps 出图（实测仿真 186 fps），Python 回调线程处理
    # 800x600 的 BGRA 拷贝会把主线程拖死，控制频率掉到 0.5 Hz，闭环无从谈起。
    # 切同步后仿真步长固定，控制频率与物理时间严格对应。
    use_sync = not args.async_mode
    prev_settings = None
    if use_sync:
        prev_settings = world.get_settings()
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = max(0.02, float(args.fixed_dt))
        world.apply_settings(s)
        print(f"  已切同步模式：fixed_delta_seconds={s.fixed_delta_seconds:.3f} s "
              f"（仿真 {1.0 / s.fixed_delta_seconds:.0f} Hz）")

    try:
        world_map = world.get_map()
    except Exception as exc:  # noqa: BLE001
        print("  [警告] 无法取地图，真值对照关闭：%s" % exc)
        world_map = None

    tick_n = 0
    try:
        while True:
            if use_sync:
                before = captured["count"]
                try:
                    world.tick()
                except Exception as exc:  # noqa: BLE001
                    print("  [警告] world.tick() 失败：%s" % exc)
                    break
                tick_n += 1
                t_wait = time.time() + 5.0
                while captured["count"] == before and time.time() < t_wait:
                    time.sleep(0.002)
                frame = captured["bgr"]
                if frame is None:
                    continue
                # 时间轴取仿真时间，才能和车辆动力学对上
                t_rel = tick_n * float(args.fixed_dt)
                now = t_rel
            else:
                frame = captured["bgr"]
                if frame is None or captured["count"] == last_count:
                    time.sleep(0.005)
                    if t_start is not None and time.time() - t_start > args.duration + 15.0:
                        print("  [警告] 长期收不到新相机帧，提前结束")
                        break
                    continue
                now = time.time()
                if t_start is None:
                    t_start = now
                t_rel = now - t_start
            last_count = captured["count"]

            res = detector.detect(frame)
            e_y = res.lateral_offset_m
            e_psi = res.heading_error_rad

            # 几何合理性校验。
            #
            # 真实图像上的失败模式很具体：右侧白线是虚线、近处段缺失，而左侧
            # 黄线连同行人道的栏杆、砖缝、树影一起被当成候选，于是「左右两条线」
            # 全都落在画面左半边，车道中心被算到左边 200 多像素，横向偏差报出
            # 13~33 m —— 车其实好端端在车道中间。
            #
            # 真压在车道里时，左右车道线必定分居画面两侧，且间距对应 3.5 m 左右。
            # 这两个条件能把这批误检整批筛掉；筛不掉时宁可不发指令（走 blind
            # 分支：停止转向 + 复位积分），也不能用错的偏差去打方向盘。
            geom_ok = False
            lane_w_m = None
            if res.detected and res.left_fit is not None and res.right_fit is not None:
                _u_l = float(np.polyval(res.left_fit, res.v_eval))
                _u_r = float(np.polyval(res.right_fit, res.v_eval))
                _mx = float(
                    detector.region.mpp_x
                    if detector.region is not None
                    else detector.cfg.meters_per_pixel_x
                )
                _w = float(frame.shape[1])  # 鸟瞰图与输入同尺寸
                lane_w_m = (_u_r - _u_l) * _mx
                geom_ok = (
                    2.6 <= lane_w_m <= 4.6
                    and _u_l < 0.45 * _w
                    and _u_r > 0.55 * _w
                )

            valid = (
                res.detected
                and e_y is not None
                and e_psi is not None
                and res.confidence >= args.min_confidence
                and geom_ok
            )
            in_disturb = t_rel < args.disturbance_sec and args.disturbance != 0.0

            if args.fixed_steer is not None:
                # 阶跃标定：固定转向，观察横向偏差向哪边漂
                steering, throttle, brake = args.fixed_steer, 0.30, 0.0
                phase = "open"
            elif in_disturb:
                # 扰动阶段：固定转向、匀速，把车推出车道中心
                steering, throttle, brake = args.disturbance, 0.30, 0.0
                phase = "disturb"
            elif valid:
                # 一阶低通：单帧检测有 ±0.05~0.1 m 抖动，直接进 PID 会被 D 项
                # 放大成方向盘打摆（实测 steer 在 ±0.3 之间来回）。
                alpha = min(1.0, max(1e-3, float(args.filter_alpha)))
                e_y_f = e_y if e_y_f is None else alpha * e_y + (1 - alpha) * e_y_f
                e_psi_f = (
                    e_psi if e_psi_f is None else alpha * e_psi + (1 - alpha) * e_psi_f
                )

                dt = 0.0 if prev_t is None else max(1e-3, now - prev_t)
                if dt > 0.0:
                    integr += e_y_f * dt
                    integr = float(
                        np.clip(integr, -args.integral_limit, args.integral_limit)
                    )
                derivative = 0.0
                if prev_err is not None and dt > 0.0:
                    derivative = (e_y_f - prev_err) / dt

                pid_out = args.kp * e_y_f + args.ki * integr + args.kd * derivative
                pid_out = float(np.clip(pid_out, -args.steer_limit, args.steer_limit))

                steering = -pid_out - args.k_psi * e_psi_f
                steering = float(np.clip(steering, -args.steer_limit, args.steer_limit))

                # 纵向：速度闭环。
                #
                # 原先是 throttle = 0.18 * target_speed（开环），实测车会一路
                # 加速到 14 m/s —— 那时 4.5 m 的预瞄距离已经不够，任何横向
                # 增益都救不回来。改成对速度误差做比例控制，超速时轻踩刹车。
                _v = vehicle.get_velocity()
                speed_ctrl = math.sqrt(_v.x ** 2 + _v.y ** 2 + _v.z ** 2)
                v_err = args.target_speed - speed_ctrl
                throttle = float(np.clip(args.speed_gain * v_err, 0.0, 1.0))
                brake = float(np.clip(-0.25 * v_err, 0.0, 0.6)) if v_err < -0.3 else 0.0
                phase = "closed"
                prev_err, prev_t = e_y_f, now
            else:
                # 感知失效：不转向、轻刹，并复位积分（与 ROS 控制器一致）
                integr, prev_err, prev_t = 0.0, None, None
                e_y_f = e_psi_f = None
                steering, throttle, brake = 0.0, 0.0, 0.3
                phase = "blind"

            vehicle.apply_control(
                carla.VehicleControl(
                    throttle=float(throttle),
                    steer=float(steering),
                    brake=float(brake),
                    hand_brake=False,
                    reverse=False,
                    gear=1,
                )
            )

            vel = vehicle.get_velocity()
            speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
            tf_now = vehicle.get_transform()
            gt_e_y, gt_e_psi = (
                (None, None)
                if world_map is None
                else lane_ground_truth(world_map, carla, tf_now)
            )

            # 存下前若干帧的原始图与鸟瞰掩码，用于事后核对「检测到底锁到了什么」
            if len(rows) < 6:
                try:
                    save_ppm(
                        os.path.join(outdir, "closed_raw_%02d.ppm" % len(rows)),
                        frame[:, :, ::-1],
                    )
                    _m = detector.warped_mask(frame)
                    save_ppm(
                        os.path.join(outdir, "closed_mask_%02d.ppm" % len(rows)),
                        (_m.astype(np.uint8) * 255),
                    )
                except Exception as exc:  # noqa: BLE001
                    print("  [警告] 保存图像失败：%s" % exc)
            rows.append(
                (
                    t_rel,
                    e_y,
                    math.degrees(e_psi) if e_psi is not None else None,
                    steering,
                    throttle,
                    speed,
                    res.confidence,
                    1 if res.detected else 0,
                    phase,
                    tf_now.location.x,
                    tf_now.location.y,
                    tf_now.rotation.yaw,
                    gt_e_y,
                    None if gt_e_psi is None else math.degrees(gt_e_psi),
                )
            )

            # 每秒打一行，方便盯着看
            if t_rel >= next_report:
                next_report = t_rel + 1.0
                e_txt = "  --  " if e_y is None else f"{e_y:+.3f}"
                lw_txt = " -- " if lane_w_m is None else f"{lane_w_m:5.2f}"
                gt_txt = "  --  " if gt_e_y is None else f"{gt_e_y:+.3f}"
                print(
                    f"  t={t_rel:5.1f}s  [{phase:7s}] e_y={e_txt} m  "
                    f"(真值 {gt_txt})  steer={steering:+.3f}  v={speed:.2f} m/s  "
                    f"conf={res.confidence:.2f}  lane_w={lw_txt} m  "
                    f"geom={'ok' if geom_ok else 'NG'}",
                    flush=True,
                )

            # 卡住保护：持续低速且不在扰动阶段，多半是撞上了
            if speed < 0.3 and not in_disturb:
                if stalled_since is None:
                    stalled_since = now
                elif now - stalled_since > 3.0:
                    print("  [警告] 车速持续 < 0.3 m/s，可能撞到障碍物，提前结束")
                    break
            else:
                stalled_since = None

            if t_rel >= args.duration:
                break
    finally:
        try:
            vehicle.apply_control(
                carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=False)
            )
        except Exception:  # noqa: BLE001
            pass
        # 务必恢复异步模式，否则仿真器会一直等客户端 tick
        if prev_settings is not None:
            try:
                world.apply_settings(prev_settings)
            except Exception:  # noqa: BLE001
                pass

    # ---------------- 汇总 ----------------
    closed = [r for r in rows if r[8] == "closed" and r[1] is not None]
    blind = [r for r in rows if r[8] == "blind"]
    print()
    print("=" * 68)
    print(f"闭环结束：共 {len(rows)} 帧，其中闭环 {len(closed)} 帧、"
          f"感知失效 {len(blind)} 帧")
    if rows:
        span = rows[-1][0] - rows[0][0]
        print(f"  平均控制频率 {len(rows) / max(1e-6, span):.1f} Hz")

    if closed:
        lat = np.array([r[1] for r in closed], dtype=np.float64)
        steer_arr = np.array([r[3] for r in closed], dtype=np.float64)
        peak = float(np.max(np.abs(lat)))
        tail_n = max(10, len(lat) // 5)
        tail = lat[-tail_n:]
        steady_abs = float(np.mean(np.abs(tail)))
        steady_std = float(np.std(tail))

        # 收敛时间：从末尾往前找最后一个 |e_y| > 0.25 m 的时刻
        threshold = 0.25
        over = np.nonzero(np.abs(lat) > threshold)[0]
        if over.size == 0:
            t_settle = 0.0
        elif over[-1] + 1 >= len(lat):
            t_settle = float("nan")  # 结束前仍超阈值，未收敛
        else:
            t_settle = closed[over[-1] + 1][0] - closed[0][0]

        print(f"  横向偏差 |e_y|：峰值 {peak:.3f} m，稳态均值 {steady_abs:.3f} m "
              f"（标准差 {steady_std:.3f}，取最后 {tail_n} 帧）")
        print(f"  收敛时间（回到 ±{threshold:.2f} m 内并保持）："
              + ("未收敛" if not np.isfinite(t_settle) else f"{t_settle:.2f} s"))
        print(f"  转向量：范围 [{steer_arr.min():+.3f}, {steer_arr.max():+.3f}]")
        v_arr = np.array([r[5] for r in closed], dtype=np.float64)
        print(f"  车速：均值 {v_arr.mean():.2f} m/s")
        pairs = [
            (r[1], r[12]) for r in rows
            if r[1] is not None and len(r) > 12 and r[12] is not None
        ]
        if pairs:
            err = np.array([a - b for a, b in pairs], dtype=np.float64)
            print(
                "  与地图真值对比（%d 帧）：误差均值 %+.3f m，"
                "平均绝对误差 %.3f m，最大 %.3f m"
                % (len(pairs), err.mean(), np.abs(err).mean(), np.abs(err).max())
            )
        within = float(np.mean(np.abs(lat) <= 0.25))
        print(
            "  落在 ±0.25 m 内的帧占比：%.1f%%（含扰动阶段的恢复过程）" % (100 * within)
        )
        if steady_abs <= 0.10 and within >= 0.90:
            print("  判定：车辆被稳定保持在车道内 ✓")
        else:
            print("  判定：效果不佳，需要检查符号方向或整定参数")

    csv_path = os.path.join(outdir, "lateral_trace.csv")
    with open(csv_path, "w", encoding="utf-8", newline="") as fh:
        fh.write(
            "t_s,e_y_m,e_psi_deg,steer,throttle,speed_mps,confidence,detected,"
            "phase,world_x,world_y,yaw_deg,gt_e_y_m,gt_e_psi_deg\n"
        )
        for r in rows:
            fh.write(
                "%.3f,%s,%s,%.4f,%.3f,%.3f,%.3f,%d,%s,%.3f,%.3f,%.2f,%s,%s\n"
                % (
                    r[0],
                    "" if r[1] is None else "%.4f" % r[1],
                    "" if r[2] is None else "%.3f" % r[2],
                    r[3],
                    r[4],
                    r[5],
                    r[6],
                    r[7],
                    r[8],
                    r[9],
                    r[10],
                    r[11],
                    "" if len(r) < 13 or r[12] is None else "%.4f" % r[12],
                    "" if len(r) < 14 or r[13] is None else "%.3f" % r[13],
                )
            )
    print(f"  逐帧数据已写入 {csv_path}")
    return 0


def main():
    ap = argparse.ArgumentParser(description="在 CARLA 中运行车道线检测")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--town", default="", help="加载的地图，留空则用当前地图")
    ap.add_argument("--frames", type=int, default=90, help="采集多少个相机帧")
    ap.add_argument("--save-every", type=int, default=25, help="每多少帧保存一次图片")
    ap.add_argument("--outdir", default=os.path.join(HERE, "out_carla"))

    # ---- 闭环模式（把检测结果反馈到方向盘）----
    ap.add_argument(
        "--closed-loop",
        action="store_true",
        help="闭环模式：每帧检测结果直接下发方向盘，跑真正的车道保持",
    )
    ap.add_argument("--duration", type=float, default=25.0, help="闭环运行秒数")
    ap.add_argument("--target-speed", type=float, default=4.0, help="目标车速 m/s")
    ap.add_argument("--kp", type=float, default=0.28)
    ap.add_argument("--ki", type=float, default=0.02)
    ap.add_argument("--kd", type=float, default=0.06)
    ap.add_argument("--k-psi", type=float, default=0.9)
    ap.add_argument(
        "--filter-alpha",
        type=float,
        default=0.35,
        help="横向/航向的一阶低通系数，1 表示不滤波",
    )
    ap.add_argument(
        "--speed-gain",
        type=float,
        default=0.35,
        help="速度环比例增益：throttle = clip(speed_gain*(v_target-v))",
    )
    ap.add_argument("--steer-limit", type=float, default=0.7)
    ap.add_argument("--integral-limit", type=float, default=2.0)
    ap.add_argument("--min-confidence", type=float, default=0.5)
    ap.add_argument(
        "--cam-height", type=float, default=1.70, help="相机离地高度（米）"
    )
    ap.add_argument(
        "--cam-pitch", type=float, default=8.0, help="相机俯角（度，向下为正）"
    )
    ap.add_argument(
        "--no-calib",
        action="store_true",
        help="不用相机标定，退回按图像比例圈梯形的旧模式（仅用于对比）",
    )
    ap.add_argument(
        "--disturbance",
        type=float,
        default=0.0,
        help="开环施加的固定转向量（制造初始偏差），0 表示不扰动",
    )
    ap.add_argument(
        "--disturbance-sec", type=float, default=1.5, help="扰动持续秒数"
    )
    ap.add_argument(
        "--fixed-steer",
        type=float,
        default=None,
        help="固定转向量，用于阶跃标定（负/正各跑一次即可判定符号方向）",
    )
    ap.add_argument(
        "--spawn",
        default="",
        help="直接指定生成点 'x,y,yaw'，跳过位置搜索（调试用）",
    )
    ap.add_argument(
        "--spawn-index",
        type=int,
        default=-1,
        help="使用地图第 N 个标准生成点（车必然在车道中心、朝向正确）",
    )
    ap.add_argument(
        "--fixed-dt",
        type=float,
        default=0.05,
        help="闭环同步模式的固定步长（秒），0.05 即 20 Hz 仿真",
    )
    ap.add_argument(
        "--async-mode",
        action="store_true",
        help="闭环时不切同步模式（不推荐：相机出图过快会拖死 Python）",
    )
    args = ap.parse_args()

    try:
        import carla
    except ImportError:
        print("[错误] 未安装 carla 包。请执行：")
        print('  py -3.12 -m pip install "D:\\CARLA\\CARLA_0.9.16\\PythonAPI\\carla\\dist\\carla-0.9.16-cp312-cp312-win_amd64.whl"')
        return 1

    os.makedirs(args.outdir, exist_ok=True)

    print("=" * 68)
    print("在 CARLA 中运行车道线检测")
    print("=" * 68)
    print(f"连接 {args.host}:{args.port} ...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    try:
        version = client.get_server_version()
    except Exception as exc:  # noqa: BLE001
        print(f"[错误] 无法连接 CARLA：{type(exc).__name__}: {exc}")
        print("       请确认 CarlaUE4.exe 已启动，并等待场景完全加载。")
        return 1
    print(f"  服务端版本 {version} ✓")

    if args.town:
        world = client.load_world(args.town)
    else:
        world = client.get_world()
    try:
        map_name = safe_str(world.get_map().name)
    except Exception as exc:  # noqa: BLE001
        map_name = f"<读取失败: {type(exc).__name__}>"
    print(f"  当前地图：{map_name}")

    bp_lib = world.get_blueprint_library()
    vehicle_bps = bp_lib.filter("vehicle.tesla.model3")
    if not vehicle_bps:
        vehicle_bps = bp_lib.filter("vehicle.*")
    vehicle_bp = vehicle_bps[0]
    if vehicle_bp.has_attribute("role_name"):
        vehicle_bp.set_attribute("role_name", "lane_test")

    cfg = LaneFitConfig(
        min_lane_pixels=150,
        window_margin_px=100,
    )
    camera = None
    if not args.no_calib:
        camera = CameraModel(
            width=800,
            height=600,
            hfov_deg=90.0,
            mount_height_m=args.cam_height,
            pitch_deg=args.cam_pitch,
        )
    detector = LaneDetector(cfg, camera=camera)
    if camera is not None:
        print("  检测器：相机标定模式（高 %.2f m，俯角 %.1f°，FOV 90°）"
              % (camera.mount_height_m, camera.pitch_deg))
    else:
        print("  检测器：比例模式（未标定，横向偏差无物理意义）")

    # 搜索一个「能正确检出车道线」的拍摄位置与朝向。
    #
    # 注意：本机 CARLA 返回的地图名字节不是合法 UTF-8，导致 world.get_map()
    # 与 client.get_trafficmanager() 都直接抛 UnicodeDecodeError，
    # 因此无法用 map.get_spawn_points() 拿到标准生成点，只能在候选坐标上
    # 逐个尝试，并用「检测结果是否合理」来反推车辆是否真的压在车道上。
    candidates = [
        (57.4, 61.1, 0.6), (26.1, 75.2, 0.6), (35.3, 53.6, 0.6),
        (49.6, 38.5, 0.6), (-28.1, 66.0, 0.6), (44.0, 37.8, 0.6),
        (110.0, 20.0, 0.6), (-60.0, 20.0, 0.6), (-95.0, 20.0, 0.6),
        (0.0, 0.0, 0.6), (10.0, 10.0, 0.6), (-117.0, 5.0, 0.6),
        (-46.0, -73.0, 0.6), (-34.0, -51.0, 0.6), (115.0, 35.0, 0.6),
        (-62.0, 123.8, 0.6), (-58.7, 145.8, 0.6), (-31.6, 33.6, 0.6),
    ]
    yaws = [0.0, 90.0, 180.0, 270.0]

    if args.spawn_index >= 0:
        # 标准生成点：位置在车道中心、朝向与道路一致，比手填坐标可靠得多。
        # （之前手填 (110,20,yaw=0) 把车横在建筑前的铺装广场上，检测只好
        #   把路灯杆的竖直边缘当成车道线，横向偏差自然乱跳。）
        try:
            sps = world.get_map().get_spawn_points()
        except Exception as exc:  # noqa: BLE001
            print("  [错误] 读取生成点失败：%s" % exc)
            sps = []
        if not sps:
            print("  [错误] 地图没有返回生成点")
            return 1
        sp = sps[args.spawn_index % len(sps)]
        spawn_loc = (sp.location.x, sp.location.y, 0.6)
        spawn_yaw = sp.rotation.yaw
        print(f"  使用标准生成点 #{args.spawn_index}（共 {len(sps)} 个）："
              f"({spawn_loc[0]:.1f}, {spawn_loc[1]:.1f}) yaw={spawn_yaw:.1f}")
    elif args.spawn:
        _p = [float(v) for v in args.spawn.split(",")]
        spawn_loc = (_p[0], _p[1], 0.6)
        spawn_yaw = _p[2] if len(_p) > 2 else 0.0
        print(f"  使用指定生成点 {spawn_loc} yaw={spawn_yaw}（跳过位置搜索）")
    else:
        print(f"  搜索拍摄位置：{len(candidates)} 个坐标 x {len(yaws)} 个朝向 ...")
        best = None
        tried = 0
        for (x, y, z) in candidates:
            for yaw in yaws:
                tried += 1
                score, info = evaluate_scene(world, bp_lib, detector, cfg, x, y, z, yaw)
                if score is None:
                    continue
                if best is None or score > best[0]:
                    best = (score, x, y, z, yaw, info)
                    print(f"    ✓ 命中 (x={x}, y={y}, yaw={yaw}) "
                          f"车道宽 {info['lane_width']:.2f} m  "
                          f"置信度 {info['confidence']:.2f}  "
                          f"细长度 {info['slenderness']}  得分 {score:.2f}",
                          flush=True)
        if best is None:
            print(f"  ⚠ 试了 {tried} 组位置/朝向，都没能稳定检出车道线。")
            print("    这不代表算法无效——合成图像上的精度已验证到 0.1 m 以内。")
            print("    更可能的原因：该地图当前视野内没有清晰的车道标线，")
            print("    或相机俯仰角与实际道路不匹配。将使用第一个可用位置继续采集。")
            spawn_loc, spawn_yaw = candidates[0], 0.0
        else:
            _, x, y, z, spawn_yaw, info = best
            spawn_loc = (x, y, z)
            print(f"  选定位置 (x={x}, y={y}, yaw={spawn_yaw})："
                  f"车道宽 {info['lane_width']:.2f} m，细长度 {info['slenderness']}")

    tf = carla.Transform(
        carla.Location(x=spawn_loc[0], y=spawn_loc[1], z=spawn_loc[2]),
        carla.Rotation(yaw=spawn_yaw),
    )
    vehicle = world.try_spawn_actor(vehicle_bp, tf)
    if vehicle is None:
        print("[错误] 选定的位置无法生成车辆（可能被之前清理的演员占用）")
        return 1
    print(f"  已生成车辆 id={vehicle.id} 位置={spawn_loc} 朝向={spawn_yaw}")

    # 相机：车前上方，略微俯视，视野 90 度
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", "800")
    cam_bp.set_attribute("image_size_y", "600")
    cam_bp.set_attribute("fov", "90")
    cam_transform = carla.Transform(
        carla.Location(x=1.8, y=0.0, z=1.6),
        carla.Rotation(pitch=-8.0, yaw=0.0, roll=0.0),
    )
    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
    print("  已挂载前视相机 800x600 FOV90")

    captured = {"bgr": None, "count": 0}

    def on_image(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        captured["bgr"] = np.ascontiguousarray(arr[:, :, :3])
        captured["count"] += 1

    camera.listen(on_image)

    if args.closed_loop:
        # 等第一帧到位，否则闭环第一帧读到 None 会白白空转
        t_wait = time.time() + 10.0
        while captured["bgr"] is None and time.time() < t_wait:
            time.sleep(0.05)
        rc = run_closed_loop(carla, world, vehicle, captured, detector, args, args.outdir)
        try:
            camera.stop()
            camera.destroy()
        except Exception:  # noqa: BLE001
            pass
        try:
            vehicle.destroy()
        except Exception:  # noqa: BLE001
            pass
        return rc

    # 驱动车辆前进。
    #
    # 不用 client.get_trafficmanager()：本机 CARLA 的 C++->Python 字符串转换
    # 存在编码缺陷（地图名、traffic manager 都会抛 UnicodeDecodeError），
    # 因此改用最基础的 VehicleControl 直接给油门与转向。
    control = carla.VehicleControl()
    control.throttle = 0.35
    control.steer = 0.0
    control.brake = 0.0
    vehicle.apply_control(control)
    print("  已施加前进油门（绕过 traffic manager），开始采集 ...")

    stats = []
    saved = 0
    first_done = False
    ascii_shown = False
    last_count = -1
    last_progress = time.time()
    deadline = time.time() + 90.0
    try:
        while captured["count"] < args.frames and time.time() < deadline:
            # 不用 world.wait_for_tick()：异步模式下它可能长时间不返回，
            # 一旦仿真器异常退出，脚本就会永久卡住且没有任何输出。
            # 改为轮询 + 超时保护。
            time.sleep(0.02)
            frame = captured["bgr"]
            if frame is None or captured["count"] == last_count:
                if time.time() - last_progress > 20.0:
                    print("  [警告] 20 秒没有收到新相机帧，停止采集。")
                    print("         仿真器可能已退出，或相机未正常发布数据。")
                    break
                continue
            last_count = captured["count"]
            last_progress = time.time()

            if captured["count"] % 10 == 0:
                print(f"  ... 已处理 {captured['count']} 帧", flush=True)

            res = detector.detect(frame)

            # 首帧无条件保存，并打印诊断信息：
            # 若检测失败，这些数据能直接指出是「没有车道标线」还是「阈值不当」。
            if not first_done:
                first_done = True
                save_ppm(
                    os.path.join(args.outdir, "frame_first.ppm"),
                    frame[:, :, ::-1],
                )
                gray = to_grayscale(frame)
                binary = detector.binary_lane_mask(frame)
                mask = detector.warped_mask(frame)
                save_ppm(
                    os.path.join(args.outdir, "frame_first_binary.ppm"),
                    (binary.astype(np.uint8) * 255),
                )
                save_ppm(
                    os.path.join(args.outdir, "frame_first_mask.ppm"),
                    (mask.astype(np.uint8) * 255),
                )
                print()
                print("  --- 首帧诊断 ---")
                print(f"  灰度范围 [{gray.min():.0f}, {gray.max():.0f}]  均值 {gray.mean():.1f}  "
                      f"标准差 {gray.std():.1f}")
                print(f"  二值掩码像素数 = {int(binary.sum())}  "
                      f"(占全图 {100.0 * binary.mean():.2f}%)")
                print(f"  鸟瞰掩码像素数 = {int(mask.sum())}  "
                      f"(占全图 {100.0 * mask.mean():.2f}%)")
                rows = np.nonzero(mask.any(axis=1))[0]
                print(f"  鸟瞰掩码有效行范围 = "
                      f"{int(rows.min()) if rows.size else 'none'} ~ "
                      f"{int(rows.max()) if rows.size else 'none'}")
                hist = mask[int(frame.shape[0] * 0.7):, :].sum(axis=0)
                lb, rb = find_lane_bases(hist)
                print(f"  底部直方图定出的基点 = 左 {lb} 右 {rb}")
                print(f"  检出结果: detected={res.detected} "
                      f"左像素={res.left_pixels} 右像素={res.right_pixels} "
                      f"置信度={res.confidence:.2f}")

                # 连通域统计：能直接看出「是否压在某条车道上」以及
                # 「车道线在相机视野里是竖直的（朝向正确）」还是倾斜的（朝向偏了）
                labels, ids = _label_components(mask)
                print(f"  鸟瞰图连通域共 {ids.size} 个（只列出较大的）:")
                shown = 0
                for lab in ids:
                    ys, xs = np.nonzero(labels == lab)
                    if ys.size < 200:
                        continue
                    width = xs.max() - xs.min() + 1
                    hh = ys.max() - ys.min() + 1
                    # 倾斜度：整条连通域的横向跨度 / 纵向跨度
                    tilt = width / max(1, hh)
                    print(f"    #{lab}: 像素={ys.size:>6} 宽={width:>3} 高={hh:>3} "
                          f"u均值={xs.mean():>6.1f} 倾斜度={tilt:.2f}")
                    shown += 1
                    if shown >= 6:
                        break
                print("  提示：真正竖直的车道线 倾斜度 应远小于 1（细长）；"
                      "若某个连通域又宽又矮，说明看到的是路面而非车道线，")
                print("        或车辆朝向与道路不平行（斜着看车道线）。")

                print(f"  已保存首帧图像与掩码到 {args.outdir}")
                print()
                saved += 1
                continue

            # 车道宽度合理性检查：真实车道约 3.5 m，明显偏离说明没落在车道上
            lane_w = None
            if res.detected:
                u_l = float(np.polyval(res.left_fit, res.v_eval))
                u_r = float(np.polyval(res.right_fit, res.v_eval))
                lane_w = (u_r - u_l) * cfg.meters_per_pixel_x
                plausible = 1.5 <= lane_w <= 8.0
            else:
                plausible = False

            if res.detected and plausible:
                stats.append(
                    (
                        res.lateral_offset_m,
                        math.degrees(res.heading_error_rad),
                        res.curvature_m,
                        res.confidence,
                        lane_w,
                    )
                )
            if captured["count"] % args.save_every == 0:
                overlay = draw_overlay(frame, res, cfg)
                save_ppm(
                    os.path.join(args.outdir, f"frame_{captured['count']:04d}.ppm"),
                    overlay[:, :, ::-1],  # BGR -> RGB
                )
                mask = detector.warped_mask(frame)
                save_ppm(
                    os.path.join(args.outdir, f"frame_{captured['count']:04d}_mask.ppm"),
                    (mask.astype(np.uint8) * 255),
                )
                saved += 1
                if not ascii_shown and res.detected:
                    print()
                    print(f"  --- 第 {captured['count']} 帧鸟瞰掩码（'#' = 车道像素）---")
                    print(ascii_view(mask))
                    print(f"  车道宽度实测 = {lane_w:.2f} m  "
                          f"(合理区间 1.5~8.0 m，用于判断是否真的压在车道上)")
                    print()
                    ascii_shown = True
    except KeyboardInterrupt:
        print("\n  用户中断采集")
    finally:
        try:
            camera.stop()
            camera.destroy()
        except Exception:  # noqa: BLE001
            pass
        try:
            vehicle.apply_control(carla.VehicleControl(brake=1.0))
            vehicle.destroy()
        except Exception:  # noqa: BLE001
            pass
        print("  已清理相机与车辆")

    total = captured["count"]
    print()
    print("=" * 68)
    print(f"采集 {total} 帧，车道线检测通过合理性检查 {len(stats)} 帧 "
          f"({100.0 * len(stats) / max(1, total):.1f}%)")
    if stats:
        lat = np.array([s[0] for s in stats])
        head = np.array([s[1] for s in stats])
        curv = np.array([s[2] for s in stats])
        width = np.array([s[4] for s in stats])
        print(f"  横向偏差  : 均值 {lat.mean():+.3f} m  范围 [{lat.min():+.3f}, {lat.max():+.3f}]")
        print(f"  航向偏差  : 均值 {head.mean():+.2f} deg  范围 [{head.min():+.2f}, {head.max():+.2f}]")
        print(f"  车道宽度  : 均值 {width.mean():.2f} m  (真实车道约 3.5 m)")
        finite = curv[np.isfinite(curv)]
        if finite.size:
            print(f"  曲率半径  : 中位数 {np.median(finite):.1f} m")
    else:
        print("  ⚠ 没有一帧通过合理性检查")
        print("    可能原因：该路段没有车道标线、相机俯仰角不合适、或阈值需要调整")
    print(f"  已保存 {saved} 组图片到 {os.path.abspath(args.outdir)}")
    print("=" * 68)
    return 0


if __name__ == "__main__":
    sys.exit(main())
