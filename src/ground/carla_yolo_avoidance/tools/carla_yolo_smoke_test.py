# -*- coding: utf-8 -*-
"""离机自检工具：不启动 ROS 也能验证"CARLA + YOLOv8 + 测距 + 避障决策"整条链路。

它的价值和用途：
    1. 上 ROS 之前先确认环境没问题（CARLA 连得上、YOLO 装好了、权重能下载）；
    2. 复用了与 ROS 节点完全相同的模块（config / detector / distance_estimator /
       decision_logic），所以这里测出来的结果就是节点里跑的逻辑；
    3. `--drive` 模式会真的开车：踩油门前进 -> YOLO 测距 -> 决策刹车，
       并把距离变化过程打印出来，用来验证避障阈值选得是否合理；
    4. 会把原始帧和画框后的帧保存成 PNG，可直接作为作业截图素材。

用法示例：
    python3 tools/carla_yolo_smoke_test.py                 # 只检测，不开车
    python3 tools/carla_yolo_smoke_test.py --drive         # 闭环：前进 + 刹车避障
    python3 tools/carla_yolo_smoke_test.py --drive --frames 120 --spawn-ahead 45
    python3 tools/carla_yolo_smoke_test.py --host 10.0.0.5 $12000

    CARLA 的 IP 与端口默认读取 config/params.yaml（与 ROS 节点同一份配置），
    只有需要临时连别的仿真服务端时才用 --host / --port 覆盖。
"""

import argparse
import math
import os
import queue
import sys
import time

import numpy as np

# 让脚本在未 colcon build 的情况下也能 import 到包内模块
_PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..')
)
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from carla_yolo_avoidance import carla_env, config, decision_logic, detector  # noqa: E402
from carla_yolo_avoidance import distance_estimator  # noqa: E402


def parse_args(default_host='', default_port=0):
    parser = argparse.ArgumentParser(description='CARLA + YOLOv8 离线自检')
    parser.add_argument('--model', default=config.YOLO_WEIGHTS, help='YOLOv8 权重文件')
    parser.add_argument('--host', default=default_host,
                        help='CARLA 服务端 IP / 主机名（默认取 config/params.yaml）')
    parser.add_argument('--port', type=int, default=default_port,
                        help='CARLA 服务端端口（默认取 config/params.yaml）')
    parser.add_argument('--frames', type=int, default=40, help='处理多少帧后退出')
    parser.add_argument('--spawn-ahead', type=float, default=config.LEAD_VEHICLE_AHEAD_M,
                        help='障碍车放在自车前方多少米')
    parser.add_argument('--drive', action='store_true',
                        help='闭环模式：按决策结果真的踩油门/刹车')
    parser.add_argument('--outdir', default='artifacts', help='截图输出目录')
    parser.add_argument('--no-save', action='store_true', help='不保存截图')
    parser.add_argument('--save-every', type=int, default=0,
                        help='每隔多少帧保存一次截图；0 = 自动（最多约 8 组）')
    parser.add_argument('--warmup', type=int, default=10, help='丢弃前多少帧（等画面稳定）')
    return parser.parse_args()


def bgra_to_bgr(carla_image):
    """CARLA 图像 -> numpy BGR（必须拷贝，CARLA 会复用缓冲区）。"""
    raw = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
    bgra = raw.reshape((carla_image.height, carla_image.width, 4))
    return np.ascontiguousarray(bgra[:, :, :3])


def pick_vehicle_blueprint(world, blueprint_id):
    """按车型名取蓝图，找不到就用第一个四轮车。"""
    bp_lib = world.get_blueprint_library()
    found = list(bp_lib.filter(blueprint_id))
    if found:
        return found[0]
    for bp in bp_lib.filter('vehicle.*'):
        if bp.has_attribute('number_of_wheels') and \
                bp.get_attribute('number_of_wheels').as_int() == 4:
            return bp
    return list(bp_lib.filter('vehicle.*'))[0]


def spawn_actors(world, carla, spawn_ahead_m):
    """生成自车 + 前方静止障碍车 + 前视相机，返回 (ego, lead, camera)。"""
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError('当前地图没有可用的出生点')

    ego_bp = pick_vehicle_blueprint(world, config.EGO_VEHICLE_BLUEPRINT)
    if ego_bp.has_attribute('role_name'):
        ego_bp.set_attribute('role_name', 'smoke_ego')

    base = spawn_points[0]
    ego = None
    for transform in spawn_points:
        ego = world.try_spawn_actor(ego_bp, transform)
        if ego is not None:
            base = transform
            break
    if ego is None:
        raise RuntimeError('所有出生点都被占用，无法生成自车')

    yaw = math.radians(base.rotation.yaw)
    lead_bp = pick_vehicle_blueprint(world, config.LEAD_VEHICLE_BLUEPRINT)
    if lead_bp.has_attribute('role_name'):
        lead_bp.set_attribute('role_name', 'smoke_lead')
    lead_location = carla.Location(
        x=base.location.x + spawn_ahead_m * math.cos(yaw),
        y=base.location.y + spawn_ahead_m * math.sin(yaw),
        z=base.location.z,
    )
    lead = world.try_spawn_actor(lead_bp, carla.Transform(lead_location, base.rotation))

    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(config.CAMERA_IMAGE_WIDTH))
    camera_bp.set_attribute('image_size_y', str(config.CAMERA_IMAGE_HEIGHT))
    camera_bp.set_attribute('fov', str(config.CAMERA_FOV_DEG))
    camera = world.spawn_actor(
        camera_bp,
        carla.Transform(carla.Location(*config.CAMERA_POSITION),
                        carla.Rotation(*config.CAMERA_ROTATION)),
        attach_to=ego,
    )
    return ego, lead, camera


def speed_of(vehicle):
    """车辆速度（m/s）。"""
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)


def distance_to(ego, other):
    """两车中心距离（m）；other 为 None 时返回 None。"""
    if other is None:
        return None
    a, b = ego.get_location(), other.get_location()
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def draw_detections(frame, detections, extra_text=''):
    """在图上画检测框，返回新图（不修改原图）。"""
    import cv2

    canvas = frame.copy()
    for det in detections:
        x1, y1, x2, y2 = (int(v) for v in det['bbox'])
        distance = det['distance_m']
        if distance is None:
            color = (160, 160, 160)
            text = '{} {:.2f}'.format(det['class'], det['confidence'])
        else:
            if distance < config.FULL_BRAKE_DISTANCE_M:
                color = (0, 0, 255)
            elif distance < config.SAFE_DISTANCE_M:
                color = (0, 215, 255)
            else:
                color = (0, 255, 0)
            text = '{} {:.1f}m {:.2f}'.format(det['class'], distance, det['confidence'])
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        cv2.putText(canvas, text, (x1, max(15, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
    if extra_text:
        cv2.putText(canvas, extra_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2, cv2.LINE_AA)
    return canvas


def main():
    # CARLA 地址取自与 ROS 节点同一份 config/params.yaml，保证两边一致
    try:
        default_host, default_port = config.load_carla_endpoint()
    except RuntimeError as exc:
        print('[警告] 读取 config/params.yaml 失败：{}'.format(exc))
        print('       请用 --host / --port 手动指定 CARLA 地址。')
        default_host, default_port = '', 0

    args = parse_args(default_host, default_port)
    # 地址缺失时给出与 ROS 节点一致的友好提示
    args.host, args.port = carla_env.check_endpoint_or_exit(args.host, args.port)

    print('=' * 78)
    print('CARLA + YOLOv8 离线自检（不依赖 ROS）')
    print('=' * 78)
    print('环境：{}'.format(carla_env.describe_environment()))

    # ---- 1) 环境检测（失败会打印友好提示并退出）----
    carla = carla_env.import_carla_or_exit()
    YOLO = carla_env.import_ultralytics_or_exit()

    model = YOLO(args.model)
    print('[OK] YOLOv8 已加载：{}（类别数 {}）'.format(args.model, len(model.names)))

    focal_px = distance_estimator.default_focal_px()
    print('[OK] 单目测距焦距 = {:.1f} px（图像 {}x{}，视场角 {}°）'.format(
        focal_px, config.CAMERA_IMAGE_WIDTH, config.CAMERA_IMAGE_HEIGHT, config.CAMERA_FOV_DEG))

    # ---- 2) 连接 CARLA（地址来自 config/params.yaml，可用 --host/--port 覆盖）----
    client, world = carla_env.connect_to_carla(carla, args.host, args.port)
    print('[OK] 已连接 CARLA {}:{}，地图 = {}'.format(
        args.host, args.port, world.get_map().name))

    ego = lead = camera = None
    frame_queue = queue.Queue()
    history = []
    try:
        ego, lead, camera = spawn_actors(world, carla, args.spawn_ahead)
        print('[OK] 自车 = {}，障碍车 = {}（前方 {:.1f} m）'.format(
            ego.type_id,
            '生成失败' if lead is None else lead.type_id,
            args.spawn_ahead,
        ))

        camera.listen(frame_queue.put)

        os.makedirs(args.outdir, exist_ok=True)
        saved_files = []
        # 截图间隔：默认自动，保证最多只写出约 8 组图，避免刷出几百个文件
        save_stride = args.save_every if args.save_every > 0 else max(1, args.frames // 8)
        if not args.no_save:
            print('截图策略：每 {} 帧保存一帧到 {}'.format(save_stride, args.outdir))
        last_detections = []

        print('-' * 78)
        print('开始处理 {} 帧（丢弃前 {} 帧等待画面稳定）...'.format(args.frames, args.warmup))
        print('-' * 78)

        for index in range(args.warmup + args.frames):
            # 只取最新一帧：丢掉队列里排队的旧帧。
            # 否则当推理比相机慢时，处理的是几秒前的旧画面（距离会明显偏大）。
            # ROS 节点里用 depth=1 的 QoS 达到同样效果。
            carla_image = None
            while True:
                try:
                    carla_image = frame_queue.get_nowait()
                except queue.Empty:
                    break
            if carla_image is None:
                try:
                    carla_image = frame_queue.get(timeout=10.0)
                except queue.Empty:
                    print('[错误] 10 秒内没有收到相机帧，请检查 CARLA 是否正常渲染')
                    return 1

            frame = bgra_to_bgr(carla_image)

            if index < args.warmup:
                continue

            start = time.monotonic()
            # 与 yolo_detect_node.py 保持一致的推理参数
            predict_kwargs = {
                'source': frame,
                'conf': config.YOLO_CONF_THRESHOLD,
                'iou': config.YOLO_IOU_THRESHOLD,
                'imgsz': config.YOLO_IMG_SIZE,
                'max_det': config.YOLO_MAX_DETECTIONS,
                'verbose': False,
            }
            if config.YOLO_DEVICE:
                predict_kwargs['device'] = config.YOLO_DEVICE
            results = model.predict(**predict_kwargs)
            infer_ms = (time.monotonic() - start) * 1000.0

            detections = []
            if results and getattr(results[0], 'boxes', None) is not None \
                    and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                detections = detector.build_detections(
                    boxes_xyxy=boxes.xyxy.cpu().numpy(),
                    confidences=boxes.conf.cpu().numpy(),
                    class_ids=boxes.cls.cpu().numpy().astype(int),
                    names=getattr(results[0], 'names', model.names),
                    image_width_px=frame.shape[1],
                    focal_px=focal_px,
                    max_detections=config.YOLO_MAX_DETECTIONS,
                )
            last_detections = detections

            min_distance, _ = decision_logic.select_nearest_in_lane(detections)
            real_distance = distance_to(ego, lead)
            speed = speed_of(ego)

            # ---- 闭环控制：用决策函数的输出真的开车 ----
            if args.drive:
                decision = decision_logic.decide(min_distance, 0.0, True)
                control = carla.VehicleControl()
                control.throttle = float(decision.throttle)
                control.brake = float(decision.brake)
                control.steer = 0.0          # 作业要求：不做转向控制
                ego.apply_control(control)
            else:
                decision = None
                control = None

            history.append({
                'index': index - args.warmup,
                'speed': speed,
                'estimate': min_distance,
                'truth': real_distance,
                'state': None if decision is None else decision.state,
                'mode': None if control is None else (control.throttle, control.brake),
            })

            if (index - args.warmup) % 5 == 0 or index == args.warmup:
                print('帧 {:3d} | 推理 {:5.0f} ms | 目标 {} 个 | 估计距离 {} | '
                      '真值距离 {} | 速度 {:5.2f} m/s | {}'.format(
                          index - args.warmup, infer_ms, len(detections),
                          '  N/A ' if min_distance is None else '{:5.2f} m'.format(min_distance),
                          '  N/A ' if real_distance is None else '{:5.2f} m'.format(real_distance),
                          speed,
                          '' if decision is None else decision.state,
                      ))

            # ---- 保存截图（按间隔抽样，避免生成几百个文件）----
            if not args.no_save and (index - args.warmup) % save_stride == 0:
                summary = 'vehicles={} min_dist={} infer={:.0f}ms'.format(
                    len(detections),
                    'N/A' if min_distance is None else '{:.1f}m'.format(min_distance),
                    infer_ms,
                )
                annotated = draw_detections(frame, detections, summary)
                raw_path = os.path.join(args.outdir, 'frame_{:03d}_raw.png'.format(index - args.warmup))
                ann_path = os.path.join(args.outdir, 'frame_{:03d}_yolo.png'.format(index - args.warmup))
                import cv2
                cv2.imwrite(raw_path, frame)
                cv2.imwrite(ann_path, annotated)
                saved_files.append(ann_path)

        # ---- 3) 结果汇总 ----
        print('-' * 78)
        print('汇总')
        print('-' * 78)
        print('处理帧数            : {}'.format(len(history)))
        if history:
            estimates = [h['estimate'] for h in history if h['estimate'] is not None]
            truths = [h['truth'] for h in history if h['truth'] is not None]
            detected_frames = sum(1 for h in history if h['estimate'] is not None)
            print('检测到车辆的帧      : {}/{}'.format(detected_frames, len(history)))
            if estimates:
                print('估计距离范围        : {:.2f} ~ {:.2f} m'.format(min(estimates), max(estimates)))
            if truths:
                print('真实距离范围        : {:.2f} ~ {:.2f} m'.format(min(truths), max(truths)))
            if estimates and truths:
                # 用相同帧的估计值与真值比对，给出平均相对误差（评估单目测距精度）
                pairs = [(h['estimate'], h['truth']) for h in history
                         if h['estimate'] is not None and h['truth'] is not None
                         and h['truth'] > 1e-6]
                if pairs:
                    error = sum(abs(e - t) / t for e, t in pairs) / len(pairs) * 100.0
                    print('单目测距平均相对误差: {:.1f}%  （{} 帧参与统计）'.format(error, len(pairs)))
                    print('说明：误差来自检测框抖动、目标真实宽度假设、以及车辆 3D 尺寸近似，'
                          '课程作业够用。')
            print('最终速度            : {:.2f} m/s'.format(history[-1]['speed']))
            if history[-1]['truth'] is not None:
                print('最终真实距离        : {:.2f} m'.format(history[-1]['truth']))

            if args.drive:
                states = [h['state'] for h in history if h['state']]
                print('决策状态序列        : {}'.format(
                    ' -> '.join([states[0]] + [s for i, s in enumerate(states[1:], 1)
                                               if s != states[i - 1]])))
                final_speed = history[-1]['speed']
                final_truth = history[-1]['truth']
                stopped = final_speed < 1.0
                safe = final_truth is None or final_truth > 2.5
                print('避障判定            : {}（速度 {:.2f} m/s，距离 {}）'.format(
                    'PASS 已安全减速停车' if (stopped and safe) else 'FAIL 未能在障碍前停住',
                    final_speed,
                    'N/A' if final_truth is None else '{:.2f} m'.format(final_truth)))

        if saved_files:
            print('截图数量            : {} 组（原图 + 画框图）'.format(len(saved_files)))
            print('截图目录            : {}'.format(os.path.abspath(args.outdir)))
            print('示例（画框图）      : {}'.format(saved_files[-1]))
        print('=' * 78)
        return 0

    finally:
        # ---- 4) 清理，避免 CARLA 里堆积 actor ----
        for actor, name in ((camera, '相机'), (lead, '障碍车'), (ego, '自车')):
            if actor is None:
                continue
            try:
                if name == '相机':
                    actor.stop()
                actor.destroy()
                print('已清理{}'.format(name))
            except Exception as exc:
                print('清理{}失败（可忽略）：{}'.format(name, exc))


if __name__ == '__main__':
    raise SystemExit(main())
