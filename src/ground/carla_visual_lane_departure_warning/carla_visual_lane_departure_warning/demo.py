"""在线演示：图像算偏移，地图算真值，两路并排输出并统计误差。

不控车，只做预警判定——和仓库里的地图版 LDW 是同一个位置的产品，
区别只在偏移的来源。
"""
import os
import time

import numpy as np
from PIL import Image, ImageDraw

from . import carla_common as cc
from . import dataset
from . import lanes
from . import metrics
from . import model as net

# 每帧打印的间隔（帧）
PRINT_EVERY = 20


def offset_from_boundaries(left, right):
    """左右边界中点相对自车的横向位置，左偏为正。"""
    return -(left + right) / 2.0


def sway_steer(args, frame, now, offset, state):
    """闭环摆动：先把「正转向让偏移变大还是变小」探出来，再按目标做比例控制。

    CARLA 里 steer 的正负对应左右，不同版本示例写法不一致，与其猜，
    不如头 15 帧给一个固定转向、看偏移往哪边走。
    """
    import math

    probe_frames = 15
    probe_steer = 0.25
    if not state['probed']:
        if state['start'] is None:
            state['start'] = offset
        if state['count'] < probe_frames:
            state['count'] += 1
            return probe_steer
        delta = offset - state['start']
        state['sign'] = 1.0 if probe_steer * delta > 0 else -1.0
        state['probed'] = True
        print('[摆动] 探测完成：偏移变化 %+.2f m，转向符号 %+.0f'
              % (delta, state['sign']), flush=True)
        return 0.0

    target = args.sway * math.sin(2.0 * math.pi * args.sway_freq * now)
    error = target - offset
    return max(-0.6, min(0.6, state['sign'] * 0.9 * error))


def draw_hud(rgb, frame, visual_offset, map_offset, state, event):
    """左上角叠加文字，图上只出现英文，避免字体缺字。"""
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    lines = [
        'frame %d' % frame,
        'visual offset %+.2f m' % visual_offset,
        'map offset    %+.2f m' % map_offset,
        'state %s' % state,
    ]
    if event:
        lines.append('event %s' % event)
    for index, text in enumerate(lines):
        y = 8 + index * 16
        draw.rectangle([6, y - 2, 230, y + 13], fill=(0, 0, 0))
        draw.text((10, y), text, fill=(0, 255, 0))
    return np.asarray(image)


def run(args):
    if not os.path.isfile(args.checkpoint):
        print('找不到权重 %s，先跑 --collect 和 --train' % args.checkpoint)
        return 1

    regressor = net.LaneRegressor.load(args.checkpoint)
    client, world = cc.connect(args.host, args.port, args.town)
    cc.enable_sync(world, args.dt)
    ego = cc.spawn_ego(world, client, args.vehicle, args.spawn_index)
    camera, frames = cc.attach_camera(world, ego, args.width, args.height, args.fov)
    sway = getattr(args, 'sway', 0.0)
    sway_state = {'sign': 0.0, 'start': None, 'probed': False, 'count': 0}
    if sway > 0.0:
        import carla
        ego.set_autopilot(False)
        print('[摆动] 关闭自动驾驶，目标横向幅度 %.2f m，频率 %.2f Hz'
              % (sway, args.sway_freq), flush=True)

    # 用交通管理器的强制变道制造真实的车道线穿越，
    # 比自己写转向控制稳：车仍然跟着车道走，只是整体挪一条道
    lane_change_every = getattr(args, 'lane_change', 0)
    manager = None
    if lane_change_every > 0 and sway <= 0.0:
        manager = client.get_trafficmanager()
        manager.set_synchronous_mode(True)
        ego.set_autopilot(True, manager.get_port())
        print('[变道] 每 %d 帧强制变道一次' % lane_change_every, flush=True)

    visual_machine = lanes.LaneDepartureStateMachine()
    map_machine = lanes.LaneDepartureStateMachine()
    visual_events, map_events = [], []
    speeds = []
    visual_series, map_series = [], []
    settle = max(0, args.settle)
    skipped = 0
    previous_visual = None
    record_dir = args.record or ''

    try:
        if record_dir:
            os.makedirs(record_dir, exist_ok=True)
            print('[录制] 逐帧写入 %s' % record_dir)

        for frame in range(args.frames):
            world.tick()
            image = frames.get(timeout=5.0)
            cc.follow_ego(world, ego)
            if frame < settle:
                if sway > 0.0:
                    ego.apply_control(carla.VehicleControl(
                        throttle=args.throttle, steer=0.0))
                continue
            rgb = cc.image_to_array(image)
            now = frame * args.dt
            gray = dataset.resize_gray(rgb)
            predicted = regressor.predict(net.flatten_images(gray[None]))[0]
            visual_offset = offset_from_boundaries(predicted[0], predicted[1])

            transform = ego.get_transform()
            waypoint = world.get_map().get_waypoint(transform.location,
                                                    project_to_road=True)
            # 真值取自车位置的垂距：往前看 10 m 的话，弯道上那段弧会被算成
            # 偏移，车摆在正中也能读出一米多，真值自己先报警
            map_offset = dataset.lane_offset(waypoint, transform)
            if sway > 0.0:
                steer = sway_steer(args, frame, now, map_offset, sway_state)
                ego.apply_control(carla.VehicleControl(
                    throttle=args.throttle, steer=steer))
            elif manager is not None and frame % lane_change_every == 0:
                direction = (frame // lane_change_every) % 2 == 0
                manager.force_lane_change(ego, direction)
                print('  [变道] 第 %d 帧 强制向%s变道'
                      % (frame, '左' if direction else '右'), flush=True)
            # 地图这一帧给的车道不合理（路口、宽路段、判错车道）时，
            # 拿它当真值算误差没有意义，直接跳过
            if not (2.8 < waypoint.lane_width < 4.2) or abs(map_offset) > args.max_offset:
                skipped += 1
                continue

            # 逐帧估计会抖，输出前先做一阶平滑；平滑系数写进文档
            if previous_visual is None:
                pass
            else:
                visual_offset = (args.smooth * previous_visual
                                 + (1.0 - args.smooth) * visual_offset)
            previous_visual = visual_offset

            visual_series.append(visual_offset)
            map_series.append(map_offset)
            speeds.append(float(ego.get_velocity().length()))

            state, event = visual_machine.update(visual_offset)
            if event:
                visual_events.append((now, event))
                print('  [视觉] t=%5.2f s  %s（偏移 %+.2f m）' % (now, event, visual_offset))
            map_state, map_event = map_machine.update(map_offset)
            if map_event:
                map_events.append((now, map_event))
                print('  [地图] t=%5.2f s  %s（偏移 %+.2f m）' % (now, map_event, map_offset))

            if frame % PRINT_EVERY == 0:
                print('帧 %4d  视觉 %+.2f m  地图 %+.2f m  误差 %+.2f m  状态 %s'
                      % (frame, visual_offset, map_offset,
                         visual_offset - map_offset, state))

            if record_dir:
                frame_image = draw_hud(rgb, frame, visual_offset, map_offset,
                                       state, event)
                Image.fromarray(frame_image).save(
                    os.path.join(record_dir, 'frame_%04d.png' % frame))
        # 汇总放在清理之前：清理阶段 CARLA 偶尔会因为 actor 已销毁而直接
        # C++ abort，那种崩溃 try/except 捕不回来，报告必须先打出来
        if not visual_series:
            print('一帧也没跑，检查 --frames')
            return 1

        print('\n跳过 %d 帧（地图无有效车道），统计只用 %d 帧'
              % (skipped, len(visual_series)))
        report(visual_series, map_series, speeds, visual_events, map_events, args)
        return 0
    finally:
        cc.destroy_all(camera, ego)
        cc.restore_async(world)


def report(visual_series, map_series, speeds, visual_events, map_events, args):
    """汇总误差与事件一致性，并把指标写成 json。"""
    result = metrics.evaluate_offsets(np.array(visual_series),
                                      np.array(map_series), speeds)
    print('\n' + metrics.format_report(result))

    comparison = lanes.compare_events(visual_events, map_events,
                                      tolerance=args.event_tolerance)
    print('\n事件对照：匹配 %d，视觉误报 %d，视觉漏报 %d，平均滞后 %+.2f s'
          % (comparison['matched'], comparison['false_alarm'],
             comparison['missed'], comparison['mean_delay']))

    if args.save_metrics:
        import json
        payload = {'metrics': result, 'events': comparison,
                   'visual_events': visual_events, 'map_events': map_events,
                   'series': {'visual_offset': visual_series,
                              'map_offset': map_series}}
        os.makedirs(os.path.dirname(os.path.abspath(args.save_metrics)), exist_ok=True)
        with open(args.save_metrics, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        print('指标写到 %s' % args.save_metrics)