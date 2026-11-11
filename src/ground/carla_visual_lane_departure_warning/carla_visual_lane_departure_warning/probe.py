"""探路脚本：抓一帧图，把地图算出来的车道边界投到图像上。

这一功能只回答一个问题——CARLA 的路面到底画没画车道线、位置对不对得上。
每次采样都算一个「线亮度得分」：投影点上的像素比左右各偏 14 像素处亮多少。
得分明显为正说明投影点正好压在画线上；接近 0 说明那里是光板路或投影偏了。

出生点常常在路口（路口没有车道线），所以默认先空跑若干帧让车开出去，
最后只保存得分最高的几张图。
"""
import math
import os

import numpy as np
from PIL import Image, ImageDraw

from . import carla_common as cc
from . import geometry as geo

# 边界采样：从车头前 3 m 到 40 m，每 2 m 一个点
AHEAD_START = 8.0
AHEAD_END = 40.0
AHEAD_STEP = 2.0

# 判断「压在线上」的取样参数
SAMPLE_HALF = 1
NEIGHBOUR_OFFSET = 14


def lane_boundaries(waypoint, half_width_scale=0.5):
    """沿车道往前采样，返回左右边界点的世界坐标 (N, 3) 两份。

    边界取在「车道中心线 ± 车道宽一半」处，如果地图土建和路面贴图一致，
    这些点应当正好落在画的线上。
    """
    left, right = [], []
    node = waypoint
    distance = AHEAD_START
    node = node.next(distance)[0]
    while distance <= AHEAD_END:
        transform = node.transform
        location = transform.location
        side = transform.get_right_vector()
        half = node.lane_width * half_width_scale
        centre = np.array([location.x, location.y, location.z])
        lateral = np.array([side.x, side.y, side.z])
        left.append(centre - lateral * half)
        right.append(centre + lateral * half)
        node = node.next(AHEAD_STEP)[0]
        distance += AHEAD_STEP
    return np.array(left), np.array(right)


def draw_polyline(image, uv, inside, colour, width=3):
    """把投影点连成折线画到图上，落在画面外的点直接跳过。"""
    draw = ImageDraw.Draw(image)
    previous = None
    for point, ok in zip(uv, inside):
        if not ok:
            previous = None
            continue
        current = (float(point[0]), float(point[1]))
        if previous is not None:
            draw.line([previous, current], fill=colour, width=width)
        previous = current


def paint_score(gray, uv, inside, half=SAMPLE_HALF, offset=NEIGHBOUR_OFFSET):
    """投影点比左右邻域亮多少（灰度）。正得越多，越像压在画线上。"""
    scores = []
    height, width = gray.shape
    for (u, v), ok in zip(uv, inside):
        if not ok:
            continue
        col, row = int(round(u)), int(round(v))
        if not (0 <= row < height):
            continue
        centre = gray[row, max(0, col - half):min(width, col + half + 1)].mean()
        left = gray[row, max(0, col - offset - half):min(width, col - offset + half + 1)].mean()
        right = gray[row, max(0, col + offset - half):min(width, col + offset + half + 1)].mean()
        scores.append(centre - 0.5 * (left + right))
    return float(np.mean(scores)) if scores else 0.0


def run(args):
    client, world = cc.connect(args.host, args.port, args.town)
    cc.enable_sync(world, args.dt)
    ego = cc.spawn_ego(world, client, args.vehicle, args.spawn_index)
    camera, frames = cc.attach_camera(world, ego, args.width, args.height, args.fov)

    os.makedirs(args.out_dir, exist_ok=True)
    k = geo.intrinsics(args.width, args.height, args.fov)
    print('开始探路：%d 帧，前 %d 帧不采样，每 %d 帧取一个点'
          % (args.frames, max(0, args.settle), args.every), flush=True)
    samples = []
    settle = max(0, args.settle)

    try:
        for index in range(args.frames):
            world.tick()
            image = frames.get(timeout=5.0)
            cc.follow_ego(world, ego)
            if index % 20 == 0:
                print('  ... 第 %d 帧' % index, flush=True)

            if index < settle:
                continue
            if index % args.every:
                continue

            rgb = cc.image_to_array(image)
            gray = rgb.mean(axis=2)
            transform = ego.get_transform()
            waypoint = world.get_map().get_waypoint(transform.location,
                                                    project_to_road=True)
            offset = geo.lateral_offset(
                (transform.location.x, transform.location.y),
                math.radians(transform.rotation.yaw),
                (waypoint.transform.location.x, waypoint.transform.location.y),
                math.radians(waypoint.transform.rotation.yaw))

            left, right = lane_boundaries(waypoint)
            pose = np.array(camera.get_transform().get_matrix())
            left_uv, _, left_inside = geo.project(left, pose, k)
            right_uv, _, right_inside = geo.project(right, pose, k)

            score = 0.5 * (paint_score(gray, left_uv, left_inside)
                           + paint_score(gray, right_uv, right_inside))
            samples.append({'index': index, 'score': score, 'offset': offset,
                            'rgb': rgb, 'left_uv': left_uv, 'left_in': left_inside,
                            'right_uv': right_uv, 'right_in': right_inside,
                            'waypoint': waypoint, 'pose': pose})
    finally:
        print('[清理] 1/3 停止相机', flush=True)
        try:
            camera.stop()
        except Exception as error:
            print('       停止相机失败：%r' % error, flush=True)
        print('[清理] 2/3 销毁自车', flush=True)
        try:
            ego.destroy()
        except Exception as error:
            print('       销毁自车失败：%r' % error, flush=True)
        print('[清理] 3/3 恢复异步模式', flush=True)
        try:
            cc.restore_async(world)
        except Exception as error:
            print('       恢复失败：%r' % error, flush=True)

    if not samples:
        print('没有采到任何帧，检查 --frames / --every / --settle 的组合')
        return 1

    print('采了 %d 个采样点，线亮度得分（正 = 投影点比两侧亮，像压在白线上）'
          % len(samples))
    for sample in samples[:12]:
        print('  帧 %4d  得分 %+6.1f  横向偏移 %+5.2f m'
              % (sample['index'], sample['score'], sample['offset']))

    samples.sort(key=lambda item: item['score'], reverse=True)
    keep = samples[:max(1, args.top)]
    for rank, sample in enumerate(keep):
        base = Image.fromarray(sample['rgb'])
        base.save(os.path.join(args.out_dir, 'probe_%03d.png' % rank))
        overlay = Image.fromarray(sample['rgb'].copy())
        draw_polyline(overlay, sample['left_uv'], sample['left_in'], (255, 60, 60))
        draw_polyline(overlay, sample['right_uv'], sample['right_in'], (60, 160, 255))
        draw_centre(overlay, sample, k)
        overlay.save(os.path.join(args.out_dir, 'probe_%03d_annot.png' % rank))
        print('  保存 probe_%03d（帧 %d，得分 %+.1f）'
              % (rank, sample['index'], sample['score']))

    best = keep[0]['score']
    print('\n最好的一帧得分 %+.1f' % best)
    if best < 5.0:
        print('得分接近 0：这些位置的路面上没有可用的车道线，'
              '多半是路口或小路。换 --spawn-index 或者加大 --settle 再试。')
    else:
        print('得分明显为正：投影压在白线上，可以用这些位置采数据。')
    return 0


def draw_centre(image, sample, k):
    """把车道中心线也画出来，方便判断投影是否整体偏移。"""
    centre = []
    node = sample['waypoint']
    distance = 0.0
    while distance <= AHEAD_END:
        location = node.transform.location
        centre.append(np.array([location.x, location.y, location.z + 0.02]))
        node = node.next(AHEAD_STEP)[0]
        distance += AHEAD_STEP
    uv, _, inside = geo.project(np.array(centre), sample['pose'], k)
    draw_polyline(image, uv, inside, (60, 220, 60), width=2)