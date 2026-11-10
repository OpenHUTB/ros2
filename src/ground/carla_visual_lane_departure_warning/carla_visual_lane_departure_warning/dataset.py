"""数据集：从 CARLA 采同期图像与车道边界真值，存成一个 npz。

真值不是人工标注，是同一帧里地图 API 算出来的左右边界横向位置：
两张数据同源同步，不存在对齐问题。
"""
import math
import os

import numpy as np
from PIL import Image

from . import carla_common as cc
from . import geometry as geo
from .model import INPUT_H, INPUT_W

# 真值取车头前方 5 m 处的左右边界。10 m 时弯道上的横向位移会整段算进标签，
# 实测左右边界均值能差出 0.6 m，5 m 把这个曲率项缩到原来的四分之一
TARGET_DISTANCE = 5.0

# 裁剪比例：上边界切掉天空，下边界切掉引擎盖（实测引擎盖前沿在第 555 行附近）
CROP_TOP = 0.55
CROP_BOTTOM = 0.92


def resize_gray(rgb):
    """裁掉天空和车头，再缩成 128x32 的灰度图。

    直接整图缩放会把 10 像素宽的白线压成不到 1 像素，网络根本看不见，
    所以先按比例裁出路面条带，横向只缩 6 倍。
    """
    height, width = rgb.shape[:2]
    top = int(height * CROP_TOP)
    bottom = int(height * CROP_BOTTOM)
    cropped = Image.fromarray(rgb[top:bottom]).convert('L')
    return np.asarray(cropped.resize((INPUT_W, INPUT_H)), dtype=np.uint8)


def boundary_targets(waypoint, ego_transform, distance=TARGET_DISTANCE):
    """返回 (左边界横向位置, 右边界横向位置)，单位米，右为正。

    `waypoint` 取自车当前位置；先沿车道往前走到 `distance` 处，
    再用「中心 ± 半个车道宽」得到左右边界，最后投影到自车的横向轴上。
    """
    node = waypoint
    travelled = 0.0
    while travelled < distance:
        step = min(2.0, distance - travelled)
        node = node.next(step)[0]
        travelled += step

    location = node.transform.location
    side = node.transform.get_right_vector()
    half = node.lane_width / 2.0
    centre = np.array([location.x, location.y, location.z])

    ego_location = ego_transform.location
    ego_side = ego_transform.get_right_vector()
    ego_right = np.array([ego_side.x, ego_side.y, ego_side.z])
    ego_centre = np.array([ego_location.x, ego_location.y, ego_location.z])

    left = centre - np.array([side.x, side.y, side.z]) * half - ego_centre
    right = centre + np.array([side.x, side.y, side.z]) * half - ego_centre
    return float(left @ ego_right), float(right @ ego_right)


def lane_offset(waypoint, ego_transform):
    """自车到当前车道中心线的垂距，左偏为正，与预测偏移同口径。

    取自车所在位置的路点（distance=0）再投影，而不是沿车道往前看 10 m：
    弯道上 10 m 外那段弧的横向位移会整段算进偏移量，车摆在车道正中也能
    读出 1 m 以上，真值自己就先报警了。
    """
    left, right = boundary_targets(waypoint, ego_transform, distance=0.0)
    return -(left + right) / 2.0


def boundary_points(waypoint, distance=TARGET_DISTANCE):
    """沿车道往前走到 distance 处，返回左右边界点的世界坐标。"""
    node = waypoint
    travelled = 0.0
    while travelled < distance:
        step = min(2.0, distance - travelled)
        node = node.next(step)[0]
        travelled += step
    location = node.transform.location
    side = node.transform.get_right_vector()
    half = node.lane_width / 2.0
    centre = np.array([location.x, location.y, location.z])
    lateral = np.array([side.x, side.y, side.z])
    return centre - lateral * half, centre + lateral * half


def visible_score(waypoint, side, gray, pose, k, distances=(6.0, 10.0, 14.0)):
    """在几处距离上找这条边界，取最亮的一次。

    车道线大多是虚线，只看 10 m 那一个点会有一半概率落在空档里，
    所以沿纵向多试几个位置，只要有一处压在画线上就算这条线可见。
    """
    best = -999.0
    node = waypoint
    travelled = 0.0
    for target in distances:
        while travelled < target:
            step = min(2.0, target - travelled)
            node = node.next(step)[0]
            travelled += step
        location = node.transform.location
        vector = node.transform.get_right_vector()
        half = node.lane_width / 2.0
        centre = np.array([location.x, location.y, location.z])
        lateral = np.array([vector.x, vector.y, vector.z])
        point = centre - lateral * half if side == 'left' else centre + lateral * half
        best = max(best, paint_score(gray, point, pose, k))
    return best


def paint_score(gray, point, pose, k, half=1, offset=14):
    """某个世界点投到图像后的「压在画线上」程度。"""
    uv, _, inside = geo.project(np.array([point]), pose, k)
    if not inside[0]:
        return -999.0
    col, row = int(round(uv[0][0])), int(round(uv[0][1]))
    height, width = gray.shape
    if not (0 <= row < height):
        return -999.0
    left_lo, left_hi = max(0, col - offset - half), min(width, col - offset + half + 1)
    right_lo, right_hi = max(0, col + offset - half), min(width, col + offset + half + 1)
    if left_lo >= left_hi or right_lo >= right_hi:
        return -999.0
    centre = gray[row, max(0, col - half):min(width, col + half + 1)].mean()
    left = gray[row, left_lo:left_hi].mean()
    right = gray[row, right_lo:right_hi].mean()
    return float(centre - 0.5 * (left + right))


def sway_steer(sway, freq, state, now, offset):
    """采集时的闭环摆动：先探出转向符号，再按正弦目标做比例控制。

    自动驾驶只会让车贴着车道中心跑，采出来的标签几乎全在 0 附近，网络最后
    只能学会「永远输出 0」。采集时主动左右摆，偏移才能覆盖到 ±sway。
    """
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
        return 0.0
    target = sway * math.sin(2.0 * math.pi * freq * now)
    return max(-0.6, min(0.6, state['sign'] * 0.9 * (target - offset)))


def save_dataset(args, images, targets, speeds, skipped_width, skipped_offset,
                 skipped_paint):
    """把采到的样本落盘并打印丢弃统计。"""
    if not images:
        raise RuntimeError('一帧也没采到，检查 --every 和地图是否有可行驶车道')
    dataset = {
        'images': np.asarray(images, dtype=np.uint8),
        'targets': np.asarray(targets, dtype=np.float32),
        'speeds': np.asarray(speeds, dtype=np.float32),
    }
    os.makedirs(os.path.dirname(args.data) or '.', exist_ok=True)
    np.savez_compressed(args.data, **dataset)
    print('\n写出 %s' % args.data)
    print('  图像 %s，真值 %s' % (dataset['images'].shape, dataset['targets'].shape))
    print('  丢弃：车道宽异常 %d，自车不在车道中间 %d，边界没画线 %d'
          % (skipped_width, skipped_offset, skipped_paint))
    mean = dataset['targets'].mean(axis=0)
    print('  左边界均值 %+.2f m，右边界均值 %+.2f m' % (mean[0], mean[1]))
    return dataset


def collect(args):
    """连 CARLA 采数据。图像与真值在同一帧取，逐帧配对。"""
    client, world = cc.connect(args.host, args.port, args.town)
    cc.enable_sync(world, args.dt)
    ego = cc.spawn_ego(world, client, args.vehicle, args.spawn_index)
    camera, frames = cc.attach_camera(world, ego, args.width, args.height, args.fov)

    sway = getattr(args, 'sway', 0.0)
    sway_state = {'sign': 0.0, 'start': None, 'probed': False, 'count': 0}
    lane_change_every = getattr(args, 'lane_change', 0)
    manager = None
    if sway > 0.0:
        import carla
        ego.set_autopilot(False)
        print('[采集] 关闭自动驾驶，横向摆动 ±%.2f m，频率 %.2f Hz'
              % (sway, args.sway_freq), flush=True)
    elif lane_change_every > 0:
        # 车始终开在车道里，靠变道瞬态把偏移扫过 0 -> ±1.75 -> 0，
        # 比强行摆动稳，也不会把车开到路肩上
        manager = client.get_trafficmanager()
        manager.set_synchronous_mode(True)
        ego.set_autopilot(True, manager.get_port())
        print('[采集] 每 %d 帧强制变道一次，用变道瞬态覆盖大偏移' % lane_change_every,
              flush=True)

    images, targets, speeds = [], [], []
    skipped_width = 0
    skipped_offset = 0
    skipped_paint = 0
    k = geo.intrinsics(args.width, args.height, args.fov)

    try:
        for index in range(args.frames):
            world.tick()
            image = frames.get(timeout=5.0)

            transform = ego.get_transform()
            waypoint = world.get_map().get_waypoint(transform.location,
                                                    project_to_road=True)
            # 摆动每一拍都要施加，被 every 跳过的帧也不能撒手
            if sway > 0.0:
                ego.apply_control(carla.VehicleControl(
                    throttle=args.throttle,
                    steer=sway_steer(sway, args.sway_freq, sway_state,
                                     index * args.dt,
                                     lane_offset(waypoint, transform))))
            if manager is not None and index % lane_change_every == 0:
                direction = (index // lane_change_every) % 2 == 0
                manager.force_lane_change(ego, direction)
            if index % args.every:
                continue

            rgb = cc.image_to_array(image)
            left, right = boundary_targets(waypoint, transform)
            # 用地图给的车道宽判断，别用投影后的边界间距：车一转向、一压线，投影出来的
            # 间距会被拉长，变道和弯道上能把整段数据都判成「宽度异常」
            if not (2.8 < waypoint.lane_width < 4.2):
                skipped_width += 1
                continue
            # 自车相对当前车道中心的偏移，用垂距；往前看 10 m 会把曲率算进来
            lateral_now = lane_offset(waypoint, transform)
            if abs(lateral_now) > args.max_offset:
                skipped_offset += 1
                continue
            # 两条边界上得真有白线，否则网络没有任何线索可学
            gray_full = rgb.mean(axis=2)
            pose = np.array(camera.get_transform().get_matrix())
            score_left = visible_score(waypoint, 'left', gray_full, pose, k)
            score_right = visible_score(waypoint, 'right', gray_full, pose, k)
            if min(score_left, score_right) < args.min_paint:
                skipped_paint += 1
                continue

            images.append(resize_gray(rgb))
            targets.append((left, right))
            speeds.append(float(ego.get_velocity().length()))
            if (index + 1) % max(1, args.frames // 10) == 0:
                print('  已采 %d 帧（%d/%d），丢弃 宽度%d 偏移%d 无线%d'
                      % (len(images), index + 1, args.frames,
                         skipped_width, skipped_offset, skipped_paint))
    finally:
        # 先落盘再清理：交通管理器强制变道后，清理阶段偶尔会因为 actor 已经被销毁
        # 而直接 C++ abort，那种崩溃 try/except 捕不回来，数据必须先写下去
        payload = save_dataset(args, images, targets, speeds, skipped_width,
                               skipped_offset, skipped_paint)
        cc.destroy_all(camera, ego)
        cc.restore_async(world)

    return payload


def load(path):
    with np.load(path) as data:
        return {key: data[key] for key in data.files}


def augment_flip(dataset):
    """左右镜像扩一倍数据。

    镜像图里左右车道互换、横向坐标取反，所以新样本的左右边界
    就是「原来的右边界取反」和「原来的左边界取反」。
    """
    images = dataset['images']
    targets = dataset['targets']
    flipped_images = images[:, :, ::-1]
    flipped_targets = np.stack([-targets[:, 1], -targets[:, 0]], axis=1)
    return {
        'images': np.concatenate([images, flipped_images], axis=0),
        'targets': np.concatenate([targets, flipped_targets], axis=0).astype(np.float32),
    }


def split(dataset, val_ratio=0.2, seed=0):
    """按帧顺序随机划分，不打乱时间近邻也够用（同一段路会同时进两边）。"""
    rng = np.random.default_rng(seed)
    total = dataset['images'].shape[0]
    order = rng.permutation(total)
    cut = int(total * (1.0 - val_ratio))
    train, val = order[:cut], order[cut:]
    train_set = {'images': dataset['images'][train], 'targets': dataset['targets'][train]}
    val_set = {'images': dataset['images'][val], 'targets': dataset['targets'][val]}
    if 'speeds' in dataset:
        train_set['speeds'] = dataset['speeds'][train]
        val_set['speeds'] = dataset['speeds'][val]
    return train_set, val_set


def lateral_from_targets(targets):
    """由左右边界反推带符号横向偏移：两者中点相对车道中心的偏移。

    注意这里是相对「地图给的边界」，不是相对车道中心线，
    所以偏移 = (左 + 右) / 2，左正右负与仓库 LDW 口径一致。
    """
    targets = np.asarray(targets, dtype=np.float64)
    return (targets[..., 0] + targets[..., 1]) / 2.0 * -1.0