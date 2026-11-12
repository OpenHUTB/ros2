#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务③ 建图 + 导航 (SLAM 同步仿真)
纯 Python 实现 + 可选 ROS1(noetic) 话题发布:
  /map   (nav_msgs/OccupancyGrid)  栅格地图
  /scan  (sensor_msgs/LaserScan)   单帧激光
  /odom  (nav_msgs/Odometry)       里程计
  /cmd_vel (geometry_msgs/Twist)   控制指令
流程: 激光 -> 占据栅格建图(log-odds) -> A* 全局规划 -> 纯跟踪局部控制 -> 边走边建图
输出: task3_slam_map.png, task3_slam_navigation.gif, task3_metrics.json
"""
import os, sys, math, json, time, queue, heapq
import numpy as np
try:
    import cv2
except ImportError:
    cv2 = None
from PIL import Image, ImageDraw
try:
    import carla
except ImportError:
    carla = None

HOST, PORT = '127.0.0.1', 2000
# ---- 启动参数支持: launch 文件通过 args 传入 host/port ----
import argparse as _argparse
_ap = _argparse.ArgumentParser(add_help=False)
_ap.add_argument('--host', default=HOST)
_ap.add_argument('--port', type=int, default=PORT)
_ap.add_argument('--out', default='output', help='输出目录(相对当前工作目录)')
_known, _unknown = _ap.parse_known_args()
HOST, PORT = _known.host, _known.port
OUT_DIR = _known.out
os.makedirs(OUT_DIR, exist_ok=True)

RES = 0.5                       # 栅格分辨率 m/cell
MAP_FRAMES = 130                # 建图阶段帧数
NAV_FRAMES = 110                # 导航阶段帧数
OUT_MAP = os.path.join(OUT_DIR, 'task3_slam_map.png')
OUT_GIF = os.path.join(OUT_DIR, 'task3_slam_navigation.gif')

# ---------------- 可选 ROS 发布 ----------------
ROS = False
try:
    import rospy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Header
    ROS = True
except Exception:
    rospy = None


class GridMap:
    """稀疏 log-odds 占据栅格"""

    def __init__(self, res=RES):
        self.res = res
        self.logodds = {}
        self.L_OCC = 0.85
        self.L_FREE = -0.4
        self.L_MAX, self.L_MIN = 3.5, -2.5

    def _k(self, x, y):
        return (int(math.floor(x / self.res)), int(math.floor(y / self.res)))

    def update_ray(self, ox, oy, px, py):
        dx, dy = px - ox, py - oy
        dist = math.hypot(dx, dy)
        if dist < 0.3 or dist > 55.0:
            return
        steps = max(int(dist / (self.res * 0.8)), 1)
        for s in range(steps):
            t = s / steps
            k = self._k(ox + dx * t, oy + dy * t)
            v = self.logodds.get(k, 0.0) + self.L_FREE
            self.logodds[k] = max(self.L_MIN, min(self.L_MAX, v))
        k = self._k(px, py)
        v = self.logodds.get(k, 0.0) + self.L_OCC
        self.logodds[k] = max(self.L_MIN, min(self.L_MAX, v))

    def is_free(self, k):
        return self.logodds.get(k, 0.0) < 0.2

    def is_occ(self, k):
        return self.logodds.get(k, 0.0) > 0.6

    def bounds(self):
        if not self.logodds:
            return 0, 0, 1, 1
        ks = list(self.logodds.keys())
        return (min(k[0] for k in ks), min(k[1] for k in ks),
                max(k[0] for k in ks), max(k[1] for k in ks))

    def to_image(self, scale=1):
        i0, j0, i1, j1 = self.bounds()
        w, h = i1 - i0 + 1, j1 - j0 + 1
        img = np.full((h, w), 128, dtype=np.uint8)
        for (i, j), v in self.logodds.items():
            c = 0 if v > 0.6 else (254 if v < -0.2 else 200)
            img[j - j0, i - i0] = c
        img = np.flipud(img)          # y 轴向上
        return img, (i0, j0)

    # ---------------- A* ----------------
    def astar(self, start, goal, max_expand=40000):
        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])
        openq = [(h(start, goal), 0.0, start)]
        came, g = {}, {start: 0.0}
        exp = 0
        nbrs = [(1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1)]
        while openq and exp < max_expand:
            _, gc, cur = heapq.heappop(openq)
            if cur == goal:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                return list(reversed(path))
            exp += 1
            for di, dj in nbrs:
                nb = (cur[0] + di, cur[1] + dj)
                if self.is_occ(nb):
                    continue
                # 斜向需两侧通行
                if di and dj:
                    if self.is_occ((cur[0] + di, cur[1])) or \
                       self.is_occ((cur[0], cur[1] + dj)):
                        continue
                step = math.hypot(di, dj)
                ng = gc + step
                if ng < g.get(nb, 1e18):
                    g[nb] = ng
                    came[nb] = cur
                    heapq.heappush(openq, (ng + h(nb, goal), ng, nb))
        return []


def lidar_points(lidar_data):
    a = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
    return a.reshape((-1, 4))[:, :3].astype(np.float64)


def to_world(tf, pts):
    """激光点(车体系) -> 世界坐标"""
    yaw = math.radians(tf.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    x = pts[:, 0] * c - pts[:, 1] * s + tf.location.x
    y = pts[:, 0] * s + pts[:, 1] * c + tf.location.y
    return x, y


def teacher(world, ego, target_speed=7.5, lookahead=8.0):
    m = world.get_map()
    loc, tf = ego.get_location(), ego.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    wp = m.get_waypoint(loc, project_to_road=True,
                        lane_type=carla.LaneType.Driving)
    path, cur = [], wp
    for _ in range(20):
        nx = cur.next(2.5)
        if not nx:
            break
        cur = nx[0]
        path.append(cur.transform.location)
    target = path[-1] if path else loc
    for p in path:
        if p.distance(loc) >= lookahead:
            target = p
            break
    dx, dy = target.x - loc.x, target.y - loc.y
    lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    alpha = math.atan2(ly, max(abs(lx), 1e-3))
    steer = math.atan2(2.0 * 2.9 * math.sin(alpha), max(math.hypot(lx, ly), 2.0))
    steer = float(np.clip(steer / math.radians(35.0), -1.0, 1.0))
    v = ego.get_velocity().length()
    err = target_speed - v
    throttle = float(np.clip(0.55 + 0.25 * err, 0.0, 1.0))
    brake = float(np.clip(-0.6 * err, 0.0, 1.0)) if err < -1.5 else 0.0
    return steer, throttle, brake


def follow_path(world, ego, path_xy, target_speed=6.0):
    """沿规划路径的纯跟踪控制"""
    loc = ego.get_location()
    if not path_xy:
        return 0.0, 0.0, 1.0
    # 最近点
    d = [(math.hypot(x - loc.x, y - loc.y), i)
         for i, (x, y) in enumerate(path_xy)]
    _, i0 = min(d)
    look = path_xy[-1]
    acc = 0.0
    for k in range(i0, len(path_xy)):
        if k > i0:
            acc += math.hypot(path_xy[k][0] - path_xy[k - 1][0],
                              path_xy[k][1] - path_xy[k - 1][1])
        if acc >= 7.0:
            look = path_xy[k]
            break
    tf = ego.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    dx, dy = look[0] - loc.x, look[1] - loc.y
    lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    alpha = math.atan2(ly, max(abs(lx), 1e-3))
    steer = math.atan2(2.0 * 2.9 * math.sin(alpha), max(math.hypot(lx, ly), 2.0))
    steer = float(np.clip(steer / math.radians(35.0), -1.0, 1.0))
    v = ego.get_velocity().length()
    err = target_speed - v
    throttle = float(np.clip(0.55 + 0.25 * err, 0.0, 1.0))
    brake = float(np.clip(-0.7 * err, 0.0, 1.0)) if err < -1.5 else 0.0
    # 接近终点减速
    remain = math.hypot(path_xy[-1][0] - loc.x, path_xy[-1][1] - loc.y)
    if remain < 8.0:
        throttle, brake = 0.0, min(1.0, 0.25 * (8.0 - remain))
    return steer, throttle, brake


def render(map_img, origin, ego_xy, path_xy, scan_xy, w=420, h=420, view_range=45.0):
    """把地图 + 车辆 + 轨迹 + 激光渲染成一张图 (世界坐标 -> 像素)"""
    i0, j0 = origin
    res = RES
    cx, cy = ego_xy
    span = view_range
    canvas = np.full((h, w, 3), 30, dtype=np.uint8)
    half = span / 2.0
    # 地图裁剪
    im_h, im_w = map_img.shape
    x0 = int((cx - half) / res) - i0
    y0 = int((cy - half) / res) - j0
    size = int(span / res)
    sub = np.full((size, size), 128, dtype=np.uint8)
    sx0, sy0 = max(x0, 0), max(y0, 0)
    sx1, sy1 = min(x0 + size, im_w), min(y0 + size, im_h)
    if sx1 > sx0 and sy1 > sy0:
        # map_img 已上下翻转: 行 = (j1-j)，需要换算
        top = im_h - (y0 + size)
        sub[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0] = \
            map_img[max(top, 0):max(top, 0) + (sy1 - sy0),
                    sx0:sx1]
    sub = cv2.resize(sub, (w, h), interpolation=cv2.INTER_NEAREST)
    canvas[:, :, 0] = sub
    canvas[:, :, 1] = sub
    canvas[:, :, 2] = sub
    canvas = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

    def w2p(x, y):
        px = int((x - (cx - half)) / span * w)
        py = int(h - (y - (cy - half)) / span * h)
        return px, py

    # 激光点
    for x, y in scan_xy[:400]:
        px, py = w2p(x, y)
        if 0 <= px < w and 0 <= py < h:
            canvas[py, px] = (255, 90, 90)
    # 规划路径
    for x, y in path_xy:
        px, py = w2p(x, y)
        if 0 <= px < w and 0 <= py < h:
            cv2.circle(canvas, (px, py), 1, (60, 160, 255), -1)
    # 车辆
    px, py = w2p(cx, cy)
    cv2.circle(canvas, (px, py), 5, (0, 230, 90), -1)
    return canvas


def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(30.0)
    world = client.get_world()
    m = world.get_map()
    print('MAP:', m.name, flush=True)

    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = 0.05
    world.apply_settings(s)
    for a in world.get_actors().filter('vehicle.*'):
        try:
            a.destroy()
        except Exception:
            pass

    sp = m.get_spawn_points()[0]
    bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
    ego = world.spawn_actor(bp, sp)

    lid_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    lid_bp.set_attribute('range', '55')
    lid_bp.set_attribute('channels', '32')
    lid_bp.set_attribute('points_per_second', '60000')
    lid_bp.set_attribute('rotation_frequency', '20')
    lid = world.spawn_actor(lid_bp, carla.Transform(carla.Location(z=2.4)),
                            attach_to=ego)
    cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '400')
    cam_bp.set_attribute('image_size_y', '300')
    cam_bp.set_attribute('fov', '90')
    cam = world.spawn_actor(cam_bp, carla.Transform(
        carla.Location(x=1.5, z=1.6), carla.Rotation(pitch=-8)), attach_to=ego)
    ql, qc = queue.Queue(), queue.Queue()
    lid.listen(ql.put)
    cam.listen(qc.put)
    for _ in range(6):
        world.tick()

    # ROS 发布者
    pubs = {}
    if ROS:
        try:
            rospy.init_node('carla_slam_nav', anonymous=True)
            pubs['map'] = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
            pubs['scan'] = rospy.Publisher('/scan', LaserScan, queue_size=1)
            pubs['odom'] = rospy.Publisher('/odom', Odometry, queue_size=1)
            pubs['cmd'] = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            print('ROS_PUBLISHERS_READY', flush=True)
        except Exception as e:
            print('ROS init failed:', e, flush=True)

    gm = GridMap()
    trail = []
    t0 = time.time()

    # ---------- 阶段 1: 边走边建图 ----------
    print('--- STAGE 1: mapping ---', flush=True)
    for i in range(MAP_FRAMES):
        steer, throttle, brake = teacher(world, ego)
        c = carla.VehicleControl()
        c.steer, c.throttle, c.brake = steer, throttle, brake
        ego.apply_control(c)
        world.tick()
        try:
            ld = ql.get(timeout=5.0)
        except queue.Empty:
            continue
        tf = ego.get_transform()
        pts = lidar_points(ld)
        wx, wy = to_world(tf, pts)
        step = max(len(wx) // 700, 1)
        for k in range(0, len(wx), step):
            gm.update_ray(tf.location.x, tf.location.y, wx[k], wy[k])
        trail.append((tf.location.x, tf.location.y))
        if (i + 1) % 30 == 0:
            print(f'[map {i+1}/{MAP_FRAMES}] cells={len(gm.logodds)} '
                  f'pos=({tf.location.x:.1f},{tf.location.y:.1f})', flush=True)

    map_img, origin = gm.to_image()
    print('MAP_CELLS:', len(gm.logodds), 'SHAPE:', map_img.shape, flush=True)

    # ---------- 阶段 2: A* 规划 ----------
    print('--- STAGE 2: planning ---', flush=True)
    ego_loc = ego.get_location()
    start = (int(math.floor(ego_loc.x / RES)), int(math.floor(ego_loc.y / RES)))
    # 目标: 沿当前车道向前 20~60m 的已建图自由栅格
    goal = None
    cur = m.get_waypoint(ego.get_location(), project_to_road=True,
                         lane_type=carla.LaneType.Driving)
    for d in [57.5, 55.0, 52.5, 50.0, 45.0, 40.0, 35.0, 30.0, 25.0, 20.0]:
        nxt = cur.next(d)
        if not nxt:
            continue
        p = nxt[0].transform.location
        cell = (int(math.floor(p.x / RES)), int(math.floor(p.y / RES)))
        if gm.is_free(cell) and not gm.is_occ(cell):
            goal = cell
            print('GOAL at %.1fm' % d, flush=True)
            break
    if goal is None:
        x, y = trail[-1]
        goal = (int(math.floor(x / RES)), int(math.floor(y / RES)))
    print('GOAL_CELL:', goal, 'free=', gm.is_free(goal), flush=True)
    path_cells = gm.astar(start, goal)
    print('A_STAR cells:', len(path_cells), flush=True)
    path_xy = [((i + 0.5) * RES, (j + 0.5) * RES) for i, j in path_cells]
    path_xy = path_xy[::2] if len(path_xy) > 2 else path_xy

    # ---------- 阶段 3: 导航 + 继续建图 ----------
    print('--- STAGE 3: navigation ---', flush=True)
    frames, speeds, min_obs = [], [], []
    for i in range(NAV_FRAMES):
        steer, throttle, brake = follow_path(world, ego, path_xy)
        c = carla.VehicleControl()
        c.steer, c.throttle, c.brake = steer, throttle, brake
        ego.apply_control(c)
        world.tick()
        try:
            ld = ql.get(timeout=5.0)
            ci = qc.get(timeout=5.0)
        except queue.Empty:
            continue
        tf = ego.get_transform()
        pts = lidar_points(ld)
        wx, wy = to_world(tf, pts)
        step = max(len(wx) // 700, 1)
        for k in range(0, len(wx), step):
            gm.update_ray(tf.location.x, tf.location.y, wx[k], wy[k])
        trail.append((tf.location.x, tf.location.y))
        # 前向障碍距离
        m_front = pts[(pts[:, 0] > 0) & (np.abs(pts[:, 1]) < 2.5), 0]
        dmin = float(m_front.min()) if m_front.size else 55.0
        speeds.append(ego.get_velocity().length() * 3.6)
        min_obs.append(dmin)

        map_img, origin = gm.to_image()
        now = time.time() - t0
        remain = math.hypot(path_xy[-1][0] - tf.location.x,
                            path_xy[-1][1] - tf.location.y) if path_xy else 0
        hud = f'frame {i+1}  v={speeds[-1]:4.1f}km/h  d_obs={dmin:4.1f}m  remain={remain:4.1f}m'
        map_view = render(map_img, origin, (tf.location.x, tf.location.y),
                          path_xy, list(zip(wx[::20], wy[::20])))
        cam_arr = np.frombuffer(ci.raw_data, dtype=np.uint8).reshape(
            (ci.height, ci.width, 4))[:, :, :3]
        map_view = cv2.resize(map_view, (420, 300))
        fuse = np.zeros((300, 820, 3), dtype=np.uint8)
        fuse[:, :400] = cam_arr[:, :, ::-1]
        fuse[:, 400:820] = map_view
        cv2.rectangle(fuse, (0, 0), (819, 24), (0, 0, 0), -1)
        cv2.putText(fuse, hud, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 255, 255), 1)
        frames.append(fuse)
        if (i + 1) % 30 == 0:
            print(f'[nav {i+1}/{NAV_FRAMES}] v={speeds[-1]:.1f}km/h '
                  f'remain={remain:.1f}m cells={len(gm.logodds)}', flush=True)

    # ---------- 保存 ----------
    map_img, origin = gm.to_image()
    colored = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
    i0, j0 = origin
    ih = map_img.shape[0]
    for (x, y) in trail:
        px = int(x / RES) - i0
        py = ih - (int(y / RES) - j0)
        if 0 <= px < colored.shape[1] and 0 <= py < ih:
            colored[py, px] = (60, 160, 255)
    for (x, y) in path_xy:
        px = int(x / RES) - i0
        py = ih - (int(y / RES) - j0)
        if 0 <= px < colored.shape[1] and 0 <= py < ih:
            colored[py, px] = (0, 90, 255)
    cv2.imwrite(OUT_MAP, colored)
    print('MAP_SAVED', OUT_MAP, colored.shape, flush=True)

    if frames:
        imgs = [Image.fromarray(cv2.cvtColor(cv2.resize(f, (410, 150)),
                                             cv2.COLOR_BGR2RGB)) for f in frames]
        imgs[0].save(OUT_GIF, save_all=True, append_images=imgs[1:],
                     duration=110, loop=0, optimize=True)
        met = {
            'map': m.name,
            'grid_cells_known': len(gm.logodds),
            'map_frames': MAP_FRAMES,
            'nav_frames': len(frames),
            'a_star_waypoints': len(path_cells),
            'avg_speed_kmh': round(float(np.mean(speeds)), 2),
            'min_obstacle_distance_m': round(float(np.min(min_obs)), 2),
            'elapsed_s': round(time.time() - t0, 1),
        }
        with open(os.path.join(OUT_DIR, 'task3_metrics.json'), 'w') as f:
            json.dump(met, f, indent=2)
        print('METRICS:', json.dumps(met), flush=True)
        print('GIF_SAVED', OUT_GIF, flush=True)

    try:
        s.synchronous_mode = False
        world.apply_settings(s)
        for a in (lid, cam, ego):
            a.destroy()
    except Exception:
        pass
    print('DONE', flush=True)


if __name__ == '__main__':
    main()
