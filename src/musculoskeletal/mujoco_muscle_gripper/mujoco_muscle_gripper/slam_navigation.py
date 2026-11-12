"""夹爪移动底座：激光占据栅格 SLAM 与训练后的神经价值迭代导航。"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import binary_dilation, distance_transform_edt, map_coordinates
import torch
from torch import nn
from torch.nn import functional as F

RESOLUTION = 0.20
SIZE = 32
DT = 0.15
TRACK = 0.32
RADIUS = 0.13
MAX_RANGE = 7.5
ANGLES = np.linspace(-np.pi, np.pi, 96, endpoint=False)


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def integrate(pose, wheels, dt=DT):
    """差速轮速积分，使用区间中点航向。"""
    v = (wheels[0] + wheels[1]) / 2
    omega = (wheels[1] - wheels[0]) / TRACK
    a = pose[2] + omega * dt / 2
    return np.array([pose[0] + v * np.cos(a) * dt,
                     pose[1] + v * np.sin(a) * dt, wrap(pose[2] + omega * dt)])


def scenarios():
    """底座携带肌肉夹爪的三种工作区，起点是已知初始坐标先验。"""
    layouts = []
    for name in ("工作台绕行", "双货架通道", "转角工位"):
        grid = np.zeros((SIZE, SIZE), dtype=bool)
        grid[[0, -1], :] = True
        grid[:, [0, -1]] = True
        if name == "工作台绕行":
            grid[4:23, 14:16] = True
            grid[8:12, 22:26] = True
        elif name == "双货架通道":
            grid[10:17, 8:14] = True
            grid[18:23, 18:27] = True
        else:
            grid[12:28, 11:13] = True
            grid[19:21, 19:29] = True
        layouts.append({"name": name, "grid": grid,
                        "start": np.array([1.0, 1.0, 0.0]),
                        "goal": np.array([5.3, 5.3])})
    return layouts


def distance_labels(obstacles, goal):
    """BFS 只生成离线训练标签和可达性评价，在线规划器不调用本函数。"""
    d = np.full(obstacles.shape, 100.0, dtype=np.float32)
    gy, gx = goal
    if obstacles[gy, gx]:
        return d
    d[gy, gx] = 0
    todo = deque([(gy, gx)])
    while todo:
        y, x = todo.popleft()
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            yy, xx = y + dy, x + dx
            if 0 <= yy < len(d) and 0 <= xx < d.shape[1] and not obstacles[yy, xx] and d[yy, xx] > d[y, x] + 1:
                d[yy, xx] = d[y, x] + 1
                todo.append((yy, xx))
    return d


class DifferentialWorld:
    """真值只留在传感器和运动仿真器内，不传入 SLAM 或导航器。"""

    def __init__(self, obstacles, initial_pose, seed):
        self.obstacles = obstacles.copy()
        self.pose = np.array(initial_pose, dtype=float)
        self.rng = np.random.default_rng(seed)
        self.collisions = 0

    def _blocked(self, xy):
        angles = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        points = np.array(xy) + RADIUS * np.c_[np.cos(angles), np.sin(angles)]
        cells = np.floor(points / RESOLUTION).astype(int)
        if np.any(cells < 0) or np.any(cells >= SIZE):
            return True
        return bool(self.obstacles[cells[:, 1], cells[:, 0]].any())

    def move_and_encode(self, v, omega):
        wheels = np.array([v - omega * TRACK / 2, v + omega * TRACK / 2])
        for _ in range(3):
            proposed = integrate(self.pose, wheels, DT / 3)
            if self._blocked(proposed[:2]):
                self.collisions += 1
                self.pose[2] = proposed[2]
                break
            self.pose = proposed
        # 固定轮径比例偏差和独立编码器噪声；离线标定训练使用同一台底座。
        return wheels * np.array([1.07, 0.94]) + self.rng.normal(0, 0.0015, 2)

    def scan(self):
        distances = np.arange(0.03, MAX_RANGE + 0.03, 0.03)
        a = self.pose[2] + ANGLES
        x = self.pose[0] + np.cos(a)[:, None] * distances
        y = self.pose[1] + np.sin(a)[:, None] * distances
        cx = np.clip((x / RESOLUTION).astype(int), 0, SIZE - 1)
        cy = np.clip((y / RESOLUTION).astype(int), 0, SIZE - 1)
        hit = self.obstacles[cy, cx]
        first = hit.argmax(axis=1)
        values = np.where(hit.any(axis=1), distances[first], MAX_RANGE)
        return np.clip(values + self.rng.normal(0, 0.007, len(ANGLES)), 0.03, MAX_RANGE)


class OccupancyMap:
    """未知栅格从零对数概率开始；最大量程束不产生障碍端点。"""

    def __init__(self, shape=(SIZE, SIZE), resolution=RESOLUTION):
        self.logodds = np.zeros(shape, dtype=float)
        self.observed = np.zeros(shape, dtype=bool)
        self.resolution = resolution

    @property
    def occupied(self):
        return self.logodds > 0.65

    def update(self, pose, ranges, angles=ANGLES):
        d = np.arange(0.0, MAX_RANGE, self.resolution * 0.4)
        valid = d[None, :] < np.maximum(ranges[:, None] - 0.08, 0)
        a = pose[2] + np.asarray(angles)
        xx = pose[0] + np.cos(a)[:, None] * d
        yy = pose[1] + np.sin(a)[:, None] * d
        free_x = np.floor(xx[valid] / self.resolution).astype(int)
        free_y = np.floor(yy[valid] / self.resolution).astype(int)
        h, w = self.logodds.shape
        keep = (free_x >= 0) & (free_y >= 0) & (free_x < w) & (free_y < h)
        free = np.unique(free_y[keep] * w + free_x[keep])
        hits = ranges < MAX_RANGE - 0.04
        hx = np.floor((pose[0] + np.cos(a[hits]) * ranges[hits]) / self.resolution).astype(int)
        hy = np.floor((pose[1] + np.sin(a[hits]) * ranges[hits]) / self.resolution).astype(int)
        keep = (hx >= 0) & (hy >= 0) & (hx < w) & (hy < h)
        occupied = np.unique(hy[keep] * w + hx[keep])
        free = np.setdiff1d(free, occupied, assume_unique=True)
        self.logodds.flat[free] -= 0.35
        self.logodds.flat[occupied] += 0.95
        self.logodds[:] = np.clip(self.logodds, -4, 4)
        self.observed.flat[free] = True
        self.observed.flat[occupied] = True


class ScanSlam:
    """输入仅编码器与激光；先对历史地图配准，再加入本帧扫描。"""

    def __init__(self, calibration, initial_prior):
        self.calibration = calibration
        self.pose = np.array(initial_prior, dtype=float)
        self.raw_odom = self.pose.copy()
        self.map = OccupancyMap()
        self.corrections = 0

    def observe(self, encoders, ranges, initial=False):
        if not initial:
            self.raw_odom = integrate(self.raw_odom, encoders)
            with torch.no_grad():
                wheels = self.calibration(torch.tensor(encoders[None], dtype=torch.float32)).numpy()[0]
            self.pose = integrate(self.pose, wheels)
        occupied = self.map.occupied
        hits = ranges < MAX_RANGE - 0.04
        # 似然场的有限差分搜索，所有评分来自上一时刻的估计地图。
        if occupied.sum() > 20 and hits.sum() > 15:
            field = distance_transform_edt(~occupied) * RESOLUTION
            local = ranges[hits, None] * np.c_[np.cos(ANGLES[hits]), np.sin(ANGLES[hits])]
            prediction = self.pose.copy()

            def score(candidate):
                c, s = np.cos(candidate[2]), np.sin(candidate[2])
                points = local @ np.array([[c, s], [-s, c]]) + candidate[:2]
                coords = np.array([points[:, 1] / RESOLUTION - 0.5,
                                   points[:, 0] / RESOLUTION - 0.5])
                distances = map_coordinates(field, coords, order=1, mode="constant", cval=1.0)
                regularization = 0.12 * np.linalg.norm(candidate[:2] - prediction[:2])
                regularization += 0.08 * abs(wrap(candidate[2] - prediction[2]))
                return float(np.minimum(distances, 0.45).mean() + regularization)

            best = prediction.copy()
            best_score = score(best)
            for dimension, delta in ((0, 0.018), (1, 0.018), (2, 0.010)):
                for sign in (-1, 1):
                    candidate = best.copy()
                    candidate[dimension] += sign * delta
                    value = score(candidate)
                    if value + 0.0005 < best_score:
                        best, best_score = candidate, value
            if np.linalg.norm(best - prediction) > 1e-6:
                self.corrections += 1
            self.pose = best
            self.pose[2] = wrap(self.pose[2])
        self.map.update(self.pose, ranges)
        return self.pose.copy()


class ValueIterationNetwork(nn.Module):
    """可训练卷积代价网络 + 四邻移位/max 的递归 Bellman 层。"""

    def __init__(self):
        super().__init__()
        self.cost = nn.Sequential(nn.Conv2d(1, 4, 3, padding=1), nn.ReLU(), nn.Conv2d(4, 1, 1))

    def forward(self, obstacles, goals, iterations=None):
        cost = F.softplus(self.cost(obstacles)) + 0.05
        value = torch.full_like(obstacles, -100.0)
        value = torch.where(goals > 0.5, torch.zeros_like(value), value)
        for _ in range(iterations or (obstacles.shape[-1] + obstacles.shape[-2])):
            padded = F.pad(value, (1, 1, 1, 1), value=-100.0)
            neighbors = torch.stack((padded[:, :, 1:-1, 2:], padded[:, :, 1:-1, :-2],
                                     padded[:, :, 2:, 1:-1], padded[:, :, :-2, 1:-1]))
            value = neighbors.max(dim=0).values - cost
            value = torch.where(obstacles > 0.5, torch.full_like(value, -100.0), value)
            value = torch.where(goals > 0.5, torch.zeros_like(value), value)
        return value

    def plan(self, estimated_map, pose, goal):
        obstacles = binary_dilation(estimated_map.occupied, iterations=1)
        obstacles[[0, -1], :] = True
        obstacles[:, [0, -1]] = True
        sy, sx = np.clip((pose[1::-1] / RESOLUTION).astype(int), 1, SIZE - 2)
        gy, gx = np.clip((goal[::-1] / RESOLUTION).astype(int), 1, SIZE - 2)
        obstacles[sy, sx] = False
        obstacles[gy, gx] = False
        goals = np.zeros((SIZE, SIZE), dtype=np.float32)
        goals[gy, gx] = 1
        with torch.no_grad():
            value = self(torch.tensor(obstacles[None, None], dtype=torch.float32),
                         torch.tensor(goals[None, None])).numpy()[0, 0]
        choices = []
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            yy, xx = sy + dy, sx + dx
            if not (0 <= yy < SIZE and 0 <= xx < SIZE) or obstacles[yy, xx]:
                continue
            if dx and dy and (obstacles[sy, xx] or obstacles[yy, sx]):
                continue
            point = (np.array([xx, yy]) + 0.5) * RESOLUTION
            heading = math.atan2(point[1] - pose[1], point[0] - pose[0])
            score = value[yy, xx] - 0.06 * abs(wrap(heading - pose[2]))
            choices.append((score, point))
        if not choices:
            return pose[:2].copy(), value
        return max(choices, key=lambda item: item[0])[1], value


def _fit_mlp(model, x, y, split, epochs=130):
    train, validation, test = split
    x, y = torch.tensor(x, dtype=torch.float32), torch.tensor(y, dtype=torch.float32)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.012)
    history = []
    for epoch in range(epochs):
        optimizer.zero_grad()
        loss = F.mse_loss(model(x[train]), y[train])
        loss.backward()
        optimizer.step()
        if epoch % 5 == 0 or epoch == epochs - 1:
            with torch.no_grad():
                history.append({"epoch": epoch + 1, "train_mse": float(loss),
                                "validation_mse": float(F.mse_loss(model(x[validation]), y[validation]))})
    with torch.no_grad():
        test_mse = float(F.mse_loss(model(x[test]), y[test]))
    return history, test_mse


def train_models(seed=21):
    """独立序列标定、独立控制样本，以及按完整地图划分的价值网络训练。"""
    torch.manual_seed(seed)
    torch.set_num_threads(1)
    rng = np.random.default_rng(seed)
    wheels = rng.uniform(-0.40, 0.50, (32, 160, 2)).astype(np.float32)
    encoders = wheels * np.array([1.07, 0.94]) + rng.normal(0, 0.0015, wheels.shape)
    ids = np.repeat(np.arange(32), 160)
    split = [np.flatnonzero(ids < 22), np.flatnonzero((ids >= 22) & (ids < 27)), np.flatnonzero(ids >= 27)]
    calibration = nn.Sequential(nn.Linear(2, 24), nn.Tanh(), nn.Linear(24, 2))
    calibration_history, calibration_test = _fit_mlp(calibration, encoders.reshape(-1, 2), wheels.reshape(-1, 2), split, 160)
    angle = rng.uniform(-np.pi, np.pi, 6000)
    distance = rng.uniform(0, 0.6, 6000)
    clearance = rng.uniform(0.2, 2.0, 6000)
    x = np.c_[np.sin(angle), np.cos(angle), distance / 0.6, clearance / 2]
    y = np.c_[0.36 * np.maximum(np.cos(angle), 0) ** 2 * np.minimum(distance / 0.25, 1)
              * np.clip((clearance - 0.18) / 0.4, 0, 1), 2.2 * np.sin(angle)]
    control = nn.Sequential(nn.Linear(4, 40), nn.Tanh(), nn.Linear(40, 24), nn.Tanh(), nn.Linear(24, 2))
    split_control = [np.arange(4200), np.arange(4200, 5100), np.arange(5100, 6000)]
    control_history, control_test = _fit_mlp(control, x, y, split_control, 150)
    maps, goals, targets = [], [], []
    for index in range(20):
        grid = np.zeros((12, 12), dtype=bool)
        grid[[0, -1], :] = True
        grid[:, [0, -1]] = True
        for _ in range(4):
            xx, yy = rng.integers(2, 9, 2)
            grid[yy:yy + 2, xx:xx + 2] = True
        goal = (10, 10)
        grid[goal] = False
        goals_map = np.zeros_like(grid, dtype=np.float32)
        goals_map[goal] = 1
        maps.append(grid)
        goals.append(goals_map)
        targets.append(-distance_labels(grid, goal))
    maps = torch.tensor(np.array(maps)[:, None], dtype=torch.float32)
    goals = torch.tensor(np.array(goals)[:, None])
    targets = torch.tensor(np.array(targets)[:, None])
    planner = ValueIterationNetwork()
    optimizer = torch.optim.Adam(planner.parameters(), lr=0.018)
    history = []
    mask = (targets > -99) & (maps < 0.5)
    for epoch in range(35):
        optimizer.zero_grad()
        predicted = planner(maps[:14], goals[:14])
        loss = ((predicted - targets[:14])[mask[:14]] ** 2).mean()
        loss.backward()
        optimizer.step()
        if epoch % 3 == 0 or epoch == 34:
            with torch.no_grad():
                val = planner(maps[14:17], goals[14:17])
                validation = ((val - targets[14:17])[mask[14:17]] ** 2).mean()
            history.append({"epoch": epoch + 1, "train_mse": float(loss), "validation_mse": float(validation)})
    with torch.no_grad():
        test = planner(maps[17:], goals[17:])
        planner_test = float(((test - targets[17:])[mask[17:]] ** 2).mean())
    metrics = {
        "wheel_calibration": {"samples": len(ids), "train_sequences": 22, "validation_sequences": 5,
                              "test_sequences": 5, "test_mse_m2_s2": calibration_test, "history": calibration_history},
        "motion_controller": {"samples": len(x), "train_samples": 4200, "validation_samples": 900,
                              "test_samples": 900, "test_mse": control_test, "history": control_history},
        "neural_value_iteration": {"train_maps": 14, "validation_maps": 3, "test_maps": 3,
                                   "test_mse_cells2": planner_test, "history": history},
    }
    return calibration.eval(), planner.eval(), control.eval(), metrics


def run_scene(spec, calibration, planner, control, seed):
    world = DifferentialWorld(spec["grid"], spec["start"], seed)
    slam = ScanSlam(calibration, spec["start"].copy())
    ranges = world.scan()
    slam.observe(np.zeros(2), ranges, initial=True)
    records, snapshots = [], []
    waypoint = spec["start"][:2].copy()
    protection = 0
    for step in range(1100):
        if step % 4 == 0 or np.linalg.norm(waypoint - slam.pose[:2]) < 0.12:
            waypoint, _ = planner.plan(slam.map, slam.pose, spec["goal"])
        delta = waypoint - slam.pose[:2]
        error = wrap(math.atan2(delta[1], delta[0]) - slam.pose[2])
        front = float(ranges[np.abs(ANGLES) < 0.35].min())
        features = np.array([[np.sin(error), np.cos(error), min(np.linalg.norm(delta), 0.6) / 0.6,
                              min(front, 2) / 2]], dtype=np.float32)
        with torch.no_grad():
            command = control(torch.tensor(features)).numpy()[0]
        v, omega = float(np.clip(command[0], 0, 0.38)), float(np.clip(command[1], -2.3, 2.3))
        if np.cos(error) < 0.2:
            v = 0.0
        if front < RADIUS + 0.12:
            v = 0.0
            protection += 1
        encoders = world.move_and_encode(v, omega)
        ranges = world.scan()
        slam.observe(encoders, ranges)
        truth = world.pose.copy()  # 以下真值只用于记录、停止验收及最终评价。
        records.append([DT * (step + 1), *truth, *slam.pose, *slam.raw_odom, v, omega,
                        np.linalg.norm(truth[:2] - slam.pose[:2]), np.linalg.norm(truth[:2] - slam.raw_odom[:2])])
        if step % 10 == 0:
            snapshots.append((np.array(records), slam.map.logodds.copy(), step))
        if np.linalg.norm(slam.pose[:2] - spec["goal"]) < 0.25:
            break
    trajectory = np.asarray(records)
    predicted = slam.map.occupied
    actual = spec["grid"]
    iou = (predicted & actual).sum() / max(1, (predicted | actual).sum())
    observed_iou = (predicted & actual & slam.map.observed).sum() / max(1, ((predicted | actual) & slam.map.observed).sum())
    result = {"name": spec["name"], "goal_reached": bool(np.linalg.norm(world.pose[:2] - spec["goal"]) < 0.35),
              "collisions": world.collisions, "steps": len(records), "mapIoU": float(iou),
              "observed_mapIoU": float(observed_iou), "map_coverage": float(slam.map.observed.mean()),
              "poseRMSE": float(np.sqrt(np.mean(trajectory[:, -2] ** 2))),
              "odomRMSE": float(np.sqrt(np.mean(trajectory[:, -1] ** 2))),
              "path_length_m": float(np.linalg.norm(np.diff(trajectory[:, 1:3], axis=0), axis=1).sum()),
              "final_goal_error_m": float(np.linalg.norm(world.pose[:2] - spec["goal"])),
              "scan_to_map_corrections": slam.corrections, "laser_stop_count": protection}
    return result, trajectory, slam.map, snapshots


def _font(size=18):
    for path in ("C:/Windows/Fonts/msyh.ttc", "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"):
        if Path(path).is_file():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def render_frame(spec, trajectory, logodds, title, size=384):
    image = Image.new("RGB", (size * 2, size + 62), "white")
    draw = ImageDraw.Draw(image)
    scale = size / SIZE
    for side in range(2):
        for y in range(SIZE):
            for x in range(SIZE):
                value = spec["grid"][y, x] if side == 0 else logodds[y, x]
                color = (45, 53, 65) if value > (0.5 if side == 0 else 0.65) else (248, 250, 252)
                if side and abs(value) < 0.05:
                    color = (190, 196, 205)
                draw.rectangle((side * size + x * scale, 54 + (SIZE - 1 - y) * scale,
                                side * size + (x + 1) * scale, 54 + (SIZE - y) * scale), fill=color)
        columns = (1, 2) if side == 0 else (4, 5)
        points = [(side * size + row[columns[0]] / RESOLUTION * scale,
                   54 + size - row[columns[1]] / RESOLUTION * scale) for row in trajectory]
        if len(points) > 1:
            draw.line(points, fill=(0, 145, 115) if side == 0 else (30, 100, 210), width=3)
        if points:
            px, py = points[-1]
            draw.ellipse((px - 6, py - 6, px + 6, py + 6), fill=(220, 80, 30))
            # 底座前方的双指图标表示承载的肌肉夹爪。
            draw.line((px + 4, py - 7, px + 12, py - 7), fill=(30, 140, 50), width=3)
            draw.line((px + 4, py + 7, px + 12, py + 7), fill=(30, 140, 50), width=3)
        gx, gy = spec["goal"] / RESOLUTION * scale
        draw.ellipse((side * size + gx - 6, 54 + size - gy - 6, side * size + gx + 6, 54 + size - gy + 6), outline=(220, 25, 65), width=3)
    draw.text((12, 6), title, fill=(20, 30, 45), font=_font(18))
    draw.text((12, 30), "真值工作区与实际轨迹（评价）", fill=(20, 30, 45), font=_font(15))
    draw.text((size + 12, 30), "在线估计地图与 SLAM 轨迹", fill=(20, 30, 45), font=_font(15))
    return image


def make_plots(out, runs, training):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname="C:/Windows/Fonts/msyh.ttc") if Path("C:/Windows/Fonts/msyh.ttc").is_file() else FontProperties()
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.4))
    for ax, (result, trajectory, _, _) in zip(axes, runs):
        ax.plot(trajectory[:, 0], trajectory[:, -1], label="原始里程计")
        ax.plot(trajectory[:, 0], trajectory[:, -2], label="神经标定与激光 SLAM")
        ax.set_xlabel("时间 / 秒", fontproperties=font)
        ax.set_ylabel("位置误差 / 米", fontproperties=font)
        ax.set_title(result["name"], fontproperties=font)
        ax.legend(prop=font, fontsize=8)
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out / "slam_errors.png", dpi=135)
    plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.4))
    for ax, (key, title) in zip(axes, (("wheel_calibration", "编码器标定网络"), ("motion_controller", "运动控制网络"), ("neural_value_iteration", "神经价值迭代网络"))):
        history = training[key]["history"]
        ax.semilogy([x["epoch"] for x in history], [x["train_mse"] for x in history], label="训练")
        ax.semilogy([x["epoch"] for x in history], [x["validation_mse"] for x in history], label="独立验证")
        ax.set_title(title, fontproperties=font)
        ax.set_xlabel("训练轮次", fontproperties=font)
        ax.set_ylabel("均方误差", fontproperties=font)
        ax.legend(prop=font)
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out / "slam_training.png", dpi=135)
    plt.close(fig)


def benchmark(out, seed=21):
    started = time.perf_counter()
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    calibration, planner, control, training = train_models(seed)
    torch.save({"calibration": calibration.state_dict(), "planner": planner.state_dict(),
                "controller": control.state_dict()}, out / "slam_models.pt")
    runs, frames = [], []
    for index, spec in enumerate(scenarios()):
        result, trajectory, grid, snapshots = run_scene(spec, calibration, planner, control, seed + index)
        runs.append((result, trajectory, grid, snapshots))
        with (out / f"scene_{index + 1}_trajectory.csv").open("w", newline="", encoding="utf-8-sig") as file:
            writer = csv.writer(file)
            writer.writerow(["时间秒", "真值x", "真值y", "真值航向", "估计x", "估计y", "估计航向", "里程计x", "里程计y", "里程计航向", "网络线速度", "网络角速度", "SLAM位置误差米", "里程计位置误差米"])
            writer.writerows(trajectory)
        np.savez_compressed(out / f"scene_{index + 1}_maps.npz", truth_map=spec["grid"], logodds=grid.logodds,
                            observed=grid.observed, goal=spec["goal"], initial_prior=spec["start"])
        for track, odds, step in snapshots[::2]:
            frames.append(render_frame(spec, track, odds, f"{spec['name']} · {step * DT:.1f} 秒").resize((640, 372)))
        frames.append(render_frame(spec, trajectory, grid.logodds,
                                   f"{spec['name']} · 到达目标").resize((640, 372)))
        print(f"{spec['name']}：到达={result['goal_reached']}，碰撞={result['collisions']}，位置误差={result['poseRMSE']:.3f}米，里程计误差={result['odomRMSE']:.3f}米", flush=True)
    panels = [render_frame(spec, track, grid.logodds, result["name"]) for spec, (result, track, grid, _) in zip(scenarios(), runs)]
    sheet = Image.new("RGB", (panels[0].width, panels[0].height * len(panels)), "white")
    for i, panel in enumerate(panels):
        sheet.paste(panel, (0, i * panel.height))
    sheet.save(out / "slam_navigation.png")
    frames[0].save(out / "slam_navigation.gif", save_all=True, append_images=frames[1:], duration=110, loop=0, optimize=True)
    make_plots(out, runs, training)
    metrics = {"passed": bool(all(r[0]["goal_reached"] and r[0]["collisions"] == 0 and r[0]["poseRMSE"] < r[0]["odomRMSE"] for r in runs)),
               "seed": seed, "simulation": "夹爪移动底座二维差速运动学、噪声编码器和激光射线",
               "online_inputs": ["编码器量测", "96束激光量测", "已知初始位姿先验", "指定目标坐标"],
               "planner": "卷积代价网络与递归移位最大值 Bellman 层；在线无 BFS 或 A*",
               "controller": "4-40-24-2 MLP 输出线速度和角速度；近距激光停车保护",
               "map_resolution_m": RESOLUTION, "dt_s": DT, "robot_radius_m": RADIUS,
               "training": training, "scenarios": [r[0] for r in runs],
               "runtime_seconds": time.perf_counter() - started, "gif_bytes": (out / "slam_navigation.gif").stat().st_size}
    (out / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = ["夹爪移动底座 SLAM 与神经导航运行结果", f"随机种子：{seed}"]
    for result, *_ in runs:
        summary.append(f"{result['name']}：到达目标 {result['goal_reached']}；碰撞 {result['collisions']}；地图IoU {result['mapIoU']:.3f}；SLAM位置RMSE {result['poseRMSE']:.3f}米；原始里程计RMSE {result['odomRMSE']:.3f}米。")
    summary.append(f"总用时：{metrics['runtime_seconds']:.2f}秒；验收通过：{metrics['passed']}")
    (out / "运行摘要.txt").write_text("\n".join(summary), encoding="utf-8")
    return metrics


def main():
    parser = argparse.ArgumentParser(description="夹爪移动底座 SLAM 与神经导航")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=21)
    args = parser.parse_args()
    print(json.dumps(benchmark(args.out, args.seed), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
