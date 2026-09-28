#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业二 · CARLA 传感器感知 + 给定轨迹跟踪（神经网络版）。

对应老师任务 (2)：用传感器数据进行感知（雷达、摄像头、深度摄像头），并对机器人进行
运动控制（感知 + 给定轨迹的控制）。满足"**感知、控制算法须为神经网络**"的硬性要求：

  * **感知 NN**：`nn_models.MLPClassifier`，由三路传感器特征 → 障碍类别（无目标/偏左/偏右）
  * **控制 NN**：`nn_models.MLPPolicy`（策略网络），由 (航向差, 距离) → 转向角 steer

三种模式：

  * `--mode train`   离线训练两个神经网络（仅需 numpy，**无需 CARLA**）
  * `--mode run`     连 CARLA 在线感知 + 沿给定轨迹自动驾驶
  * `--headless --demo`  离线自检取证：训练 + 合成轨迹跟踪回放，导出损失/误差曲线 PNG
                         （无需 CARLA、无需图形界面，可用于无 3D 加速的虚拟机出证据）

用法：
    # 离线训练（无需 CARLA）
    python3 main.py --mode train --epochs 300 --out models/nn_percept.json
    # 在线感知 + 轨迹跟踪
    python3 main.py --mode run --model models/nn_percept.json --waypoints "40,-8 40,12 25,20"
    # 离线取证（导出曲线图）
    python3 main.py --headless --demo --save_dir ~/shots
    # ROS 2 / ROS 1 launch
    ros2 launch carla_perception_control main.launch.py host:=<宿主机IP>
    roslaunch carla_perception_control main.launch host:=<宿主机IP>
"""

import argparse
import json
import math
import os
import struct
import sys
import zlib

import numpy as np

from carla_perception_control.nn_models import MLPClassifier, MLPPolicy

try:
    from carla_perception_control import carla_common as cc
    _HAVE_CARLA = getattr(cc, "carla", None) is not None
except Exception:  # noqa: BLE001
    cc = None
    _HAVE_CARLA = False

DT = 0.05  # 同步步进固定步长（秒）


# ============================================================ 传感器特征提取
def _feat_feature(off, depth_v, front_n, front_d):
    """把 (横向偏移, 深度密集度, 前方点数, 最近距离) 归一化到相近尺度。

    归一化对神经网络训练至关重要：各特征量纲差异大（depth∈[0,255]、off∈[-1,1]、
    dist∈[0.5,20]），不归一化会由大数值特征主导梯度，导致小量纲特征学不到。
    """
    off_n = np.asarray(off, dtype=np.float32) / 1.0
    depth_n = np.asarray(depth_v, dtype=np.float32) / 255.0
    fn_n = np.asarray(front_n, dtype=np.float32) / 60.0
    fd_n = np.clip(np.asarray(front_d, dtype=np.float32) / 20.0, 0.0, 1.0)
    return np.stack([off_n, depth_n, fn_n, fd_n], axis=-1).astype(np.float32)


def _rgb_offset(img):
    """RGB 下半区亮色像素的横向重心，归一化到 [-1,1]（正=目标偏右）。"""
    if img is None:
        return 0.0
    h, w = img.shape[:2]
    roi = img[int(h * 0.55):, :, :].mean(axis=2)
    mask = roi > 100
    if mask.sum() == 0:
        return 0.0
    _ys, xs = np.where(mask)
    return float(np.clip((xs.mean() - w / 2.0) / (w / 2.0), -1, 1))


def _depth_density(img):
    """深度图 ROI 均值（越大表示近处障碍越密）。"""
    if img is None:
        return 0.0
    h, w = img.shape[:2]
    roi = img[int(h * 0.5):, int(w * 0.2):int(w * 0.8), :]
    return float(roi.mean())


def _lidar_front(pc):
    """雷达前方（x>0 且 |y|<3m）的点数与最近距离。"""
    if pc is None or len(pc) == 0:
        return 0, None
    x, y = pc[:, 0], pc[:, 1]
    pts = pc[(x > 0) & (np.abs(y) < 3.0)]
    if len(pts) == 0:
        return 0, None
    return len(pts), float(pts[:, 0].min())


def extract_features(rgb, depth, lidar):
    """三路传感器 → 4 维归一化特征。返回 (feat, raw)。"""
    off = _rgb_offset(rgb)
    depth_v = _depth_density(depth)
    front_n, front_d = _lidar_front(lidar)
    if front_d is None:
        front_d = 20.0  # 无障碍视为很远
    if front_n is None:
        front_n = 0
    raw = (off, depth_v, front_n, front_d)
    return _feat_feature(off, depth_v, front_n, front_d), raw


# ================================================================ 离线训练
# 车辆与转向几何参数（与离线回放、监督标签保持一致）
WHEELBASE = 2.5       # 轴距（m）
# CARLA 的 VehicleControl.steer ∈ [-1,1] 线性映射到前轮转角约 ±70°：
#   δ_max = 70° = 1.2217 rad  →  g = δ_max / 1.0
# 该折算系数取真实几何值后，纯跟踪律的标签不再饱和（|steer| 全部 < 1），
# 控制网络的回归误差显著下降。
STEER_GAIN = 1.2217   # steer(归一化) → 前轮转角(rad) 的折算系数
LOOKAHEAD = 6.0       # 前视距离（m）
GOAL_TOL = 5.0        # 终点判定阈值（m）：进入该范围视为到达并停车
MAX_SPEED = 7.2       # 稳态速度（m/s）= throttle(0.6) × 12

# 离线取证用的给定轨迹：起点与默认出生点一致（朝 +x），转弯均为缓弯。
# 注意：转弯半径受车辆运动学限制（R_min ≈ L/tan(δ_max)），原始示例路点
# (36,-5)→(40,-8)→(40,12) 要求 5 m 内转过 127°，物理上无法完成，故演示
# 采用缓弯路线；在线运行时可用 --waypoints 传入任意给定轨迹。
DEMO_ROUTE = [(36.0, -5.0), (80.0, -5.0), (120.0, -5.0), (150.0, 10.0), (160.0, 40.0)]


def pure_pursuit_law(e_psi, dist, wheelbase=None, steer_gain=None):
    """纯跟踪几何律：由航向差与前视距离解析算出转向量。

    $$
    \\delta = \\arctan\\!\\Big(\\frac{2 L \\sin e_\\psi}{d}\\Big),\\qquad
    \\text{steer} = \\mathrm{clip}\\!\\Big(\\frac{\\delta}{g},\\,-1,\\,1\\Big)
    $$

    其中 $L$ 为轴距、$d$ 为前视距离、$g$ 为 CARLA `steer` 到前轮转角的折算系数。
    **该式同时用作控制神经网络的监督标签**：网络学习「状态 → 转向」这一映射后，
    即可在在线运行时直接推理，无需再解析求解。

    注意：`wheelbase` / `steer_gain` 的默认值在**函数体内**取模块常量，而不是写成
    默认参数值——默认参数只在函数定义时求值一次，之后修改模块常量将不再生效。
    """
    if wheelbase is None:
        wheelbase = WHEELBASE
    if steer_gain is None:
        steer_gain = STEER_GAIN
    d = max(float(dist), 0.5)  # 避免除零
    delta = math.atan2(2.0 * wheelbase * math.sin(float(e_psi)), d)
    return float(np.clip(delta / steer_gain, -1.0, 1.0))


def synth_dataset(n=400, seed=0):
    """生成合成训练数据（无需 CARLA），构造"传感器特征 → 感知/控制"的监督信号。

    返回 (feat_X, feat_Yc, ctrl_X, ctrl_Y)：
      * 感知任务：横向偏移 < -0.2 记为「偏左」(1)，> 0.2 记为「偏右」(2)，否则「无目标」(0)
      * 控制任务：标签由**纯跟踪几何律** `pure_pursuit_law(e_ψ, d)` 生成，
        因此航向差为 0 时标签恰为 0（直线行驶不产生转向），符合真实驾驶逻辑。
    """
    rng = np.random.RandomState(seed)
    off_true = rng.uniform(-1, 1, n)
    depth_r = rng.uniform(0, 255, n)
    front_n = rng.randint(0, 60, n)
    front_d = rng.uniform(0.5, 20, n)

    cls = np.zeros(n, dtype=int)
    cls[off_true < -0.2] = 1
    cls[off_true > 0.2] = 2
    feat_X = _feat_feature(off_true, depth_r, front_n, front_d)

    # 控制标签：航向差覆盖 [-π, π]，前视距离覆盖实际用到的范围
    e_psi = rng.uniform(-math.pi, math.pi, n)
    dist = rng.uniform(3.0, 15.0, n)
    steer_true = np.array([pure_pursuit_law(e, d) for e, d in zip(e_psi, dist)],
                          dtype=np.float32)
    ctrl_X = np.stack([e_psi, dist], axis=1).astype(np.float32)
    return feat_X, cls, ctrl_X, steer_true


def train(feat_X, feat_Yc, ctrl_X, ctrl_Y, epochs=300, verbose=None):
    """训练感知分类器与策略网络，返回 (sens, ctrl, hist)。

    控制网络的输入是 (e_ψ, d) 两维**量纲差异大**的状态（角度∈[-π,π]、距离∈[3,15]），
    因此在训练前按各维标准差做标准化，训练/推理时使用同一组均值方差，保证一致。
    """
    if verbose is None:
        verbose = max(1, epochs // 5)

    sens = MLPClassifier([4, 12, 3], seed=0)
    print("== 训练感知 NN：4 维传感器特征 → 3 类障碍方向 ==")
    h1 = sens.train(feat_X, feat_Yc, epochs=epochs, lr=0.2, verbose=verbose)

    # 控制网络：先标准化输入，再训练（提升回归精度）
    ctrl_X = np.asarray(ctrl_X, dtype=np.float32)
    mu = ctrl_X.mean(axis=0, keepdims=True)
    sd = ctrl_X.std(axis=0, keepdims=True) + 1e-6
    ctrl_Xn = (ctrl_X - mu) / sd

    ctrl = MLPPolicy([2, 32, 1], seed=1)
    print("== 训练控制 NN：2 维状态(航向差,前视距离) → 转向角 ==")
    h2 = ctrl.train(ctrl_Xn, ctrl_Y.reshape(-1, 1), epochs=epochs, lr=0.2, verbose=verbose)
    ctrl.input_mu = mu
    ctrl.input_sd = sd
    return sens, ctrl, {"sens": h1, "ctrl": h2}


# ============================================================ 控制（NN / 对比）
def pure_pursuit(pos, yaw, waypoints, lookahead=2.0, gain=1.5, base=0.6):
    """纯跟踪算法（作为 NN 控制器的对比基准）。返回 (throttle, steer)。"""
    cur = np.array(pos)
    d = [np.hypot(p[0] - cur[0], p[1] - cur[1]) for p in waypoints]
    idx = int(np.argmin(d))
    target = waypoints[min(idx + 1, len(waypoints) - 1)]
    dx, dy = target[0] - cur[0], target[1] - cur[1]
    diff = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
    return base, float(np.clip(diff * gain, -1, 1))


# ============================================================ 路点与误差度量
def interpolate_waypoints(waypoints, step=2.0):
    """把稀疏路点按固定间距加密（线性插值）。

    给定轨迹的原始路点可能相隔十几米，直接用它做前视追踪会"跳点"、用最近路点
    距离做误差度量也会虚高。加密到约 step 米一个点后，两个问题都能解决。
    """
    if len(waypoints) < 2:
        return list(waypoints)
    out = [tuple(waypoints[0])]
    for (x0, y0), (x1, y1) in zip(waypoints[:-1], waypoints[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(seg / step))
        for i in range(1, n + 1):
            t = i / n
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    return out


def _dist_to_polyline(pos, waypoints):
    """点到路点折线的**最短垂距**（真正的横向误差）。

    比"到最近路点的距离"准确得多：当车位于两个稀疏路点之间时，最近路点距离会
    高达半个路段长，而垂距才反映真实的轨迹偏离程度。
    """
    px, py = float(pos[0]), float(pos[1])
    best = float("inf")
    for (x0, y0), (x1, y1) in zip(waypoints[:-1], waypoints[1:]):
        dx, dy = x1 - x0, y1 - y0
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-12:
            d = math.hypot(px - x0, py - y0)
        else:
            t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / seg2))
            d = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
        best = min(best, d)
    return best


def _goal_state(pos, yaw, waypoints, lookahead=6.0):
    """由当前位置/航向与路点序列给出 (航向差 e_ψ, 目标点距离 d, 目标点索引)。

    采用**前视点**策略（纯跟踪思想）：不只选最近路点，而是沿路点序列向前找到
    第一个距车**至少 lookahead 米**的路点作为追踪目标。前视距离过短会让车在
    高速下"冲过"路点来回振荡，过长则切弯。默认 6 m 适配 3~6 m/s 的车速。
    """
    cur = np.array(pos, dtype=np.float64)
    dists = [float(np.hypot(p[0] - cur[0], p[1] - cur[1])) for p in waypoints]
    # 1) 先找最近路点，作为搜索起点
    idx = int(np.argmin(dists))
    # 2) 从最近点向前找第一个距离 >= lookahead 的点（若都不够远则取最后一个）
    target_idx = len(waypoints) - 1
    for j in range(idx, len(waypoints)):
        if dists[j] >= lookahead:
            target_idx = j
            break
    target = waypoints[target_idx]
    dx, dy = target[0] - cur[0], target[1] - cur[1]
    diff = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
    return diff, float(math.hypot(dx, dy)), target_idx


def _ctrl_input(ctrl_net, diff, dist):
    """构造控制网络输入：与训练时使用同一组均值/方差做标准化。"""
    x = np.array([[diff, dist]], dtype=np.float32)
    mu = getattr(ctrl_net, "input_mu", None)
    sd = getattr(ctrl_net, "input_sd", None)
    if mu is not None and sd is not None:
        x = (x - mu) / sd
    return x


def nn_control(pos, yaw, waypoints, ctrl_net, throttle=0.6, lookahead=LOOKAHEAD):
    """神经网络控制：把状态 (航向差, 前视距离) 喂给策略网算出 steer。

    转向由控制神经网络给出；**油门按航向差自适应**——偏差大（弯道）时减速，
    偏差小（直道）时全速。这与真实驾驶一致，也能避免高速入弯冲出轨迹：
    车辆最小转弯半径随速度增大而增大，不减速就必然转不过来。
    """
    diff, dist, _idx = _goal_state(pos, yaw, waypoints, lookahead)
    steer = float(ctrl_net.predict(_ctrl_input(ctrl_net, diff, dist))[0, 0])
    # 弯道减速：|e_ψ|=0 时全油门，|e_ψ|=π 时降到 25%
    throttle_eff = float(throttle) * max(0.25, 1.0 - 0.75 * abs(diff) / math.pi)
    return throttle_eff, float(np.clip(steer, -1, 1))


# ============================================================== 模型存取
def save_model(path, sens, ctrl):
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    payload = {
        "sens": {"layers": sens.layers,
                 "W": [w.tolist() for w in sens.W], "b": [t.tolist() for t in sens.b]},
        "ctrl": {"layers": ctrl.layers,
                 "W": [w.tolist() for w in ctrl.W], "b": [t.tolist() for t in ctrl.b],
                 # 控制网络输入的标准化参数，推理时必须沿用
                 "input_mu": np.asarray(getattr(ctrl, "input_mu", [[0.0, 0.0]])).tolist(),
                 "input_sd": np.asarray(getattr(ctrl, "input_sd", [[1.0, 1.0]])).tolist()},
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f)
    print("模型已保存:", path)


def load_model(path):
    with open(path, encoding="utf-8") as f:
        obj = json.load(f)
    sens = MLPClassifier(obj["sens"]["layers"])
    sens.W = [np.asarray(w, dtype=np.float32) for w in obj["sens"]["W"]]
    sens.b = [np.asarray(t, dtype=np.float32) for t in obj["sens"]["b"]]
    ctrl = MLPPolicy(obj["ctrl"]["layers"])
    ctrl.W = [np.asarray(w, dtype=np.float32) for w in obj["ctrl"]["W"]]
    ctrl.b = [np.asarray(t, dtype=np.float32) for t in obj["ctrl"]["b"]]
    ctrl.input_mu = np.asarray(obj["ctrl"].get("input_mu", [[0.0, 0.0]]), dtype=np.float32)
    ctrl.input_sd = np.asarray(obj["ctrl"].get("input_sd", [[1.0, 1.0]]), dtype=np.float32)
    return {"sens": sens, "ctrl": ctrl}


# ============================================================ PNG 导出（取证用）
def _write_png(path, rgb):
    """纯 zlib 写 PNG（不依赖 pygame/Pillow/opencv），rgb 为 (H,W,3) uint8。"""
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "wb") as f:
        f.write(png)


def _draw_curves(path, series, width=640, height=360, title=None):
    """把若干条曲线画成 PNG。series = [(name, values, (r,g,b)), ...]。"""
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    # 边框
    canvas[10:height - 10, 10:12] = 200
    canvas[10:height - 10, width - 12:width - 10] = 200
    canvas[10:12, 10:width - 10] = 200
    canvas[height - 12:height - 10, 10:width - 10] = 200
    allv = np.concatenate([np.asarray(v, dtype=np.float32).ravel() for _n, v, _c in series])
    vmin, vmax = float(np.min(allv)), float(np.max(allv))
    if vmax - vmin < 1e-9:
        vmax = vmin + 1.0
    x0, x1 = 20, width - 20
    y0, y1 = 20, height - 20
    for _name, values, color in series:
        v = np.asarray(values, dtype=np.float32).ravel()
        if v.size < 2:
            continue
        xs = np.linspace(x0, x1, v.size).astype(int)
        ys = (y1 - (v - vmin) / (vmax - vmin) * (y1 - y0)).astype(int)
        for i in range(v.size - 1):
            _line(canvas, xs[i], ys[i], xs[i + 1], ys[i + 1], color)
    _write_png(path, canvas)
    return path


def _line(canvas, x0, y0, x1, y1, color):
    """Bresenham 画线（用于曲线导出）。"""
    h, w = canvas.shape[:2]
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        if 0 <= x0 < w and 0 <= y0 < h:
            canvas[y0, x0] = color
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


# ======================================================== 离线取证（无需 CARLA）
def run_offline_demo(save_dir, epochs=200, sim_time=20.0, seed=0):
    """离线自检：训练两个 NN，再在合成车辆运动学模型上沿给定轨迹跟踪并导出曲线。

    **不需要 CARLA、不需要图形界面**，用于在无 3D 加速的虚拟机/CI 中产出可运行性证据。
    导出：感知损失曲线、控制损失曲线、横向误差曲线。
    """
    print("=" * 62)
    print("  作业二 离线自检取证模式（合成数据，不需要 CARLA）")
    print("=" * 62)

    raw_wps = DEMO_ROUTE
    waypoints = interpolate_waypoints(raw_wps, step=2.0)
    print(f"给定轨迹：{len(raw_wps)} 个原始路点 → 加密为 {len(waypoints)} 个（约 2 m 间距）")

    feat_X, feat_Yc, ctrl_X, ctrl_Y = synth_dataset(400, seed=seed)
    sens, ctrl, hist = train(feat_X, feat_Yc, ctrl_X, ctrl_Y, epochs=epochs)

    acc = float(np.mean(sens.predict(feat_X) == feat_Yc))
    mse = float(np.mean(np.square(
        ctrl.predict((ctrl_X - ctrl.input_mu) / ctrl.input_sd).ravel() - ctrl_Y)))
    print(f"\n感知 NN 训练集准确率 = {acc:.3f}")
    print(f"控制 NN 训练集 MSE   = {mse:.5f}")

    # ---- 合成车辆运动学回放：沿路点用控制 NN 跟踪 ----
    # 车辆运动学与转向几何参数统一取自模块常量，保证"训练用的标签"与
    # "回放时车辆实际响应"出自同一套几何模型，横向误差才有意义。
    x, y, yaw = waypoints[0][0], waypoints[0][1], 0.0
    goal = waypoints[-1]
    v = 0.0
    lateral, steers, speeds = [], [], []
    reached_at = None
    n = int(sim_time / DT)
    for k in range(n):
        # 终点判定：进入 goal_tol 米内视为到达，停车结束（否则会一直绕终点打转）
        if math.hypot(goal[0] - x, goal[1] - y) < GOAL_TOL:
            reached_at = k
            print(f"\n[到达] t={k * DT:.1f}s 抵达终点 ({goal[0]:.1f},{goal[1]:.1f})，停车。")
            break

        throttle, steer = nn_control((x, y), yaw, waypoints, ctrl)
        steers.append(steer)
        # 一阶速度响应 + 自行车模型航向更新（教学用简化模型）
        v += (throttle * 12.0 - v) * 0.1
        yaw += v * math.tan(steer * STEER_GAIN) / WHEELBASE * DT
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        speeds.append(v)
        lateral.append(_dist_to_polyline((x, y), waypoints))
        if k % 40 == 0:
            diff, dist, _i = _goal_state((x, y), yaw, waypoints)
            print(f"[t={k * DT:5.1f}s] pos=({x:6.1f},{y:6.1f}) yaw={math.degrees(yaw):+6.1f}° "
                  f"v={v:5.2f}m/s e_psi={diff:+.2f} 前视距={dist:5.2f} steer={steer:+.2f} "
                  f"横向误差={lateral[-1]:.2f}m")

    rmse = float(np.sqrt(np.mean(np.square(lateral)))) if lateral else float("nan")
    print(f"\n轨迹跟踪完成：横向误差 RMSE = {rmse:.3f} m（峰值 {max(lateral):.3f} m）"
          if lateral else "\n未产生轨迹数据")
    if speeds:
        print(f"平均速度 = {float(np.mean(speeds)):.2f} m/s，最大速度 = {float(np.max(speeds)):.2f} m/s")
    if reached_at is not None:
        remain = min(math.hypot(goal[0] - x, goal[1] - y) for _ in [0])
        print(f"终点残余距离 = {remain:.2f} m（判定阈值 {GOAL_TOL} m）")

    # ---- 导出曲线（可运行性证据）----
    os.makedirs(save_dir, exist_ok=True)
    p1 = _draw_curves(os.path.join(save_dir, "percept_loss.png"),
                      [("感知NN交叉熵损失", hist["sens"]["loss"], (220, 60, 60))])
    p2 = _draw_curves(os.path.join(save_dir, "control_loss.png"),
                      [("控制NN MSE", hist["ctrl"]["loss"], (60, 90, 220))])
    p3 = _draw_curves(os.path.join(save_dir, "lateral_error.png"),
                      [("横向误差(m)", lateral, (30, 150, 90))])
    p4 = _draw_curves(os.path.join(save_dir, "steer_cmd.png"),
                      [("转向指令", steers, (170, 90, 200))])
    print("\n已导出取证图：")
    for p in (p1, p2, p3, p4):
        print("   ", p)
    print(f"\n合计：感知精度 {acc:.3f}，控制 MSE {mse:.5f}，横向误差 RMSE {rmse:.3f} m")
    return 0


# ================================================================ 在线运行
def run_carla(host, port, town, model_path, waypoints, sim_time=20.0,
              use_nn_control=True, save_dir=None):
    """连 CARLA：三路传感器 → 感知 NN → 障碍类别；控制 NN / 纯跟踪 → 转向。"""
    if not _HAVE_CARLA:
        print("[错误] 缺少 carla 模块，run 模式需要 CARLA 服务端。请安装 CARLA 0.9.16 客户端。")
        print("       离线验证可改用： python3 main.py --headless --demo --save_dir ~/shots")
        return 1
    model = load_model(model_path)
    sens = model["sens"]

    # 原始路点加密，便于前视追踪与横向误差度量
    waypoints = interpolate_waypoints(waypoints, step=2.0)

    client, world = cc.connect(host, port, town)
    vehicle, tf = cc.spawn_vehicle(world)
    holder = {"rgb": None, "depth": None, "lidar": None}
    cc.make_rgb_camera(world, vehicle, lambda im: holder.__setitem__("rgb", im), tick=True)
    cc.make_depth_camera(world, vehicle, lambda d: holder.__setitem__("depth", d), tick=True)
    cc.make_lidar(world, vehicle, lambda pc: holder.__setitem__(
        "lidar", np.frombuffer(pc.raw_data, dtype=np.float32).reshape(-1, 4)), tick=True)

    print(f"[就绪] 自车@{tf.location}，NN 感知 + 控制，路点={len(waypoints)}（已加密）")
    frames_seen = 0
    shot = 0
    lateral_errors = []
    warned = False
    n_ticks = int(sim_time / DT)
    for k in range(n_ticks):
        pos = cc.get_location(vehicle)
        yaw = cc.get_yaw(vehicle)
        feat, raw = extract_features(holder["rgb"], holder["depth"], holder["lidar"])
        if holder["rgb"] is not None:
            frames_seen += 1
        elif not warned and k * DT > 3.0:
            warned = True
            print("[警告] 3 秒内未收到任何相机帧，请检查 CARLA 服务端与传感器回调。")
        cls = int(sens.predict(feat[None, :])[0])
        cls_name = {0: "无目标", 1: "偏左", 2: "偏右"}[cls]

        # 终点判定：进入阈值范围则刹车停车，避免在终点附近绕圈
        goal = waypoints[-1]
        if math.hypot(goal[0] - pos[0], goal[1] - pos[1]) < GOAL_TOL:
            cc.apply_control(vehicle, throttle=0.0, brake=1.0)
            world.tick()
            print(f"\n[到达] t={k * DT:.1f}s 抵达终点 ({goal[0]:.1f},{goal[1]:.1f})，停车。")
            break

        if use_nn_control:
            throttle, steer = nn_control(pos, yaw, waypoints, model["ctrl"])
        else:
            throttle, steer = pure_pursuit(pos, yaw, waypoints)
        cc.apply_control(vehicle, throttle=throttle, steer=steer)

        lateral_errors.append(_dist_to_polyline(pos, waypoints))

        # 取证：按间隔导出前视相机画面
        if save_dir and holder["rgb"] is not None and k % 20 == 0:
            shot += 1
            _write_png(os.path.join(save_dir, f"percept_frame_{shot:02d}.png"), holder["rgb"])

        world.tick()
        if k % 20 == 0:
            off, d_, fn, fd = raw
            print(f"[t={k * DT:.1f}] pos=({pos[0]:.1f},{pos[1]:.1f}) NN感知={cls_name} "
                  f"steer={steer:+.2f} rgb={off:+.2f} depth={d_:.0f} lidar({fn},{fd}) "
                  f"横向误差={lateral_errors[-1]:.2f}m")

    rmse = float(np.sqrt(np.mean(np.square(lateral_errors))))
    print(f"\n共收到相机帧 {frames_seen} 张，导出截图 {shot} 张")
    print(f"轨迹跟踪完成：横向误差 RMSE = {rmse:.3f} m")
    vehicle.destroy()
    return 0


# ==================================================================== CLI
def build_parser():
    p = argparse.ArgumentParser(description="CARLA 传感器感知 + 给定轨迹跟踪（神经网络版）")
    p.add_argument("--mode", choices=["train", "run"], default="run")
    p.add_argument("--host", default="127.0.0.1", help="CARLA 服务端地址（虚拟机填宿主机 IP）")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--town", default="Town05")
    p.add_argument("--waypoints", default="40,-8 40,12 25,20", help="给定轨迹路点 \"x,y x,y ...\"")
    p.add_argument("--sim_time", type=float, default=20.0)
    p.add_argument("--epochs", type=int, default=300)
    p.add_argument("--out", default="models/nn_percept.json")
    p.add_argument("--model", default="models/nn_percept.json")
    p.add_argument("--no_nn_control", action="store_true", help="改用纯跟踪作对比")
    p.add_argument("--headless", action="store_true", help="无窗口模式（离线取证）")
    p.add_argument("--demo", action="store_true", help="运行内置演示序列")
    p.add_argument("--save_dir", default=None, help="取证图/截图导出目录")
    p.add_argument("--launch", action="store_true", help="由 ROS launch 启动（等价 run）")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.headless or args.demo:
        return run_offline_demo(args.save_dir or "shots", epochs=min(args.epochs, 200),
                                sim_time=args.sim_time)

    wps = [tuple(map(float, t.split(","))) for t in args.waypoints.split()]
    if not wps:
        print("[错误] 至少需要 1 个路点")
        return 1

    if args.mode == "train":
        feat_X, feat_Y, ctrl_X, ctrl_Y = synth_dataset()
        sens, ctrl, _hist = train(feat_X, feat_Y, ctrl_X, ctrl_Y, args.epochs)
        print(f"感知 NN train_acc = {np.mean(sens.predict(feat_X) == feat_Y):.3f}")
        mse = np.mean(np.square(
            ctrl.predict((ctrl_X - ctrl.input_mu) / ctrl.input_sd).ravel() - ctrl_Y))
        print(f"控制 NN 最终 MSE  = {mse:.4f}")
        save_model(args.out, sens, ctrl)
        return 0

    return run_carla(args.host, args.port, args.town, args.model, wps, args.sim_time,
                     use_nn_control=not args.no_nn_control, save_dir=args.save_dir)


if __name__ == "__main__":
    sys.exit(main())
