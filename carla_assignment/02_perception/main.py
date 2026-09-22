#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业二 · CARLA 传感器感知 + 给定轨迹跟踪（神经网络版）。

对应老师任务 (2) 与"算法需为神经网络"的硬性要求：
- **感知 NN**：`nn_models.MLPClassifier` 由传感器特征 → 障碍类别 + 横向偏移
- **控制 NN**：`nn_models.MLPPolicy` 由 (航向差, 距离) → steer（策略网络）

两种数据来源：
  * RGB/深度相机 + 雷达三路传感器（在线，连 CARLA）
  * 离线合成特征（train 模式，仅 numpy，无需 CARLA）

两种模式：
  * `--mode train`：离线训练感知/控制 NN，保存到 json（可在无 CARLA 的本机执行）
  * `--mode run`  ：加载 NN，连 CARLA 在线感知 + 沿给定轨迹自动驾驶

用法：
    # 在线：连 CARLA 自动驾驶，感知与转向均由 NN 给出
    python 02_perception/main.py --mode run --model nn_model.json --waypoints "40,-8 40,12 25,20"

    # 离线训练（无需 CARLA）
    python 02_perception/main.py --mode train --out nn_model.json --epochs 300

ROS launch：见 launch/（main.launch / main.launch.py）
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from nn_models import MLPClassifier, MLPPolicy  # noqa: E402

try:  # CARLA 可选：train 模式不依赖它
    from common import (  # noqa: F401
        connect, spawn_vehicle, make_rgb_camera, make_depth_camera, make_lidar,
        apply_control, get_location, get_yaw, get_speed, _tick_once,
        DEFAULT_HOST, DEFAULT_PORT, DEFAULT_MAP, DT,
    )
    _HAVE_CARLA = True
except Exception:  # noqa: BLE001
    _HAVE_CARLA = False


# ---------------------------------------------------------------- 特征提取
def extract_features(rgb, depth, lidar):
    """把三路传感器压缩为固定长度特征向量（供感知 NN 输入）。

    返回 (feat, raw)：feat 为 (4,) 归一化特征，raw 为各感知的原始标量（供可视化/HUD）。
    """
    off = _rgb_offset(rgb)
    depth_v = _depth_density(depth)
    front_n, front_d = _lidar_front(lidar)
    if front_d is None:
        front_d = 20.0  # 无障碍当作很远
    if front_n is None:
        front_n = 0
    return _feat_feature(off, depth_v, front_n, front_d), (off, depth_v, front_n, front_d)


def _feat_feature(off, depth_v, front_n, front_d):
    """把 (横向偏移, 深度密集度, 前方点数, 最近距离) 归一化到相近尺度。

    归一化对神经网络训练很关键：各特征量纲/范围差异大（如 depth∈[0,255]、
    off∈[-1,1]、dist∈[0.5,20]），不归一化会由大数值特征主导梯度。
    """
    off_n = off / 1.0
    depth_n = depth_v / 255.0
    fn_n = front_n / 60.0
    fd_n = np.clip(front_d / 20.0, 0.0, 1.0)
    return np.stack([off_n, depth_n, fn_n, fd_n], axis=-1).astype(np.float32)


def _rgb_offset(img):
    if img is None:
        return 0.0
    h, w = img.shape[:2]
    roi = img[int(h * 0.55):, :, :].mean(axis=2)
    mask = roi > 100
    if mask.sum() == 0:
        return 0.0
    ys, xs = np.where(mask)
    return float(np.clip((xs.mean() - w / 2.0) / (w / 2.0), -1, 1))


def _depth_density(img):
    if img is None:
        return 0.0
    h, w = img.shape[:2]
    roi = img[int(h * 0.5):, int(w * 0.2):int(w * 0.8), :]
    return float(roi.mean())


def _lidar_front(pc):
    if pc is None or len(pc) == 0:
        return 0, None
    x, y = pc[:, 0], pc[:, 1]
    front = (x > 0) & (np.abs(y) < 3.0)
    pts = pc[front]
    if len(pts) == 0:
        return 0, None
    return len(pts), float(pts[:, 0].min())


# ---------------------------------------------------------------- 离线训练
def train(feat_X, feat_Yc, ctrl_X, ctrl_Y, epochs=300):
    """用合成/真实数据训练感知分类器与策略网络，返回两个 net。"""
    # 感知分类器：特征(4) -> 3 类（0=无碍,1=左障碍,2=右障碍）
    sens = MLPClassifier([4, 12, 3], seed=0)
    print("== 训练感知 NN (特征->障碍类别) ==")
    sens.train(feat_X, feat_Yc, epochs=epochs, lr=0.2)
    # 控制策略：状态(2)=[航向差,距离] -> steer
    ctrl = MLPPolicy([2, 16, 1], seed=1)
    print("== 训练控制 NN (航向差/距离->转向) ==")
    ctrl.train(ctrl_X, ctrl_Y.reshape(-1, 1), epochs=epochs, lr=0.1)
    return sens, ctrl


def synth_dataset(n=400, seed=0):
    """生成合成训练数据（无需 CARLA），模拟"传感器特征->感知/控制"。

    返回 (feat_X, feat_Yc, ctrl_X, ctrl_Y)。
    """
    rng = np.random.RandomState(seed)
    # ---- 感知：真实横向偏移（用图像offset模拟）+ 障碍类别 ----
    off_true = rng.uniform(-1, 1, n)          # 目标横向偏移
    depth_r = rng.uniform(0, 255, n)          # 深度密集度
    front_n = rng.randint(0, 60, n)           # 前方点数
    front_d = rng.uniform(0.5, 20, n)         # 最近距离
    # 类别：障碍在左（<0）=>1，偏右=>2，中性=>0
    cls = np.zeros(n, dtype=int)
    cls[off_true < -0.2] = 1
    cls[off_true > 0.2] = 2
    # 特征归一化（尺度差异大，不归一化会导致梯度被大数主导）
    feat_X = _feat_feature(off_true, depth_r, front_n, front_d)
    feat_Y = cls

    # ---- 控制：航向差 e_psi∈[-pi,pi]，距离 d>0 -> steer（tanh）----
    e_psi = rng.uniform(-math.pi, math.pi, n)
    dist = rng.uniform(0.5, 25, n)
    steer_true = np.clip(0.6 * np.tanh(1.2 * e_psi) + 0.1 * (1 - 1 / (1 + dist / 8)), -1, 1)
    ctrl_X = np.stack([e_psi, dist], axis=1).astype(np.float32)
    ctrl_Y = steer_true.astype(np.float32)
    return feat_X, feat_Y, ctrl_X, ctrl_Y


def pure_pursuit(pos, yaw, waypoints, lookahead=2.0, gain=1.5, base=0.6):
    """纯跟踪（作为 NN 控制器的对比/参考）。返回 (throttle, steer)。"""
    cur = np.array(pos)
    d = [np.hypot(p[0] - cur[0], p[1] - cur[1]) for p in waypoints]
    idx = int(np.argmin(d))
    target = waypoints[min(idx + 1, len(waypoints) - 1)]
    dx, dy = target[0] - cur[0], target[1] - cur[1]
    diff = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
    steer = float(np.clip(diff * gain, -1, 1))
    return base, steer


def nn_control(pos, yaw, waypoints, ctrl_net):
    """神经网络控制：把 (航向差, 最近距离) 喂给策略网算出 steer。"""
    cur = np.array(pos)
    d = [np.hypot(p[0] - cur[0], p[1] - cur[1]) for p in waypoints]
    idx = int(np.argmin(d))
    target = waypoints[min(idx + 1, len(waypoints) - 1)]
    dx, dy = target[0] - cur[0], target[1] - cur[1]
    diff = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
    # 神经网络前向：状态 -> steer
    steer = float(ctrl_net.predict(np.array([[diff, d[idx]]], dtype=np.float32))[0, 0])
    steer = float(np.clip(steer, -1, 1))
    return 0.6, steer


# ---------------------------------------------------------------- 在线运行
def run_carla(host, port, town, model_path, waypoints, sim_time=20.0,
              use_nn_control=True):
    """连 CARLA：三路传感器 -> 感知 NN -> 类别/偏航；控制 NN 或纯跟踪给控制。"""
    if not _HAVE_CARLA:
        print("[错误] 缺少 carla 模块，run 需要 CARLA 服务端。请先装 carla 0.9.16。")
        return 1
    # 载入感知/控制两个网络（模型文件为打包 dict {sens,ctrl}）
    model = _load_model(model_path)
    sens = model["sens"]

    client, world = connect(host, port, town)
    vehicle, tf = spawn_vehicle(world)
    holder = {"rgb": None, "depth": None, "lidar": None}
    make_rgb_camera(world, vehicle, lambda im: holder.__setitem__("rgb", im), tick=True)
    make_depth_camera(world, vehicle, lambda d: holder.__setitem__("depth", d), tick=True)
    make_lidar(world, vehicle, lambda pc: holder.__setitem__(
        "lidar", np.frombuffer(pc.raw_data, dtype=np.float32).reshape(-1, 4)), tick=True)

    print(f"[就绪] 自车@{tf.location}，使用 NN 感知+控制，路点={len(waypoints)}")
    lateral_errors = []
    n_ticks = int(sim_time / DT)
    for k in range(n_ticks):
        pos = get_location(vehicle)
        yaw = get_yaw(vehicle)
        # 感知：特征 -> NN 类别
        feat, raw = extract_features(holder["rgb"], holder["depth"], holder["lidar"])
        cls = int(sens.predict(feat[None, :])[0])
        cls_name = {0: "无目标", 1: "偏左", 2: "偏右"}[cls]
        # 控制
        if use_nn_control:
            throttle, steer = nn_control(pos, yaw, waypoints, model["ctrl"])
        else:
            throttle, steer = pure_pursuit(pos, yaw, waypoints)
        apply_control(vehicle, throttle=throttle, steer=steer)

        cur = np.array(pos)
        best = min(np.hypot(p[0] - cur[0], p[1] - cur[1]) for p in waypoints)
        lateral_errors.append(best)

        world.tick()
        if k % 20 == 0:
            off, d_, fn, fd = raw
            print(f"[t={k*DT:.1f}] pos=({pos[0]:.1f},{pos[1]:.1f}) "
                  f"NN感知={cls_name} steer={steer:+.2f} rgb={off:+.2f} "
                  f"depth={d_:.0f} lidar({fn},{fd}) nearest={best:.2f}")

    rmse = float(np.sqrt(np.mean(np.square(lateral_errors))))
    print(f"\n轨迹跟踪完成：横向误差 RMSE = {rmse:.3f} m")
    vehicle.destroy()
    return 0


def _load_model(path):
    # 模型文件为打包 dict {sens,ctrl}
    import json
    with open(path) as f:
        obj = json.load(f)
    sens = MLPClassifier(obj["sens"]["layers"])
    sens.W = [np.asarray(w, dtype=np.float32) for w in obj["sens"]["W"]]
    sens.b = [np.asarray(t, dtype=np.float32) for t in obj["sens"]["b"]]
    ctrl = MLPPolicy(obj["ctrl"]["layers"])
    ctrl.W = [np.asarray(w, dtype=np.float32) for w in obj["ctrl"]["W"]]
    ctrl.b = [np.asarray(t, dtype=np.float32) for t in obj["ctrl"]["b"]]
    return {"sens": sens, "ctrl": ctrl}


def _save_model(path, sens, ctrl):
    import json
    import os
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    payload = {
        "sens": {"layers": sens.layers, "W": [w.tolist() for w in sens.W],
                 "b": [t.tolist() for t in sens.b]},
        "ctrl": {"layers": ctrl.layers, "W": [w.tolist() for w in ctrl.W],
                 "b": [t.tolist() for t in ctrl.b]},
    }
    with open(path, "w") as f:
        json.dump(payload, f)
    print("模型已保存:", path)


def main():
    p = argparse.ArgumentParser(description="CARLA 感知+轨迹跟踪（神经网络版）")
    p.add_argument("--mode", choices=["train", "run"], default="run")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--town", default=DEFAULT_MAP)
    p.add_argument("--waypoints", default="40,-8 40,12 25,20")
    p.add_argument("--sim_time", type=float, default=20.0)
    p.add_argument("--epochs", type=int, default=300)
    p.add_argument("--out", default="models/nn_percept.json")
    p.add_argument("--model", default="models/nn_percept.json")
    args = p.parse_args()

    wps = [tuple(map(float, t.split(","))) for t in args.waypoints.split()]
    if len(wps) < 1:
        print("[错误] 至少 1 个路点"); return 1

    if args.mode == "train":
        feat_X, feat_Y, ctrl_X, ctrl_Y = synth_dataset()
        sens, ctrl = train(feat_X, feat_Y, ctrl_X, ctrl_Y, args.epochs)
        print(f"感知 NN train_acc={np.mean(sens.predict(feat_X)==feat_Y):.3f}")
        print(f"控制 NN 最终MSE={np.mean(np.square(ctrl.predict(ctrl_X).ravel()-ctrl_Y)):.4f}")
        _save_model(args.out, sens, ctrl)
        return 0

    return run_carla(args.host, args.port, args.town, args.model, wps, args.sim_time)


if __name__ == "__main__":
    sys.exit(main())
