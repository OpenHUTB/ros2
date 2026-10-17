#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业四 · CARLA 端到端神经网络【图像 → 控制】。

对应老师任务 (4)：端到端模型——**输入图像、输出控制指令**，中间不再有
"感知目标 → 解析规划"的人工分层。

数据流：

    前视相机图像 (60×80×3) --CNN--> steer ∈ [-1,1] --应用到车辆--> (油门, 转向)

四个模式：

  * `--mode collect`  连 CARLA，用**专家控制器**驾驶并采集 (图像, 转向) 对（行为克隆）
  * `--mode train`    训练 CNN（有监督回归，MAE 损失）。默认**纯 numpy**，无需 TensorFlow
  * `--mode test`     连 CARLA，加载模型做端到端自主驾驶
  * `--headless --demo`  离线自检取证：合成道路图像 → 训练 CNN → 导出损失与预测曲线
                         （**无需 CARLA、无需图形界面、无需 TensorFlow**）

用法：
    python3 main.py --mode collect --host <宿主机IP> --frames 300 --out_dir dataset
    python3 main.py --mode train   --epochs 40 --data_dir dataset --model_path models/cnn.json
    python3 main.py --mode test    --host <宿主机IP> --model_path models/cnn.json --sim_time 20
    python3 main.py --headless --demo --save_dir ~/shots
    ros2 launch carla_end_to_end_nn main.launch.py host:=<宿主机IP>
    roslaunch carla_end_to_end_nn main.launch host:=<宿主机IP>
"""

import argparse
import math
import os
import sys

import numpy as np

from carla_end_to_end_nn.nn_models import SimpleCNN

try:
    import carla  # noqa: F401
    from carla_end_to_end_nn import carla_common as cc
    _HAVE_CARLA = True
except Exception:  # noqa: BLE001
    cc = None
    _HAVE_CARLA = False

DT = 0.05
IMG_H, IMG_W = 60, 80          # 相机图像分辨率（高 × 宽）
WHEELBASE = 2.5
STEER_GAIN = 1.2217            # steer(归一化) → 前轮转角(rad)
LOOKAHEAD = 6.0                # 专家控制器的前视距离（m）
DEFAULT_THROTTLE = 0.45


# ============================================================ 合成道路图像
def synth_road_image(steer, rng, h=IMG_H, w=IMG_W):
    """合成一张"前方道路"图像：透视车道线，**弯曲方向与弯曲量由 steer 决定**。

    这是端到端学习正确的数据构造方式：图像里必须**真的含有**"路往哪弯"的信息，
    标签才是"应该打多大转向"。二者有因果关系，网络才学得到映射。

    （反面例子：用与图像无关的正弦信号当标签——那样图像与标签无因果关系，
    网络只能学到标签的均值，永远学不会看路。）

    透视约定：行号 y 越小越远（近地平线），越大越近（画面底部）。
    车道线在半宽处收窄，并按 steer 产生二次弯曲，模拟转弯时的视角变化。
    """
    img = np.zeros((h, w, 3), dtype=np.float32)
    horizon = int(h * 0.38)

    # 天空（渐变）与路面
    for y in range(horizon):
        t = y / max(1, horizon)
        img[y] = (120 + 40 * t, 165 + 35 * t, 215 + 20 * t)
    img[horizon:] = (72, 72, 78)                      # 沥青

    # 车道线：两条白线，随 steer 弯曲
    for y in range(horizon, h):
        t = (y - horizon) / max(1, (h - 1 - horizon))  # 0=远处, 1=近处
        lane_w = 0.55 * w * t + 0.06 * w               # 透视收窄
        curve = -steer * (1.0 - t) ** 2 * w * 0.42     # steer>0(左转) → 车道向左弯
        center = w / 2.0 + curve
        for x_line in (center - lane_w / 2.0, center + lane_w / 2.0):
            xi = int(round(x_line))
            img[y, max(0, xi - 1):min(w, xi + 2)] = (235, 235, 238)

    # 传感器噪声：模拟真实相机的暗电流与颗粒
    img += rng.normal(0.0, 5.0, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def synth_dataset(n=400, seed=0, steer_range=0.8):
    """合成端到端数据集：N 张道路图像 + 对应转向标签。"""
    rng = np.random.RandomState(seed)
    steers = np.linspace(-steer_range, steer_range, n)   # 覆盖全转向区间
    rng.shuffle(steers)
    X = np.zeros((n, IMG_H, IMG_W, 3), dtype=np.uint8)
    for i, s in enumerate(steers):
        X[i] = synth_road_image(float(s), rng)
    return X, steers.reshape(-1, 1).astype(np.float32)


# ============================================================ 专家控制器
def expert_steer(pos, yaw, waypoints, lookahead=LOOKAHEAD):
    """专家控制器：纯跟踪几何律，由**前方路点**算出转向（行为克隆的标签来源）。

    $$
    \\delta = \\arctan\\!\\Big(\\frac{2L\\sin e_\\psi}{d}\\Big),\\qquad
    \\text{steer} = \\mathrm{clip}\\!\\Big(\\frac{\\delta}{g},-1,1\\Big)
    $$

    端到端学习（行为克隆）的关键：标签来自"看着路开"的专家，
    因此**图像与标签天然相关**，网络学到的是真正的"看图打方向"。
    """
    if not waypoints:
        return 0.0, 0.0
    # 选第一个距车至少 lookahead 的路点作为前视目标
    tx, ty = waypoints[-1]
    for (wx, wy) in waypoints:
        if math.hypot(wx - pos[0], wy - pos[1]) >= lookahead:
            tx, ty = wx, wy
            break
    dx, dy = tx - pos[0], ty - pos[1]
    dist = max(math.hypot(dx, dy), 0.5)
    diff = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
    delta = math.atan2(2.0 * WHEELBASE * math.sin(diff), dist)
    return float(np.clip(delta / STEER_GAIN, -1.0, 1.0)), diff


def carla_expert_steer(world, vehicle, lookahead=LOOKAHEAD):
    """CARLA 场景中的专家转向：用路点 API 沿车道向前看，算纯跟踪转向。

    这是行为克隆采集标签用的专家。相比"人为正弦转向"，本专家确实在**沿车道行驶**，
    因此它打的转向与相机看到的画面一致。
    """
    tf = vehicle.get_transform()
    loc, yaw_deg = tf.location, tf.rotation.yaw
    yaw = math.radians(yaw_deg)
    cmap = world.get_map()
    wp = cmap.get_waypoint(loc, project_to_road=True)
    if wp is None:
        return 0.0
    ahead = wp.next(lookahead)
    if not ahead:
        return 0.0
    tgt = ahead[0].transform.location
    dx, dy = tgt.x - loc.x, tgt.y - loc.y
    dist = max(math.hypot(dx, dy), 0.5)
    diff = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
    delta = math.atan2(2.0 * WHEELBASE * math.sin(diff), dist)
    return float(np.clip(delta / STEER_GAIN, -1.0, 1.0))


# ============================================================ 训练
def train_cnn(images, labels, model_path, epochs=60, backend="numpy",
              lr=0.15, batch=16, verbose=None):
    """训练图像 → 转向 的端到端 CNN。

    backend="numpy"（默认）：纯 numpy `SimpleCNN`，无需 TensorFlow，模型存 JSON
    backend="tf"          ：tf.keras CNN，模型存 .h5（需自行安装 TensorFlow）

    返回 (net_or_model, history)。
    """
    if verbose is None:
        verbose = max(1, epochs // 5)
    labels = np.asarray(labels, dtype=np.float32).ravel()
    print(f"数据集: X{images.shape} Y{labels.shape} backend={backend}")

    d = os.path.dirname(model_path)
    if d:
        os.makedirs(d, exist_ok=True)

    if backend == "tf":
        return _train_tf(images, labels, model_path, epochs)

    net = SimpleCNN(in_channels=3, hidden=8, out=1, img=(IMG_H, IMG_W), seed=0)
    hist = net.train(images, labels, epochs=epochs, lr=lr, batch=batch, verbose=verbose)
    net.save(model_path)
    pred = net.predict(images.astype(np.float32))
    mae = float(np.mean(np.abs(pred.ravel() - labels)))
    print(f"端到端 CNN 训练 MAE = {mae:.5f}")
    print("模型已保存(纯 numpy JSON):", model_path)
    # SimpleCNN.train 返回的是逐 epoch 损失 list（不是 dict）
    return net, {"loss": list(hist), "mae": mae}


def _train_tf(images, labels, model_path, epochs):
    """可选：用 tf.keras 训练（需安装 TensorFlow）。"""
    import tensorflow as tf
    from tensorflow.keras import layers, models
    inp = layers.Input(shape=(IMG_H, IMG_W, 3), name="image")
    x = layers.Rescaling(1.0 / 255.0)(inp)
    x = layers.Conv2D(16, 3, activation="relu", padding="same")(x)
    x = layers.MaxPooling2D(2)(x)
    x = layers.Conv2D(32, 3, activation="relu", padding="same")(x)
    x = layers.MaxPooling2D(2)(x)
    x = layers.Conv2D(64, 3, activation="relu", padding="same")(x)
    x = layers.GlobalAveragePooling2D()(x)
    x = layers.Dense(64, activation="relu")(x)
    x = layers.Dropout(0.3)(x)
    out = layers.Dense(1, activation="tanh", name="steer")(x)
    model = models.Model(inp, out)
    model.compile(optimizer="adam", loss="mae", metrics=["mae"])
    h = model.fit(images, labels, epochs=epochs, batch_size=32,
                  validation_split=0.1, verbose=2)
    model.save(model_path)
    print("模型已保存(tf.keras):", model_path)
    return model, {"loss": list(h.history.get("loss", [])), "mae": float(h.history["mae"][-1])}


def cnn_predict(net_or_model, img, backend="numpy"):
    """单张图像 → 转向角。img 为 HWC uint8。"""
    if backend == "tf":
        return float(net_or_model.predict(img[np.newaxis].astype(np.float32), verbose=0)[0][0])
    return float(net_or_model.predict(img.astype(np.float32))[0, 0])


def load_cnn(model_path, backend="numpy"):
    """加载模型。backend="numpy" 走 SimpleCNN JSON。"""
    if backend == "tf":
        import tensorflow as tf
        return tf.keras.models.load_model(model_path, compile=False)
    return SimpleCNN.load(model_path)


# ============================================================ PNG 导出（取证）
def _write_png(path, rgb):
    """纯 zlib 写 PNG（不依赖 Pillow / opencv）。rgb 为 (H,W,3) uint8。"""
    import struct
    import zlib
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
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
    return path


def _line(canvas, x0, y0, x1, y1, color):
    """Bresenham 画线。"""
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


def _axes(canvas, margin=28):
    """画坐标轴边框。"""
    h, w = canvas.shape[:2]
    canvas[margin:h - margin, margin:margin + 2] = 190
    canvas[margin:h - margin, w - margin - 2:w - margin] = 190
    canvas[margin:margin + 2, margin:w - margin] = 190
    canvas[h - margin - 2:h - margin, margin:w - margin] = 190
    # y=0 参考线（转向为 0 的中线）
    mid = h // 2
    canvas[mid - 1:mid + 1, margin:w - margin] = 220
    return margin


def render_loss_png(losses, path, width=640, height=360):
    """训练损失曲线（MAE）。"""
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    m = _axes(canvas)
    v = np.asarray(losses, dtype=np.float32).ravel()
    if v.size < 2:
        return _write_png(path, canvas)
    vmin, vmax = float(v.min()), float(v.max())
    if vmax - vmin < 1e-9:
        vmax = vmin + 1.0
    xs = np.linspace(m + 6, width - m - 6, v.size).astype(int)
    ys = (height - m - 6 - (v - vmin) / (vmax - vmin) * (height - 2 * m - 12)).astype(int)
    for i in range(v.size - 1):
        _line(canvas, xs[i], ys[i], xs[i + 1], ys[i + 1], (200, 60, 60))
    return _write_png(path, canvas)


def render_pred_vs_true_png(y_true, y_pred, path, width=640, height=360):
    """预测转向 vs 真实转向：蓝=标签，红=CNN 预测。两条曲线越贴合越好。"""
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    m = _axes(canvas)
    n = len(y_true)
    if n < 2:
        return _write_png(path, canvas)

    def to_xy(vals):
        xs = np.linspace(m + 6, width - m - 6, n).astype(int)
        ys = (height // 2 - np.clip(vals, -1, 1) * (height / 2 - m - 12)).astype(int)
        return xs, ys

    xs, ys_t = to_xy(np.asarray(y_true, dtype=np.float32))
    _xs, ys_p = to_xy(np.asarray(y_pred, dtype=np.float32))
    for i in range(n - 1):
        _line(canvas, xs[i], ys_t[i], xs[i + 1], ys_t[i + 1], (60, 90, 200))
        _line(canvas, xs[i], ys_p[i], xs[i + 1], ys_p[i + 1], (210, 50, 50))
    return _write_png(path, canvas)


def render_samples_png(images, steers, preds, path, cols=4, scale=3):
    """把若干张样本图像横向拼成一张图，便于在文档里展示训练数据。"""
    n = min(cols, len(images))
    if n == 0:
        return _write_png(path, np.full((IMG_H, IMG_W, 3), 255, np.uint8))
    h, w = images[0].shape[:2]
    gap = 4
    canvas = np.full((h, n * w + (n - 1) * gap, 3), 30, dtype=np.uint8)
    for i in range(n):
        x0 = i * (w + gap)
        canvas[:, x0:x0 + w] = images[i][:, :, :3]
    if scale > 1:
        canvas = np.repeat(np.repeat(canvas, scale, axis=0), scale, axis=1)
    return _write_png(path, canvas)


# ======================================================== 离线取证（无需 CARLA）
def run_offline_demo(save_dir, epochs=60, n_samples=150, seed=0):
    """离线自检：合成道路图像 → 训练端到端 CNN → 导出证据图。

    **不需要 CARLA、不需要图形界面、不需要 TensorFlow**，用于在无 3D 加速的
    虚拟机 / CI 中产出"可运行性"证据，并验证「图像 → 转向」确实被学到。
    """
    print("=" * 64)
    print("  作业四 离线自检取证模式（合成道路图像，不需要 CARLA）")
    print("=" * 64)

    # ---- 合成数据集：图像里的车道线弯曲量与标签一致 ----
    X, Y = synth_dataset(n=n_samples, seed=seed)
    print(f"合成道路图像数据集: {X.shape}，转向标签范围 [{Y.min():+.2f}, {Y.max():+.2f}]")

    # ---- 训练端到端 CNN ----
    os.makedirs(save_dir, exist_ok=True)
    model_path = os.path.join(save_dir, "cnn_steer.json")
    net, hist = train_cnn(X, Y, model_path, epochs=epochs, backend="numpy")

    # ---- 评估：MAE + 符号一致率（转向方向判对的比例）----
    pred = net.predict(X.astype(np.float32)).ravel()
    mae = float(np.mean(np.abs(pred - Y.ravel())))
    rmse = float(np.sqrt(np.mean((pred - Y.ravel()) ** 2)))
    sign_acc = float(np.mean(np.sign(pred) == np.sign(Y.ravel())))
    # 与"恒输出 0"的基线比较，证明网络确实学到了东西
    base_mae = float(np.mean(np.abs(Y.ravel())))
    print(f"\n端到端 CNN 评估：MAE = {mae:.5f}，RMSE = {rmse:.5f}，"
          f"转向方向一致率 = {sign_acc * 100:.1f}%")
    print(f"零输出基线 MAE = {base_mae:.5f}（MAE 明显低于基线才说明网络学到了映射）")

    # ---- 导出证据图 ----
    p1 = render_loss_png(hist["loss"], os.path.join(save_dir, "cnn_loss.png"))
    order = np.argsort(Y.ravel())          # 按真实转向排序，便于看两条曲线贴合程度
    p2 = render_pred_vs_true_png(Y.ravel()[order], pred[order],
                                 os.path.join(save_dir, "steer_pred_vs_true.png"))
    p3 = render_samples_png(X[:4], Y.ravel()[:4], pred[:4],
                            os.path.join(save_dir, "road_samples.png"))

    print("\n已导出取证图：")
    for p in (p1, p2, p3):
        print("   ", p)
    return 0


# ============================================================ CARLA：采集
def collect_carla(host, port, town, frames, out_dir, backend="numpy"):
    """**行为克隆**采集：(前视相机图像, 专家转向) 对。

    与"人为施加正弦转向"的错误做法不同，这里由**专家控制器**（沿车道前视的纯跟踪律）
    实际驾驶车辆，同时记录相机图像与该时刻专家打出的转向。
    图像与标签因此**天然对应**：画面里路往左弯，标签就是左转。

    采集顺序（避免图文错帧）：先 `tick()` 让场景与相机更新 → 读当前图像与位姿
    → 用专家算出此刻应打的转向 → 记为标签 → 应用到车辆 → 进入下一帧。
    """
    if not _HAVE_CARLA:
        print("[错误] collect 模式需要 CARLA，请先启动服务端并安装 carla 客户端。")
        print("       离线验证可改用： python3 main.py --headless --demo --save_dir ~/shots")
        return 1

    try:
        client, world = cc.connect(host, port, town)
    except Exception as exc:  # noqa: BLE001
        print(f"\n[错误] 连接 CARLA 服务端失败：{exc}")
        print(f"       目标 {host}:{port}，地图 {town}。请确认服务端已启动、"
              "防火墙放行 2000 端口、host 填宿主机 VMnet8 地址。")
        return 1

    # spawn / 采集 / 清理全部包进 try-finally：万一中途抛异常，
    # 也要销毁传感器与自车并把世界恢复为异步模式。
    vehicle = None
    sensors = []
    try:
        vehicle, tf = cc.spawn_vehicle(world)

        holder = {"img": None}
        # 必须保留 sensor 对象的引用！否则 spawn_actor 返回的 sensor 无引用，
        # 函数作用域结束后被 Python 垃圾回收，Actor 虽留在仿真里但回调不再触发，
        # 表现为「采集到 0 帧」并伴随 CARLA 警告：
        #   sensor object went out of the scope but the sensor is still alive in the simulation
        sensors.append(cc.make_rgb_camera(
            world, vehicle, lambda im: holder.update(img=im),
            width=IMG_W, height=IMG_H, tick=True))
        os.makedirs(os.path.join(out_dir, "images"), exist_ok=True)
        lbl_path = os.path.join(out_dir, "labels.txt")

        saved = 0
        with open(lbl_path, "w", encoding="utf-8") as f:
            for i in range(frames):
                world.tick()                  # 先推进：图像与位姿反映当前状态
                img = holder["img"]
                if img is None:
                    continue
                steer = carla_expert_steer(world, vehicle)  # 专家此刻该打的转向
                _write_jpg_like(os.path.join(out_dir, "images", f"{saved:05d}.npy"), img)
                f.write(f"{steer:.6f}\n")
                cc.apply_control(vehicle, throttle=DEFAULT_THROTTLE, steer=steer)
                saved += 1
                if i % 50 == 0:
                    print(f"  collect {i}/{frames}  已保存 {saved}  专家 steer={steer:+.3f}")

        print(f"数据采集完成：{saved} 帧 → {out_dir}")
        print(f"  图像: {os.path.join(out_dir, 'images')}（.npy，uint8 HWC）")
        print(f"  标签: {lbl_path}")
        if saved == 0:
            print("[警告] 未采集到任何帧，请检查 CARLA 服务端与相机回调。")
    finally:
        for s in sensors:
            try:
                s.stop()
                s.destroy()
            except Exception:  # noqa: BLE001
                pass
        if vehicle is not None:
            try:
                vehicle.destroy()
            except Exception:  # noqa: BLE001
                pass
        # 恢复异步模式：同步模式下服务端只在客户端 tick 时推进，
        # 脚本退出后没人 tick，CARLA 窗口会看起来「停住不动」。
        if cc.restore_async(world):
            print("[清理] 已销毁传感器与自车，世界已恢复异步模式。")
        else:
            print("[清理] 已销毁传感器与自车；世界仍为同步模式，"
                  "如需恢复可重启 CARLA 服务端。")
    return 0


def _write_jpg_like(path, img):
    """保存图像为 .npy（避免依赖 opencv/Pillow，纯 numpy 可读回）。"""
    np.save(path, np.ascontiguousarray(img, dtype=np.uint8))


def load_dataset(data_dir):
    """读取采集的数据集：.npy 图像 + labels.txt 标签。"""
    img_dir = os.path.join(data_dir, "images")
    names = sorted(n for n in os.listdir(img_dir) if n.endswith(".npy"))
    labels = []
    with open(os.path.join(data_dir, "labels.txt"), encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                labels.append(float(line.split()[0]))
    X, Y = [], []
    for i, name in enumerate(names):
        if i >= len(labels):
            break
        X.append(np.load(os.path.join(img_dir, name)))
        Y.append(labels[i])
    if not X:
        return np.zeros((0, IMG_H, IMG_W, 3), np.uint8), np.zeros((0, 1), np.float32)
    return np.asarray(X, dtype=np.uint8), np.asarray(Y, dtype=np.float32).reshape(-1, 1)


# ============================================================ CARLA：端到端驾驶
def test_carla(host, port, town, model_path, sim_time=20.0, backend="numpy", save_dir=None):
    """加载端到端 CNN，仅凭相机图像输出转向，实现自主驾驶。"""
    if not _HAVE_CARLA:
        print("[错误] test 模式需要 CARLA。离线验证请用 --headless --demo")
        return 1

    # 模型文件不存在时**现场训练并保存**，而不是直接崩掉。
    # 与 ROS 节点（end_to_end_node.py）的行为保持一致：那边同样有
    # os.path.isfile 守卫并回退到现场训练，保证"节点总能运行"。
    #
    # 注意预算：纯 numpy 的 CNN 训练较慢，实测约 15 s/epoch（400 样本）。
    # 若沿用离线取证的 60 epochs，回退训练要十几分钟——使用者会以为程序卡死。
    # 因此回退只用小样本 + 少轮次，并明确打印预计耗时。
    if not os.path.isfile(model_path):
        n_fb, ep_fb = 150, 20
        print(f"[提示] 未找到模型文件 {model_path}，先现场训练一个可用的 CNN。")
        print(f"       预算：{n_fb} 合成样本 × {ep_fb} 轮（纯 numpy 约 2 分钟）。")
        print("       需要更高精度请先离线训练：")
        print(f"         python3 main.py --mode train --epochs 60 --samples 400 "
              f"--model_path {model_path}")
        X, Y = synth_dataset(n=n_fb)
        train_cnn(X, Y, model_path, epochs=ep_fb, backend="numpy")

    net = load_cnn(model_path, backend=backend)

    try:
        client, world = cc.connect(host, port, town)
    except Exception as exc:  # noqa: BLE001
        print(f"\n[错误] 连接 CARLA 服务端失败：{exc}")
        print(f"       目标 {host}:{port}，地图 {town}。请确认服务端已启动、"
              "防火墙放行 2000 端口、host 填宿主机 VMnet8 地址。")
        return 1

    # spawn / 推理 / 清理全部包进 try-finally：万一中途抛异常，
    # 也要销毁传感器与自车并把世界恢复为异步模式。
    vehicle = None
    sensors = []
    try:
        vehicle, tf = cc.spawn_vehicle(world)

        holder = {"img": None}
        # 必须保留 sensor 对象的引用，否则会被 Python GC 回收、回调失效
        # （表现为 steer 恒为 0、图像始终为 None）。
        sensors.append(cc.make_rgb_camera(
            world, vehicle, lambda im: holder.update(img=im),
            width=IMG_W, height=IMG_H, tick=True))
        print(f"[就绪] 端到端 CNN 已加载：{model_path}")
        print("[说明] 转向完全由相机图像经 CNN 得出，不含任何解析规划环节")

        steer = 0.0
        frames_seen = 0
        for k in range(int(sim_time / DT)):
            world.tick()
            img = holder["img"]
            if img is not None:
                frames_seen += 1
                steer = cnn_predict(net, img, backend=backend)
                cc.apply_control(vehicle, throttle=DEFAULT_THROTTLE, steer=steer)
            if k % 40 == 0:
                x, y = cc.get_location(vehicle)
                print(f"[t={k * DT:5.1f}s] CNN steer={steer:+.3f} pos=({x:7.2f},{y:7.2f})")
        print("端到端自主驾驶完成")
        print(f"共收到相机帧 {frames_seen} 张")
        if frames_seen == 0:
            print("[警告] 全程未收到任何相机帧，转向输出恒为默认值，"
                  "本轮结果不具参考意义。")
    finally:
        for s in sensors:
            try:
                s.stop()
                s.destroy()
            except Exception:  # noqa: BLE001
                pass
        if vehicle is not None:
            try:
                vehicle.destroy()
            except Exception:  # noqa: BLE001
                pass
        if cc.restore_async(world):
            print("[清理] 已销毁传感器与自车，世界已恢复异步模式。")
        else:
            print("[清理] 已销毁传感器与自车；世界仍为同步模式，"
                  "如需恢复可重启 CARLA 服务端。")
    return 0


# ==================================================================== CLI
def build_parser():
    p = argparse.ArgumentParser(description="CARLA 端到端神经网络【图像 → 控制】")
    p.add_argument("--mode", choices=["collect", "train", "test"], default="test")
    p.add_argument("--host", default="127.0.0.1", help="CARLA 服务端地址（虚拟机填宿主机 IP）")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--town", default="Town05")
    p.add_argument("--frames", type=int, default=300, help="collect：采集帧数")
    p.add_argument("--out_dir", default="dataset")
    p.add_argument("--data_dir", default="dataset")
    p.add_argument("--epochs", type=int, default=60)
    p.add_argument("--model_path", default="models/cnn.json")
    p.add_argument("--sim_time", type=float, default=20.0)
    p.add_argument("--backend", choices=["numpy", "tf"], default="numpy",
                   help="numpy=纯numpy SimpleCNN（默认，无需 TensorFlow）；tf=tf.keras")
    p.add_argument("--samples", type=int, default=150, help="离线取证：合成样本数")
    _add_bool(p, "--headless",
              "离线取证模式（无需 CARLA/图形界面）")
    _add_bool(p, "--demo",
              "运行内置演示序列")
    p.add_argument("--save_dir", nargs="?", const=None, default=None, help="取证图导出目录")
    _add_bool(p, "--launch",
              "由 ROS launch 启动")
    return p


def _parse_bool(value):
    """Parse a boolean that arrives as text.

    ROS 1 launch files can only pass arguments as plain text (e.g.
    ``--follow true`` / ``--follow false``) -- unlike ROS 2, which passes a real
    parameter and keeps the bool type.  Without this, ``--follow false`` makes
    ``store_true`` set follow=True and the stray ``false`` is swallowed by
    ``parse_known_args`` as an unknown positional.
    """
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on", "y", "t"):
        return True
    if text in ("0", "false", "no", "off", "n", "f", ""):
        return False
    raise argparse.ArgumentTypeError(f"无法识别的布尔值：{value!r}")


def _add_bool(parser, name, help_text):
    """Boolean switch: bare ``--flag`` or explicit ``--flag true|false``."""
    parser.add_argument(name, type=_parse_bool, nargs="?", const=True,
                        default=False, help=help_text)


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.headless or args.demo:
        return run_offline_demo(args.save_dir or "shots", epochs=args.epochs,
                                n_samples=args.samples)

    if args.mode == "train":
        X, Y = load_dataset(args.data_dir)
        if len(X) == 0:
            print(f"[错误] 数据集为空：{args.data_dir}")
            print("       请先采集：python3 main.py --mode collect --host <IP> --frames 300")
            print("       或离线取证：python3 main.py --headless --demo --save_dir ~/shots")
            return 1
        train_cnn(X, Y, args.model_path, epochs=args.epochs, backend=args.backend)
        return 0

    if args.mode == "collect":
        return collect_carla(args.host, args.port, args.town, args.frames,
                             args.out_dir, backend=args.backend)
    return test_carla(args.host, args.port, args.town, args.model_path,
                      args.sim_time, backend=args.backend, save_dir=args.save_dir)


if __name__ == "__main__":
    sys.exit(main())
