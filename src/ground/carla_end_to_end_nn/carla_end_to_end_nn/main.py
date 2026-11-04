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

from carla_end_to_end_nn.bitmap_font import draw_text, text_width
from carla_end_to_end_nn.gif_writer import rgb_frames_to_gif
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
def synth_road_image(steer, rng, h=IMG_H, w=IMG_W, noise=5.0):
    """合成一张"前方道路"图像：透视车道线，**弯曲方向与弯曲量由 steer 决定**。

    这是端到端学习正确的数据构造方式：图像里必须**真的含有**"路往哪弯"的信息，
    标签才是"应该打多大转向"。二者有因果关系，网络才学得到映射。

    （反面例子：用与图像无关的正弦信号当标签——那样图像与标签无因果关系，
    网络只能学到标签的均值，永远学不会看路。）

    透视约定：行号 y 越小越远（近地平线），越大越近（画面底部）。
    车道线在半宽处收窄，并按 steer 产生二次弯曲，模拟转弯时的视角变化。

    `noise` 是相机噪声的标准差（像素值）。导出文档动图时传 0：GIF 只有 256 色，
    加了噪声后每张图会有上万种颜色，无法索引化，压缩率也会崩掉。
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
    if noise:
        img += rng.normal(0.0, float(noise), img.shape)
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


# ============================================================ 留出集与数据增强
# 为什么补这一节：本模块原先只在**训练集**上报指标（文档 6.2 自注"训练集口径"）。
# 端到端模型最容易掩盖的问题就是过拟合——训练集 MAE 0.09 很好看，换一批同分布的
# 图像却可能明显变差，而"只在训练集上好看"正是行为克隆最常见的隐性缺陷。
# 因此这里补上两件事：
#   1) 确定性留出集切分（训练/验证互不重叠），给出**验证集**口径的指标；
#   2) 保持标签语义的数据增强，并用"仅复制扩容"做控制组，量化增强本身的效果。
VAL_SEED = 1234


def split_dataset(X, Y, val_ratio=0.25, seed=VAL_SEED):
    """确定性切分训练集 / 留出验证集（两者互不重叠）。

    用固定 seed 的 permutation，同一份数据每次切分结果完全一致，便于复现与写测试。
    `val_ratio<=0` 时不分验证集，`X_val is None`（调用方据此跳过验证指标）。

    返回 `(X_tr, Y_tr, X_val, Y_val)`，Y 保持 `(N, 1)` 形状，与 `synth_dataset` 一致。
    """
    X = np.asarray(X)
    Y = np.asarray(Y, dtype=np.float32)
    n = len(X)
    if n < 2:
        return X, Y, None, None

    n_val = int(round(n * float(val_ratio)))
    if n_val <= 0:
        return X, Y, None, None
    n_val = min(n_val, n - 1)                      # 至少留 1 个训练样本

    idx = np.random.RandomState(seed).permutation(n)
    val_idx, tr_idx = idx[:n_val], idx[n_val:]
    return X[tr_idx], Y[tr_idx], X[val_idx], Y[val_idx]


def mirror_image(img):
    """水平镜像一张 HWC 图像（车道线左右互换）。"""
    return np.ascontiguousarray(img[:, ::-1, :])


def augment_dataset(X, Y, copies=1, seed=0, mirror_prob=0.5,
                    gain_range=(0.45, 1.45), noise=12.0):
    """对训练样本做**保持标签语义**的数据增强，返回纯增强样本 `(X_aug, Y_aug)`。

    只做两类增强，理由如下（这也是本节最容易被写错的地方）：

    * **水平镜像**：镜像后"左弯"变"右弯"，**标签必须同时取负**，物理关系才成立。
      若镜像了图像却沿用原标签，等于给网络灌入错误监督，比不增强更糟。
    * **光度扰动**（亮度增益 + 加性噪声）：不改变道路几何，标签保持不变。
      它模拟相机自动曝光与暗电流差异，让网络不靠"某几档亮度"走捷径。

    **刻意不做横向平移 / 裁剪**：平移会改变车道线相对车体的位置，"正确转向"随之
    改变；标签不跟着改就破坏了图像与标签的因果关系——与文档 §2.5 里"用人为正弦
    信号当标签"是同一类错误（网络只会学到标签均值）。

    参数
    ----
    copies      : 生成几轮增强样本（每轮每个训练样本一张新图）
    mirror_prob : 每张图做水平镜像的概率（0=不镜像，1=全镜像）
    gain_range  : 亮度增益的均匀分布区间
    noise       : 加性高斯噪声标准差（像素值）

    默认 `gain_range=(0.45, 1.45)`、`noise=12` 是**实测选出来的**：增强范围必须
    覆盖部署时的光度变化，否则不但没用，还会在强偏移下更差（见文档 6.3 的消融表：
    窄范围 0.85~1.15/σ=3 在强偏移下 0.326，反而劣于不增强的 0.283）。

    返回的样本需要与原始训练集**拼接**后使用；`copies=0` 时返回空数组，
    调用方可据此判断"没有做增强"。
    """
    X = np.asarray(X)
    Ya = np.asarray(Y, dtype=np.float32).ravel()
    n = len(X)
    rng = np.random.RandomState(seed)

    out_X = np.zeros((max(0, int(copies)) * n, X.shape[1], X.shape[2], 3), dtype=np.uint8)
    out_Y = np.zeros(max(0, int(copies)) * n, dtype=np.float32)
    k = 0
    for _ in range(max(0, int(copies))):
        for i in range(n):
            img = X[i]
            y = float(Ya[i])
            # ① 几何：镜像 → 标签取负（左右手互换）
            if mirror_prob > 0.0 and rng.rand() < float(mirror_prob):
                img = mirror_image(img)
                y = -y
            # ② 光度：增益 + 噪声 → 标签不变
            g = rng.uniform(float(gain_range[0]), float(gain_range[1]))
            arr = img.astype(np.float32) * g
            if noise:
                arr = arr + rng.normal(0.0, float(noise), arr.shape)
            out_X[k] = np.clip(arr, 0, 255).astype(np.uint8)
            out_Y[k] = y
            k += 1
    return out_X[:k], out_Y[:k].reshape(-1, 1)


def photometric_shift(X, gain=1.0, noise=0.0, seed=99):
    """对整批图像做**与标签无关**的光度偏移：整体压暗 + 加重噪声。

    用来模拟"训练时是理想光照、部署时更暗且噪声更大"的分布偏移——这正是文档
    Q5「合成图像上表现好、实车上不 work」的量化形式。标签不用改：亮度与噪声不
    改变"路往哪弯"，只改变网络的输入分布。

    与 `augment_dataset` 的关系：增强是"训练时随机扰动"，本函数是"评测时固定
    偏移"，两者共同回答"增强范围要覆盖多大的部署偏移才管用"。
    """
    X = np.asarray(X, dtype=np.float32)
    rng = np.random.RandomState(seed)
    Y = X * float(gain)
    if noise:
        Y = Y + rng.normal(0.0, float(noise), Y.shape)
    return np.clip(Y, 0, 255).astype(np.uint8)


# 分布偏移评测的三档光度条件：clean 是留出集原样；mild/strong 模拟更暗、
# 噪声更大的夜间/雨雾相机。数值取"比训练增强范围更大一档"，用于检验泛化边界。
SHIFT_LEVELS = [("clean", 1.00, 0.0), ("mild", 0.75, 8.0), ("strong", 0.60, 18.0)]


def evaluate_steer(pred, y_true):
    """转向回归的评估指标：MAE / RMSE / 方向一致率 / 相关系数 / 零输出基线。

    `base_mae` 是"恒输出 0"的基线：网络的 MAE 必须明显低于它，才说明真的学到了
    "看图打方向"，否则等价于一个常数预测器。

    相关系数在预测或标签方差为 0（网络塌缩成常数）时无定义，此时返回 0.0，
    而不是让 numpy 抛 NaN 警告——塌缩本身已经由 `sign_acc≈50%` 与 MAE 暴露。
    """
    pred = np.asarray(pred, dtype=np.float32).ravel()
    y = np.asarray(y_true, dtype=np.float32).ravel()
    n = int(y.size)
    if n == 0:
        return {"n": 0, "mae": float("nan"), "rmse": float("nan"),
                "sign_acc": float("nan"), "corr": 0.0, "base_mae": float("nan")}
    diff = pred - y
    if pred.std() > 1e-9 and y.std() > 1e-9:
        corr = float(np.corrcoef(pred, y)[0, 1])
    else:
        corr = 0.0
    return {
        "n": n,
        "mae": float(np.mean(np.abs(diff))),
        "rmse": float(np.sqrt(np.mean(diff ** 2))),
        "sign_acc": float(np.mean(np.sign(pred) == np.sign(y))),
        "corr": corr,
        "base_mae": float(np.mean(np.abs(y))),
    }


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
              lr=0.15, batch=16, verbose=None, X_val=None, Y_val=None):
    """训练图像 → 转向 的端到端 CNN。

    backend="numpy"（默认）：纯 numpy `SimpleCNN`，无需 TensorFlow，模型存 JSON
    backend="tf"          ：tf.keras CNN，模型存 .h5（需自行安装 TensorFlow）

    `X_val` / `Y_val` 为留出验证集：传入后每个 epoch 结束会额外记录验证集 MSE
    （`history["val"]`），训练结束再给出验证集指标（`history["val_metrics"]`）。
    验证集**不参与梯度更新**，仅用于观察泛化。

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
        return _train_tf(images, labels, model_path, epochs, X_val=X_val, Y_val=Y_val)

    net = SimpleCNN(in_channels=3, hidden=8, out=1, img=(IMG_H, IMG_W), seed=0)
    hist = net.train(images, labels, epochs=epochs, lr=lr, batch=batch, verbose=verbose,
                     x_val=X_val, y_val=(None if Y_val is None else np.asarray(Y_val).ravel()))
    net.save(model_path)
    pred = net.predict(images.astype(np.float32))
    mae = float(np.mean(np.abs(pred.ravel() - labels)))
    print(f"端到端 CNN 训练 MAE = {mae:.5f}")
    print("模型已保存(纯 numpy JSON):", model_path)
    out = {"loss": list(hist), "mae": mae,
           "val": list(getattr(net, "val_history", [])),
           "val_metrics": None}
    if X_val is not None and Y_val is not None and len(X_val):
        pv = net.predict(np.asarray(X_val, dtype=np.float32))
        out["val_metrics"] = evaluate_steer(pv, Y_val)
        print(f"留出集（{out['val_metrics']['n']} 样本）MAE = "
              f"{out['val_metrics']['mae']:.5f}")
    return net, out


def _train_tf(images, labels, model_path, epochs, X_val=None, Y_val=None):
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
    # 有留出集就交给 keras 自己记验证曲线，否则退化成原先的 validation_split
    if X_val is not None and Y_val is not None and len(X_val):
        h = model.fit(images, labels, epochs=epochs,
                      validation_data=(np.asarray(X_val), np.asarray(Y_val).ravel()),
                      batch_size=32, verbose=2)
    else:
        h = model.fit(images, labels, epochs=epochs, batch_size=32,
                      validation_split=0.1, verbose=2)
    model.save(model_path)
    print("模型已保存(tf.keras):", model_path)
    return model, {"loss": list(h.history.get("loss", [])), "mae": float(h.history["mae"][-1]),
                   "val": list(h.history.get("val_loss", [])), "val_metrics": None}


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


# ------------------------------------------------------- 取证图：带标注的绘图区
# 评审指出取证图「没有标题、坐标轴、图例，图本身不能自证」。本模块的 PNG 是
# 纯 numpy + zlib 手写的（不依赖 Pillow / matplotlib），所以没有 set_xlabel 可用，
# 这里用 bitmap_font 的位图字体把标题/刻度/轴标签/图例直接写进像素。
_TXT = (40, 40, 40)
_TXT_LIGHT = (235, 235, 238)


def _plot_frame(canvas, left, right, top, bottom, title, xlabel, ylabel, y_ticks):
    """画带标题 / 坐标轴 / 刻度文字的绘图区，返回内框 (x0, y0, x1, y1)。"""
    h, w = canvas.shape[:2]
    x0, x1 = left, w - right
    y0, y1 = top, h - bottom

    draw_text(canvas, 8, 4, title, _TXT)                    # 标题
    canvas[y0:y1 + 1, x0:x0 + 2] = 190                      # y 轴
    canvas[y0:y1 + 1, x1 - 2:x1] = 190                      # 右边框
    canvas[y0:y0 + 2, x0:x1] = 190                          # 上边框
    canvas[y1 - 2:y1, x0:x1] = 190                          # x 轴

    draw_text(canvas, 6, y0 - 22, ylabel, _TXT)             # y 轴标签（横排，不旋转）
    for frac, label in y_ticks:                             # 刻度值 + 短刻度线
        yy = int(round(y1 - frac * (y1 - y0)))
        draw_text(canvas, max(2, x0 - text_width(label) - 8), yy - 7, label, _TXT)
        canvas[max(y0, yy - 1):yy + 1, x0 - 5:x0] = 150
    draw_text(canvas, (x0 + x1) // 2 - text_width(xlabel) // 2, y1 + 26, xlabel, _TXT)
    return x0, y0, x1, y1


def render_loss_png(losses, path, width=640, height=360):
    """训练损失曲线（MAE）：带标题、坐标轴、刻度值与轴标签。"""
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    v = np.asarray(losses, dtype=np.float32).ravel()
    if v.size < 2:
        return _write_png(path, canvas)
    vmin, vmax = float(v.min()), float(v.max())
    if vmax - vmin < 1e-9:
        vmax = vmin + 1.0
    x0, y0, x1, y1 = _plot_frame(
        canvas, 68, 22, 44, 58, "training loss (MAE)", "epoch", "MAE",
        [(1.0, f"{vmax:.4f}"), (0.5, f"{(vmin + vmax) / 2:.4f}"), (0.0, f"{vmin:.4f}")])
    xs = np.linspace(x0 + 6, x1 - 6, v.size).astype(int)
    ys = (y1 - 6 - (v - vmin) / (vmax - vmin) * (y1 - y0 - 12)).astype(int)
    for i in range(v.size - 1):
        _line(canvas, xs[i], ys[i], xs[i + 1], ys[i + 1], (200, 60, 60))
    draw_text(canvas, x0, y1 + 8, "1", _TXT)                    # x 轴端点：epoch
    draw_text(canvas, x1 - text_width(str(v.size)), y1 + 8, str(v.size), _TXT)
    return _write_png(path, canvas)


def render_pred_vs_true_png(y_true, y_pred, path, width=640, height=360):
    """预测转向 vs 真实转向：蓝=标签，红=CNN 预测。带标题、轴标签与图例。"""
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    n = len(y_true)
    if n < 2:
        return _write_png(path, canvas)
    x0, y0, x1, y1 = _plot_frame(
        canvas, 60, 22, 44, 58, "steer: pred (red) vs true (blue)",
        "sample (sorted by steer)", "steer", [(1.0, "+1"), (0.5, "0"), (0.0, "-1")])
    mid = (y0 + y1) // 2
    canvas[mid - 1:mid + 1, x0 + 2:x1 - 2] = 225        # steer=0 参考线

    def to_xy(vals):
        xs = np.linspace(x0 + 6, x1 - 6, n).astype(int)
        ys = (mid - np.clip(vals, -1, 1) * ((y1 - y0) / 2 - 8)).astype(int)
        return xs, ys

    xs, ys_t = to_xy(np.asarray(y_true, dtype=np.float32))
    _xs, ys_p = to_xy(np.asarray(y_pred, dtype=np.float32))
    for i in range(n - 1):
        _line(canvas, xs[i], ys_t[i], xs[i + 1], ys_t[i + 1], (60, 90, 200))
        _line(canvas, xs[i], ys_p[i], xs[i + 1], ys_p[i + 1], (210, 50, 50))

    # 图例（绘图区内右上角）：红线=预测，蓝线=标签
    lx, ly = x1 - 128, y0 + 10
    canvas[ly:ly + 4, lx:lx + 18] = (210, 50, 50)
    draw_text(canvas, lx + 24, ly - 5, "pred", _TXT)
    canvas[ly + 20:ly + 24, lx:lx + 18] = (60, 90, 200)
    draw_text(canvas, lx + 24, ly + 15, "true", _TXT)
    return _write_png(path, canvas)


def render_train_val_png(train_curve, val_curve, path, width=640, height=360):
    """训练 / 留出集 MSE 曲线对比（蓝=训练，红=留出集），带标题、坐标轴与图例。

    为什么值得单独画一张：只看训练损失无法判断过拟合——训练曲线一路下降、
    留出集曲线却可能在中途抬头。两条线一起看，"学没学会"与"有没有背题"才分得开。
    """
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    tr = np.asarray(train_curve, dtype=np.float32).ravel()
    va = np.asarray(val_curve, dtype=np.float32).ravel()
    if tr.size < 2:
        return _write_png(path, canvas)
    vmin = float(min(tr.min(), va.min() if va.size else tr.min()))
    vmax = float(max(tr.max(), va.max() if va.size else tr.max()))
    if vmax - vmin < 1e-9:
        vmax = vmin + 1.0
    x0, y0, x1, y1 = _plot_frame(
        canvas, 68, 22, 44, 58, "MSE: train (blue) vs hold-out val (red)",
        "epoch", "MSE",
        [(1.0, f"{vmax:.4f}"), (0.5, f"{(vmin + vmax) / 2:.4f}"), (0.0, f"{vmin:.4f}")])

    def to_xy(v):
        xs = np.linspace(x0 + 6, x1 - 6, v.size).astype(int)
        ys = (y1 - 6 - (v - vmin) / (vmax - vmin) * (y1 - y0 - 12)).astype(int)
        return xs, ys

    xs, ys = to_xy(tr)
    for i in range(tr.size - 1):
        _line(canvas, xs[i], ys[i], xs[i + 1], ys[i + 1], (60, 90, 200))
    if va.size >= 2:
        xs2, ys2 = to_xy(va)
        for i in range(va.size - 1):
            _line(canvas, xs2[i], ys2[i], xs2[i + 1], ys2[i + 1], (210, 50, 50))

    # 图例（绘图区内右上角）
    lx, ly = x1 - 118, y0 + 10
    canvas[ly:ly + 4, lx:lx + 18] = (60, 90, 200)
    draw_text(canvas, lx + 24, ly - 5, "train", _TXT)
    canvas[ly + 20:ly + 24, lx:lx + 18] = (210, 50, 50)
    draw_text(canvas, lx + 24, ly + 15, "val", _TXT)
    draw_text(canvas, x0, y1 + 8, "1", _TXT)
    draw_text(canvas, x1 - text_width(str(tr.size)), y1 + 8, str(tr.size), _TXT)
    return _write_png(path, canvas)


def render_ablation_png(rows, shifts, path, width=720, height=400):
    """数据增强消融：**分组柱状图**——横轴是光度条件，每组内是各配置的留出集 MAE。

    为什么按"光度条件"分组：干净留出集上各配置差异很小（任务接近饱和），真正
    拉开差距的是**分布偏移**。把 clean / mild / strong 并排放，才能一眼看出
    "增强带来的不是干净集精度，而是偏移下的鲁棒性"。

    `rows` 形如 `[{"label": "base", "mae": {"clean": 0.058, "mild": 0.19}}, ...]`；
    `shifts` 是条件名列表；`rows[0]["base_mae"]` 若存在则画零输出基线虚线。
    文字只能用 ASCII（位图字体只含 ASCII 32..126），中文说明放在文档表格里。
    """
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    if not rows or not shifts:
        return _write_png(path, canvas)
    all_vals = [v for r in rows for v in r["mae"].values()]
    base = rows[0].get("base_mae")
    top = max(all_vals + ([float(base)] if base else [])) * 1.18
    if top <= 0:
        top = 1.0
    x0, y0, x1, y1 = _plot_frame(
        canvas, 68, 22, 44, 86, "hold-out MAE under photometric shift",
        "shift level", "MAE",
        [(1.0, f"{top:.3f}"), (0.5, f"{top / 2:.3f}"), (0.0, "0")])

    colors = [(120, 120, 120), (60, 160, 90), (60, 120, 200), (210, 120, 60)]
    n_cfg = len(rows)
    slot = (x1 - x0 - 10) / float(len(shifts))
    bw = int(slot / (n_cfg + 0.8))
    for gi, sh in enumerate(shifts):
        gx = x0 + 5 + int(slot * gi)
        for ci, r in enumerate(rows):
            val = float(r["mae"].get(sh, float("nan")))
            if not np.isfinite(val):
                continue
            cx = gx + int(bw * (ci + 0.35))
            hgt = int(val / top * (y1 - y0 - 14))
            canvas[y1 - 2 - hgt:y1 - 2, cx:cx + bw - 2] = colors[ci % len(colors)]
            # 柱顶标数值：图离开文档表格也能自证（评审要求"图本身能说明问题"）
            vtxt = f"{val:.3f}"
            draw_text(canvas, cx + max(0, (bw - text_width(vtxt)) // 2),
                      max(y0 + 2, y1 - 6 - hgt), vtxt, _TXT)
        draw_text(canvas, gx + slot // 2 - text_width(sh) // 2, y1 + 8, sh, _TXT)

    # 图例（绘图区内右上角）：色块 + 配置名
    ly = y0 + 8
    for ci, r in enumerate(rows):
        lx = x1 - 150
        canvas[ly:ly + 4, lx:lx + 14] = colors[ci % len(colors)]
        draw_text(canvas, lx + 20, ly - 5, str(r["label"]), _TXT)
        ly += 18

    if base:
        yy = int(round(y1 - 2 - float(base) / top * (y1 - y0 - 14)))
        for xx in range(x0 + 4, x1 - 4, 8):          # 虚线：恒输出 0 的基线
            canvas[yy:yy + 2, xx:min(x1 - 4, xx + 4)] = (150, 150, 150)
        label = f"zero-output {float(base):.3f}"
        draw_text(canvas, x0 + 6, max(y0 + 2, yy - 16), label, _TXT)
    return _write_png(path, canvas)


def render_samples_png(images, steers, preds, path, cols=4, scale=3):
    """把若干张样本图像横向拼成一张图，每张下方标注真实/预测转向。"""
    n = min(cols, len(images))
    if n == 0:
        return _write_png(path, np.full((IMG_H, IMG_W, 3), 255, np.uint8))
    h, w = images[0].shape[:2]
    gap = 8
    cap_h = 36
    canvas = np.full((h + cap_h, n * w + (n - 1) * gap, 3), 30, dtype=np.uint8)
    for i in range(n):
        x0 = i * (w + gap)
        canvas[:h, x0:x0 + w] = images[i][:, :, :3]
        draw_text(canvas, x0 + 2, h + 3, f"true {float(steers[i]):+.2f}", _TXT_LIGHT)
        draw_text(canvas, x0 + 2, h + 20, f"pred {float(preds[i]):+.2f}", _TXT_LIGHT)
    if scale > 1:
        canvas = np.repeat(np.repeat(canvas, scale, axis=0), scale, axis=1)
    return _write_png(path, canvas)


# ======================================================== 离线取证（无需 CARLA）
GIF_SCALE = 4          # 动图里把 60×80 的相机图放大 4 倍
GIF_BAR_H = 48         # 转向指示条高度
GIF_PROG_H = 6         # 底部播放进度条高度


def render_e2e_frame(img, y_true, y_pred, frac, scale=GIF_SCALE):
    """把「相机图像 + 转向指示条」画成一帧 RGB，用于合成端到端演示动图。

    指示条里白色竖线是专家标签（这条弯道应该打多少转向），红色竖线是 CNN 的
    预测；中间灰线是零转向。两条线贴得越近，说明「看到的弯」与「打的方向」越一致。
    最底部细条是播放进度。
    """
    h, w = img.shape[:2]
    ih, iw = h * scale, w * scale
    total_h = ih + GIF_BAR_H + GIF_PROG_H
    canvas = np.full((total_h, iw, 3), 30, dtype=np.uint8)
    canvas[:ih] = np.repeat(np.repeat(img[:, :, :3], scale, axis=0), scale, axis=1)

    top = ih
    cx = iw // 2
    half = iw // 2 - 8
    canvas[top:top + GIF_BAR_H] = (30, 30, 30)
    canvas[top:top + GIF_BAR_H, cx - 1:cx + 1] = (90, 90, 90)     # 零转向参考线
    # 两条竖线上下错开一点：预测正确时二者重合，若完全叠画就只剩红色，
    # 看不出「标签在哪」。错开后上段是白（标签）、下段是红（预测），重合也都能看见。
    for val, color, r0, r1 in ((y_true, (240, 240, 240), 4, GIF_BAR_H - 10),
                               (y_pred, (220, 50, 50), 10, GIF_BAR_H - 4)):
        x = int(round(cx + float(np.clip(val, -1.0, 1.0)) * half))
        x = max(3, min(iw - 5, x))
        canvas[top + r0:top + r1, x - 2:x + 2] = color

    n = max(1, int(round(float(frac) * iw)))
    canvas[total_h - GIF_PROG_H:] = (55, 55, 55)
    canvas[total_h - GIF_PROG_H:, :n] = (60, 120, 230)
    return canvas


def run_augment_ablation(X_tr, Y_tr, X_val, Y_val, epochs, save_dir, seed=0,
                         aug_gain=(0.45, 1.45), aug_noise=12.0):
    """数据增强消融：三组配置 × 三档光度条件，全部在同一份留出集上评测。

    三组配置用的是**同一份训练数据、同一组验证样本、同一个随机初始化**，
    只有喂进网络的样本不同，否则比出来的差异说明不了问题：

      A `base`    只用原始训练样本；
      B `copy x2` 把同一批样本复制一份 —— **控制组**：数据量翻倍但没有新信息；
      C `aug`     镜像 + 光度扰动（同样 2 倍数据量，但带来新的视角/亮度组合）。

    评测分三档光度条件（`SHIFT_LEVELS`）：干净 / 轻度偏移 / 强偏移。
    只看干净留出集会得出"增强没用甚至更差"的错误结论——干净集上任务已接近饱和；
    增强真正的收益体现在**输入端分布变化**时（对应文档 Q5「合成好、实车不 work」）。

    返回 `(rows, png_path, csv_path)`；`rows` 每项含 `label` 与 `mae`（按条件）。
    """
    Xa, Ya = augment_dataset(X_tr, Y_tr, copies=1, seed=seed + 7,
                             gain_range=aug_gain, noise=aug_noise)
    configs = [
        ("base", np.asarray(X_tr), np.asarray(Y_tr)),
        ("copy x2", np.concatenate([X_tr, X_tr]), np.concatenate([Y_tr, Y_tr])),
        ("aug", np.concatenate([X_tr, Xa]), np.concatenate([Y_tr, Ya])),
    ]

    # 预先算好三档条件下的评测图像（同一 seed：各配置看到完全相同的偏移）
    eval_sets = []
    for name, gain, noise in SHIFT_LEVELS:
        Xe = (np.asarray(X_val) if (gain == 1.0 and noise == 0.0)
              else photometric_shift(X_val, gain, noise, seed=seed + 31))
        eval_sets.append((name, Xe))

    rows, csv_rows, base_mae = [], [], None
    for label, Xf, Yf in configs:
        # 每组都用同一 seed 初始化：权重初值一致，差异只来自数据构成
        net = SimpleCNN(in_channels=3, hidden=8, out=1, img=(IMG_H, IMG_W), seed=0)
        net.train(Xf, Yf.ravel(), epochs=epochs, lr=0.15, batch=16, verbose=0)
        per_shift, cells = {}, []
        for name, Xe in eval_sets:
            m = evaluate_steer(net.predict(np.asarray(Xe, dtype=np.float32)), Y_val)
            per_shift[name] = m["mae"]
            csv_rows.append(f"{label},{name},{m['mae']:.6f},{m['rmse']:.6f},"
                            f"{m['sign_acc']:.4f},{m['corr']:+.6f},{len(Xf)},{m['n']}")
            cells.append(f"{name}={m['mae']:.5f}")
        if base_mae is None:
            base_mae = m["base_mae"]
        rows.append({"label": label, "mae": per_shift, "base_mae": base_mae})
        print(f"  [{label:8s}] n_train={len(Xf):4d}  " + "  ".join(cells))

    shift_names = [name for name, _gain, _noise in SHIFT_LEVELS]
    png = render_ablation_png(rows, shift_names,
                              os.path.join(save_dir, "e2e_shift_ablation.png"))
    csv_path = os.path.join(save_dir, "e2e_shift_ablation.csv")
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("config,shift,val_mae,val_rmse,val_sign_acc,val_corr,n_train,n_val\n")
        f.write("\n".join(csv_rows) + "\n")

    # 用"强偏移 / 干净"的退化倍数讲清收益：增强该把退化压下去多少
    base_row = rows[0]
    for r in rows:
        if r["label"] == "aug" and base_row["mae"]["clean"] > 1e-9:
            print(f"  强偏移下：base {base_row['mae']['strong']:.5f} → "
                  f"aug {r['mae']['strong']:.5f}"
                  f"（退化倍数 {base_row['mae']['strong'] / base_row['mae']['clean']:.1f}× → "
                  f"{r['mae']['strong'] / max(1e-9, r['mae']['clean']):.1f}×）")
    return rows, png, csv_path


def run_offline_demo(save_dir, epochs=60, n_samples=150, seed=0,
                     val_ratio=0.25, augment=False, ablation=False,
                     aug_gain=(0.45, 1.45), aug_noise=12.0):
    """离线自检：合成道路图像 → 训练端到端 CNN → 导出证据图。

    **不需要 CARLA、不需要图形界面、不需要 TensorFlow**，用于在无 3D 加速的
    虚拟机 / CI 中产出"可运行性"证据，并验证「图像 → 转向」确实被学到。

    默认就把数据切成**训练 / 留出**两部分，指标同时报两个口径：
    只在训练集上看 MAE 会掩盖过拟合（训练集 0.09、换一批同分布图像明显变差），
    而"训练集上很漂亮"恰恰是行为克隆最常见的隐性缺陷。

    `val_ratio` 留出集比例（0 表示不切分，退回旧行为：只报训练集口径）；
    `augment`   训练前对训练集做镜像 + 光度扰动增强；
    `ablation`  额外跑三组对照并把留出集 MAE 画成柱状图（见 run_augment_ablation）。
    """
    print("=" * 64)
    print("  作业四 离线自检取证模式（合成道路图像，不需要 CARLA）")
    print("=" * 64)

    # ---- 合成数据集：图像里的车道线弯曲量与标签一致 ----
    X, Y = synth_dataset(n=n_samples, seed=seed)
    print(f"合成道路图像数据集: {X.shape}，转向标签范围 [{Y.min():+.2f}, {Y.max():+.2f}]")

    # ---- 切成训练 / 留出：验证集不参与任何梯度更新 ----
    X_tr, Y_tr, X_val, Y_val = split_dataset(X, Y, val_ratio=val_ratio)
    has_val = X_val is not None and len(X_val) > 0
    if has_val:
        print(f"切分：训练 {len(X_tr)} 样本 / 留出验证 {len(X_val)} 样本"
              f"（val_ratio={val_ratio:g}，seed={VAL_SEED}）")
    else:
        print("未切分留出集（val_ratio=0），指标仅为训练集口径")

    # ---- 可选数据增强：只加在训练集上，验证集保持原始分布 ----
    X_fit, Y_fit = X_tr, Y_tr
    if augment:
        Xa, Ya = augment_dataset(X_tr, Y_tr, copies=1, seed=seed + 1,
                                 gain_range=aug_gain, noise=aug_noise)
        X_fit = np.concatenate([X_tr, Xa])
        Y_fit = np.concatenate([Y_tr, Ya])
        print(f"数据增强：镜像+光度扰动（增益 {aug_gain[0]:.2f}~{aug_gain[1]:.2f}，"
              f"噪声 σ={aug_noise:g}），训练样本 {len(X_tr)} → {len(X_fit)}"
              "（验证集不增强，保持真实分布）")

    # ---- 训练端到端 CNN ----
    os.makedirs(save_dir, exist_ok=True)
    model_path = os.path.join(save_dir, "cnn_steer.json")
    net, hist = train_cnn(X_fit, Y_fit, model_path, epochs=epochs, backend="numpy",
                          X_val=(X_val if has_val else None),
                          Y_val=(Y_val if has_val else None))

    # ---- 评估：训练集与留出集分开报（两个口径不能混）----
    m_tr = evaluate_steer(net.predict(np.asarray(X_tr, dtype=np.float32)), Y_tr)
    pred_val = None
    m_val = None
    if has_val:
        pred_val = net.predict(np.asarray(X_val, dtype=np.float32)).ravel()
        m_val = evaluate_steer(pred_val, Y_val)

    def _show(tag, m):
        print(f"{tag}：MAE = {m['mae']:.5f}，RMSE = {m['rmse']:.5f}，"
              f"转向方向一致率 = {m['sign_acc'] * 100:.1f}%，"
              f"相关系数 = {m['corr']:+.5f}")

    print()
    _show(f"训练集口径（n={m_tr['n']}）", m_tr)
    if m_val:
        _show(f"留出集口径（n={m_val['n']}）", m_val)
    base_ref = m_val if m_val else m_tr
    print(f"零输出基线 MAE = {base_ref['base_mae']:.5f}"
          "（MAE 明显低于基线才说明网络学到了映射）")

    # CSV：留出集优先（更能代表泛化），没有留出集才退回训练集
    pred_tr = net.predict(np.asarray(X_tr, dtype=np.float32)).ravel()
    csv_src_y = (Y_val.ravel() if has_val else Y_tr.ravel())
    csv_src_p = (pred_val if has_val else pred_tr)
    csv_path = os.path.join(save_dir, "steer_pred_vs_true.csv")
    np.savetxt(csv_path, np.column_stack([csv_src_y, csv_src_p]), delimiter=",",
               header="steer_true,steer_pred", comments="", fmt="%.6f")
    print(f"预测与标签已存 CSV（{'留出集' if has_val else '训练集'}）：{csv_path}")

    # ---- 导出证据图 ----
    if has_val and hist.get("val"):
        p1 = render_train_val_png(hist["loss"], hist["val"],
                                  os.path.join(save_dir, "cnn_loss.png"))
    else:
        p1 = render_loss_png(hist["loss"], os.path.join(save_dir, "cnn_loss.png"))

    fig_y = csv_src_y
    fig_p = csv_src_p
    order = np.argsort(fig_y)              # 按真实转向排序，便于看两条曲线贴合程度
    p2 = render_pred_vs_true_png(fig_y[order], fig_p[order],
                                 os.path.join(save_dir, "steer_pred_vs_true.png"))

    show_X = X_val if has_val else X_tr
    show_Y = (Y_val.ravel() if has_val else Y_tr.ravel())
    show_P = (pred_val if has_val else fig_p)
    p3 = render_samples_png(show_X[:4], show_Y[:4], show_P[:4],
                            os.path.join(save_dir, "road_samples.png"))

    # ---- 动图：图像 → 转向（按真实转向从小到大扫一遍，看得清映射关系）----
    # 用 noise=0 重新合成：GIF 只有 256 色，带噪声的图有上万种颜色，索引化不了。
    n_gif = 24
    pick = np.argsort(show_Y)[np.linspace(0, len(show_X) - 1, n_gif).astype(int)]
    gif_rng = np.random.RandomState(seed)
    gif_frames = []
    for i, k in enumerate(pick):
        clean = synth_road_image(float(show_Y[k]), gif_rng, noise=0.0)
        gif_frames.append(render_e2e_frame(
            clean, float(show_Y[k]), float(show_P[k]), (i + 1) / float(n_gif)))
    p4 = rgb_frames_to_gif(os.path.join(save_dir, "e2e_demo.gif"),
                           gif_frames, delay_cs=12)

    print("\n已导出取证图：")
    for p in (p1, p2, p3, p4):
        print("   ", p)
    print(f"    动图 {len(gif_frames)} 帧，{os.path.getsize(p4) / 1024:.0f} KB")

    # ---- 可选消融：同样数据量下，增强 vs 仅复制；并对比三档光度偏移 ----
    if ablation and has_val:
        print("\n数据增强消融（同一初始化、同一留出集，只改训练样本构成；"
              "按光度偏移分档评测）：")
        rows, p5, p6 = run_augment_ablation(X_tr, Y_tr, X_val, Y_val, epochs, save_dir,
                                            seed=seed, aug_gain=aug_gain,
                                            aug_noise=aug_noise)
        print(f"    已导出消融图：{p5}")
        print(f"    已导出指标表：{p6}")
        best = min(rows, key=lambda r: r["mae"]["strong"])
        print(f"    强偏移下留出集 MAE 最优配置：{best['label']}"
              f"（{best['mae']['strong']:.5f}）")
    elif ablation:
        print("\n[提示] val_ratio=0 时无法做消融（需要留出集来比较），已跳过。")
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
    p.add_argument("--val_ratio", type=float, default=0.25,
                   help="离线取证：留出验证集比例（0 表示不切分，只报训练集口径）")
    _add_bool(p, "--augment",
              "离线取证：训练集启用数据增强（镜像+光度扰动，验证集不增强）")
    p.add_argument("--aug_gain", default="0.45,1.45", type=parse_gain,
                   help="增强亮度增益区间，形如 0.45,1.45（要覆盖部署时的光照变化，"
                        "范围过窄实测反而更差）")
    p.add_argument("--aug_noise", type=float, default=12.0,
                   help="增强加性噪声标准差（像素值）")
    _add_bool(p, "--ablation",
              "离线取证：运行数据增强消融（基线/复制扩容/增强三组 + 三档光度偏移）")
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


def parse_gain(text):
    """解析 `--aug_gain 0.45,1.45` 这类区间参数（也接受单个数表示上下界相同）。

    公开（非 `_` 前缀）是因为单元测试与外部脚本都要用它校验参数解析；
    包根的 `main.py` 是 `import *` 转发壳，下划线开头的名字不会被导出。
    """
    parts = [p.strip() for p in str(text).replace("~", ",").split(",") if p.strip()]
    if len(parts) == 1:
        lo = hi = float(parts[0])
    elif len(parts) == 2:
        lo, hi = float(parts[0]), float(parts[1])
    else:
        raise argparse.ArgumentTypeError(f"增益区间应形如 0.45,1.45，收到：{text!r}")
    if not (0.0 < lo <= hi):
        raise argparse.ArgumentTypeError(f"增益区间需满足 0 < low <= high，收到：{text!r}")
    return (lo, hi)


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.headless or args.demo:
        return run_offline_demo(args.save_dir or "shots", epochs=args.epochs,
                                n_samples=args.samples, val_ratio=args.val_ratio,
                                augment=bool(args.augment), ablation=bool(args.ablation),
                                aug_gain=args.aug_gain, aug_noise=args.aug_noise)

    if args.mode == "train":
        X, Y = load_dataset(args.data_dir)
        if len(X) == 0:
            print(f"[错误] 数据集为空：{args.data_dir}")
            print("       请先采集：python3 main.py --mode collect --host <IP> --frames 300")
            print("       或离线取证：python3 main.py --headless --demo --save_dir ~/shots")
            return 1
        # 真实采集的数据同样切留出集：训练集指标好看、换一段路就崩，是行为克隆
        # 最常见的失败方式；留出集能提前暴露这个问题。
        X_tr, Y_tr, X_val, Y_val = split_dataset(X, Y, val_ratio=args.val_ratio)
        train_cnn(X_tr, Y_tr, args.model_path, epochs=args.epochs, backend=args.backend,
                  X_val=X_val, Y_val=Y_val)
        return 0

    if args.mode == "collect":
        return collect_carla(args.host, args.port, args.town, args.frames,
                             args.out_dir, backend=args.backend)
    return test_carla(args.host, args.port, args.town, args.model_path,
                      args.sim_time, backend=args.backend, save_dir=args.save_dir)


if __name__ == "__main__":
    sys.exit(main())
