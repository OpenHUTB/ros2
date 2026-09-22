#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业四 · CARLA 端到端神经网络【图像 → 控制】（任务 4）

对应老师任务 (4)：端到端模型（输入图像、输出控制指令）。

用 CARLA 0.9.16 前视相机图像驱动一个 CNN：
    相机图像(60×80×3) --CNN--> steer ∈ [-1,1] --差速/转向--> (油门, 转向)
三阶段：
   collect   采集 (图像, steer 标签)
   train     训练 CNN（有监督回归，MAE 损失）
   test      加载模型自主驾驶

用法（独立，需先启动 CARLA server）：
    采集：  python main.py --mode collect --frames 200 --out_dir dataset
    训练：  python main.py --mode train --epochs 30 --data_dir dataset
    测试：  python main.py --mode test --model_path models/cnn.h5

ROS launch：
    roslaunch 04_end_to_end/launch/main.launch.py mode:=test
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import (  # noqa: E402
    connect, spawn_vehicle, make_rgb_camera,
    apply_control, get_location, get_yaw, _tick_once,
    DEFAULT_HOST, DEFAULT_PORT, DEFAULT_MAP, DT,
)

IMG_H, IMG_W = 60, 80


# ===================================================== CNN 模型
def build_cnn(img_shape=(IMG_H, IMG_W, 3)):
    """图像 -> 转向角（单输出 tanh）。"""
    import tensorflow as tf
    from tensorflow.keras import layers, models
    inputs = layers.Input(shape=img_shape, name="input")
    x = layers.Normalization()(inputs)
    x = layers.Conv2D(16, 3, activation="relu", padding="same")(x)
    x = layers.MaxPooling2D(2)(x)
    x = layers.Conv2D(32, 3, activation="relu", padding="same")(x)
    x = layers.MaxPooling2D(2)(x)
    x = layers.Conv2D(64, 3, activation="relu", padding="same")(x)
    x = layers.GlobalAveragePooling2D()(x)
    x = layers.Dense(64, activation="relu")(x)
    x = layers.Dropout(0.3)(x)
    out = layers.Dense(1, activation="tanh", name="steer")(x)
    model = models.Model(inputs, out)
    model.compile(optimizer="adam", loss="mae", metrics=["mae"])
    return model


# ===================================================== 采集
def collect(world, vehicle, frames, out_dir):
    """用正弦转向生成多样轨迹，采集 (图像, steer)。"""
    import cv2
    holder = {"img": None, "yaw": 0.0}
    make_rgb_camera(world, vehicle,
                    lambda im: holder.update(img=im),
                    width=IMG_W, height=IMG_H, tick=True)
    os.makedirs(os.path.join(out_dir, "images"), exist_ok=True)
    labels = []
    with open(os.path.join(out_dir, "labels.txt"), "w") as f:
        for i in range(frames):
            # 用正弦转向生成多样控制轨迹
            steer = 0.5 * math.sin(i / 12.0) + 0.2 * math.cos(i / 5.0)
            throttle = 0.5 + 0.2 * np.sin(i / 20.0)
            apply_control(vehicle, throttle=throttle, steer=steer)
            world.tick()
            img = holder["img"]
            if img is None:
                continue
            # 用车道偏置作为标签的更稳健做法：这里仍用真实施加的 steer 作为标签
            f.write(f"{steer:.6f}\n")
            labels.append(steer)
            cv2.imwrite(os.path.join(out_dir, "images", f"{i:05d}.jpg"), img)
            if i % 50 == 0:
                print(f"  collect {i}/{frames}")
    print("数据采集完成 ->", out_dir)


# ===================================================== 训练
def train(model_path, data_dir, epochs, backend="tf"):
    """读取采集的 (图像, steer) 训练一个图像->转向 端到端网络。

    backend:
      - "tf"    : tf.keras CNN（默认，需要 TensorFlow，模型 .h5）
      - "numpy" : 纯 numpy SimpleCNN（无需 TensorFlow，模型 .json），便于离线验证
    """
    import cv2
    img_dir = os.path.join(data_dir, "images")
    lbl_file = os.path.join(data_dir, "labels.txt")
    names = sorted(os.listdir(img_dir))
    with open(lbl_file) as f:
        labels = [float(l.split()[0]) for l in f]
    X, Y = [], []
    assert len(labels) == len(names), "标签数与图像数不一致"
    for i, name in enumerate(names):
        img = cv2.imread(os.path.join(img_dir, name))
        if img is None:
            continue
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
        Y.append(labels[i])
        X.append(img)
    X_arr = np.asarray(X).astype(np.uint8) if len(X) else np.zeros((0, IMG_H, IMG_W, 3), np.uint8)
    Y_arr = np.asarray(Y).reshape(-1, 1) if len(Y) else np.zeros((0, 1))
    print(f"数据集: X{X_arr.shape} Y{Y_arr.shape}  backend={backend}")

    d = os.path.dirname(model_path)
    if d:
        os.makedirs(d, exist_ok=True)

    if backend == "numpy":
        from nn_models import SimpleCNN
        net = SimpleCNN(in_channels=3, hidden=8, out=1, img=(IMG_H, IMG_W), seed=0)
        net.train(X_arr, Y_arr.ravel(), epochs=epochs, lr=0.05, batch=16)
        json_path = model_path.rsplit(".", 1)[0] + ".json"
        net.save(json_path)
        print("模型已保存(纯numpy JSON):", json_path)
        return

    m = build_cnn()
    m.fit(X_arr, Y_arr, epochs=epochs, batch_size=32, validation_split=0.1)
    m.save(model_path)
    print("模型已保存(tf.keras .h5):", model_path)


# ===================================================== 测试
def test(world, vehicle, model_path, sim_time=10.0, backend="tf"):
    import tensorflow as tf
    if backend == "numpy":
        from nn_models import SimpleCNN
        json_path = model_path.rsplit(".", 1)[0] + ".json"
        net = SimpleCNN.load(json_path)
        model = None  # numpy 网络用 predict 接口
    else:
        net = None
        model = tf.keras.models.load_model(model_path, compile=False)
    holder = {"img": None}
    make_rgb_camera(world, vehicle,
                    lambda im: holder.update(img=im),
                    width=IMG_W, height=IMG_H, tick=True)
    n_ticks = int(sim_time / DT)
    for k in range(n_ticks):
        img = holder["img"]
        if img is not None:
            if backend == "numpy":
                steer = float(net.predict(img.astype(np.uint8))[0, 0])
            else:
                img_in = img[np.newaxis].astype(np.float32)
                steer = float(model.predict(img_in, verbose=0)[0][0])
            throttle = 0.5
            apply_control(vehicle, throttle=throttle, steer=steer)
        world.tick()
        if k % 20 == 0:
            x, y = get_location(vehicle)
            print(f"[t={k*DT:.1f}] steer={steer:+.3f} pos=({x:.2f},{y:.2f})")
    print("端到端驾驶完成")


def cli():
    p = argparse.ArgumentParser(description="CARLA 端到端 CNN 任务4")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--town", default=DEFAULT_MAP)
    p.add_argument("--mode", choices=["collect", "train", "test"], required=True)
    p.add_argument("--frames", type=int, default=200)
    p.add_argument("--out_dir", default="dataset")
    p.add_argument("--data_dir", default="dataset")
    p.add_argument("--epochs", type=int, default=30)
    p.add_argument("--model_path", default="models/cnn.h5")
    p.add_argument("--sim_time", type=float, default=10.0)
    # backend：tf（默认，tf.keras CNN）或 numpy（纯 numpy SimpleCNN，无需 TensorFlow）
    p.add_argument("--backend", choices=["tf", "numpy"], default="tf")
    args = p.parse_args()

    if args.mode == "train":
        train(args.model_path, args.data_dir, args.epochs, backend=args.backend)
        return

    # collect / test 都需要连 CARLA
    client, world = connect(args.host, args.port, args.town)
    vehicle, tf = spawn_vehicle(world)
    if args.mode == "collect":
        collect(world, vehicle, args.frames, args.out_dir)
    elif args.mode == "test":
        test(world, vehicle, args.model_path, args.sim_time, backend=args.backend)
    vehicle.destroy()


if __name__ == "__main__":
    cli()
