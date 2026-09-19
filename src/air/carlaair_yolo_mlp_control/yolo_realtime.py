# -*- coding: utf-8 -*-

import os
import csv
import time
from datetime import datetime

import airsim
import cv2
import numpy as np
import torch
from ultralytics import YOLO


# ============================================================
# 配置
# ============================================================

AIRSIM_IP = "127.0.0.1"
AIRSIM_PORT = 41451

CAMERA_NAME = "0"

MODEL_PATH = "yolo11n.pt"

CONF_THRESHOLD = 0.15
IMAGE_SIZE = 960

OUTPUT_DIR = "output"
SCREENSHOT_DIR = os.path.join(OUTPUT_DIR, "screenshots")
CSV_PATH = os.path.join(OUTPUT_DIR, "results.csv")


# ============================================================
# 创建输出文件夹
# ============================================================

os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(SCREENSHOT_DIR, exist_ok=True)


# ============================================================
# GPU
# ============================================================

print("=" * 60)
print("CarlaAir + YOLO 实时目标检测")
print("=" * 60)

if torch.cuda.is_available():
    device = 0
    gpu_name = torch.cuda.get_device_name(0)

    print("CUDA: True")
    print("GPU:", gpu_name)
else:
    device = "cpu"
    gpu_name = "CPU"

    print("CUDA: False")
    print("使用 CPU")


# ============================================================
# 加载 YOLO
# ============================================================

print("\n正在加载 YOLO...")

model = YOLO(MODEL_PATH)

print("YOLO 加载完成:", MODEL_PATH)


# ============================================================
# 连接 AirSim
# ============================================================

print("\n正在连接 AirSim...")

client = airsim.MultirotorClient(
    ip=AIRSIM_IP,
    port=AIRSIM_PORT
)

client.confirmConnection()

print("AirSim 连接成功")


# ============================================================
# 测试相机
# ============================================================

test = client.simGetImages([
    airsim.ImageRequest(
        CAMERA_NAME,
        airsim.ImageType.Scene,
        False,
        True
    )
])

if not test or test[0].width == 0:
    print("无法获取 RGB 图像")
    raise SystemExit

print(
    "RGB 相机正常:",
    test[0].width,
    "x",
    test[0].height
)


# ============================================================
# CSV
# ============================================================

csv_file = open(
    CSV_PATH,
    "w",
    newline="",
    encoding="utf-8-sig"
)

writer = csv.writer(csv_file)

writer.writerow([
    "frame",
    "time",
    "fps",
    "inference_ms",
    "capture_ms",
    "objects"
])


# ============================================================
# 统计
# ============================================================

frame_id = 0
fps = 0.0

fps_counter = 0
fps_start = time.time()

fps_list = []
inference_list = []
capture_list = []


# ============================================================
# OpenCV 窗口
# ============================================================

WINDOW_NAME = "CarlaAir + YOLO"

cv2.namedWindow(
    WINDOW_NAME,
    cv2.WINDOW_NORMAL
)

cv2.resizeWindow(
    WINDOW_NAME,
    1280,
    720
)


print("\n实时检测开始")
print("Q = 退出")
print("S = 保存截图\n")


# ============================================================
# 主循环
# ============================================================

try:

    while True:

        # ----------------------------------------------------
        # AirSim RGB
        # ----------------------------------------------------

        capture_start = time.perf_counter()

        responses = client.simGetImages([
            airsim.ImageRequest(
                CAMERA_NAME,
                airsim.ImageType.Scene,
                False,
                True
            )
        ])

        capture_ms = (
            time.perf_counter()
            - capture_start
        ) * 1000


        if not responses:
            continue

        response = responses[0]

        if response.width == 0:
            continue


        # ----------------------------------------------------
        # bytes -> OpenCV
        # ----------------------------------------------------

        image_data = np.frombuffer(
            response.image_data_uint8,
            dtype=np.uint8
        )

        frame = cv2.imdecode(
            image_data,
            cv2.IMREAD_COLOR
        )

        if frame is None:
            continue


        # ----------------------------------------------------
        # YOLO
        # ----------------------------------------------------

        results = model.predict(
            frame,
            conf=CONF_THRESHOLD,
            imgsz=IMAGE_SIZE,
            device=device,
            verbose=False
        )

        result = results[0]

        inference_ms = float(
            result.speed["inference"]
        )

        object_count = len(result.boxes)


        # ----------------------------------------------------
        # 检测框
        # ----------------------------------------------------

        annotated = result.plot()


        # ----------------------------------------------------
        # FPS
        # ----------------------------------------------------

        frame_id += 1
        fps_counter += 1

        elapsed = time.time() - fps_start

        if elapsed >= 1.0:

            fps = fps_counter / elapsed

            fps_counter = 0
            fps_start = time.time()


        if fps > 0:
            fps_list.append(fps)

        inference_list.append(inference_ms)
        capture_list.append(capture_ms)


        # ----------------------------------------------------
        # 信息显示
        # ----------------------------------------------------

        cv2.rectangle(
            annotated,
            (10, 10),
            (510, 175),
            (0, 0, 0),
            -1
        )

        info = [
            f"FPS: {fps:.1f}",
            f"YOLO inference: {inference_ms:.1f} ms",
            f"AirSim capture: {capture_ms:.1f} ms",
            f"Objects: {object_count}",
            f"GPU: {gpu_name}"
        ]

        y = 40

        for text in info:

            cv2.putText(
                annotated,
                text,
                (25, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2
            )

            y += 30


        # ----------------------------------------------------
        # CSV
        # ----------------------------------------------------

        writer.writerow([
            frame_id,
            datetime.now().strftime(
                "%Y-%m-%d %H:%M:%S"
            ),
            round(fps, 2),
            round(inference_ms, 2),
            round(capture_ms, 2),
            object_count
        ])

        if frame_id % 10 == 0:
            csv_file.flush()


        # ----------------------------------------------------
        # 显示
        # ----------------------------------------------------

        cv2.imshow(
            WINDOW_NAME,
            annotated
        )


        # ----------------------------------------------------
        # 键盘
        # ----------------------------------------------------

        key = cv2.waitKey(1) & 0xFF


        # Q 退出
        if key == ord("q"):
            break


        # S 截图
        if key == ord("s"):

            filename = datetime.now().strftime(
                "yolo_%Y%m%d_%H%M%S.jpg"
            )

            save_path = os.path.join(
                SCREENSHOT_DIR,
                filename
            )

            cv2.imwrite(
                save_path,
                annotated
            )

            print(
                "截图已保存:",
                save_path
            )


except KeyboardInterrupt:

    print("\nCtrl+C")


finally:

    csv_file.close()

    cv2.destroyAllWindows()


# ============================================================
# 最终统计
# ============================================================

print("\n" + "=" * 60)
print("实验结果")
print("=" * 60)

print("总帧数:", frame_id)

if fps_list:

    print(
        "平均 FPS:",
        round(
            float(np.mean(fps_list)),
            2
        )
    )

if inference_list:

    print(
        "平均 YOLO 推理时间:",
        round(
            float(np.mean(inference_list)),
            2
        ),
        "ms"
    )

if capture_list:

    print(
        "平均 AirSim 获取时间:",
        round(
            float(np.mean(capture_list)),
            2
        ),
        "ms"
    )

print("CSV:", os.path.abspath(CSV_PATH))
print("截图:", os.path.abspath(SCREENSHOT_DIR))

print("=" * 60)