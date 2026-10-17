"""Carla 交通标志识别（Windows 端脚本版）

从 Carla RGB 相机获取图像，使用 YOLOv8 神经网络识别交通标志（stop sign），
并通过键盘控制车辆。与 ROS 版 carla_tsr_ros 互补。

运行前置条件：
1. Carla/OpenHUTB 服务端已启动并完成地图加载（默认 127.0.0.1:2000）
2. 已安装 Carla Python API（hutb whl）
3. yolov8n.pt 权重放在模块目录下

键盘操作需先点击 OpenCV 窗口激活焦点，否则按键无效。
"""
import os
import sys
import time
import random
import argparse

import cv2
import numpy as np

# ============================================================================
# 参数配置（集中管理，便于扩展）
# ============================================================================
CONFIG = {
    # Carla 连接
    'host': '127.0.0.1',
    'port': 2000,
    'timeout': 10.0,
    # 相机
    'camera_width': 640,
    'camera_height': 360,
    'camera_fov': 90,
    # 神经网络
    'model_filename': 'yolov8n.pt',
    'target_class': 11,           # COCO: stop sign
    'infer_every_n_frames': 4,    # 每 N 帧推理一次，降频
    # 控制
    'throttle_step': 0.06,
    'steer_step': 0.10,
    'key_hold_timeout': 0.18,
    # 主循环帧率控制
    'loop_sleep': 0.02,
}


# ============================================================================
# 权重路径解析：优先模块目录，其次当前目录
# ============================================================================
def resolve_model_path(filename):
    module_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(module_dir, filename),
        os.path.join(os.getcwd(), filename),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    print("[ERROR] 未找到权重文件 " + filename)
    print("[ERROR] 请下载 yolov8n.pt 并放到以下任一位置：")
    for path in candidates:
        print("  - " + path)
    print("  下载地址：https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt")
    sys.exit(1)


# ============================================================================
# 全局状态
# ============================================================================
current_frame = None
latest_annotated_frame = None
frame_count = 0
model = None


def camera_callback(image):
    """相机回调：图像解码 + 降频推理"""
    global current_frame, latest_annotated_frame, frame_count

    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    frame = array[:, :, :3]

    frame_count += 1
    if frame_count % CONFIG['infer_every_n_frames'] == 0:
        results = model(frame, classes=[CONFIG['target_class']], verbose=False)
        latest_annotated_frame = results[0].plot()

    current_frame = latest_annotated_frame if latest_annotated_frame is not None else frame


def main():
    global model

    parser = argparse.ArgumentParser(description='Carla Traffic Sign Recognition')
    parser.add_argument('--host', default=CONFIG['host'], help='Carla 服务端地址')
    parser.add_argument('--port', type=int, default=CONFIG['port'], help='Carla 服务端端口')
    args = parser.parse_args()

    # 1. 加载权重
    model_path = resolve_model_path(CONFIG['model_filename'])
    print("[INFO] 加载 YOLO 权重: " + model_path)
    from ultralytics import YOLO
    model = YOLO(model_path)

    # 2. 连接 Carla
    import carla
    print("[INFO] 连接 Carla: " + args.host + ":" + str(args.port))
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(CONFIG['timeout'])
        world = client.get_world()
    except Exception as e:
        print("[ERROR] 连接 Carla 失败: " + str(e))
        print("[ERROR] 请检查：")
        print("  1. Carla/OpenHUTB 服务端是否已启动")
        print("  2. 地图是否加载完成")
        print("  3. host/port 参数是否正确（默认 127.0.0.1:2000）")
        sys.exit(1)

    blueprint_library = world.get_blueprint_library()

    # 3. 生成车辆
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    spawn_point = random.choice(world.get_map().get_spawn_points())
    try:
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    except Exception as e:
        print("[ERROR] 生成车辆失败: " + str(e))
        sys.exit(1)
    print("[INFO] 车辆已生成")

    # 4. 挂载相机
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(CONFIG['camera_width']))
    camera_bp.set_attribute('image_size_y', str(CONFIG['camera_height']))
    camera_bp.set_attribute('fov', str(CONFIG['camera_fov']))
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(lambda image: camera_callback(image))

    # 5. 控制状态
    control = carla.VehicleControl()
    last_input_time = 0.0
    target_throttle = 0.0
    target_steer = 0.0
    target_reverse = False

    print()
    print("=========================================")
    print("准备就绪！请点击 OpenCV 窗口激活键盘焦点。")
    print("键盘操作说明（必须点击 OpenCV 窗口后再按键）：")
    print("  [W] : 油门前进")
    print("  [S] : 刹车 / 倒车")
    print("  [A] : 左转")
    print("  [D] : 右转")
    print("  [Q] : 退出程序")
    print("=========================================")
    print()

    # 6. 主循环
    try:
        while True:
            world.wait_for_tick()

            if current_frame is not None:
                cv2.imshow("Carla Traffic Sign Recognition", current_frame)

            key = cv2.waitKey(1) & 0xFF
            now = time.monotonic()

            if key == ord('q'):
                break
            elif key == ord('w'):
                target_throttle = 0.55
                target_steer = 0.0
                target_reverse = False
                last_input_time = now
            elif key == ord('s'):
                target_throttle = 0.35
                target_steer = 0.0
                target_reverse = True
                last_input_time = now
            elif key == ord('a'):
                target_steer = -0.45
                last_input_time = now
            elif key == ord('d'):
                target_steer = 0.45
                last_input_time = now
            elif now - last_input_time > CONFIG['key_hold_timeout']:
                target_throttle = 0.0
                target_steer = 0.0

            current_speed = vehicle.get_velocity().length()
            if current_speed < 0.2:
                control.reverse = target_reverse

            control.throttle += np.clip(target_throttle - control.throttle,
                                        -CONFIG['throttle_step'], CONFIG['throttle_step'])
            control.steer += np.clip(target_steer - control.steer,
                                     -CONFIG['steer_step'], CONFIG['steer_step'])
            control.brake = 0.0

            vehicle.apply_control(control)

            # 帧率控制，避免满载 CPU
            time.sleep(CONFIG['loop_sleep'])

    except KeyboardInterrupt:
        print()
        print("[INFO] 手动中断")
    finally:
        print("[INFO] 清理资源...")
        try:
            camera.stop()
            camera.destroy()
        except Exception:
            pass
        try:
            vehicle.destroy()
        except Exception:
            pass
        cv2.destroyAllWindows()
        print("[INFO] 清理完成，程序退出")


if __name__ == '__main__':
    main()
