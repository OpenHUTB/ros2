"""Carla 交通标志识别（Windows 端脚本版）"""
import os
import sys
import time
import random
import argparse
import cv2
import numpy as np

CONFIG = {
    'host': '127.0.0.1', 'port': 2000, 'timeout': 10.0,
    'camera_width': 640, 'camera_height': 360, 'camera_fov': 90,
    'model_filename': 'yolov8n.pt', 'target_class': 11,
    'infer_interval': 0.2, 'throttle_step': 0.06, 'steer_step': 0.10,
    'key_hold_timeout': 0.18, 'loop_sleep': 0.02,
}

def resolve_model_path(filename):
    module_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(module_dir, filename), os.path.join(os.getcwd(), filename)]
    for path in candidates:
        if os.path.exists(path): return path
    print("[ERROR] 未找到权重文件 " + filename)
    sys.exit(1)

current_frame = None
latest_annotated_frame = None
last_infer_time = time.monotonic()
model = None

def camera_callback(image):
    global current_frame, latest_annotated_frame, last_infer_time
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    frame = array[:, :, :3]
    
    now = time.monotonic()
    if now - last_infer_time > CONFIG['infer_interval']:
        last_infer_time = now
        results = model(frame, classes=[CONFIG['target_class']], verbose=False)
        latest_annotated_frame = results[0].plot()
        
    current_frame = latest_annotated_frame if latest_annotated_frame is not None else frame

def main():
    global model
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default=CONFIG['host'])
    parser.add_argument('--port', type=int, default=CONFIG['port'])
    args = parser.parse_args()

    model_path = resolve_model_path(CONFIG['model_filename'])
    from ultralytics import YOLO
    model = YOLO(model_path)
    # 预热：消除首帧推理延迟（避免阻塞相机回调）
    dummy = np.zeros((CONFIG['camera_height'], CONFIG['camera_width'], 3), dtype=np.uint8)
    model(dummy, verbose=False)
    print("[INFO] 模型预热完成")

    print("[INFO] 连接 Carla: " + args.host + ":" + str(args.port))
    try:
        import carla
        client = carla.Client(args.host, args.port)
        client.set_timeout(CONFIG['timeout'])
        world = client.get_world()
    except ImportError:
        print("[ERROR] 无法导入 carla，请先安装本地 whl（见 README）。")
        sys.exit(1)
    except Exception as e:
        print("[ERROR] 连接 Carla 失败: " + str(e))
        sys.exit(1)

    bp_lib = world.get_blueprint_library()
    vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    vehicle = None
    for spawn_point in spawn_points:
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle is not None:
            break
    if vehicle is None:
        print("[ERROR] 无法生成车辆，所有出生点均被占用。")
        sys.exit(1)

    try:
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(CONFIG['camera_width']))
        camera_bp.set_attribute('image_size_y', str(CONFIG['camera_height']))
        camera_bp.set_attribute('fov', str(CONFIG['camera_fov']))
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        camera.listen(lambda image: camera_callback(image))
    except Exception as e:
        print("[ERROR] 相机生成失败: " + str(e))
        vehicle.destroy()
        sys.exit(1)

    control = carla.VehicleControl()
    last_input_time = 0.0
    target_throttle = 0.0
    target_steer = 0.0
    target_reverse = False

    print("准备就绪！点击 OpenCV 窗口后按 W/A/S/D 控制。")
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
                control.brake = 0.0
                target_reverse = False
                last_input_time = now
            elif key == ord('s'):
                target_steer = 0.0
                current_speed = vehicle.get_velocity().length()
                if current_speed > 0.2:
                    target_throttle = 0.0
                    control.brake = 1.0
                    target_reverse = False
                else:
                    control.brake = 0.0
                    target_throttle = 0.35
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
                control.brake = 0.0

            if vehicle.get_velocity().length() < 0.2:
                control.reverse = target_reverse

            control.throttle += np.clip(target_throttle - control.throttle, -CONFIG['throttle_step'], CONFIG['throttle_step'])
            control.steer += np.clip(target_steer - control.steer, -CONFIG['steer_step'], CONFIG['steer_step'])
            vehicle.apply_control(control)
            time.sleep(CONFIG['loop_sleep'])

    finally:
        try:
            camera.stop()
        except Exception:
            pass
        try:
            camera.destroy()
        except Exception:
            pass
        try:
            vehicle.destroy()
        except Exception:
            pass
        cv2.destroyAllWindows()
        print("清理完成。")

if __name__ == '__main__':
    main()
