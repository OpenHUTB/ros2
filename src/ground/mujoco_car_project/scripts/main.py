import mujoco
import cv2
import numpy as np
import os
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, '../models/car.xml')
model = mujoco.MjModel.from_xml_path(model_path)

data = mujoco.MjData(model)

cam_main_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'front_camera') 
cam_fpv_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'fpv_camera')    

renderer = mujoco.Renderer(model, height=720, width=1280)

print("W/A/S/D 控制移动 | Q 退出")

data.ctrl[:] = 0.0
cv2.namedWindow('car_control', cv2.WINDOW_NORMAL)
cv2.resizeWindow('car_control', 1200, 800)

while True:
    start_time = time.time()

    # 1. 渲染主视角
    renderer.update_scene(data, camera=cam_main_id)
    img_main = renderer.render()
    img_main_bgr = cv2.cvtColor(img_main, cv2.COLOR_RGB2BGR)

    # 2. 渲染车头视角
    renderer.update_scene(data, camera=cam_fpv_id)
    img_fpv = renderer.render()
    img_fpv_bgr = cv2.cvtColor(img_fpv, cv2.COLOR_RGB2BGR)
    
    # 【关键修改】：把画中画缩小到 1/4 大小
    fpv_h, fpv_w, _ = img_fpv_bgr.shape
    small_fpv = cv2.resize(img_fpv_bgr, (fpv_w//4, fpv_h//4))
    
    main_h, main_w, _ = img_main_bgr.shape
    small_h, small_w, _ = small_fpv.shape
    
    # 放置在右上角
    offset_x = main_w - small_w - 20
    offset_y = 20
    
    # 贴图并加细白框
    img_main_bgr[offset_y:offset_y+small_h, offset_x:offset_x+small_w] = small_fpv
    cv2.rectangle(img_main_bgr, (offset_x-2, offset_y-2), (offset_x+small_w+2, offset_y+small_h+2), (255, 255, 255), 2)
    
    # 在画中画左上角写个字
    cv2.putText(img_main_bgr, "FPV", (offset_x + 5, offset_y + 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    cv2.imshow('car_control', img_main_bgr)
    key = cv2.waitKey(1) & 0xFF

    # --- 控制逻辑不变 ---
    target_left = 0.0
    target_right = 0.0

    if key == ord('w') or key == 82:
        target_left = 8.0
        target_right = 8.0
    elif key == ord('s') or key == 84:
        target_left = -8.0
        target_right = -8.0
    elif key == ord('a') or key == 81:
        target_left = -10.0
        target_right = 10.0
    elif key == ord('d') or key == 83:
        target_left = 10.0
        target_right = -10.0
    elif key == ord('q'):
        break

    data.ctrl[0] += (target_left - data.ctrl[0]) * 0.9
    data.ctrl[1] += (target_right - data.ctrl[1]) * 0.9
    data.ctrl[2] += (target_left - data.ctrl[2]) * 0.9
    data.ctrl[3] += (target_right - data.ctrl[3]) * 0.9

    mujoco.mj_step(model, data)
    
    elapsed = time.time() - start_time
    if elapsed < 0.01:
        time.sleep(0.01 - elapsed)

cv2.destroyAllWindows()