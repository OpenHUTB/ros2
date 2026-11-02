import mujoco
import cv2
import numpy as np
import os

# 【修复】：动态获取脚本所在路径，向上找一级到 models 文件夹
current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, '../models/car.xml')
model = mujoco.MjModel.from_xml_path(model_path)

data = mujoco.MjData(model)

cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'front_camera')
renderer = mujoco.Renderer(model, height=480, width=640)

print("W/上 = 前进    S/下 = 后退")
print("A/左 = 左转    D/右 = 右转")
print("Q = 退出")

while True:
    # 默认速度为 0 (每帧都重置，松开就停)
    speed_left = 0.0
    speed_right = 0.0

    renderer.update_scene(data, camera=cam_id)
    img = renderer.render()
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    cv2.imshow('car_control', img_bgr)
    key = cv2.waitKey(1) & 0xFF

    # 只有当前按下了对应键，才设置速度
    if key == ord('w') or key == 82:
        speed_left = 3.0
        speed_right = 3.0
    elif key == ord('s') or key == 84:
        speed_left = -3.0
        speed_right = -3.0
    elif key == ord('a') or key == 81:
        speed_left = -2.0
        speed_right = 2.0
    elif key == ord('d') or key == 83:
        speed_left = 2.0
        speed_right = -2.0
    elif key == ord('q'):
        break

    data.ctrl[0] = speed_left
    data.ctrl[1] = speed_right
    data.ctrl[2] = speed_left
    data.ctrl[3] = speed_right

    mujoco.mj_step(model, data)

cv2.destroyAllWindows()