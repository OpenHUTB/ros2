import mujoco
import cv2
import numpy as np
import os

# 绝对路径死刑，动态获取路径
current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, '../models/car.xml')
model = mujoco.MjModel.from_xml_path(model_path)

data = mujoco.MjData(model)

cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'front_camera')
renderer = mujoco.Renderer(model, height=480, width=640)

print("W/上 = 前进    S/下 = 后退")
print("A/左 = 左转    D/右 = 右转")
print("Q = 退出")

# 初始化电机速度
data.ctrl[:] = 0.0

while True:
    renderer.update_scene(data, camera=cam_id)
    img = renderer.render()
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    cv2.imshow('car_control', img_bgr)
    key = cv2.waitKey(1) & 0xFF

    # 速度设定
    target_left = 0.0
    target_right = 0.0

    if key == ord('w') or key == 82:
        target_left = 8.0
        target_right = 8.0
    elif key == ord('s') or key == 84:
        target_left = -8.0
        target_right = -8.0
    elif key == ord('a') or key == 81:
        # 【关键修改】：左轮猛退，右轮猛进，实现极速原地左转
        target_left = -10.0
        target_right = 10.0
    elif key == ord('d') or key == 83:
        target_left = 10.0
        target_right = -10.0
    elif key == ord('q'):
        break

    # 平滑系数提高到 0.9，极速响应
    data.ctrl[0] += (target_left - data.ctrl[0]) * 0.9
    data.ctrl[1] += (target_right - data.ctrl[1]) * 0.9
    data.ctrl[2] += (target_left - data.ctrl[2]) * 0.9
    data.ctrl[3] += (target_right - data.ctrl[3]) * 0.9

    mujoco.mj_step(model, data)

cv2.destroyAllWindows()