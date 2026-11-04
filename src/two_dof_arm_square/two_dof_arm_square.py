import os
import mujoco
import numpy as np

XML_PATH = os.path.join(os.path.dirname(__file__), "two_dof_arm_square.xml")
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

mujoco.mj_forward(model, data)

L1 = 0.5
L2 = 0.5

def inverse_kinematics(x, y):
    d = np.sqrt(x**2 + y**2)
    d = min(d, L1 + L2 - 0.01)
    cos_q2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_q2 = np.clip(cos_q2, -1, 1)
    q2 = np.arccos(cos_q2)
    q1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    return q1, q2

def get_ee_pos():
    x = data.body('ee').xpos[0]
    y = data.body('ee').xpos[1]
    return np.array([x, y])

def run(headless=False):
    v = None
    if not headless:
        try:
            from mujoco import viewer
            v = viewer.launch_passive(model, data)
        except:
            print("没有显示器，切换到无头模式")
            headless = True
    
    square_points = [
        (0.3, 0.3),
        (0.7, 0.3),
        (0.7, 0.7),
        (0.3, 0.7),
        (0.3, 0.3),
    ]
    
    trail_idx = 0
    step = 0
    
    for target_x, target_y in square_points:
        q1_target, q2_target = inverse_kinematics(target_x, target_y)
        print(f"目标点: ({target_x:.2f}, {target_y:.2f}), q1={q1_target:.3f}, q2={q2_target:.3f}")
        
        for i in range(500):
            data.ctrl[0] = q1_target
            data.ctrl[1] = q2_target
            
            mujoco.mj_step(model, data)
            step += 1
            
            if step % 20 == 0 and trail_idx < 16:
                ee_pos = get_ee_pos()
                trail_name = f"trail{trail_idx}"
                geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, trail_name)
                model.geom_rgba[geom_id, 3] = 1.0
                model.geom_pos[geom_id, 0] = ee_pos[0]
                model.geom_pos[geom_id, 1] = ee_pos[1]
                model.geom_pos[geom_id, 2] = 0.5
                trail_idx += 1
            
            if not headless and v is not None and v.is_running():
                v.sync()
    
    print(f"画完正方形！总步数: {step}")

if __name__ == "__main__":
    import sys
    headless = "--headless" in sys.argv
    run(headless=headless)
