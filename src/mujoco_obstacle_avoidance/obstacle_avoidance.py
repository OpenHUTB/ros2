import os
import mujoco
import numpy as np

XML_PATH = os.path.join(os.path.dirname(__file__), "diff_drive.xml")
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

mujoco.mj_forward(model, data)

WHEEL_RADIUS = 0.05
LINEAR_SPEED = 0.5
TURN_SPEED = 1.0
LINEAR_WHEEL = LINEAR_SPEED / WHEEL_RADIUS
TURN_WHEEL = TURN_SPEED / WHEEL_RADIUS

GOAL = np.array([3.0, 0.0])
GOAL_DIST = 0.15

def get_car_pose():
    x = data.body('chassis').xpos[0]
    y = data.body('chassis').xpos[1]
    theta = np.arctan2(data.body('chassis').xmat[3], data.body('chassis').xmat[0])
    return np.array([x, y, theta])

def run(headless=False):
    step = 0
    max_steps = 10000
    v = None
    avoid_steps = 0
    
    if not headless:
        try:
            from mujoco import viewer
            v = viewer.launch_passive(model, data)
        except:
            print("没有显示器，切换到无头模式")
            headless = True
    
    while step < max_steps:
        pose = get_car_pose()
        x, y, theta = pose
        dx = GOAL[0] - x
        dy = GOAL[1] - y
        dist_to_goal = np.sqrt(dx**2 + dy**2)
        
        if dist_to_goal < GOAL_DIST:
            print(f"到达目标！steps={step}, final=({x:.3f},{y:.3f})")
            break
        
        if avoid_steps > 0:
            avoid_steps -= 1
            left = LINEAR_WHEEL
            right = LINEAR_WHEEL
        else:
            cos_t = np.cos(theta)
            sin_t = np.sin(theta)
            need_avoid = False
            avoid_side = 0
            
            for obs_name in ['obstacle1', 'obstacle2']:
                obs_x = data.geom(obs_name).xpos[0]
                obs_y = data.geom(obs_name).xpos[1]
                dx_world = obs_x - x
                dy_world = obs_y - y
                dist = np.sqrt(dx_world**2 + dy_world**2)
                dx_car = cos_t * dx_world + sin_t * dy_world
                dy_car = -sin_t * dx_world + cos_t * dy_world
                if dx_car > 0 and dist < 0.5:
                    need_avoid = True
                    avoid_side = np.sign(dy_car)
                    break
            
            if need_avoid:
                avoid_steps = 200
                if avoid_side > 0:
                    left = LINEAR_WHEEL * 1.1
                    right = LINEAR_WHEEL * 0.9
                else:
                    left = LINEAR_WHEEL * 0.9
                    right = LINEAR_WHEEL * 1.1
            else:
                target_angle = np.arctan2(dy, dx)
                angle_diff = target_angle - theta
                angle_diff = (angle_diff + np.pi) % (2*np.pi) - np.pi
                
                if abs(angle_diff) > 0.15:
                    left = TURN_WHEEL * np.sign(angle_diff)
                    right = -TURN_WHEEL * np.sign(angle_diff)
                else:
                    left = LINEAR_WHEEL
                    right = LINEAR_WHEEL
        
        data.ctrl[0] = left
        data.ctrl[1] = right
        
        mujoco.mj_step(model, data)
        step += 1
        
        if not headless and v is not None and v.is_running():
            v.sync()
        
        if step % 500 == 0:
            print(f"step={step}, pos=({x:.2f},{y:.2f}), theta={theta:.2f}, dist={dist_to_goal:.2f}")
    
    if step >= max_steps:
        print(f"超时，final=({x:.3f},{y:.3f})")

if __name__ == "__main__":
    import sys
    headless = "--headless" in sys.argv
    run(headless=headless)
