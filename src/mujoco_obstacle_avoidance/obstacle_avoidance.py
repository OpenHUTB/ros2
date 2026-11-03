import mujoco
import numpy as np
import glfw
from scipy.spatial.transform import Rotation as R

model = mujoco.MjModel.from_xml_path("obstacle_avoidance.xml")
data = mujoco.MjData(model)

camera = mujoco.MjvCamera()
mujoco.mjv_defaultFreeCamera(model, camera)
camera.distance = 5
camera.elevation = -20
camera.azimuth = 45

renderer = mujoco.Renderer(model)

GOAL_X = 3.0
GOAL_Y = 2.0
LINEAR_SPEED = 0.08
ANGULAR_SPEED = 0.1
OBSTACLE_THRESHOLD = 0.8
LEFT_AVOID_SPEED = 0.15
RIGHT_AVOID_SPEED = 0.0

obstacles = np.array([[1.0, 0.5], [2.0, -0.5]])

def get_car_pose():
    pos = data.body("chassis").xpos.copy()
    quat = data.body("chassis").xquat.copy()
    r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
    yaw = r.as_euler("xyz")[2]
    return pos[0], pos[1], yaw

def run():
    if not glfw.init():
        return
    window = glfw.create_window(1200, 900, "MuJoCo 小车避障", None, None)
    glfw.make_context_current(window)

    while not glfw.window_should_close(window):
        car_x, car_y, car_yaw = get_car_pose()

        dx = GOAL_X - car_x
        dy = GOAL_Y - car_y
        dist_to_goal = np.sqrt(dx**2 + dy**2)

        if dist_to_goal < 0.2:
            data.ctrl[0] = 0
            data.ctrl[1] = 0
            print("到达目标点！")
        else:
            goal_yaw = np.arctan2(dy, dx)
            yaw_error = goal_yaw - car_yaw
            yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

            min_obs_dist = 999
            for obs in obstacles:
                dist = np.sqrt((car_x - obs[0])**2 + (car_y - obs[1])**2)
                min_obs_dist = min(min_obs_dist, dist)

            if min_obs_dist < OBSTACLE_THRESHOLD:
                left_speed = LEFT_AVOID_SPEED
                right_speed = RIGHT_AVOID_SPEED
            else:
                left_speed = LINEAR_SPEED - ANGULAR_SPEED * yaw_error
                right_speed = LINEAR_SPEED + ANGULAR_SPEED * yaw_error

            data.ctrl[0] = left_speed
            data.ctrl[1] = right_speed

        mujoco.mj_step(model, data)
        renderer.update_scene(data, camera)
        pixels = renderer.render()
        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.terminate()

if __name__ == "__main__":
    run()
    print("仿真结束")
