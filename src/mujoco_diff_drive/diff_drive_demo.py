import mujoco
import mujoco.viewer
import time
import math

GOAL_X = 3.0
GOAL_Y = 2.0
WHEEL_RADIUS = 0.05
WHEEL_SEP = 0.22

reached = False
step_count = 0

def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

def control(model, data):
    global reached, step_count
    step_count += 1

    x = data.qpos[0]
    y = data.qpos[1]
    yaw = yaw_from_quat(data.qpos[3:7])

    dx = GOAL_X - x
    dy = GOAL_Y - y
    dist = math.hypot(dx, dy)
    goal_angle = math.atan2(dy, dx)
    angle_diff = goal_angle - yaw
    while angle_diff > math.pi: angle_diff -= 2 * math.pi
    while angle_diff < -math.pi: angle_diff += 2 * math.pi

    if step_count % 50 == 0:
        print("x=%.2f y=%.2f yaw=%.2f angle_diff=%.2f dist=%.2f" %
              (x, y, yaw, angle_diff, dist))

    if dist < 0.15:
        data.ctrl[0] = 0.0
        data.ctrl[1] = 0.0
        if not reached:
            print(">>> 到达目标点！当前 (%.2f, %.2f)" % (x, y))
            reached = True
        return

    if abs(angle_diff) > 0.3:
        linear = 0.0
        angular = 0.1 * math.copysign(1.0, angle_diff)
    else:
        linear = 0.08
        angular = 0.0

    v_left = (linear - angular * WHEEL_SEP / 2) / WHEEL_RADIUS
    v_right = (linear + angular * WHEEL_SEP / 2) / WHEEL_RADIUS
    data.ctrl[0] = v_left
    data.ctrl[1] = v_right

if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path("diff_drive.xml")
    data = mujoco.MjData(model)
    mujoco.set_mjcb_control(control)
    print("小车从原点出发，目标点 (%.1f, %.1f)" % (GOAL_X, GOAL_Y))
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)
