import mujoco as mj
from mujoco import viewer
import numpy as np
import math
import time

model = mj.MjModel.from_xml_path("two_dof_arm_square.xml")
data = mj.MjData(model)

L1 = 0.4
L2 = 0.25
square_size = 0.2
center_x = 0.4
center_y = 0.0

def ik(x, y):
    r_sq = x**2 + y**2
    cos_q2 = (r_sq - L1**2 - L2**2) / (2*L1*L2)
    cos_q2 = max(-1.0, min(1.0, cos_q2))
    q2 = math.acos(cos_q2)
    q1 = math.atan2(y, x) - math.atan2(L2*math.sin(q2), L1 + L2*math.cos(q2))
    return q1, q2

if __name__ == "__main__":
    v = viewer.launch_passive(model, data)
    start_q1, start_q2 = ik(center_x - square_size/2, center_y - square_size/2)
    data.ctrl[0] = start_q1
    data.ctrl[1] = start_q2
    for i in range(500):
        mj.mj_step(model, data)
        v.sync()
        time.sleep(0.01)
    corners = [
        (center_x - square_size/2, center_y - square_size/2),
        (center_x + square_size/2, center_y - square_size/2),
        (center_x + square_size/2, center_y + square_size/2),
        (center_x - square_size/2, center_y + square_size/2),
    ]
    total_steps_per_edge = 1000
    step = 0
    while v.is_running():
        seg = step // total_steps_per_edge
        t = (step % total_steps_per_edge) / total_steps_per_edge
        x0, y0 = corners[seg % 4]
        x1, y1 = corners[(seg+1) % 4]
        x = x0 + (x1-x0)*t
        y = y0 + (y1-y0)*t
        q1, q2 = ik(x, y)
        data.ctrl[0] = q1
        data.ctrl[1] = q2
        mj.mj_step(model, data)
        v.sync()
        step += 1
        time.sleep(0.005)
