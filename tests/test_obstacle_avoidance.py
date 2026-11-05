import os
import mujoco
import numpy as np

XML_PATH = os.path.join(os.path.dirname(__file__), '..', 'src', 'mujoco_obstacle_avoidance', 'obstacle_avoidance.xml')

def test_model_load():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model is not None
    assert data is not None

def test_goal_reached():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    
    WHEEL_RADIUS = 0.05
    LINEAR_SPEED = 0.5
    LINEAR_WHEEL = LINEAR_SPEED / WHEEL_RADIUS
    
    GOAL = np.array([3.0, 0.0])
    GOAL_DIST = 0.15
    
    reached = False
    for step in range(5000):
        x = data.body('chassis').xpos[0]
        y = data.body('chassis').xpos[1]
        dist = np.sqrt((GOAL[0]-x)**2 + (GOAL[1]-y)**2)
        if dist < GOAL_DIST:
            reached = True
            break
        data.ctrl[0] = LINEAR_WHEEL
        data.ctrl[1] = LINEAR_WHEEL
        mujoco.mj_step(model, data)
    
    assert reached, "小车未到达目标点"
