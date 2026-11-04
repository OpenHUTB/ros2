import os
import mujoco
import numpy as np

XML_PATH = os.path.join(os.path.dirname(__file__), '..', 'src', 'two_dof_arm_square', 'two_dof_arm_square.xml')

def test_model_load():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model is not None
    assert data is not None

def test_ee_pos_reachable():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    
    x = data.body('ee').xpos[0]
    y = data.body('ee').xpos[1]
    assert 0.5 < x < 1.0
    assert -0.1 < y < 0.1
