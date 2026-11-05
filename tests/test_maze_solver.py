import os
import mujoco
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'diff_drive_maze'))

def test_model_load():
    xml_path = os.path.join(os.path.dirname(__file__), '..', 'src', 'diff_drive_maze', 'diff_drive_maze.xml')
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.nq == 7
    assert model.nu == 2

def test_robot_initial_pos():
    xml_path = os.path.join(os.path.dirname(__file__), '..', 'src', 'diff_drive_maze', 'diff_drive_maze.xml')
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    x = data.body('robot').xpos[0]
    y = data.body('robot').xpos[1]
    assert abs(x - (-2.0)) < 0.01
    assert abs(y - (-2.0)) < 0.01
