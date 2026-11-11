#!/usr/bin/env python3
import os
import mujoco

def test_model_loads():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(script_dir, "..", "scene.xml")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    assert model.nq == 7
    assert data.qpos.shape[0] == 7
