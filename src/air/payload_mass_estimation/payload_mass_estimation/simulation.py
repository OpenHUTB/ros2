"""Vertical-flight experiments with a fixed controller independent of payload."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
DT = .05
EMPTY_MASS = .85


def collect(seed, payload_kg, duration=12., thrust_scale=1., render_path=None):
    rng = np.random.default_rng(seed)
    xml = ET.parse(ROOT / 'models/community_quadrotor.xml')
    body = xml.find("worldbody/body[@name='base_link']")
    body.find('inertial').set('mass', str(EMPTY_MASS + payload_kg))
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml.getroot(), encoding='unicode'))
    data = mujoco.MjData(model)
    data.qpos[:3] = [0, 0, 3.]
    data.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    # Fixed nominal controller. The payload label is never used in this control law.
    integral = 0.
    phase = rng.uniform(0, 2*np.pi)
    rows = []
    substeps = round(DT / model.opt.timestep)
    for k in range(round(duration / DT)):
        target = 3. + .25*np.sin(.8*data.time + phase)
        error = target - data.qpos[2]
        integral = np.clip(integral + error*DT, -2., 2.)
        thrust = np.clip(EMPTY_MASS*9.81 + 5.*error - 3.*data.qvel[2] + 2.*integral, 0., 22.)
        # Equal rotor inputs retain the level attitude in this vertical-only experiment.
        data.ctrl[:] = thrust * thrust_scale / 4.
        rows.append([data.time, data.qpos[2], data.qvel[2], thrust])
        for _ in range(substeps):
            mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or data.qpos[2] < .3:
            raise RuntimeError('Invalid flight; do not include in training silently')
        if render_path and k==100:
            model.vis.global_.offwidth=900;model.vis.global_.offheight=600
            camera=mujoco.MjvCamera();camera.lookat[:]=data.qpos[:3];camera.distance=1.5;camera.azimuth=135;camera.elevation=-20
            with mujoco.Renderer(model,height=600,width=900) as renderer:
                renderer.update_scene(data,camera=camera)
                import matplotlib.pyplot as plt
                plt.imsave(render_path,renderer.render())
    return np.asarray(rows)
