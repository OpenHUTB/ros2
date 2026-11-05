"""MuJoCo landing-contact experiments; labels come from physical contacts."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
DT = .05
FOOT_Z = .073


def rotation(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def angles(q):
    w, x, y, z = q
    return np.array([np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
                     np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))])


def make_model(seed):
    rng = np.random.default_rng(seed)
    root = ET.fromstring((ROOT/'models/community_quadrotor.xml').read_text())
    root.find('option').set('timestep', '.001')
    root.find('option').set('integrator', 'implicitfast')
    world = root.find('worldbody')
    ET.SubElement(world, 'geom', name='landing_pad', type='plane', size='4 4 .05',
                  material='grid', friction='0.7 0.01 0.001', contype='1', conaffinity='1')
    body = world.find("body[@name='base_link']")
    body.find('inertial').set('mass', str(rng.uniform(.70, 1.10)))
    softness = rng.uniform(.009, .024)
    for i, (x,y) in enumerate([(.065,.065),(.065,-.065),(-.065,.065),(-.065,-.065)]):
        ET.SubElement(body, 'geom', name='foot'+str(i), type='sphere', size='.018',
                      pos=f'{x} {y} -.055', mass='.003', rgba='.85 .45 .12 1',
                      contype='1', conaffinity='1', solref=f'{softness} 1',
                      friction='0.7 0.01 0.001')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    return model, {'seed':int(seed), 'pad_time_constant_s':float(softness)}


def episode(seed, render_path=None):
    rng = np.random.default_rng(seed+1000000)
    model, info = make_model(seed)
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
    mass = float(model.body_subtreemass[body])
    height = rng.uniform(.85, 1.45)
    descent = rng.uniform(.30, 1.65)
    hover_end = rng.uniform(.85, 1.35)
    data.qpos[:3] = [0,0,height+FOOT_Z]
    data.qpos[3:7] = [1,0,0,0]
    mujoco.mj_forward(model,data)
    mix = np.array([[1,1,1,1],[-.106,.106,.106,-.106],
                    [-.106,.106,-.106,.106],[-.018,-.018,.018,.018]])
    inverse = np.linalg.inv(mix)
    contact_time = None
    peak = 0.
    records = []
    wind = rng.uniform(-.10,.10,2)
    image_saved = False
    for step in range(10000):
        t = data.time
        attitude = angles(data.qpos[3:7])
        data.xfrc_applied[:] = 0
        if contact_time is None:
            target_z = height+FOOT_Z-max(0,t-hover_end)*descent
            target_vz = -descent if t >= hover_end else 0.
            thrust = mass*(9.81 + 12*(target_z-data.qpos[2]) + 5*(target_vz-data.qvel[2]))
            desired = np.array([(.8*data.qvel[1]+1.5*data.qpos[1])/9.81,
                               (-.8*data.qvel[0]-1.5*data.qpos[0])/9.81,0.])
            torque = -.18*(attitude-desired)-.045*data.qvel[3:6]
            data.ctrl[:] = np.clip(inverse@np.r_[np.clip(thrust,0,22),torque],0,6)
            rotation = data.xmat[body].reshape(3,3)
            data.xfrc_applied[body,3:] = rotation@np.array([0,0,torque[2]])
            data.xfrc_applied[body,:2] = wind
        else:
            data.ctrl[:] = 0
        mujoco.mj_step(model,data)
        normal = 0.
        for i in range(data.ncon):
            force = np.zeros(6)
            mujoco.mj_contactForce(model,data,i,force)
            normal += max(0,float(force[0]))
        if normal > .1 and contact_time is None:
            contact_time = float(data.time)
        if contact_time is not None and data.time-contact_time <= .25:
            peak = max(peak,normal/(mass*9.81))
        if step % 50 == 0:
            phase = 0 if data.time < hover_end else 1
            if contact_time is not None:
                phase = 3 if data.time-contact_time>.30 and np.linalg.norm(data.qvel[:3])<.10 else 2
            position = data.qpos[:3].copy()+rng.normal(0,.0008,3)
            velocity = data.qvel[:3].copy()+rng.normal(0,.008,3)
            omega = data.qvel[3:6].copy()+rng.normal(0,.004,3)
            records.append(np.r_[data.time,position,velocity,data.qpos[3:7].copy(),omega,normal,phase])
            if render_path and not image_saved and data.time>hover_end+.25:
                import matplotlib.pyplot as plt
                renderer = mujoco.Renderer(model,height=600,width=900)
                camera = mujoco.MjvCamera();camera.lookat[:]=[0,0,.45]
                camera.distance=2.5;camera.azimuth=130;camera.elevation=-20
                renderer.update_scene(data,camera=camera)
                plt.imsave(str(render_path),renderer.render());renderer.close();image_saved=True
        if contact_time is not None and data.time-contact_time>1.25:
            break
    if contact_time is None or not np.isfinite(peak) or peak <= 1:
        raise RuntimeError(f'Invalid landing episode {seed}: contact={contact_time}, peak={peak}')
    info.update(mass_kg=mass,descent_command_mps=float(descent),hover_end_s=float(hover_end),
                contact_time_s=contact_time,peak_load_factor=peak,engine='MuJoCo '+mujoco.__version__)
    return np.asarray(records,dtype=np.float64),info


def collect(output, final=False):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    count=24 if final else 120
    entries=[]
    for i in range(count):
        seed=(20000 if final else 1000)+i
        split='final' if final else ('train' if i<80 else 'val' if i<96 else 'test')
        raw,meta=episode(seed)
        name=f'episode_{seed}.npz';np.savez_compressed(output/name,raw=raw)
        meta.update(file=name,split=split,sha256=hashlib.sha256((output/name).read_bytes()).hexdigest())
        entries.append(meta)
    (output/'manifest.json').write_text(json.dumps(entries,indent=2))
    print('Collected',len(entries),'actual MuJoCo landing experiments',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',default='data/raw');p.add_argument('--final',action='store_true')
    a=p.parse_args();collect(a.output,a.final)
