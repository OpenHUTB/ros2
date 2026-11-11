"""Closed-loop rollouts, keyboard control, and actual simulator recordings."""
import json
from pathlib import Path
import sys
import time

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from .scene import IndoorSimulator
from .vision import VisionPolicy, mask_command


def rollout(assets, mode, weights, output, seed=7, seconds=45, record=False):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    sim = IndoorSimulator(assets,seed)
    policy = VisionPolicy(Path(weights)/f'{mode}.pt', mode) if mode in ('perception','end2end') else None
    trajectory, frames = [], []
    overview = mujoco.Renderer(sim.model,height=240,width=320) if record else None
    camera = mujoco.MjvCamera()
    camera.azimuth, camera.elevation, camera.distance = 140, -65, 3.5
    collision_steps = 0
    previous = np.zeros(2)
    start = time.time()
    try:
        for tick in range(int(seconds/.02)):
            if tick%10 == 0:
                rgb, mask = sim.image(labels=True)
                command = policy.command(rgb)[0] if policy else mask_command(mask)
                if tick<100:
                    command[:] = 0
                previous = .6*previous+.4*command
                sim.set_command(float(previous[0]),wz=float(previous[1]))
                trajectory.append([float(sim.data.time), *sim.data.qpos[:3].tolist(), sim.yaw,
                                   *previous.tolist(), *sim.odom.tolist()])
                if overview and tick%20 == 0:
                    camera.lookat[:] = sim.data.qpos[:3]
                    overview.update_scene(sim.data,camera=camera)
                    frame = Image.fromarray(overview.render())
                    inset = Image.fromarray(rgb).resize((128,96))
                    frame.paste(inset,(192,0))
                    ImageDraw.Draw(frame).text((8,8),f'{mode}  t={sim.data.time:.1f}s',fill='white')
                    frames.append(np.asarray(frame))
            sim.step()
            for contact in sim.data.contact:
                names = [mujoco.mj_id2name(sim.model,mujoco.mjtObj.mjOBJ_GEOM,int(g)) or ''
                         for g in (contact.geom1,contact.geom2)]
                if any(n.endswith('wall') or n.startswith('door_') or n=='cabinet' for n in names):
                    collision_steps += 1
                    break
            if sim.fallen or sim.data.qpos[0]>=7.5:
                break
        result = dict(mode=mode,scene_seed=seed,simulated_seconds=float(sim.data.time),
            wall_seconds=time.time()-start,fallen=bool(sim.fallen),
            reached=bool(sim.data.qpos[0]>=7.5 and not sim.fallen),
            final_position=sim.data.qpos[:3].tolist(),collision_steps=collision_steps,
            odometry_error_m=float(np.linalg.norm(sim.odom[:2]-sim.data.qpos[:2])))
        (output/f'{mode}_{seed}.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        np.savetxt(output/f'{mode}_{seed}.csv',trajectory,delimiter=',',
            header='time,x,y,z,yaw,vx,wz,odom_x,odom_y,odom_yaw',comments='')
        if frames:
            imageio.mimsave(output/f'{mode}_{seed}.gif',frames,duration=.4,loop=0)
            Image.fromarray(frames[-1]).save(output/f'{mode}_{seed}.png')
        return result
    finally:
        if overview:
            overview.close()
        sim.close()


def teleop(assets,seed=0,seconds=45,output=None):
    """Interactive MuJoCo viewer; arrow/WASD commands expire after 0.6s."""
    import mujoco.viewer
    sim = IndoorSimulator(assets,seed)
    command = np.zeros(2)
    speed = [.25]
    last_key = [0.]
    events = []
    def key(code):
        if code in (87,265): command[:] = [speed[0],0]
        elif code in (83,264): command[:] = [-speed[0],0]
        elif code in (65,263): command[:] = [0,.35]
        elif code in (68,262): command[:] = [0,-.35]
        elif code == 32: command[:] = 0
        elif code in (61,334): speed[0] = min(.4,speed[0]+.05)
        elif code in (45,333): speed[0] = max(.1,speed[0]-.05)
        last_key[0] = time.monotonic()
        events.append(dict(key=code,command=command.tolist(),time=float(sim.data.time)))
    print('WASD/arrows: move, space: stop, +/-: speed; repeat presses to keep moving.',flush=True)
    try:
        with mujoco.viewer.launch_passive(sim.model,sim.data,key_callback=key) as viewer:
            while viewer.is_running() and not sim.fallen and sim.data.time<seconds:
                start = time.monotonic()
                if time.monotonic()-last_key[0]>.6: command[:] = 0
                sim.set_command(command[0],wz=command[1])
                sim.step()
                viewer.sync()
                time.sleep(max(0,.02-(time.monotonic()-start)))
    finally:
        sim.set_command()
        if output:
            output=Path(output)
            output.mkdir(parents=True,exist_ok=True)
            (output/'teleop_check.json').write_text(json.dumps(dict(events=events,
                final_position=sim.data.qpos[:3].tolist(),fallen=bool(sim.fallen),
                simulated_seconds=float(sim.data.time)),indent=2),encoding='utf-8')
        sim.close()
